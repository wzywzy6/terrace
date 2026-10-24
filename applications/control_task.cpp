
#include "control_task.hpp"

#include <math.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/bmi088/bmi088.hpp"
#include "task.h"
#include "tools/detect.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"
#include "trajectory_task.hpp"

GimbalData rm_motor_data_x;
GimbalData rm_motor_data_y;
GimbalData rm_motor_data_z;
extern sp::Mahony imu;

/* ===== 红外触发状态 ===== */
static bool pitch_locked = false;	  // false=锁定保持 kPitchBias；true=已检测到 <3cm 解锁
static uint8_t dist_confirm_cnt = 0;  // 连续确认计数，做防抖
static constexpr float kDistTriggerCm = 60.0f;	// 触发距离
static constexpr uint8_t kDistConfirmN = 2;		// 连续 2 拍确认

/*前馈增益*/
static constexpr float kPitchVelFf = 1.0f;	//参考速度放大多少倍
static constexpr float kYawVelFf = 1.3f;
static constexpr float kPitchAccFf = 0.0025f;  //参考加速度要乘多大惯量变成力矩 kg*m^2
static constexpr float kYawAccFf = 0.00338f;   /*0.0032f*/
;

static constexpr float kPitchBias = -4.4f;	//绝对目标角 rad
static constexpr float kYawBias = -1.65f;

static constexpr float kPitchSign = 1.0f;  //方向标定（电机与机构正方向比例）
static constexpr float kYawSign = 1.0f;

/* ===== 进入 MID 后先保持的时长：至少 2s 只锁 kPitchBias ===== */
static constexpr uint32_t kMidBootHoldMs = 2000U;

/* ===== 进入 UP 后先保持的时长（UP 挡原逻辑保持不变） ===== */
static constexpr uint32_t kStartupHoldMs = 500U;

/* ===== 重力补偿参数 ===== */
static constexpr float kPitchGravAmp = -0.33f;	 // 重力力矩幅值 N·m
static constexpr float kPitchGravPhase = 0.35f;	 // 重力力矩相位 rad
/* UP 挡位进入后重力力矩的爬升时间,0 表示立即满输出 */
static constexpr uint32_t kUpGravRampMs = 300U;

float imu_last_yaw = 0.0f;
float yaw_ratio;
float pitch_ratio;
float M = 0.00415;
float G = 0.29516;
float B = 0.005634;
float F = 0.033428;

float Fsign(float fsign)
{
	if (fsign > 0) {
		return 1;
	}
	else if (fsign < 0) {
		return -1;
	}
	else {
		return 0;
	}
}

// 重力压在 pitch 轴上的力矩,只与 IMU 测到的重力方向 imu.roll 有关
static float pitch_gravity_torque() { return kPitchGravAmp * cosf(kPitchGravPhase - imu.roll); }

sp::PID rm_motor_pid_angle_x(0.001f, 28.0f, 0.0f, 1.0f, 50.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_x(0.001f, 0.06f, 0.0f, 0.0f, 0.5f, 0.5f, 1.0f, false, true);
sp::PID rm_motor_pid_angle_y(0.001f, 40.0f, 0.0f, 1.0f, 400.0f, 5.0f, 0.8f, true, true);
sp::PID rm_motor_pid_speed_y(0.001f, 0.165f, 0.0f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);
sp::PID rm_motor_pid_angle_z(0.001f, 20.0f, 0.0f, 0.0f, 400.0f, 5.0f, 1.0f, false, true);
sp::PID rm_motor_pid_speed_z(0.001f, 0.0002f, 0.0001f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

extern "C" void control_task()
{
	remote.request();
	gy56.begin();
	osDelay(100);
	can_start();
	osDelay(100);

	rm_motor_data_x.given_torque = 0.0f;
	rm_motor_data_y.given_torque = 0.0f;

	/* ===== 进入 MID 时刻与状态 ===== */
	TickType_t mid_enter_tick = 0;
	bool mid_active = false;
	/* ===== 进入 UP(调重力)挡位时刻与状态 ===== */
	TickType_t up_enter_tick = 0;
	bool up_active = false;
	/* ===== 上一拍的挡位,用于识别挡位切换 ===== */
	sp::DBusSwitchMode last_sw = sp::DBusSwitchMode::DOWN;
	uint32_t tick_ms = 0;  // 避障之后恢复原来位置毫秒计数
	while (true) {
		/*调用*/

		pitch_detect(
			imu.roll + 0.269f, pitch,
			&pitch_ratio);	//imu.pitch是云台实际的pitch,plan_pitch是对应的期望值,以弧度为单位,pitch_ratio为自己定义的变量名
		//调用时需要放在一个 以1ms为周期freertos任务里
		TrajectoryPoint traj = trajectory_read();

		/* ---------- GY-56：只在这里处理一次 ---------- */
		const uint32_t now_ms = osKernelSysTick();
		gy56_dist_ok = gy56.is_alive(now_ms) && gy56.is_valid();
		if (gy56_dist_ok) {
			gy56_dist_cm = gy56.distance();
		}
		/* ------------------------------------------ */

		const sp::DBusSwitchMode sw = remote.sw_r;

		/* ===== 挡位一变就先清 PID,避免切挡瞬间积分/微分突跳 ===== */
		if (sw != last_sw) {
			mid_active = false;
			up_active = false;
			rm_motor_pid_angle_x.clear();
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_angle_y.clear();
			rm_motor_pid_speed_y.clear();
			if (sw == sp::DBusSwitchMode::MID) {
				mid_enter_tick = xTaskGetTickCount();
				mid_active = true;
				pitch_locked = false;  // 每次进 MID 重新锁定
				dist_confirm_cnt = 0;
				tick_ms = 0;
			}
			last_sw = sw;
		}

		if (sw == sp::DBusSwitchMode::MID) {
			/* ===== Z 轴 2006 跟随：保持期内也要工作，放在保持判断之前 ===== */
			rm_motor_data_z.target_angle_set = (-imu.roll - 0.15f) * 36.0f;
			rm_motor_data_z.target_speed_set = 0.0f;

			rm_motor_pid_angle_z.calc(rm_motor_data_z.target_angle_set, rm_motor_z.angle);
			rm_motor_data_z.target_speed_set = rm_motor_pid_angle_z.out;
			rm_motor_pid_speed_z.calc(rm_motor_data_z.target_speed_set, rm_motor_z.speed);
			rm_motor_data_z.given_torque = rm_motor_pid_speed_z.out;  //2006电机跟随

			/* ===== 进入 MID 后的保持判断 ===== */
			const TickType_t now_tick = xTaskGetTickCount();
			const uint32_t mid_elapsed_ms =
				static_cast<uint32_t>(now_tick - mid_enter_tick);  // tick==ms @1kHz
			const bool mid_boot_hold = (mid_elapsed_ms < kMidBootHoldMs);

			if (mid_boot_hold) {
				/* ---------- 保持期：pitch 锁 kPitchBias，yaw 锁 -1.65f，红外不判定 ---------- */
				rm_motor_pid_angle_x.calc(kPitchBias, rm_motor_x.angle);
				rm_motor_pid_speed_x.calc(rm_motor_pid_angle_x.out, rm_motor_x.speed);

				float ffm_x = 0.0f;
				if (fabsf(rm_motor_pid_speed_x.out) > 4.0f) {
					ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.02f;
				}

				rm_motor_data_x.given_torque =
					rm_motor_pid_speed_x.out + pitch_gravity_torque() + ffm_x;

				rm_motor_pid_angle_y.calc(-1.65f, rm_motor_y.angle);  //YAW轴锁
				rm_motor_pid_speed_y.calc(rm_motor_pid_angle_y.out, rm_motor_y.speed);
				rm_motor_data_y.given_torque = rm_motor_pid_speed_y.out;

				/* 保持期内不判定红外，清状态，等保持结束再开始检测 */
				pitch_locked = false;
				dist_confirm_cnt = 0;
				tick_ms = 0;
				imu_last_yaw = imu.yaw;
			}
			else {
				/* ---------- 保持期结束：原 MID 挡逻辑 ---------- */
				/* ---------- 红外判定：连续 N 拍 <50cm 才解锁，防抖 ---------- */
				if (!pitch_locked) {
					if (gy56_dist_ok && gy56_dist_cm < kDistTriggerCm) {  //解锁条件
						if (dist_confirm_cnt < kDistConfirmN) dist_confirm_cnt++;
						if (dist_confirm_cnt >= kDistConfirmN) {
							pitch_locked = true;  // 触发解锁
							tick_ms = 0;		  //触发避障时计时归零
						}
					}
					else {
						dist_confirm_cnt = 0;  // 距离回弹或数据无效，重新计数
					}
				}

				if (!pitch_locked) {
					/* 锁定阶段：一直保持 kPitchBias，不叠加轨迹 */
					rm_motor_pid_angle_x.calc(kPitchBias, rm_motor_x.angle);
					rm_motor_pid_speed_x.calc(rm_motor_pid_angle_x.out, rm_motor_x.speed);

					float ffm_x = 0.0f;
					if (fabsf(rm_motor_pid_speed_x.out) > 4.0f) {
						ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.02f;
					}

					rm_motor_data_x.given_torque =
						rm_motor_pid_speed_x.out + pitch_gravity_torque() + ffm_x;
					rm_motor_pid_angle_y.calc(-1.65f, rm_motor_y.angle);  //YAW轴锁
					rm_motor_pid_speed_y.calc(rm_motor_pid_angle_y.out, rm_motor_y.speed);

					rm_motor_data_y.given_torque = rm_motor_pid_speed_y.out;
				}
				else { /* 已检测到 <50cm：触发避障 */
					tick_ms++;
					rm_motor_pid_angle_x.calc(-3.50f, rm_motor_x.angle);
					rm_motor_pid_speed_x.calc(rm_motor_pid_angle_x.out, rm_motor_x.speed);

					float ffm_x = 0.0f;
					if (fabsf(rm_motor_pid_speed_x.out) > 4.0f) {
						ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.02f;
					}

					rm_motor_data_x.given_torque =
						rm_motor_pid_speed_x.out + pitch_gravity_torque() + ffm_x;
					rm_motor_pid_angle_y.calc(-1.65f, rm_motor_y.angle);  //YAW轴锁
					rm_motor_pid_speed_y.calc(rm_motor_pid_angle_y.out, rm_motor_y.speed);

					rm_motor_data_y.given_torque = rm_motor_pid_speed_y.out;

					if (tick_ms >= 3000 && tick_ms <= 6000)	 //避障3秒后，复位
					{
						rm_motor_pid_angle_x.calc(kPitchBias, rm_motor_x.angle);
						rm_motor_pid_speed_x.calc(rm_motor_pid_angle_x.out, rm_motor_x.speed);

						float ffm_x2 = 0.0f;
						if (fabsf(rm_motor_pid_speed_x.out) > 4.0f) {
							ffm_x2 = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.02f;
						}

						rm_motor_data_x.given_torque =
							rm_motor_pid_speed_x.out + pitch_gravity_torque() + ffm_x2;
						rm_motor_pid_angle_y.calc(-1.65f, rm_motor_y.angle);  //YAW轴锁
						rm_motor_pid_speed_y.calc(rm_motor_pid_angle_y.out, rm_motor_y.speed);

						rm_motor_data_y.given_torque = rm_motor_pid_speed_y.out;
						imu_last_yaw = imu.yaw;
					}
					if (tick_ms >= 6000) {
						rm_motor_pid_angle_x.calc(kPitchBias, rm_motor_x.angle);
						rm_motor_pid_speed_x.calc(rm_motor_pid_angle_x.out, rm_motor_x.speed);

						float ffm_x3 = 0.0f;
						if (fabsf(rm_motor_pid_speed_x.out) > 4.0f) {
							ffm_x3 = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.02f;
						}

						rm_motor_data_x.given_torque = rm_motor_pid_speed_x.out +
													   pitch_gravity_torque() +
													   ffm_x3;	//pitch轴锁住

						const float yaw_target = kYawBias + kYawSign * traj.yaw;

						rm_motor_pid_angle_y.calc(yaw_target, rm_motor_y.angle);
						const float yaw_vel_ref =
							rm_motor_pid_angle_y.out + kYawVelFf * kYawSign * traj.yaw_vel;
						rm_motor_pid_speed_y.calc(yaw_vel_ref, rm_motor_y.speed);

						rm_motor_data_y.given_torque =
							rm_motor_pid_speed_y.out +
							kYawAccFf * kYawSign * traj.yaw_acc;  //yaw轴执行任务
					}
				}
			}
		}
		else if (sw == sp::DBusSwitchMode::UP) {
		}

		else {
			/* DOWN 挡位:不输出力矩 */
			rm_motor_data_x.given_torque = 0.0f;
			rm_motor_data_y.given_torque = 0.0f;
			rm_motor_data_z.given_torque = 0.0f;
		}
		yaw_detect(
			imu.yaw - imu_last_yaw, yaw,
			&yaw_ratio);  //imu.yaw是云台实际的yaw,plan_yaw是对应的期望值,以弧度为单位,yaw_ratio为自己定义的变量名

		rm_motor_x.cmd(rm_motor_data_x.given_torque);
		rm_motor_y.cmd(rm_motor_data_y.given_torque);
		rm_motor_z.cmd(rm_motor_data_z.given_torque);
		send_rm_motor();
		osDelay(1);
	}
}