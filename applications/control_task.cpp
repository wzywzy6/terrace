#if 0
#include "control_task.hpp"

#include <math.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/bmi088/bmi088.hpp"
#include "task.h"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"
#include "trajectory_task.hpp"

GimbalData rm_motor_data_x;
GimbalData rm_motor_data_y;
GimbalData rm_motor_data_z;
extern sp::Mahony imu;

static constexpr float kPitchVelFf = 2.35f;
static constexpr float kYawVelFf = 2.0f;
static constexpr float kPitchAccFf = 0.0025f;
static constexpr float kYawAccFf = 0.006f;

static constexpr float kPitchBias = -4.00f;
static constexpr float kYawBias = 0.0f;

static constexpr float kPitchSign = 1.0f;
static constexpr float kYawSign = 1.0f;
float rm_motor_z_out_angle = rm_motor_z.angle / 36.0f;
float rm_motor_z_out_speed = rm_motor_z.speed / 36.0f;
float rm_motor_z_out_torgue = rm_motor_z.temperature * 36.0f;  //设置减速比
//重力矩相关
float g_torgue = 0.0f;
float k1 = 0.0f, k2 = 0.0f, k3 = 0.0f;

//
float in_angle = 0.0f;
/* ===== 新增:进入 MID 后先保持的时长 ===== */
static constexpr uint32_t kStartupHoldMs = 500U;

sp::PID rm_motor_pid_angle_x(0.001f, 28.0f, 0.1f, 1.0f, 50.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_x(0.001f, 0.06f, 0.0f, 0.0f, 0.5f, 0.5f, 1.0f, false, true);
sp::PID rm_motor_pid_angle_y(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_y(0.001f, 0.007f, 0.0f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);
sp::PID rm_motor_pid_angle_z(0.001f, 20.0f, 0.0f, 0.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_z(0.001f, 0.0002f, 0.0001f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

extern "C" void control_task()
{
	gy56.begin();
	remote.request();
	osDelay(100);
	can_start();
	osDelay(100);

	rm_motor_data_x.given_torque = 0.0f;
	rm_motor_data_y.given_torque = 0.0f;

	/* ===== 新增:进入 MID 时刻与状态 ===== */
	TickType_t mid_enter_tick = 0;
	bool mid_active = false;
	sp::DBusSwitchMode last_sw = sp::DBusSwitchMode::DOWN;

	while (true) {
		TrajectoryPoint traj = trajectory_read();
		/* ---------- GY-56：只在这里处理一次 ---------- */
		const uint32_t now_ms = osKernelSysTick();
		gy56_dist_ok = gy56.is_alive(now_ms) && gy56.is_valid();
		if (gy56_dist_ok) {
			gy56_dist_cm = gy56.distance();
		}
		/* ------------------------------------------ */

		const sp::DBusSwitchMode sw = remote.sw_r;

		/* ===== 新增:挡位一变就先清 PID,避免切挡瞬间积分/微分突跳 ===== */
		if (sw != last_sw) {
			mid_active = false;
			rm_motor_pid_angle_x.clear();
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_angle_y.clear();
			rm_motor_pid_speed_y.clear();
			if (remote.sw_r == sp::DBusSwitchMode::UP) {
				in_angle = rm_motor_z.angle;
			}
			/*if (remote.sw_r == sp::DBusSwitchMode::MID) {
				rm_motor_pid_angle_x.calc(kPitchBias, rm_motor_x.angle);
				rm_motor_data_x.target_speed_set = rm_motor_pid_angle_x.out;
				rm_motor_pid_speed_x.calc(rm_motor_data_x.target_speed_set, rm_motor_x.speed);

				float ffm_x = 0.0f;
				if (fabsf(rm_motor_pid_speed_x.out) > 1.0f) {
					ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.02f;
				}*/

			/*rm_motor_data_x.given_torque =
					rm_motor_pid_speed_x.out + (imu.roll > (-1.07f)
													? (0.35f) * sinf(-1.07f - imu.roll) + ffm_x
													: (0.30f) * sinf(-1.07f - imu.roll) + ffm_x);*/
		}
		last_sw = sw;
	}

	if (remote.sw_r == sp::DBusSwitchMode::MID) {
		/* ===== 新增:首次进入 MID 记录时刻 ===== */
		/*const TickType_t now_tick = xTaskGetTickCount();
			if (!mid_active) {
				mid_enter_tick = now_tick;
				mid_active = true;
			}
			
			const uint32_t elapsed_ms =
				static_cast<uint32_t>(now_tick - mid_enter_tick);  // tick==ms @1kHz
			const bool startup_hold = (elapsed_ms < kStartupHoldMs);
			const float pitch_traj_pos = startup_hold ? 0.0f : traj.pitch;
			const float pitch_traj_vel = startup_hold ? 0.0f : traj.pitch_vel;
			const float pitch_traj_acc = startup_hold ? 0.0f : traj.pitch_acc;
			const float yaw_traj_pos = startup_hold ? 0.0f : traj.yaw;
			const float yaw_traj_vel = startup_hold ? 0.0f : traj.yaw_vel;
			const float yaw_traj_acc = startup_hold ? 0.0f : traj.yaw_acc;*/

		/*if (gy56_dist_cm <= 5.0f) {
				rm_motor_pid_angle_x.calc(0.0f, rm_motor_x.angle);
				rm_motor_data_x.target_speed_set = rm_motor_pid_angle_x.out;
				rm_motor_pid_speed_x.calc(rm_motor_data_x.target_speed_set, rm_motor_x.speed);

				float ffm_x = 0.0f;
				if (fabsf(rm_motor_pid_speed_x.out) > 1.0f) {
					ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.02f;
				}

				rm_motor_data_x.given_torque =
					rm_motor_pid_speed_x.out + (-0.26f) * cosf(0.30f - imu.roll) + ffm_x;
		}*/

		/* ---------- X 轴(pitch) ---------- */
		const float pitch_target = kPitchBias + kPitchSign * pitch_traj_pos;

		rm_motor_pid_angle_x.calc(pitch_target, rm_motor_x.angle);
		const float pitch_vel_ref =
			rm_motor_pid_angle_x.out + kPitchVelFf * kPitchSign * pitch_traj_vel;
		rm_motor_pid_speed_x.calc(pitch_vel_ref, rm_motor_x.speed);

		float ffm_x = 0.0f;
		if (fabsf(rm_motor_pid_speed_x.out) > 1.0f) {
			ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.02f;
		}

		rm_motor_data_x.given_torque = rm_motor_pid_speed_x.out + (-0.10f) * cosf(0.0f - imu.roll) +
									   ffm_x + kPitchAccFf * kPitchSign * pitch_traj_acc;

		/* ---------- Y 轴(yaw) ---------- */
		/*const float yaw_target = kYawBias + kYawSign * yaw_traj_pos;

            rm_motor_pid_angle_y.calc(yaw_target, rm_motor_y.angle);
            const float yaw_vel_ref =
                rm_motor_pid_angle_y.out + kYawVelFf * kYawSign * yaw_traj_vel;
            rm_motor_pid_speed_y.calc(yaw_vel_ref, rm_motor_y.speed);

            rm_motor_data_y.given_torque =
                rm_motor_pid_speed_y.out
                + kYawAccFf * kYawSign * yaw_traj_acc;*/
	}
	else if (remote.sw_r == sp::DBusSwitchMode::UP) {
		rm_motor_data_z.target_angle_set = 2.0f * 36.0f + in_angle;
		rm_motor_data_z.target_speed_set = 0.0f;

		rm_motor_pid_angle_z.calc(rm_motor_data_z.target_angle_set, rm_motor_z.angle);
		rm_motor_data_z.target_speed_set = rm_motor_pid_angle_z.out;
		rm_motor_pid_speed_z.calc(rm_motor_data_z.target_speed_set, rm_motor_z.speed);
		rm_motor_data_z.given_torque = rm_motor_pid_speed_z.out;
	}
	else {
		mid_active = false; /* 退出 MID,下次进入重新计时 */
		rm_motor_data_x.given_torque = 0.0f;
		rm_motor_data_y.given_torque = 0.0f;
		rm_motor_data_z.given_torque = 0.0f;
		rm_motor_pid_angle_x.clear();
		rm_motor_pid_speed_x.clear();
		rm_motor_pid_angle_y.clear();
		rm_motor_pid_speed_y.clear();
	}

	rm_motor_x.cmd(rm_motor_data_x.given_torque);
	rm_motor_y.cmd(rm_motor_data_y.given_torque);
	rm_motor_z.cmd(rm_motor_data_z.given_torque);

	send_rm_motor();
	osDelay(1);
}
#endif
#include <math.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "control_task.hpp"
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
static constexpr float kDistTriggerCm = 10.0f;	// 触发距离
static constexpr uint8_t kDistConfirmN = 3;		// 连续 3 拍确认（1kHz 下约 3ms）

/*前馈增益*/
static constexpr float kPitchVelFf = 1.0f;	//参考速度放大多少倍
static constexpr float kYawVelFf = 2.2f;
static constexpr float kPitchAccFf = 0.0025f;  //参考加速度要乘多大惯量变成力矩 kg*m^2
static constexpr float kYawAccFf = 0.0047f;

static constexpr float kPitchBias = -4.00f;	 //绝对目标角 rad
static constexpr float kYawBias = 0.0f;

static constexpr float kPitchSign = 1.0f;  //方向标定（电机与机构正方向比例）
static constexpr float kYawSign = 1.0f;

/* ===== 新增:进入 MID 后先保持的时长 ===== */
static constexpr uint32_t kStartupHoldMs = 500U;

/* ===== 重力补偿参数:UP 挡位只输出这一项,专门用来调这两个值 =====
   现场手调时,可临时把 kPitchGravAmp 改成 kPitchGravAmp + 0.05f * remote.ch_lv
   (左摇杆前后推)看效果,调稳后把数写回常量即可。 */
static constexpr float kPitchGravAmp = -0.33f;	 // 重力力矩幅值 N·m
static constexpr float kPitchGravPhase = 0.35f;	 // 重力力矩相位 rad
/* UP 挡位进入后重力力矩的爬升时间,0 表示立即满输出 */
static constexpr uint32_t kUpGravRampMs = 300U;
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
// UP / MID 挡位共用这一份参数:在 UP 挡调好,切回 MID 直接生效
static float pitch_gravity_torque() { return kPitchGravAmp * cosf(kPitchGravPhase - imu.roll); }
//dt  kp  ki  kd  mo  mio  alpha  ang ?  dynamic?
sp::PID rm_motor_pid_angle_x(0.001f, 28.0f, 0.0f, 1.0f, 50.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_x(0.001f, 0.06f, 0.0f, 0.0f, 0.5f, 0.5f, 1.0f, false, true);
sp::PID rm_motor_pid_angle_y(0.001f, 60.0f, 0.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_y(0.001f, 0.07f, 0.0f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);
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

	/* ===== 新增:进入 MID 时刻与状态 ===== */
	TickType_t mid_enter_tick = 0;
	bool mid_active = false;
	/* ===== 新增:进入 UP(调重力)挡位时刻与状态 ===== */
	TickType_t up_enter_tick = 0;
	bool up_active = false;
	/* ===== 新增:上一拍的挡位,用于识别挡位切换 ===== */
	sp::DBusSwitchMode last_sw = sp::DBusSwitchMode::DOWN;
	uint32_t tick_ms = 0;  // 避障之后恢复原来位置毫秒计数
	while (true) {
		/*调用*/
		yaw_detect(
			imu.yaw, yaw,
			&yaw_ratio);  //imu.yaw是云台实际的yaw,plan_yaw是对应的期望值,以弧度为单位,yaw_ratio为自己定义的变量名
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

		/* ===== 新增:挡位一变就先清 PID,避免切挡瞬间积分/微分突跳 ===== */
		if (sw != last_sw) {
			mid_active = false;
			up_active = false;
			rm_motor_pid_angle_x.clear();
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_angle_y.clear();
			rm_motor_pid_speed_y.clear();
			if (sw == sp::DBusSwitchMode::UP) {
				mid_enter_tick = xTaskGetTickCount();
				mid_active = true;
				pitch_locked = false;  // 每次进 UP 重新锁定
				dist_confirm_cnt = 0;
			}
			last_sw = sw;
		}

		if (sw == sp::DBusSwitchMode::UP) {
			rm_motor_data_z.target_angle_set = -imu.roll * 36.0f;
			rm_motor_data_z.target_speed_set = 0.0f;

			rm_motor_pid_angle_z.calc(rm_motor_data_z.target_angle_set, rm_motor_z.angle);
			rm_motor_data_z.target_speed_set = rm_motor_pid_angle_z.out;
			rm_motor_pid_speed_z.calc(rm_motor_data_z.target_speed_set, rm_motor_z.speed);
			rm_motor_data_z.given_torque = rm_motor_pid_speed_z.out;  //2006电机跟随
			/* ---------- 红外判定：连续 N 拍 <3cm 才解锁，防抖 ---------- */
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
			}
			else { /* 已检测到 <3cm：触发避障 */

				tick_ms++;
				rm_motor_pid_angle_x.calc(-3.50f, rm_motor_x.angle);
				rm_motor_pid_speed_x.calc(rm_motor_pid_angle_x.out, rm_motor_x.speed);

				float ffm_x = 0.0f;
				if (fabsf(rm_motor_pid_speed_x.out) > 4.0f) {
					ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.02f;
				}

				rm_motor_data_x.given_torque =
					rm_motor_pid_speed_x.out + pitch_gravity_torque() + ffm_x;
				if (tick_ms >= 3000 && tick_ms <= 6000)	 //避障3秒后，复位
				{
					rm_motor_pid_angle_x.calc(kPitchBias, rm_motor_x.angle);
					rm_motor_pid_speed_x.calc(rm_motor_pid_angle_x.out, rm_motor_x.speed);

					float ffm_x = 0.0f;
					if (fabsf(rm_motor_pid_speed_x.out) > 4.0f) {
						ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.02f;
					}

					rm_motor_data_x.given_torque =
						rm_motor_pid_speed_x.out + pitch_gravity_torque() + ffm_x;
				}
				if (tick_ms >= 6000)  //避障6秒后，追踪轨迹
				{
				}
			}
		}
		else if (sw == sp::DBusSwitchMode::MID) {
			const TickType_t now_tick = xTaskGetTickCount();
			if (!mid_active) {
				mid_enter_tick = now_tick;
				mid_active = true;
			}
			// const uint32_t elapsed_ms = pdTICKS_TO_MS(now_tick - mid_enter_tick);   // 删
			const uint32_t elapsed_ms =
				static_cast<uint32_t>(now_tick - mid_enter_tick);  // tick==ms @1kHz 现在时间
			const bool startup_hold = (elapsed_ms < kStartupHoldMs);
			/* 保持期内:轨迹贡献为 0,只保留初始位置 需要的轨迹*/
			const float pitch_traj_pos = startup_hold ? 0.0f : traj.pitch;		//参考角度偏移
			const float pitch_traj_vel = startup_hold ? 0.0f : traj.pitch_vel;	//参考角速度
			const float pitch_traj_acc = startup_hold ? 0.0f : traj.pitch_acc;	//参考角加速度
			const float yaw_traj_pos = startup_hold ? 0.0f : traj.yaw;
			const float yaw_traj_vel = startup_hold ? 0.0f : traj.yaw_vel;
			const float yaw_traj_acc = startup_hold ? 0.0f : traj.yaw_acc;

			/* ---------- X 轴(pitch) ---------- */
			const float pitch_target = kPitchBias + kPitchSign * pitch_traj_pos;

			rm_motor_pid_angle_x.calc(pitch_target, rm_motor_x.angle);
			const float pitch_vel_ref =
				rm_motor_pid_angle_x.out + kPitchVelFf * kPitchSign * pitch_traj_vel;
			rm_motor_pid_speed_x.calc(pitch_vel_ref, rm_motor_x.speed);
			rm_motor_data_x.given_torque = rm_motor_pid_speed_x.out + (pitch_traj_acc)*M -
										   G * cosf(imu.roll) + B * rm_motor_x.speed +
										   F * Fsign(rm_motor_x.speed);
			/* -- -- -- -- --Y 轴(yaw)-- -- -- -- --*/
#if 0
			const float yaw_target = kYawBias + kYawSign * yaw_traj_pos;

			rm_motor_pid_angle_y.calc(yaw_target, rm_motor_y.angle);
			const float yaw_vel_ref =
				rm_motor_pid_angle_y.out + kYawVelFf * kYawSign * yaw_traj_vel;
			rm_motor_pid_speed_y.calc(yaw_vel_ref, rm_motor_y.speed);

			rm_motor_data_y.given_torque =
				rm_motor_pid_speed_y.out + kYawAccFf * kYawSign * yaw_traj_acc;
#endif
		}

		else {
			/* DOWN 挡位:不输出力矩 */
			rm_motor_data_x.given_torque = 0.0f;
			rm_motor_data_y.given_torque = 0.0f;
			rm_motor_data_z.given_torque = 0.0f;
		}

		rm_motor_x.cmd(rm_motor_data_x.given_torque);
		rm_motor_y.cmd(rm_motor_data_y.given_torque);
		rm_motor_z.cmd(rm_motor_data_z.given_torque);
		send_rm_motor();
		osDelay(1);
	}
}