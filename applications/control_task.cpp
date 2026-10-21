
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
extern sp::Mahony imu;

/*前馈增益*/
static constexpr float kPitchVelFf = 2.35f;	 //参考速度放大多少倍
static constexpr float kYawVelFf = 0.003f;
static constexpr float kPitchAccFf = 0.0025f;  //参考加速度要乘多大惯量变成力矩 kg*m^2
static constexpr float kYawAccFf = 0.0f;

static constexpr float kPitchBias = -4.00f;	 //绝对目标角 rad
static constexpr float kYawBias = 0.0f;

static constexpr float kPitchSign = 1.0f;  //方向标定（电机与机构正方向比例）
static constexpr float kYawSign = 1.0f;

/* ===== 新增:进入 MID 后先保持的时长 ===== */
static constexpr uint32_t kStartupHoldMs = 500U;

/* ===== 重力补偿参数:UP 挡位只输出这一项,专门用来调这两个值 =====
   现场手调时,可临时把 kPitchGravAmp 改成 kPitchGravAmp + 0.05f * remote.ch_lv
   (左摇杆前后推)看效果,调稳后把数写回常量即可。 */
static constexpr float kPitchGravAmp = -0.25f;	 // 重力力矩幅值 N·m
static constexpr float kPitchGravPhase = 0.17f;	 // 重力力矩相位 rad
/* UP 挡位进入后重力力矩的爬升时间,0 表示立即满输出 */
static constexpr uint32_t kUpGravRampMs = 300U;

// 重力压在 pitch 轴上的力矩,只与 IMU 测到的重力方向 imu.roll 有关
// UP / MID 挡位共用这一份参数:在 UP 挡调好,切回 MID 直接生效
static float pitch_gravity_torque() { return kPitchGravAmp * cosf(kPitchGravPhase - imu.roll); }

sp::PID rm_motor_pid_angle_x(0.001f, 28.0f, 0.0f, 1.0f, 50.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_x(0.001f, 0.06f, 0.01f, 0.0f, 0.5f, 0.5f, 1.0f, false, true);
sp::PID rm_motor_pid_angle_y(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_y(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

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
			up_active = false;
			rm_motor_pid_angle_x.clear();
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_angle_y.clear();
			rm_motor_pid_speed_y.clear();
			last_sw = sw;
		}

		if (sw == sp::DBusSwitchMode::MID) {
			/* ===== 新增:首次进入 MID 记录时刻 ===== */
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

			float ffm_x = 0.0f;
			if (fabsf(rm_motor_pid_speed_x.out) > 1.0f) {
				ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.02f;
			}

			rm_motor_data_x.given_torque = rm_motor_pid_speed_x.out + pitch_gravity_torque() +
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
		else if (sw == sp::DBusSwitchMode::UP) {
			/* ===== 新增:UP 挡位,只输出重力 =====
			   刻意不做 PID、不加摩擦前馈、不跟轨迹,看到的就是纯重力力矩,
			   方便单独调 kPitchGravAmp / kPitchGravPhase */
			const TickType_t now_tick = xTaskGetTickCount();
			if (!up_active) {
				up_enter_tick = now_tick;
				up_active = true;
			}

			/* 刚切进来时按 kUpGravRampMs 线性爬升,防止重力力矩一步顶到位 */
			const uint32_t up_elapsed_ms = static_cast<uint32_t>(now_tick - up_enter_tick);
			float grav_ramp = 1.0f;
			if (kUpGravRampMs > 0U && up_elapsed_ms < kUpGravRampMs) {
				grav_ramp = static_cast<float>(up_elapsed_ms) / static_cast<float>(kUpGravRampMs);
			}

			rm_motor_data_x.given_torque = grav_ramp * pitch_gravity_torque();
		}
		else {
			/* DOWN 挡位:不输出力矩 */
			rm_motor_data_x.given_torque = 0.0f;
			rm_motor_data_y.given_torque = 0.0f;
		}

		rm_motor_x.cmd(rm_motor_data_x.given_torque);
		rm_motor_y.cmd(rm_motor_data_y.given_torque);
		send_rm_motor();
		osDelay(1);
	}
}