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
#include "tools/obstacle_guard.hpp"
#include "tools/pid/pid.hpp"
#include "trajectory_task.hpp"

GimbalData rm_motor_data_x;
GimbalData rm_motor_data_y;
GimbalData rm_motor_data_z;
extern sp::Mahony imu;

/* ===== 避障决策模块：纯逻辑，不碰电机、不建任务 ===== */
static ObstacleGuard obstacle_guard;

/*前馈增益*/
static constexpr float kPitchVelFf = 1.0f;	//参考速度放大多少倍
static constexpr float kYawVelFf = 1.0f;
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
float YawBias = 0.0f;
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
uint32_t timeMID = 0U;	//进入MID挡位的时间
// 重力压在 pitch 轴上的力矩,只与 IMU 测到的重力方向 imu.roll 有关
// UP / MID 挡位共用这一份参数:在 UP 挡调好,切回 MID 直接生效
static float pitch_gravity_torque() { return kPitchGravAmp * cosf(kPitchGravPhase - imu.roll); }
//dt  kp  ki  kd  mo  mio  alpha  ang ?  dynamic?
sp::PID rm_motor_pid_angle_x(0.001f, 8.6f, 0.0f, 0.18f, 1.0f, 0.15f, 0.5f, true, false);
sp::PID rm_motor_pid_speed_x(0.001f, 0.06f, 0.0f, 0.0f, 0.5f, 0.5f, 0.85f, false, true);
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
	rm_motor_data_z.given_torque = 0.0f;

	/* ===== 新增:进入 MID 时刻与状态 ===== */
	TickType_t mid_enter_tick = 0;
	bool mid_active = false;
	/* UP 挡以进入时的位置为基准，避免重启后多圈零点变化导致 Z 轴突转。 */
	float up_z_angle = 0.0f;
	float up_roll = 0.0f;
	/* ===== 新增:上一拍的挡位,用于识别挡位切换 ===== */
	sp::DBusSwitchMode last_sw = sp::DBusSwitchMode::DOWN;
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
		// 同一拍读取有效标记、距离和帧时间，避免串口中断更新到一半。
		taskENTER_CRITICAL();
		const uint32_t now_ms = osKernelSysTick();
		const uint32_t gy56_frame_ms = gy56.last_frame_ms();
		const bool gy56_alive = gy56.is_alive(now_ms);
		gy56_dist_ok = gy56_alive && gy56.is_valid();
		gy56_out_of_range = gy56_alive && gy56.out_of_range();
		if (gy56_dist_ok) {
			gy56_dist_cm = gy56.distance();
		}
		taskEXIT_CRITICAL();
		/* ------------------------------------------ */

		const sp::DBusSwitchMode sw = remote.sw_r;
		const float roll_now = imu.roll;

		/* ===== 新增:挡位一变就先清 PID,避免切挡瞬间积分/微分突跳 ===== */
		if (sw != last_sw) {
			mid_active = false;
			rm_motor_pid_angle_x.clear();
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_angle_y.clear();
			rm_motor_pid_speed_y.clear();
			rm_motor_pid_angle_z.clear();
			rm_motor_pid_speed_z.clear();
			obstacle_guard.reset();	 // 换挡一律回 NORMAL
			if (sw == sp::DBusSwitchMode::UP) {
				up_z_angle = rm_motor_z.angle;
				up_roll = roll_now;
			}
			else if (sw == sp::DBusSwitchMode::MID) {
				timeMID = 0.0f;
			}
			last_sw = sw;
		}

		if (sw == sp::DBusSwitchMode::UP) {
			/* ---- 1. 避障决策：唯一入口，1kHz 调用 ---- */
			obstacle_guard.update(gy56_dist_ok, gy56_dist_cm, gy56_frame_ms, now_ms);

			/* ---- 2. X 轴：目标由避障模块整形，一次 PID ---- */
			float x_vel_ff = 0.0f;
			const float x_target = obstacle_guard.apply(kPitchBias, 0.001f, x_vel_ff);

			rm_motor_pid_angle_x.calc(x_target, rm_motor_x.angle);
			const float x_ref = rm_motor_pid_angle_x.out + x_vel_ff;
			rm_motor_pid_speed_x.calc(x_ref, rm_motor_x.speed);

			float ffm_x = 0.0f;
			if (fabsf(x_ref) > 4.0f) {
				ffm_x = (x_ref > 0 ? 1.0f : -1.0f) * 0.02f;
			}

			rm_motor_data_x.given_torque =
				rm_motor_pid_speed_x.out + pitch_gravity_torque() + ffm_x;

			/* ---- 3. Z 轴：相对进入 UP 时的姿态补偿，首次目标等于当前 Z 角度 ---- */
			rm_motor_data_z.target_angle_set =
				up_z_angle - (roll_now - up_roll) * 36.0f + obstacle_guard.z_comp_motor();

			rm_motor_pid_angle_z.calc(rm_motor_data_z.target_angle_set, rm_motor_z.angle);
			const float z_ref = rm_motor_pid_angle_z.out + obstacle_guard.z_vel_ff_motor();
			rm_motor_pid_speed_z.calc(z_ref, rm_motor_z.speed);
			rm_motor_data_z.given_torque = rm_motor_pid_speed_z.out;
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
			const bool startup_hold = (timeMID < kStartupHoldMs);
			/* 保持期内:轨迹贡献为 0,只保留初始位置 需要的轨迹*/
			const float pitch_traj_pos = startup_hold ? 0.0f : traj.pitch;		//参考角度偏移
			const float pitch_traj_vel = startup_hold ? 0.0f : traj.pitch_vel;	//参考角速度
			const float pitch_traj_acc = startup_hold ? 0.0f : traj.pitch_acc;	//参考角加速度
			const float yaw_traj_pos = startup_hold ? 0.0f : traj.yaw;
			const float yaw_traj_vel = startup_hold ? 0.0f : traj.yaw_vel;
			const float yaw_traj_acc = startup_hold ? 0.0f : traj.yaw_acc;
#if 1
			/* ---------- X 轴(pitch) ---------- */
			const float pitch_target = kPitchBias + kPitchSign * pitch_traj_pos;

			rm_motor_pid_angle_x.calc(pitch_target, rm_motor_x.angle);
			const float pitch_vel_ref = pitch_traj_vel
				/*rm_motor_pid_angle_x.out + kPitchVelFf * kPitchSign * pitch_traj_vel*/;
			//rm_motor_pid_speed_x.calc(pitch_vel_ref, rm_motor_x.speed);
			rm_motor_data_x.given_torque = rm_motor_pid_angle_x.out + (pitch_traj_acc)*M -
										   G * cosf(imu.roll - 0.25) + B * rm_motor_x.speed +
										   F * Fsign(rm_motor_x.speed);
#endif
			/* -- -- -- -- --Y 轴(yaw)-- -- -- -- --*/
#if 0
			const float yaw_target = YawBias + kYawSign * pitch_traj_pos;

			rm_motor_pid_angle_y.calc(yaw_target, rm_motor_y.angle);
			const float yaw_vel_ref = rm_motor_pid_angle_y.out;
			rm_motor_pid_speed_y.calc(yaw_vel_ref, rm_motor_y.speed);

			rm_motor_data_y.given_torque = rm_motor_pid_speed_y.out;
#endif
			/* z 轴在轨迹挡不参与，清零，避免沿用 UP 挡最后一拍的力矩 */
			rm_motor_data_z.given_torque = 0.0f;
			timeMID = timeMID + 1U;
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
