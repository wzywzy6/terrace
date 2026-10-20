/*#include "control_task.hpp"

#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/bmi088/bmi088.hpp"	 // ← 新增
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

GimbalData rm_motor_data;
sp::DBusSwitchMode last_sw_l = remote.sw_l;

//                           dt     kp    ki    kd    mo   mio   alpha  ang? dynamic?
sp::PID rm_motor_pid_angle(0.001f, 30.0f, 1.0f, 3.0f, 400.0f, 5.0f, 1.0f, true, true);
//                           dt     kp    ki    kd    mo    mio   alpha  ang? dynamic?
sp::PID rm_motor_pid_speed(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

extern "C" void control_task()
{
	remote.request();

	osDelay(100);
	can_start();
	osDelay(100);

	// 初始化
	rm_motor_data.target_angle_set = 0.0f;
	rm_motor_data.target_speed_set = 5.0f;
	rm_motor_data.given_torque = 0.3f;
	float ffs = (rm_motor_data.target_speed_set > 0 ? 1.0f : -1.0f) * 0.08f;
	float ff = (rm_motor_data.target_speed_set > 0 ? 1.0f : -1.0f) * 0.055f;
	while (true) {
		if (remote.sw_r == sp::DBusSwitchMode::MID) {
			// 单速度环pid
			//rm_motor_pid_speed.calc(rm_motor_data.target_speed_set, rm_motor_x.speed);
			//rm_motor_data.given_torque = rm_motor_pid_speed.out + ff;

			//双环pid
			rm_motor_pid_angle.calc(rm_motor_data.target_angle_set, rm_motor_x.angle);
			rm_motor_data.target_speed_set = rm_motor_pid_angle.out;
			rm_motor_pid_speed.calc(rm_motor_data.target_speed_set, rm_motor_x.speed);
			rm_motor_data.given_torque =
				rm_motor_pid_speed.out + (abs(rm_motor_x.speed) > 0.001f ? ff : ffs);
		}
		else if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
			rm_motor_data.given_torque = 0.0f;
		}
		else if (remote.sw_r == sp::DBusSwitchMode::UP) {
		}

		rm_motor_x.cmd(rm_motor_data.given_torque);
		send_rm_motor();

		last_sw_l = remote.sw_l;

		osDelay(1);
	}
}*/
/*#include "control_task.hpp"

#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

// imu_task.cpp 里的全局对象
extern sp::BMI088 bmi088;
extern sp::Mahony imu;

GimbalData rm_motor_data;

// 角度环（外环）：输出目标速度，angular=true 自动 ±π 归一化
sp::PID rm_motor_pid_angle(0.001f, 30.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
//                           dt     kp    ki    kd    mo    mio   alpha  ang? dynamic?
sp::PID rm_motor_pid_speed(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

//                           dt     kp     ki     kd   mo    mio   alpha  ang?  dynamic?

// ===== 全局状态 =====
static constexpr float DIR = 1.0f;	// 方向系数，实测后改 +1 或 -1
static constexpr float KFF = 1.0f;	// 速度前馈系数，通常 1.0
static float yaw_offset = 0.0f;		// 零点偏移
static bool was_disabled = true;	// 上一帧是否不在 MID

extern "C" void control_task()
{
	remote.request();
	osDelay(100);
	can_start();
	osDelay(100);

	rm_motor_data.target_angle_set = 0.0f;
	rm_motor_data.target_speed_set = 0.0f;
	rm_motor_data.given_torque = 0.0f;

	while (true) {
		if (remote.sw_r == sp::DBusSwitchMode::MID) {
			// ===== 1. 刚进 MID 的瞬间，对齐零点 =====
			if (was_disabled) {
				yaw_offset = imu.yaw * DIR - rm_motor_x.angle;
				rm_motor_pid_angle.clear();
				rm_motor_pid_speed.clear();
				was_disabled = false;
			}

			// ===== 2. 外环：目标角度 = IMU yaw =====
			rm_motor_data.target_angle_set = imu.yaw * DIR - yaw_offset;
			rm_motor_pid_angle.calc(rm_motor_data.target_angle_set, rm_motor_x.angle);

			// ===== 3. 速度前馈：IMU yaw 角速度直接给速度环 =====
			float yaw_rate_ff = bmi088.gyro[2] * DIR * KFF;
			rm_motor_data.target_speed_set = rm_motor_pid_angle.out + yaw_rate_ff;

			// ===== 4. 内环：速度环 =====
			rm_motor_pid_speed.calc(rm_motor_data.target_speed_set, rm_motor_x.speed);
			rm_motor_data.given_torque =
				rm_motor_pid_speed.out +
				(rm_motor_data.target_speed_set > 0 ? 1.0f : -1.0f) * 0.052f;
		}
		else {
			rm_motor_data.given_torque = 0.0f;
			rm_motor_pid_angle.clear();
			rm_motor_pid_speed.clear();
			was_disabled = true;
		}

		rm_motor_x.cmd(rm_motor_data.given_torque);
		send_rm_motor();

		osDelay(1);	 // 100Hz
	}
}*/
/*#include "control_task.hpp"

#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

extern sp::BMI088 bmi088;
extern sp::Mahony imu;

GimbalData rm_motor_data_x;
GimbalData rm_motor_data_y;

// 电机 X 的两环 PID
sp::PID rm_motor_pid_angle_x(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_x(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

// 电机 Y 的两环 PID（参数可以先和 X 一样）
sp::PID rm_motor_pid_angle_y(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_y(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);
static constexpr float DIR_X = 1.0f;  // 电机 X 方向系数
static constexpr float DIR_Y = 1.0f;  // 电机 Y 方向系数（根据安装方向改）
static constexpr float KFF = 1.0f;

static float yaw_offset_x = 0.0f;
static float yaw_offset_y = 0.0f;
static bool was_disabled = true;

extern "C" void control_task()
{
	remote.request();
	osDelay(100);
	can_start();
	osDelay(100);

	while (true) {
		if (remote.sw_r == sp::DBusSwitchMode::MID) {
			// 刚进 MID：两个电机各自对齐零点
			if (was_disabled) {
				yaw_offset_x = imu.yaw * DIR_X - rm_motor_x.angle;
				yaw_offset_y = imu.yaw * DIR_Y - rm_motor_y.angle;
				rm_motor_pid_angle_x.clear();
				rm_motor_pid_speed_x.clear();
				rm_motor_pid_angle_y.clear();
				rm_motor_pid_speed_y.clear();
				was_disabled = false;
			}

			// 共享的 IMU 输入
			float target_angle_x = imu.yaw * DIR_X - yaw_offset_x;
			float target_angle_y = imu.yaw * DIR_Y - yaw_offset_y;

			// ---- 电机 X ----
			rm_motor_pid_angle_x.calc(target_angle_x, rm_motor_x.angle);
			float target_speed_x = rm_motor_pid_angle_x.out + bmi088.gyro[2] * DIR_X * KFF;
			rm_motor_pid_speed_x.calc(target_speed_x, rm_motor_x.speed);
			rm_motor_x.cmd(
				rm_motor_pid_speed_x.out +
				(rm_motor_data_x.target_speed_set > 0 ? 1.0f : -1.0f) * 0.052f);

			// ---- 电机 Y ----
			rm_motor_pid_angle_y.calc(target_angle_y, rm_motor_y.angle);
			float target_speed_y = rm_motor_pid_angle_y.out + bmi088.gyro[2] * DIR_Y * KFF;
			rm_motor_pid_speed_y.calc(target_speed_y, rm_motor_y.speed);
			rm_motor_y.cmd(
				rm_motor_pid_speed_y.out +
				(rm_motor_data_y.target_speed_set > 0 ? 1.0f : -1.0f) * 0.052f);

			// ---- 电机 Y ----
		}

		else {
			rm_motor_x.cmd(0.0f);
			rm_motor_y.cmd(0.0f);
			rm_motor_pid_angle_x.clear();
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_angle_y.clear();
			rm_motor_pid_speed_y.clear();
			was_disabled = true;
		}

		// 一次发送两个电机
		send_rm_motor();

		osDelay(1);
	}
}*/
/*#include "control_task.hpp"

#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

extern sp::BMI088 bmi088;
extern sp::Mahony imu;

GimbalData rm_motor_data_x;
GimbalData rm_motor_data_y;

// 电机 X 的两环 PID
sp::PID rm_motor_pid_angle_x(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_x(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

// 电机 Y 的两环 PID
sp::PID rm_motor_pid_angle_y(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_y(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

static constexpr float DIR_X = 1.0f;
static constexpr float DIR_Y = 1.0f;
static constexpr float KFF = 1.0f;

static float yaw_offset_x = 0.0f;
static float yaw_offset_y = 0.0f;
static bool was_disabled = true;

// ===== 手动跟随参数 =====
static constexpr float IMU_MOVING_THRESHOLD = 0.05f;  // IMU 在动阈值 rad/s
static constexpr float MANUAL_THRESHOLD = 0.15f;	  // 手动偏差阈值 rad
static constexpr uint32_t IMU_STILL_MS = 100;		  // IMU 静止持续 ms
static constexpr float DAMPING = 0.005f;			  // 手动时阻尼系数
static uint32_t imu_still_since_ms = 0;

extern "C" void control_task()
{
	remote.request();
	osDelay(100);
	can_start();
	osDelay(100);

	while (true) {
		if (remote.sw_r == sp::DBusSwitchMode::MID) {
			// ===== 刚进 MID：对齐零点 =====
			if (was_disabled) {
				yaw_offset_x = imu.yaw * DIR_X - rm_motor_x.angle;
				yaw_offset_y = imu.yaw * DIR_Y - rm_motor_y.angle;
				imu_still_since_ms = HAL_GetTick();

				rm_motor_pid_angle_x.clear();
				rm_motor_pid_speed_x.clear();
				rm_motor_pid_angle_y.clear();
				rm_motor_pid_speed_y.clear();

				was_disabled = false;
			}

			// ===== 计算目标角度 =====
			float target_angle_x = imu.yaw * DIR_X - yaw_offset_x;
			float target_angle_y = imu.yaw * DIR_Y - yaw_offset_y;

			// ===== 手动检测 =====
			bool imu_moving = fabsf(bmi088.gyro[2]) > IMU_MOVING_THRESHOLD;
			if (imu_moving) imu_still_since_ms = HAL_GetTick();
			bool imu_still = (HAL_GetTick() - imu_still_since_ms) > IMU_STILL_MS;

			float err_x = fabsf(target_angle_x - rm_motor_x.angle);
			float err_y = fabsf(target_angle_y - rm_motor_y.angle);

			bool manual_x = imu_still && (err_x > MANUAL_THRESHOLD);
			bool manual_y = imu_still && (err_y > MANUAL_THRESHOLD);

			// 谁被推：只有一台偏差大 = 那台被推
			//         两台都偏差大 = 用偏差大的那台做参考
			bool x_push = manual_x && !manual_y;
			bool y_push = manual_y && !manual_x;
			bool both_push = manual_x && manual_y;

			bool use_x_as_ref = x_push || (both_push && err_x >= err_y);
			bool use_y_as_ref = y_push || (both_push && err_y > err_x);

			// ===== 手动模式：更新两台的 offset，两台共用同一物理参考角 =====
			if (use_x_as_ref) {
				yaw_offset_x = imu.yaw * DIR_X - rm_motor_x.angle;
				yaw_offset_y = imu.yaw * DIR_Y - rm_motor_x.angle;
				rm_motor_pid_angle_x.clear();
				rm_motor_pid_speed_x.clear();
				rm_motor_pid_angle_y.clear();
				rm_motor_pid_speed_y.clear();
				target_angle_x = rm_motor_x.angle;
				target_angle_y = rm_motor_x.angle;
			}
			else if (use_y_as_ref) {
				yaw_offset_x = imu.yaw * DIR_X - rm_motor_y.angle;
				yaw_offset_y = imu.yaw * DIR_Y - rm_motor_y.angle;
				rm_motor_pid_angle_x.clear();
				rm_motor_pid_speed_x.clear();
				rm_motor_pid_angle_y.clear();
				rm_motor_pid_speed_y.clear();
				target_angle_x = rm_motor_y.angle;
				target_angle_y = rm_motor_y.angle;
			}

			// ===== 电机 X 输出 =====
			if (use_x_as_ref) {
				// X 被推：只给阻尼，不抵抗手
				rm_motor_x.cmd(-DAMPING * rm_motor_x.speed);
			}
			else {
				// X 正常跟随
				rm_motor_pid_angle_x.calc(target_angle_x, rm_motor_x.angle);
				float target_speed_x = rm_motor_pid_angle_x.out + bmi088.gyro[2] * DIR_X * KFF;
				rm_motor_pid_speed_x.calc(target_speed_x, rm_motor_x.speed);
				rm_motor_x.cmd(
					rm_motor_pid_speed_x.out +
					(rm_motor_data_x.target_speed_set > 0 ? 1.0f : -1.0f) * 0.052f);
			}

			// ===== 电机 Y 输出 =====
			if (use_y_as_ref) {
				// Y 被推：只给阻尼
				rm_motor_y.cmd(-DAMPING * rm_motor_y.speed);
			}
			else {
				// Y 正常跟随
				rm_motor_pid_angle_y.calc(target_angle_y, rm_motor_y.angle);
				float target_speed_y = rm_motor_pid_angle_y.out + bmi088.gyro[2] * DIR_Y * KFF;
				rm_motor_pid_speed_y.calc(target_speed_y, rm_motor_y.speed);
				rm_motor_y.cmd(
					rm_motor_pid_speed_y.out +
					(rm_motor_data_y.target_speed_set > 0 ? 1.0f : -1.0f) * 0.052f);
			}
		}
		else {
			// ===== 非 MID：清空 =====
			rm_motor_x.cmd(0.0f);
			rm_motor_y.cmd(0.0f);
			rm_motor_pid_angle_x.clear();
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_angle_y.clear();
			rm_motor_pid_speed_y.clear();
			was_disabled = true;
		}

		// 一次发送两个电机
		send_rm_motor();

		osDelay(1);
	}
}*/
/*#include "control_task.hpp"

#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

extern sp::BMI088 bmi088;
extern sp::Mahony imu;

GimbalData rm_motor_data_x;
GimbalData rm_motor_data_y;

// 电机 X 的两环 PID
sp::PID rm_motor_pid_angle_x(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_x(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

// 电机 Y 的两环 PID
sp::PID rm_motor_pid_angle_y(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_y(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

static constexpr float DIR_X = 1.0f;
static constexpr float DIR_Y = 1.0f;
static constexpr float KFF = 1.0f;

// 电机 X 固定 1:1
static constexpr float RATIO_X = 1.0f;

static float yaw_offset_x = 0.0f;
static float yaw_offset_y = 0.0f;
static bool was_disabled = true;

// ===== 左拨杆比例 =====
static float last_ratio_y = 1.0f;

// ===== 手动跟随参数 =====
static constexpr float IMU_MOVING_THRESHOLD = 0.05f;  // IMU 在动阈值 rad/s
static constexpr float MANUAL_THRESHOLD = 0.15f;	  // 手动偏差阈值 rad
static constexpr uint32_t IMU_STILL_MS = 100;		  // IMU 静止持续 ms
static constexpr float DAMPING = 0.005f;			  // 手动时阻尼系数
static uint32_t imu_still_since_ms = 0;

// 从左拨杆读取 Y 的比例
static float get_ratio_y(sp::DBusSwitchMode sw)
{
	switch (sw) {
		case sp::DBusSwitchMode::UP:
			return 3.0f;  // 上档 1:3
		case sp::DBusSwitchMode::MID:
			return -1.0f;  // 中档 1:-1
		case sp::DBusSwitchMode::DOWN:
			return 0.5f;  // 下档 1:0.5
		default:
			return 1.0f;
	}
}

extern "C" void control_task()
{
	remote.request();
	osDelay(100);
	can_start();
	osDelay(100);

	while (true) {
		if (remote.sw_r == sp::DBusSwitchMode::MID) {
			// ===== 读取左拨杆，更新 Y 的比例 =====
			float ratio_y = get_ratio_y(remote.sw_l);

			// 比例变化时，调整 offset_y 保持 Y 的目标连续（不跳变）
			if (ratio_y != last_ratio_y) {
				float cur_target_y = imu.yaw * DIR_Y * last_ratio_y - yaw_offset_y;
				yaw_offset_y = imu.yaw * DIR_Y * ratio_y - cur_target_y;
				last_ratio_y = ratio_y;
			}

			// ===== 刚进 MID：对齐零点 =====
			if (was_disabled) {
				yaw_offset_x = imu.yaw * DIR_X * RATIO_X - rm_motor_x.angle;
				yaw_offset_y = imu.yaw * DIR_Y * ratio_y - rm_motor_y.angle;
				imu_still_since_ms = HAL_GetTick();
				last_ratio_y = ratio_y;

				rm_motor_pid_angle_x.clear();
				rm_motor_pid_speed_x.clear();
				rm_motor_pid_angle_y.clear();
				rm_motor_pid_speed_y.clear();

				was_disabled = false;
			}

			// ===== 计算目标角度 =====
			float target_angle_x = imu.yaw * DIR_X * RATIO_X - yaw_offset_x;
			float target_angle_y = imu.yaw * DIR_Y * ratio_y - yaw_offset_y;

			// ===== 手动检测 =====
			bool imu_moving = fabsf(bmi088.gyro[2]) > IMU_MOVING_THRESHOLD;
			if (imu_moving) imu_still_since_ms = HAL_GetTick();
			bool imu_still = (HAL_GetTick() - imu_still_since_ms) > IMU_STILL_MS;

			float err_x = fabsf(target_angle_x - rm_motor_x.angle);
			float err_y = fabsf(target_angle_y - rm_motor_y.angle);

			bool manual_x = imu_still && (err_x > MANUAL_THRESHOLD);
			bool manual_y = imu_still && (err_y > MANUAL_THRESHOLD);

			bool x_push = manual_x && !manual_y;
			bool y_push = manual_y && !manual_x;
			bool both_push = manual_x && manual_y;

			bool use_x_as_ref = x_push || (both_push && err_x >= err_y);
			bool use_y_as_ref = y_push || (both_push && err_y > err_x);

			// ===== 手动模式：更新 offset，另一台按比例跟随 =====
			if (use_x_as_ref) {
				// X 被推，Y 按 RATIO_Y/RATIO_X 比例跟随
				float delta_x = rm_motor_x.angle - target_angle_x;
				yaw_offset_x = imu.yaw * DIR_X * RATIO_X - rm_motor_x.angle;

				float new_target_y = target_angle_y + delta_x * (ratio_y / RATIO_X);
				yaw_offset_y = imu.yaw * DIR_Y * ratio_y - new_target_y;

				target_angle_x = rm_motor_x.angle;
				target_angle_y = new_target_y;

				rm_motor_pid_angle_x.clear();
				rm_motor_pid_speed_x.clear();
				rm_motor_pid_angle_y.clear();
				rm_motor_pid_speed_y.clear();
			}
			else if (use_y_as_ref) {
				// Y 被推，X 按 RATIO_X/RATIO_Y 比例跟随
				float delta_y = rm_motor_y.angle - target_angle_y;
				yaw_offset_y = imu.yaw * DIR_Y * ratio_y - rm_motor_y.angle;

				float new_target_x = target_angle_x + delta_y * (RATIO_X / ratio_y);
				yaw_offset_x = imu.yaw * DIR_X * RATIO_X - new_target_x;

				target_angle_x = new_target_x;
				target_angle_y = rm_motor_y.angle;

				rm_motor_pid_angle_x.clear();
				rm_motor_pid_speed_x.clear();
				rm_motor_pid_angle_y.clear();
				rm_motor_pid_speed_y.clear();
			}

			// ===== 电机 X 输出 =====
			if (use_x_as_ref) {
				// X 被推：只给阻尼
				rm_motor_x.cmd(-DAMPING * rm_motor_x.speed);
			}
			else {
				rm_motor_pid_angle_x.calc(target_angle_x, rm_motor_x.angle);
				float target_speed_x = rm_motor_pid_angle_x.out + bmi088.gyro[2] * DIR_X * KFF;
				rm_motor_pid_speed_x.calc(target_speed_x, rm_motor_x.speed);
				rm_motor_x.cmd(
					rm_motor_pid_speed_x.out +
					(rm_motor_data_x.target_speed_set > 0 ? 1.0f : -1.0f) * 0.052f);
			}

			// ===== 电机 Y 输出 =====
			if (use_y_as_ref) {
				// Y 被推：只给阻尼
				rm_motor_y.cmd(-DAMPING * rm_motor_y.speed);
			}
			else {
				rm_motor_pid_angle_y.calc(target_angle_y, rm_motor_y.angle);
				float target_speed_y = rm_motor_pid_angle_y.out + bmi088.gyro[2] * DIR_Y * KFF;
				rm_motor_pid_speed_y.calc(target_speed_y, rm_motor_y.speed);
				rm_motor_y.cmd(
					rm_motor_pid_speed_y.out +
					(rm_motor_data_y.target_speed_set > 0 ? 1.0f : -1.0f) * 0.052f);
			}
		}
		else {
			// ===== 非 MID：清空 =====
			rm_motor_x.cmd(0.0f);
			rm_motor_y.cmd(0.0f);
			rm_motor_pid_angle_x.clear();
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_angle_y.clear();
			rm_motor_pid_speed_y.clear();
			was_disabled = true;
		}

		send_rm_motor();
		osDelay(1);
	}
}*/
/*#include "control_task.hpp"

#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

extern sp::BMI088 bmi088;
extern sp::Mahony imu;

GimbalData rm_motor_data_x;
GimbalData rm_motor_data_y;

// 电机 X 的两环 PID
sp::PID rm_motor_pid_angle_x(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_x(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

// 电机 Y 的两环 PID
sp::PID rm_motor_pid_angle_y(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_y(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

static constexpr float DIR_X = 1.0f;
static constexpr float DIR_Y = 1.0f;
static constexpr float KFF = 1.0f;
static sp::DBusSwitchMode last_sw_r = sp::DBusSwitchMode::DOWN;

// 电机 X 固定 1:1
static constexpr float RATIO_X = 1.0f;

static float yaw_offset_x = 0.0f;
static float yaw_offset_y = 0.0f;
static bool was_disabled = true;

// ===== 左拨杆比例 =====
static float last_ratio_y = 1.0f;

// ===== 右拨杆复位相关 =====
static float aligned_offset_x = 0.0f;			 // 初始对齐位置（X）
static float aligned_offset_y = 0.0f;			 // 初始对齐位置（Y）
static bool aligned_captured = false;			 // 是否已捕获初始对齐
static constexpr float RESET_MAX_SPEED = 30.0f;	 // 复位时最大速度 rad/s

// ===== 手动跟随参数 =====
static constexpr float IMU_MOVING_THRESHOLD = 0.05f;
static constexpr float MANUAL_THRESHOLD = 0.15f;
static constexpr uint32_t IMU_STILL_MS = 100;
static constexpr float DAMPING = 0.005f;
static float motor_x_angle_at_mid_entry = 0.0f;
static float motor_y_angle_at_mid_entry = 0.0f;
static float imu_yaw_at_mid_entry = 0.0f;
static uint32_t imu_still_since_ms = 0;

// 从左拨杆读取 Y 的比例
static float get_ratio_y(sp::DBusSwitchMode sw)
{
	switch (sw) {
		case sp::DBusSwitchMode::UP:
			return 3.0f;  // 上档 1:3
		case sp::DBusSwitchMode::MID:
			return -1.0f;  // 中档 1:-1
		case sp::DBusSwitchMode::DOWN:
			return 0.5f;  // 下档 1:0.5
		default:
			return 1.0f;
	}
}

extern "C" void control_task()
{
	remote.request();
	osDelay(100);
	can_start();
	osDelay(100);

	while (true) {
		// ===== 读取左拨杆比例 =====
		float ratio_y = get_ratio_y(remote.sw_l);

		// ===== 首次进入非 DOWN 状态：捕获初始对齐位置 =====

		// ========================================
		//         右拨杆 UP：复位模式
		// ========================================
		if (remote.sw_r == sp::DBusSwitchMode::UP) {
			was_disabled = true;
			if (ratio_y != last_ratio_y) {
				float cur_target_y = imu.yaw * DIR_Y * last_ratio_y - yaw_offset_y;
				yaw_offset_y = imu.yaw * DIR_Y * ratio_y - cur_target_y;
				last_ratio_y = ratio_y;
			}
			// 复位目标 = 对齐方向（IMU yaw 相对对齐点）
			//float target_angle_x = imu.yaw * DIR_X * RATIO_X - aligned_offset_x;
			//float target_angle_y = imu.yaw * DIR_Y * 1 + -aligned_offset_y;
			float delta_yaw = imu.yaw - imu_yaw_at_mid_entry;
			float target_angle_x = motor_x_angle_at_mid_entry + delta_yaw * DIR_X * RATIO_X;
			float target_angle_y =
				target_angle_x - motor_x_angle_at_mid_entry + motor_y_angle_at_mid_entry;

			// 同步更新工作 offset（切回 MID 时不跳变）
			yaw_offset_x = aligned_offset_x;
			yaw_offset_y = aligned_offset_y;

			// 清 PID（首次进入 UP 时）

			// ---- 电机 X：限速转向对齐位置 ----
			rm_motor_pid_angle_x.calc(target_angle_x, rm_motor_x.angle);
			float speed_x = rm_motor_pid_angle_x.out;
			if (speed_x > RESET_MAX_SPEED) speed_x = RESET_MAX_SPEED;
			if (speed_x < -RESET_MAX_SPEED) speed_x = -RESET_MAX_SPEED;
			rm_motor_pid_speed_x.calc(speed_x, rm_motor_x.speed);
			float ff_x = 0.0f;
			if (fabsf(speed_x) > 0.5f) {  // 目标速度够大才补偿
				ff_x = (speed_x > 0 ? 1.0f : -1.0f) * 0.052f;
			}
			rm_motor_x.cmd(rm_motor_pid_speed_x.out + ff_x);

			// ---- 电机 Y：限速转向对齐位置 ----
			rm_motor_pid_angle_y.calc(target_angle_y, rm_motor_y.angle);
			float speed_y = rm_motor_pid_angle_y.out;
			if (speed_y > RESET_MAX_SPEED) speed_y = RESET_MAX_SPEED;
			if (speed_y < -RESET_MAX_SPEED) speed_y = -RESET_MAX_SPEED;
			rm_motor_pid_speed_y.calc(speed_y, rm_motor_y.speed);
			float ff_y = 0.0f;
			if (fabsf(speed_y) > 0.5f) {  // 目标速度够大才补偿
				ff_y = (speed_y > 0 ? 1.0f : -1.0f) * 0.052f;
			}
			rm_motor_y.cmd(rm_motor_pid_speed_y.out + ff_y);
		}

		// ========================================
		//         右拨杆 MID：联动模式
		// ========================================
		else if (remote.sw_r == sp::DBusSwitchMode::MID) {
			if (last_sw_r != sp::DBusSwitchMode::MID) {
				aligned_offset_x = imu.yaw * DIR_X * RATIO_X - rm_motor_x.angle;
				aligned_offset_y = imu.yaw * DIR_Y * ratio_y - rm_motor_y.angle;
				motor_x_angle_at_mid_entry = rm_motor_x.angle;
				motor_y_angle_at_mid_entry = rm_motor_y.angle;
				imu_yaw_at_mid_entry = imu.yaw;
				aligned_captured = true;
			}

			// 比例变化时，调整 offset_y 保持 Y 的目标连续（不跳变）
			if (ratio_y != last_ratio_y) {
				float cur_target_y = imu.yaw * DIR_Y * last_ratio_y - yaw_offset_y;
				yaw_offset_y = imu.yaw * DIR_Y * ratio_y - cur_target_y;
				last_ratio_y = ratio_y;
			}

			// 刚进 MID：对齐零点
			if (was_disabled) {
				yaw_offset_x = imu.yaw * DIR_X * RATIO_X - rm_motor_x.angle;
				yaw_offset_y = imu.yaw * DIR_Y * ratio_y - rm_motor_y.angle;
				imu_still_since_ms = HAL_GetTick();
				last_ratio_y = ratio_y;

				rm_motor_pid_angle_x.clear();
				rm_motor_pid_speed_x.clear();
				rm_motor_pid_angle_y.clear();
				rm_motor_pid_speed_y.clear();

				was_disabled = false;
			}

			float target_angle_x = imu.yaw * DIR_X * RATIO_X - yaw_offset_x;
			float target_angle_y = imu.yaw * DIR_Y * ratio_y - yaw_offset_y;

			// ===== 手动检测 =====
			bool imu_moving = fabsf(bmi088.gyro[2]) > IMU_MOVING_THRESHOLD;
			if (imu_moving) imu_still_since_ms = HAL_GetTick();
			bool imu_still = (HAL_GetTick() - imu_still_since_ms) > IMU_STILL_MS;

			float err_x = fabsf(target_angle_x - rm_motor_x.angle);
			float err_y = fabsf(target_angle_y - rm_motor_y.angle);

			bool manual_x = imu_still && (err_x > MANUAL_THRESHOLD);
			bool manual_y = imu_still && (err_y > MANUAL_THRESHOLD);

			bool x_push = manual_x && !manual_y;
			bool y_push = manual_y && !manual_x;
			bool both_push = manual_x && manual_y;

			bool use_x_as_ref = x_push || (both_push && err_x >= err_y);
			bool use_y_as_ref = y_push || (both_push && err_y > err_x);

			// ===== 手动模式：更新 offset，另一台按比例跟随 =====
			if (use_x_as_ref) {
				float delta_x = rm_motor_x.angle - target_angle_x;
				yaw_offset_x = imu.yaw * DIR_X * RATIO_X - rm_motor_x.angle;

				float new_target_y = target_angle_y + delta_x * (ratio_y / RATIO_X);
				yaw_offset_y = imu.yaw * DIR_Y * ratio_y - new_target_y;

				target_angle_x = rm_motor_x.angle;
				target_angle_y = new_target_y;

				rm_motor_pid_angle_x.clear();
				rm_motor_pid_speed_x.clear();
				rm_motor_pid_angle_y.clear();
				rm_motor_pid_speed_y.clear();
			}
			else if (use_y_as_ref) {
				float delta_y = rm_motor_y.angle - target_angle_y;
				yaw_offset_y = imu.yaw * DIR_Y * ratio_y - rm_motor_y.angle;

				float new_target_x = target_angle_x + delta_y * (RATIO_X / ratio_y);
				yaw_offset_x = imu.yaw * DIR_X * RATIO_X - new_target_x;

				target_angle_x = new_target_x;
				target_angle_y = rm_motor_y.angle;

				rm_motor_pid_angle_x.clear();
				rm_motor_pid_speed_x.clear();
				rm_motor_pid_angle_y.clear();
				rm_motor_pid_speed_y.clear();
			}

			// ===== 电机 X 输出 =====
			if (use_x_as_ref) {
				rm_motor_x.cmd(-DAMPING * rm_motor_x.speed);
			}
			else {
				rm_motor_pid_angle_x.calc(target_angle_x, rm_motor_x.angle);
				float target_speed_x = rm_motor_pid_angle_x.out + bmi088.gyro[2] * DIR_X * KFF;
				rm_motor_pid_speed_x.calc(target_speed_x, rm_motor_x.speed);
				float ffm_x = 0.0f;
				if (fabsf(rm_motor_pid_speed_x.out) > 0.5f) {  // 目标速度够大才补偿
					ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.052f;
				}
				rm_motor_x.cmd(rm_motor_pid_speed_x.out + ffm_x);
			}

			// ===== 电机 Y 输出 =====
			if (use_y_as_ref) {
				rm_motor_y.cmd(-DAMPING * rm_motor_y.speed);
			}
			else {
				rm_motor_pid_angle_y.calc(target_angle_y, rm_motor_y.angle);
				float target_speed_y = rm_motor_pid_angle_y.out + bmi088.gyro[2] * DIR_Y * KFF;
				rm_motor_pid_speed_y.calc(target_speed_y, rm_motor_y.speed);
				float ffm_y = 0.0f;
				if (fabsf(rm_motor_pid_speed_y.out) > 0.5f) {  // 目标速度够大才补偿
					ffm_y = (rm_motor_pid_speed_y.out > 0 ? 1.0f : -1.0f) * 0.052f;
				}
				rm_motor_y.cmd(rm_motor_pid_speed_y.out + ffm_y);
			}
		}

		// ========================================
		//         右拨杆 DOWN：禁用
		// ========================================
		else {
			rm_motor_x.cmd(0.0f);
			rm_motor_y.cmd(0.0f);
			rm_motor_pid_angle_x.clear();
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_angle_y.clear();
			rm_motor_pid_speed_y.clear();
			was_disabled = true;
		}

		send_rm_motor();
		last_sw_r = remote.sw_r;
		osDelay(1);
	}
}*/
/*#include "control_task.hpp"

#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

GimbalData rm_motor_data_x;
GimbalData rm_motor_data_y;
extern sp::Mahony imu;

//                           dt     kp    ki    kd    mo   mio   alpha  ang? dynamic?
// 电机 X 的两环 PID
sp::PID rm_motor_pid_angle_x(0.001f, 28.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_x(0.001f, 0.06f, 0.02f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

// 电机 Y 的两环 PID
sp::PID rm_motor_pid_angle_y(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_y(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);
extern "C" void control_task()
{
	remote.request();

	osDelay(100);
	can_start();
	osDelay(100);

	// 初始化
	rm_motor_data_x.target_angle_set = -3.65f;
	rm_motor_data_x.target_speed_set = 0.0f;
	rm_motor_data_x.given_torque = 0.0f;
	rm_motor_data_y.target_angle_set = 0.0f;
	rm_motor_data_y.target_speed_set = 0.0f;
	rm_motor_data_y.given_torque = 0.0f;
	while (true) {
		if (remote.sw_r == sp::DBusSwitchMode::MID) {
			// 单速度环pid
			//rm_motor_pid_speed.calc(rm_motor_data_x.target_speed_set, rm_motor_x.speed);
			//rm_motor_data_x.given_torque = rm_motor_pid_speed_x.out + ff;

			//双环pid
			rm_motor_pid_angle_x.calc(rm_motor_data_x.target_angle_set, rm_motor_x.angle);
			rm_motor_data_x.target_speed_set = rm_motor_pid_angle_x.out;
			rm_motor_pid_speed_x.calc(rm_motor_data_x.target_speed_set, rm_motor_x.speed);

			float ffm_x = 0.0f;
			if (fabsf(rm_motor_pid_speed_x.out) > 0.5f) {  // 目标速度够大才补偿
				ffm_x = (rm_motor_pid_speed_x.out > 0 ? 1.0f : -1.0f) * 0.05f;
			}

			rm_motor_data_x.given_torque =
				rm_motor_pid_speed_x.out + (-0.26f) * cosf(0.30f - imu.roll) + ffm_x;  //重力前馈
		}
		else if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
			rm_motor_data_x.given_torque = 0.0f;
		}
		else if (remote.sw_r == sp::DBusSwitchMode::UP) {
			rm_motor_data_x.given_torque = 0.0f;
		}

		rm_motor_x.cmd(rm_motor_data_x.given_torque);
		send_rm_motor();
		osDelay(1);
	}
}*/
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
<<<<<<< HEAD
/*前馈增益*/
static constexpr float kPitchVelFf = 2.0f;	//参考速度放大多少倍
static constexpr float kYawVelFf = 0.0f;
static constexpr float kPitchAccFf = 0.0f;	//参考加速度要乘多大惯量变成力矩 kg*m^2
=======

static constexpr float kPitchVelFf = 2.35f;
static constexpr float kYawVelFf = 0.0f;
static constexpr float kPitchAccFf = 0.0025f;
>>>>>>> bda8a53d1697a2f05285230c36934d2e824db6e9
static constexpr float kYawAccFf = 0.0f;

static constexpr float kPitchBias = -4.00f;	 //绝对目标角 rad
static constexpr float kYawBias = 0.0f;

static constexpr float kPitchSign = 1.0f;  //方向标定（电机与机构正方向比例）
static constexpr float kYawSign = 1.0f;

/* ===== 新增:进入 MID 后先保持的时长 ===== */
static constexpr uint32_t kStartupHoldMs = 500U;

sp::PID rm_motor_pid_angle_x(0.001f, 28.0f, 0.1f, 1.0f, 50.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_x(0.001f, 0.06f, 0.01f, 0.0f, 0.5f, 0.5f, 1.0f, false, true);
sp::PID rm_motor_pid_angle_y(0.001f, 50.0f, 1.0f, 1.0f, 400.0f, 5.0f, 1.0f, true, true);
sp::PID rm_motor_pid_speed_y(0.001f, 0.007f, 0.01f, 0.0f, 5.0f, 0.5f, 1.0f, false, true);

extern "C" void control_task()
{
	remote.request();
	osDelay(100);
	can_start();
	osDelay(100);

	rm_motor_data_x.given_torque = 0.0f;
	rm_motor_data_y.given_torque = 0.0f;

	/* ===== 新增:进入 MID 时刻与状态 ===== */
	TickType_t mid_enter_tick = 0;
	bool mid_active = false;

	while (true) {
		TrajectoryPoint traj = trajectory_read();

		if (remote.sw_r == sp::DBusSwitchMode::MID) {
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

			rm_motor_data_x.given_torque = rm_motor_pid_speed_x.out +
										   (-0.10f) * cosf(0.0f - imu.roll) + ffm_x +
										   kPitchAccFf * kPitchSign * pitch_traj_acc;

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
		else {
			mid_active = false; /* 退出 MID,下次进入重新计时 */
			rm_motor_data_x.given_torque = 0.0f;
			rm_motor_data_y.given_torque = 0.0f;
			rm_motor_pid_angle_x.clear();
			rm_motor_pid_speed_x.clear();
			rm_motor_pid_angle_y.clear();
			rm_motor_pid_speed_y.clear();
		}

		rm_motor_x.cmd(rm_motor_data_x.given_torque);
		rm_motor_y.cmd(rm_motor_data_y.given_torque);
		send_rm_motor();
		osDelay(1);
	}
}