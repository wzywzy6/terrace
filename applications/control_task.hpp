#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP

#include <cstdint>

#include "motor/rm_motor/rm_motor.hpp"

inline sp::RM_Motor rm_motor_x(1, sp::RM_Motors::GM6020);
inline sp::RM_Motor rm_motor_y(4, sp::RM_Motors::GM6020);
inline sp::RM_Motor rm_motor_z(3, sp::RM_Motors::M2006, 1.0f, true);

typedef struct
{
	float target_angle_set;	 // 绝对角度的目标值，rad
	float target_speed_set;	 // 绝对速度的目标值，rad/s
	float given_torque;		 // 电机给定的力矩，Nm
} GimbalData;

// GY-56 的处理结果：control_task 写，plot_task 只读，保证两边是同一份数据
inline uint16_t gy56_dist_cm = 0;		// 单位 cm，控制实际使用的值
inline bool gy56_dist_ok = false;		// 最近一帧是否有效
inline bool gy56_out_of_range = false;	// 活着但没有有效距离（超量程/无回波）
float Fsign(float fsign);

#endif	// CONTROL_TASK_HPP