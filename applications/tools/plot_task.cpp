#include "cmsis_os.h"
#include "control_task.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"	// ← 如果还没 include

extern sp::Mahony imu;	// ← 声明外部对象
extern volatile float pitch;
extern volatile float yaw;
extern volatile float yaw_ratio;
extern volatile float pitch_ratio;
extern volatile float imu_last_yaw;
extern GimbalData rm_motor_data_z;

sp::Plotter plotter(&huart1);

extern "C" void plot_task()
{
	while (1) {
		plotter.plot(
			yaw_ratio, yaw, imu.yaw - imu_last_yaw,
			gy56_dist_ok ? static_cast<float>(gy56_dist_cm) : -1.0f
			//rm_motor_data_z.target_angle_set
			/*pitch_ratio, imu.roll + 0.269f + 0.023f, pitch, imu.roll - pitch*/
			/*gy56_dist_ok ? static_cast<float>(gy56_dist_cm) : -1.0f*/);

		osDelay(5);
	}
}
// clang-format on