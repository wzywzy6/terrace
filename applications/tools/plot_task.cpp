#include "cmsis_os.h"
#include "control_task.hpp"
#include "data_interfaces/uart/uart.hpp"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"	// ← 如果还没 include

extern sp::Mahony imu;	// ← 声明外部对象
extern volatile float pitch;

sp::Plotter plotter(&huart1);

extern "C" void plot_task()
{
	while (1) {
		plotter.plot(
			/*rm_motor_x.angle,
      rm_motor_x.speed,
      rm_motor_x.torque,*/
			imu.roll + 0.269f /* - 0.35*/, pitch, imu.roll - pitch
			//sw_to_float(remote.sw_r)
		);

		osDelay(5);
	}
}
// clang-format on