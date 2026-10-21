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
			imu.roll + 0.269f,
			pitch,
			imu.roll - pitch,
			gy56_dist_ok ? static_cast<float>(gy56_dist_cm) : -1.0f
		);

		osDelay(5);
	}
}
// clang-format on