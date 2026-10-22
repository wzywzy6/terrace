#include "cmsis_os.h"
#include "uart.hpp"

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	auto stamp_ms = osKernelSysTick();

	if (huart == &huart3) {
		remote.update(Size, stamp_ms);
		remote.request();
	}
	else if (huart == &huart6) {
		gy56.update(Size, stamp_ms);
		gy56.request();
	}
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if (huart == &huart3) {
		remote.request();
	}
	else if (huart == &huart6) {
		gy56.request();
	}
}