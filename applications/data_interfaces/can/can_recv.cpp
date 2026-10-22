#include "cmsis_os.h"
#include "data_interfaces/can/can.hpp"

void can1_recv(uint32_t stamp_ms)
{
	can1.recv();
	if (can1.rx_id == rm_motor_x.rx_id) rm_motor_x.read(can1.rx_data, stamp_ms);
	if (can1.rx_id == rm_motor_y.rx_id) rm_motor_y.read(can1.rx_data, stamp_ms);
	if (can1.rx_id == rm_motor_z.rx_id) rm_motor_z.read(can1.rx_data, stamp_ms);
	;
}

void can2_recv(uint32_t stamp_ms) { can2.recv(); }

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
	auto stamp_ms = osKernelSysTick();

	while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
		if (hcan == &hcan1) {
			can1_recv(stamp_ms);
		}

		if (hcan == &hcan2) {
			can2_recv(stamp_ms);
		}
	}
}