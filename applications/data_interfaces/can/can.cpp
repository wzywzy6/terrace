#include "can.hpp"

#include "cmsis_os.h"

void can_start()
{
	can1.config();
	can1.start();
	can2.config();
	can2.start();
}

// can.cpp
void send_rm_motor()
{
	// ---- 第一帧：0x1FF，发 GM6020 (1–4) ----
	for (int i = 0; i < 8; i++) can1.tx_data[i] = 0;
	rm_motor_x.write(can1.tx_data);	 // GM6020 id=1
	rm_motor_y.write(can1.tx_data);	 // GM6020 id=4
	can1.send(rm_motor_x.tx_id);	 // 0x1FF

	// ---- 第二帧：0x200，发 M2006 (1–4) ----
	for (int i = 0; i < 8; i++) can1.tx_data[i] = 0;
	rm_motor_z.write(can1.tx_data);	 // M2006 id=3
	can1.send(rm_motor_z.tx_id);	 // 应为 0x200
}
