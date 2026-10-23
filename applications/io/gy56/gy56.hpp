#ifndef SP__GY56_HPP
#define SP__GY56_HPP

#include <cstdint>

#include "usart.h"

namespace sp
{
/**
 * GY-56 红外激光测距模块（VL53L0X + 片内 MCU）串口驱动
 *
 * 模块默认工作在串口模式（PS 焊点断开），9600 8N1，
 * 上电后按 5A 5A 15 03/04 ... 的帧格式连续输出，距离单位 cm。
 *
 * 用法：
 *   gy56.begin();                       // 上电初始化一次
 *   gy56.distance();                    // 读距离，单位 cm
 *   gy56.is_alive(osKernelSysTick());   // 200ms 内有新数据说明通信正常
 *
 * 接收采用 DMA + 空闲中断，解析在 HAL_UARTEx_RxEventCallback 里完成，
 * 具体见 applications/data_interfaces/uart/uart_recv.cpp。
 */
class GY56
{
  public:
	// 测量模式，对应帧里的 Byte6（出厂默认是高精度 0x02）
	enum class RangeMode : uint8_t
	{
		NORMAL = 0x00,		   // 一般测量
		FAST = 0x01,		   // 高速测量
		HIGH_ACCURACY = 0x02,  // 高精度测量
		LONG_RANGE = 0x03,	   // 长距离测量
	};

	explicit GY56(UART_HandleTypeDef * huart);

	void begin();
	void request();
	void update(uint16_t size, uint32_t stamp_ms);

	void send_command(uint8_t cmd, uint8_t value);
	void set_mode(RangeMode mode);
	void save_settings();

	bool is_alive(uint32_t now_ms) const;
	bool is_valid() const { return valid_; }
	// 最近一帧校验通过、但距离不在有效量程内（超量程/无回波）
	bool out_of_range() const { return out_of_range_; }

	uint16_t distance() const { return distance_; }	 // 单位: cm
	uint8_t mode() const { return mode_; }			 // 模块测量模式
	int8_t temperature() const { return temperature_; }
	uint32_t last_update_ms() const { return last_update_ms_; }  // 最近有效距离
	uint32_t last_frame_ms() const { return last_frame_ms_; }    // 最近校验通过的帧，含超量程

  private:
	UART_HandleTypeDef * huart_;

	uint8_t rx_buff_[32];  // DMA 接收落点
	uint8_t frame_[12];	   // 组帧缓存
	uint8_t index_;		   // 当前帧已收字节数
	uint8_t frame_len_;	   // 本帧总长度

	uint16_t distance_;
	uint8_t mode_;
	int8_t temperature_;

	bool valid_;
	bool out_of_range_;
	bool has_read_;
	uint32_t last_update_ms_;
	uint32_t last_frame_ms_;
};

}  // namespace sp

#endif	// SP__GY56_HPP
