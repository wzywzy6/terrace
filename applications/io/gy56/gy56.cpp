#include "gy56.hpp"

namespace sp
{
namespace
{
constexpr uint8_t kHeadByte = 0x5A;		  // 帧头
constexpr uint8_t kTypeByte = 0x15;		  // 本帧数据类型
constexpr uint8_t kMinDataLen = 3;		  // 距离(2) + 模式(1)
constexpr uint8_t kMaxDataLen = 4;		  // 距离(2) + 模式(1) + 温度(1)
constexpr uint16_t kMinDistanceCm = 3;	  // 小于最小测距距离按“错误值”处理（无回波可能给 1~2）
constexpr uint16_t kMaxDistanceCm = 350;  // 按当前传感器实测约 3.5m 限制有效量程
}  // namespace

GY56::GY56(UART_HandleTypeDef * huart)
: huart_(huart),
  index_(0),
  frame_len_(0),
  distance_(0),
  mode_(0),
  temperature_(0),
  valid_(false),
  out_of_range_(false),
  has_read_(false),
  last_update_ms_(0),
  last_frame_ms_(0)
{
}

void GY56::begin()
{
	// 波特率由 CubeMX 配好（9600 8N1），这里只负责挂接收 + 配置模块。
	request();

	// 1) 改成一般测量：A5 54 00
	set_mode(RangeMode::NORMAL);
	HAL_Delay(20);

	// 2) 存进模块 Flash：A5 87 01，不存的话下次上电又回到高精度
	save_settings();
	HAL_Delay(50);	// 等模块写完 Flash 再发下一条

	// 3) 连续输出：A5 65 01
	send_command(0x65, 0x01);
}

void GY56::request()
{
	// 重新挂起 DMA + 空闲中断接收（和 DBus 的用法一致）
	HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buff_, sizeof(rx_buff_));

	// 关掉 DMA 半传输中断，否则每半缓冲也会进一次回调
	__HAL_DMA_DISABLE_IT(huart_->hdmarx, DMA_IT_HT);
}

void GY56::send_command(uint8_t cmd, uint8_t value)
{
	// 命令格式：A5 + 指令 + 参数 + 校验（前三字节累加和取低 8 位）
	uint8_t buff[4] = {0xA5, cmd, value, static_cast<uint8_t>(0xA5 + cmd + value)};
	HAL_UART_Transmit(huart_, buff, sizeof(buff), 100);
}

void GY56::set_mode(RangeMode mode)
{
	// 0x54 的指令值就是帧里的 Byte6 模式号：00 一般 / 01 高速 / 02 高精度 / 03 长距
	send_command(0x54, static_cast<uint8_t>(mode));
}

void GY56::save_settings()
{
	// 0x87 01：把当前配置写进模块 Flash，下次上电仍然生效
	send_command(0x87, 0x01);
}

void GY56::update(uint16_t size, uint32_t stamp_ms)
{
	if (size > sizeof(rx_buff_)) size = sizeof(rx_buff_);

	for (uint16_t i = 0; i < size; ++i) {
		const uint8_t byte = rx_buff_[i];

		// 帧头 1
		if (index_ == 0) {
			if (byte == kHeadByte) frame_[index_++] = byte;
			continue;
		}

		// 帧头 2
		if (index_ == 1) {
			if (byte == kHeadByte)
				frame_[index_++] = byte;
			else
				index_ = 0;
			continue;
		}

		// 帧类型 0x15
		if (index_ == 2) {
			if (byte == kTypeByte)
				frame_[index_++] = byte;
			else
				index_ = 0;
			continue;
		}

		// 数据量（3 或 4），顺便算出本帧总长度
		if (index_ == 3) {
			if (byte < kMinDataLen || byte > kMaxDataLen) {
				index_ = 0;
				continue;
			}
			frame_[index_++] = byte;
			frame_len_ =
				static_cast<uint8_t>(byte + 5);	 // 2 帧头 + 1 类型 + 1 长度 + n 数据 + 1 校验
			continue;
		}

		// 数据 + 校验
		frame_[index_++] = byte;
		if (index_ < frame_len_) continue;

		// 一帧收满，先校验再解析
		uint8_t sum = 0;
		for (uint8_t k = 0; k + 1 < frame_len_; ++k) sum = static_cast<uint8_t>(sum + frame_[k]);

		if (sum == frame_[frame_len_ - 1]) {
			has_read_ = true;
			last_frame_ms_ = stamp_ms;
			const uint16_t dist = static_cast<uint16_t>((frame_[4] << 8) | frame_[5]);
			if (dist >= kMinDistanceCm && dist <= kMaxDistanceCm) {
				distance_ = dist;
				mode_ = frame_[6];
				if (frame_len_ == 9) temperature_ = static_cast<int8_t>(frame_[7]);
				valid_ = true;
				out_of_range_ = false;
				last_update_ms_ = stamp_ms;
			}
			else {
				valid_ = false;		   // 错误值(0/1/2) / 超出量程 / 无回波
				out_of_range_ = true;  // 帧本身是好的，只是这一帧没有可信距离
			}
		}
		else {
			// 校验失败：只打断“有效”标记，不改 out_of_range_。
			// 坏帧既不能算“远”也不能算“近”，掉线由 is_alive() 判定。
			valid_ = false;
		}

		index_ = 0;
	}
}

bool GY56::is_alive(uint32_t now_ms) const { return has_read_ && (now_ms - last_frame_ms_ < 200); }

}  // namespace sp
