/*cpp中定义*/
#include <math.h>
#include <stdint.h>
//yaw定义:
void yaw_detect(float gimbal_angle, float plan_angle, float * fire_ratio)
{
	if (fire_ratio == nullptr) {
		return;
	}
	constexpr float alpha = 0.001f;			   // 1ms的freertos对应的滤波系数
	constexpr float threshold = 0.5f / 57.3f;  // 角度转换成弧度

	static uint8_t fire = 0;

	fire = (fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;

	*fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}

//pitch定义:
void pitch_detect(float gimbal_angle, float plan_angle, float * fire_ratio)
{
	if (fire_ratio == nullptr) {
		return;
	}
	constexpr float alpha = 0.001f;			   // 1ms的freertos对应的滤波系数
	constexpr float threshold = 1.5f / 57.3f;  // 角度转换成弧度

	static uint8_t fire = 0;

	fire = (fabs(gimbal_angle - plan_angle) < threshold) ? 1u : 0u;

	*fire_ratio = alpha * static_cast<float>(fire) + (1.0f - alpha) * (*fire_ratio);
}
