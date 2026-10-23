#include "obstacle_guard.hpp"

#include <cmath>

namespace
{
float median3(float a, float b, float c)
{
	if (a > b) {
		const float t = a;
		a = b;
		b = t;
	}
	if (b > c) {
		const float t = b;
		b = c;
		c = t;
	}
	if (a > b) {
		const float t = a;
		a = b;
		b = t;
	}
	return b;
}
}  // namespace

void ObstacleGuard::reset()
{
	state_ = State::NORMAL;
	shaped_ = 0.0f;
	shaped_init_ = false;

	med_init_ = false;
	med_[0] = med_[1] = med_[2] = 0.0f;
	last_med_ = 0.0f;

	below_cnt_ = 0;
	above_cnt_ = 0;
	hold_ms_ = 0.0f;
	settle_ms_ = 0.0f;

	sensor_lost_ = false;
	measurement_valid_ = false;
	z_comp_motor_ = 0.0f;
	z_vel_ff_motor_ = 0.0f;

	now_ms_ = 0;
	invalid_active_ = false;
	invalid_since_ms_ = 0;
	invalid_release_ = false;
	rearm_active_ = false;
	rearm_ms_ = 0;

	// last_stamp_ 故意不清零：清掉会让“没有新帧”被误判成新帧
}

void ObstacleGuard::update(bool dist_ok, uint16_t dist_cm, uint32_t stamp_ms, uint32_t now_ms)
{
	now_ms_ = now_ms;
	const bool new_frame = (stamp_ms != last_stamp_);
	last_stamp_ = stamp_ms;
	sensor_lost_ = ((now_ms - stamp_ms) >= cfg_.sensor_timeout_ms);

	measurement_valid_ = dist_ok && !sensor_lost_ && (dist_cm != 0);

	// 只要这一拍没有可信距离，就累计“错误时间”。
	// 不依赖是否收到帧、也不依赖超量程标记：掉线、校验错、1~2cm 错误值都照样累计。
	if (measurement_valid_) {
		invalid_active_ = false;
	}
	else if (!invalid_active_) {
		invalid_active_ = true;
		invalid_since_ms_ = now_ms;
	}
	invalid_release_ = cfg_.invalid_release_enable && invalid_active_ &&
					   ((now_ms - invalid_since_ms_) >= cfg_.invalid_release_ms);

	// 无效帧和断流都打断连续判据；恢复后从第一帧重新初始化中值滤波。
	// 必须在 new_frame 判断外处理：超时发生时并没有新帧。
	if (!measurement_valid_) {
		below_cnt_ = 0;
		above_cnt_ = 0;
		med_init_ = false;
		last_med_ = 0.0f;
		return;
	}

	if (!new_frame) return;
	const float d = static_cast<float>(dist_cm);

	if (!med_init_) {
		med_[0] = med_[1] = med_[2] = d;
		med_init_ = true;
	}
	else {
		med_[2] = med_[1];
		med_[1] = med_[0];
		med_[0] = d;
	}

	last_med_ = median3(med_[0], med_[1], med_[2]);

	if (last_med_ < cfg_.trig_cm) {
		if (below_cnt_ < 255) below_cnt_++;
	}
	else {
		below_cnt_ = 0;
	}

	if (last_med_ > cfg_.rel_cm) {
		if (above_cnt_ < 255) above_cnt_++;
	}
	else {
		above_cnt_ = 0;
	}
}

float ObstacleGuard::apply(float normal_target, float dt, float & vel_ff)
{
	if (dt <= 0.0f) dt = 0.001f;
	const float dt_ms = dt * 1000.0f;

	if (!shaped_init_) {
		shaped_ = normal_target;
		shaped_init_ = true;
	}

	/* ===== 1. 状态迁移（只在这里做，保证和整形同拍）===== */
	switch (state_) {
		case State::NORMAL:
			// 无回波抬回后的冷静期：防止“折叠位看不到目标”时折-抬-折反复
			if (rearm_active_ && ((now_ms_ - rearm_ms_) >= cfg_.fold_rearm_ms)) {
				rearm_active_ = false;
				below_cnt_ = 0;	 // 冷静期里攒下的近距帧不算数，必须冷静期后重新确认
			}
			if (measurement_valid_ && (below_cnt_ >= cfg_.trig_frames) && !rearm_active_) {
				state_ = State::LOWERING;
				hold_ms_ = 0.0f;
				settle_ms_ = 0.0f;
				below_cnt_ = 0;
				above_cnt_ = 0;
			}
			break;

		case State::LOWERING:
			hold_ms_ += dt_ms;
			if (std::fabs(shaped_ - cfg_.avoid_pitch) < cfg_.done_tol) {
				state_ = State::LOWERED;
				settle_ms_ = 0.0f;
				above_cnt_ = 0;	 // 折叠过程中的读数不计入释放
			}
			break;

		case State::LOWERED: {
			hold_ms_ += dt_ms;
			settle_ms_ += dt_ms;

			const bool settled = (settle_ms_ >= static_cast<float>(cfg_.aim_settle_ms));
			const bool far_release =
				measurement_valid_ && (above_cnt_ >= cfg_.rel_frames) &&
				(hold_ms_ >= static_cast<float>(cfg_.min_hold_ms)) && settled;
			const bool invalid_release =
				invalid_release_ &&
				(hold_ms_ >= static_cast<float>(cfg_.fold_lockout_ms)) && settled;

			if (far_release) {
				state_ = State::RAISING;
				above_cnt_ = 0;
			}
			else if (invalid_release) {
				state_ = State::RAISING;
				above_cnt_ = 0;
				rearm_active_ = true;
				rearm_ms_ = now_ms_;
			}
			break;
		}

		case State::RAISING:
			if (std::fabs(shaped_ - normal_target) < cfg_.done_tol) {
				state_ = State::NORMAL;
				below_cnt_ = 0;
			}
			break;
	}

	/* ===== 2. x 目标整形 ===== */
	if (state_ == State::NORMAL) {
		shaped_ = normal_target;
		vel_ff = 0.0f;
	}
	else if (state_ == State::LOWERED) {
		shaped_ = cfg_.avoid_pitch;
		vel_ff = 0.0f;
	}
	else {
		const float goal = (state_ == State::LOWERING) ? cfg_.avoid_pitch : normal_target;
		const float dmax = cfg_.slew_rad_s * dt;

		float step = goal - shaped_;
		if (step > dmax) step = dmax;
		if (step < -dmax) step = -dmax;

		shaped_ += step;
		vel_ff = step / dt;
	}

	/* ===== 3. z 瞄准补偿（与 x 同步，抬回时自动反向）===== */
	if (cfg_.z_comp_enable) {
		const float dx = shaped_ - normal_target;  // 输出端折叠量
		float zc = cfg_.z_ratio * dx;

		const bool clamped = (std::fabs(zc) >= cfg_.z_comp_max_rad);
		if (clamped) zc = (zc > 0.0f) ? cfg_.z_comp_max_rad : -cfg_.z_comp_max_rad;

		z_comp_motor_ = cfg_.z_gear * zc;
		z_vel_ff_motor_ = clamped ? 0.0f : (cfg_.z_gear * cfg_.z_ratio * vel_ff);
	}
	else {
		z_comp_motor_ = 0.0f;
		z_vel_ff_motor_ = 0.0f;
	}

	return shaped_;
}
