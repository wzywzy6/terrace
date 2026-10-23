#ifndef OBSTACLE_GUARD_HPP
#define OBSTACLE_GUARD_HPP

#include <cstdint>

// 红外避障：测距预处理 + 状态机 + 折叠目标整形 + z 轴瞄准补偿
//
// 纯逻辑模块：不访问电机、不建任务、不含 HAL 调用。
// 只应在需要避障的挡位（当前为 UP）里按 1kHz 调用：
//
//   obstacle_guard.update(gy56_dist_ok, gy56_dist_cm, gy56_frame_ms, now_ms);
//   float x_vel_ff = 0.0f;
//   const float x_target = obstacle_guard.apply(kPitchBias, 0.001f, x_vel_ff);
//   const float z_comp   = obstacle_guard.z_comp_motor();    // M2006 电机侧 rad
//   const float z_vel_ff = obstacle_guard.z_vel_ff_motor();  // M2006 电机侧 rad/s
//
// 单位：
//   - apply() 的 normal_target / 返回值 / vel_ff：输出端（GM6020）rad、rad/s；
//   - z_comp_motor() / z_vel_ff_motor()：M2006 电机侧 rad、rad/s（已乘 36 减速比），
//     直接叠加到 -imu.roll * 36.0f 上。
//
// 关于 z 补偿（很重要）：
//   - C 板（IMU）固定在 Pitch 上、不随 Z 转动时，imu.roll 已经包含折叠角，
//     控制层原有的 -imu.roll * 36 已经在跟随折叠；此时本模块的 z 补偿必须关闭
//     （z_comp_enable = false），否则同一个折叠量会被算两遍（重复或抵消）。
//   - 只有 IMU 不随 Pitch 转动（例如装在底座）时，才需要打开本模块的 z 补偿。
class ObstacleGuard
{
  public:
	enum class State : uint8_t
	{
		NORMAL = 0,
		LOWERING,
		LOWERED,
		RAISING,
	};

	struct Config
	{
		/* ---- 传感器判据（按实测帧率与车速改）---- */
		float trig_cm = 35.0f;		   // 触发距离
		float rel_cm = 50.0f;		   // 释放距离，必须 > trig_cm
		uint8_t trig_frames = 2;	   // 连续多少个“新帧”才触发
		uint8_t rel_frames = 5;		   // 连续多少个“新帧”才释放
		uint32_t min_hold_ms = 500;	   // 触发后最短保持
		uint32_t aim_settle_ms = 150;  // 折叠到位后等光轴稳定
		uint32_t sensor_timeout_ms = 200;

		/* ---- 错误值/无回波策略 ----
		   未折叠时：错误值绝不参与触发（触发只认有效距离）。
		   已折叠时：只要连续一段时间“没有可信距离”（超量程 / 0 / 1 / 2 / 校验错 /
		   断流都算），就必须抬回，不依赖模块是否还在发帧。 */
		bool invalid_release_enable = true;	 // false = 退回“错误值保持折叠”
		uint32_t invalid_release_ms = 600;	 // 连续多久没有可信距离就抬回
		uint32_t fold_lockout_ms = 700;		 // 折叠后至少保持这么久才允许抬回
		uint32_t fold_rearm_ms = 1200;		 // 抬回后的冷静期，期间不重新折叠

		/* ---- 折叠目标 ---- */
		float avoid_pitch = -3.50f;	 // 与 kPitchBias 同一坐标系
		float slew_rad_s = 3.0f;	 // 目标斜坡限速
		float done_tol = 0.02f;		 // 到位判定

		/* ---- z 轴瞄准补偿 ----
		   默认关闭：C 板固定在 Pitch 上时，imu.roll 已经包含折叠角，
		   控制层的 -imu.roll * 36 已经在补偿；这里再按指令折叠量叠加会重复/抵消。
		   只有 IMU 不随 Pitch 转动（例如装在底座）时才置 true。 */
		bool z_comp_enable = false;
		float z_ratio = 1.0f;		   // 输出端 z / 输出端 x，符号按实测
		float z_gear = 36.0f;		   // M2006 减速比
		float z_comp_max_rad = 0.60f;  // 输出端补偿上限
	};

	ObstacleGuard() = default;
	explicit ObstacleGuard(const Config & cfg) : cfg_(cfg) {}

	void configure(const Config & cfg) { cfg_ = cfg; }
	void reset();

	// 1kHz 调用：stamp_ms 为最近校验通过帧的时间，dist_ok 表示距离有效。
	void update(bool dist_ok, uint16_t dist_cm, uint32_t stamp_ms, uint32_t now_ms);

	// 1kHz 调用：状态迁移 + 目标整形；返回整形后的 x 目标
	float apply(float normal_target, float dt, float & vel_ff);

	State state() const { return state_; }
	bool active() const { return state_ != State::NORMAL; }
	bool sensor_lost() const { return sensor_lost_; }
	float z_comp_motor() const { return z_comp_motor_; }
	float z_vel_ff_motor() const { return z_vel_ff_motor_; }
	float filtered_cm() const { return last_med_; }

  private:
	Config cfg_{};

	State state_ = State::NORMAL;
	float shaped_ = 0.0f;
	bool shaped_init_ = false;

	float med_[3] = {0.0f, 0.0f, 0.0f};
	bool med_init_ = false;
	float last_med_ = 0.0f;
	uint32_t last_stamp_ = 0;
	uint8_t below_cnt_ = 0;
	uint8_t above_cnt_ = 0;

	uint32_t now_ms_ = 0;			// 最近一次 update 的时间，供 apply 里的冷静期计时使用
	bool invalid_active_ = false;	// “没有可信距离”计时是否在走
	uint32_t invalid_since_ms_ = 0;
	bool invalid_release_ = false;	// 无有效距离已超时，可以抬回
	bool rearm_active_ = false;		// 抬回后的冷静期
	uint32_t rearm_ms_ = 0;

	float hold_ms_ = 0.0f;
	float settle_ms_ = 0.0f;

	bool sensor_lost_ = false;
	bool measurement_valid_ = false;
	float z_comp_motor_ = 0.0f;
	float z_vel_ff_motor_ = 0.0f;
};

#endif	// OBSTACLE_GUARD_HPP
