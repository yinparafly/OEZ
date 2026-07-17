/**
 * FlightTaskBalloonDrop.cpp
 *
 * 气球投放飞行任务 — 实现文件
 *
 * ===== 新增文件 =====
 */

#include "FlightTaskBalloonDrop.hpp"
#include <matrix/math.hpp>

using namespace matrix;

bool FlightTaskBalloonDrop::activate(const trajectory_setpoint_s &last_setpoint)
{
	_phase = DropPhase::FREE_FALL;
	_motor_ramp_elapsed = 0.0f;
	_initial_yaw_captured = false;
	_pullup_start_time = 0.0f;

	_trigger_airspeed = _param_bd_trigger_ratio.get() * _param_bd_cruise_speed.get();

	float g = 9.81f;
	float n = _param_bd_load_factor.get();
	_pullup_omega = (n - 1.0f) * g / _trigger_airspeed;

	return true;
}

bool FlightTaskBalloonDrop::updateInitialize()
{
	_evaluateVehicleLocalPosition();
	return true;
}

bool FlightTaskBalloonDrop::update()
{
	_resetSetpoints();

	switch (_phase) {
	case DropPhase::FREE_FALL:
		_updateFreeFall();
		break;
	case DropPhase::PULL_UP:
		_updatePullUp();
		break;
	case DropPhase::MOTOR_START:
		_updateMotorStart();
		break;
	case DropPhase::CRUISE:
		_updateCruise();
		break;
	}

	return true;
}

// =========================================================================
// Phase 1: 自由下落 — 保持航向，油门=0，检测空速触发
// =========================================================================
void FlightTaskBalloonDrop::_updateFreeFall()
{
	if (!_initial_yaw_captured) {
		_initial_yaw = _yaw;
		_initial_yaw_captured = true;
	}

	_yaw_setpoint = _initial_yaw;
	_yawspeed_setpoint = 0.0f;

	// 用速度向量估算空速（忽略风的影响）
	float vx = _velocity(0);
	float vy = _velocity(1);
	float vz = _velocity(2);
	float speed = sqrtf(vx * vx + vy * vy + vz * vz);

	if (speed >= _trigger_airspeed) {
		_phase = DropPhase::PULL_UP;
		_pullup_start_time = _time_stamp_current / 1e6f;
	}
}

// =========================================================================
// Phase 2: 拉起 — 恒定俯仰角速度，油门=0
// =========================================================================
void FlightTaskBalloonDrop::_updatePullUp()
{
	// 用速度估算当前空速
	float vx = _velocity(0);
	float vy = _velocity(1);
	float vz = _velocity(2);
	float current_airspeed = sqrtf(vx * vx + vy * vy + vz * vz);

	if (current_airspeed < 1.0f) {
		current_airspeed = 1.0f;
	}

	float g = 9.81f;
	float n = _param_bd_load_factor.get();
	_pullup_omega = (n - 1.0f) * g / current_airspeed;

	_yaw_setpoint = _initial_yaw;

	float current_time = _time_stamp_current / 1e6f;
	float elapsed = current_time - _pullup_start_time;
	float total_time = (M_PI_F / 2.0f) / _pullup_omega;

	if (elapsed >= total_time * 1.1f) {
		_phase = DropPhase::MOTOR_START;
		_motor_ramp_elapsed = 0.0f;
	}
}

// =========================================================================
// Phase 3: 电机启动 — 固定翼：油门渐增，保持平飞加速
// =========================================================================
void FlightTaskBalloonDrop::_updateMotorStart()
{
	_motor_ramp_elapsed += _deltatime;

	// 固定翼：保持高度，向前加速
	// 设置前向速度指令，让飞机加速到巡航速度
	_velocity_setpoint = Vector3f(
		_param_bd_cruise_speed.get() * cosf(_initial_yaw),  // 前向分量
		_param_bd_cruise_speed.get() * sinf(_initial_yaw),  // 侧向分量
		0.0f                                                  // 高度保持
	);
	_yaw_setpoint = _initial_yaw;

	float ramp_time = _param_bd_motor_ramp_time.get();

	if (_motor_ramp_elapsed >= ramp_time) {
		_phase = DropPhase::CRUISE;
	}
}

// =========================================================================
// Phase 4: 巡航 — 固定翼：盘旋待命，等待切换 AUTO 模式执行航点
// =========================================================================
void FlightTaskBalloonDrop::_updateCruise()
{
	// 固定翼不能悬停，必须保持前向速度
	// 设置巡航速度，保持当前航向和高度
	_velocity_setpoint = Vector3f(
		_param_bd_cruise_speed.get() * cosf(_initial_yaw),
		_param_bd_cruise_speed.get() * sinf(_initial_yaw),
		0.0f
	);
	_yaw_setpoint = _initial_yaw;

	// [BALLOON_DROP] 在实际部署中，这里应该：
	// 1. 发布切换到 AUTO 模式的请求
	// 2. 让 navigator 接管执行航点任务
	// 当前实现：保持巡航速度平飞
}
