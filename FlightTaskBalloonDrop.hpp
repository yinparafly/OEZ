/**
 * FlightTaskBalloonDrop.hpp
 *
 * 气球投放专用飞行任务（固定翼版本）。
 * 固定翼不能悬停，Phase 4 改为保持巡航速度平飞。
 *
 * ===== 新增文件 =====
 */

#pragma once

#include "FlightTask.hpp"
#include <lib/mathlib/mathlib.h>

class FlightTaskBalloonDrop : public FlightTask
{
public:
	FlightTaskBalloonDrop() = default;
	virtual ~FlightTaskBalloonDrop() = default;

	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;
	bool update() override;

private:
	enum class DropPhase : uint8_t {
		FREE_FALL,
		PULL_UP,
		MOTOR_START,
		CRUISE
	};

	DropPhase _phase{DropPhase::FREE_FALL};

	DEFINE_PARAMETERS_CUSTOM_PARENT(FlightTask,
		(ParamFloat<px4::params::BD_TRIG_RATIO>)    _param_bd_trigger_ratio,
		(ParamFloat<px4::params::BD_LOAD_FACTOR>)    _param_bd_load_factor,
		(ParamFloat<px4::params::BD_CRUISE_SPD>)     _param_bd_cruise_speed,
		(ParamFloat<px4::params::BD_MOTOR_RAMP>)     _param_bd_motor_ramp_time,
		(ParamFloat<px4::params::BD_CRUISE_THR>)     _param_bd_cruise_throttle,
		(ParamFloat<px4::params::BD_PITCH_THRESH>)   _param_bd_pitch_threshold
	)

	float _trigger_airspeed{0.0f};
	float _pullup_omega{0.0f};
	float _motor_ramp_elapsed{0.0f};
	float _initial_yaw{0.0f};
	bool _initial_yaw_captured{false};
	float _pullup_start_time{0.0f};

	void _updateFreeFall();
	void _updatePullUp();
	void _updateMotorStart();
	void _updateCruise();
};
