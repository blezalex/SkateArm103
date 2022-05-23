#pragma once


#include "global.h"
#include "pid.hpp"
#include "lpf.hpp"


class BalanceController  {
public:
	explicit BalanceController(const Config* settings) :
		settings_(settings), angle_pid_(&settings->balance_pid), rate_pid_(&settings->rate_pid) {
		reset();
	}

	void reset() {
		angle_pid_.reset();
		rate_pid_.reset();
		prev_error_ = 0;
	}

	float getInput(float angle, float balance_angle) {
		return balance_angle - angle;
	}

	float calcRatePid(float rateRequest,  float rate) {
		float error = rateRequest * 400 - rate;
		float d_term  = error - prev_error_;
		prev_error_ = error;
		float result = rate_pid_.compute(error, d_term);

		// shared with balance_settings.max_update_limiter ConstrainedOut
		// result = constrain(result, -settings_->balance_settings.max_update_limiter, settings_->balance_settings.max_update_limiter);
		return result;
	}

	// Compute torque needed while board in normal mode.
	// Returns torque request based on current imu and gyro readings. Expected range is -1:1,
	// but not limited here to that range.
	float compute(float angle, float rate) {
		float rateRequest = angle_pid_.compute(angle);
		return calcRatePid(rateRequest, rate);
	}

	// Compute torque needed while board in starting up phase (coming from one side to balanced state).
	// Returns torque request based on current imu and gyro readings. Expected range is -1:1,
	// but not limited here to that range.
	int16_t computeStarting(float angle, float rate, float pid_P_multiplier) {
		rate_pid_.resetI();
		angle_pid_.resetI();
		float rateRequest = angle_pid_.compute(angle);
		rateRequest *= pid_P_multiplier;
		return calcRatePid(rateRequest, rate);
	}

private:
	const Config* settings_;
	PidController angle_pid_;
	PidController rate_pid_;
	float prev_error_;

};
