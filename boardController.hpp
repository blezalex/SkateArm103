#pragma once
#include <math.h>
#include <limits>

#include "global.h"
#include "pid.hpp"
#include "imu/imu.hpp"
#include "stateTracker.hpp"
#include "lpf.hpp"
#include "drv/vesc/vesc.hpp"
#include "io/genericOut.hpp"
#include "io/pwm_out.hpp"

#include "cmsis_boot/stm32f10x.h"
#include "stm_lib/inc/stm32f10x_tim.h"
#include "stm_lib/inc/stm32f10x_rcc.h"
#include "stm_lib/inc/stm32f10x_gpio.h"
#include "io/rx.h"

#define BRAKE_VIA_USART

class StepperOut {

public:
	StepperOut(int idx) : idx_(idx) {
		if (idx >= 3) while(1);
		set(0);
	}

	void set(float v_steps_sec) {
		comm_can_set_current(idx_ + 1, v_steps_sec);
	}

	int idx_;
};

class ConstrainedOut {
public:
	ConstrainedOut(StepperOut* motor_out, Config_BalancingConfig* balance_settings) :
		settings_(balance_settings),
		motor_out_(motor_out),
		motor_out_lpf_(&(balance_settings->output_lpf_rc)) {
			reset();
		}

	void set(float new_out) {
		float prev_val = motor_out_lpf_.getVal();

		new_out = motor_out_lpf_.compute(new_out);

		motor_out_->set(new_out);
	}

	float get() {
		return motor_out_lpf_.getVal();
	}

	void reset() {
		motor_out_lpf_.reset(0);
		motor_out_->set(0);
	}

private:
	Config_BalancingConfig* settings_;
	StepperOut* motor_out_;
	// This lpf is to smooth out motor output so stepper does not get spikes and does not skip steps.
	BiQuadLpf motor_out_lpf_;
};


class BoardController  : public UpdateListener  {
public:
	BoardController(Config* settings, IMU& imu, GenericOut& status_led,
			GenericOut& beeper, Guard** guards, int guards_count, GenericOut& green_led, VescComm* vesc)
	  : settings_(settings),
		imu_(imu),
		state_(guards, guards_count),
		pitch_balancer_(settings_),
		motor1_(&out[0], &settings->balance_settings),
		motor2_(&out[1], &settings->balance_settings),
		status_led_(status_led),
		beeper_(beeper),
		green_led_(green_led),
		fwd_lpf_(&settings->balance_settings.output_lpf_rc),
		vesc_(vesc) {
	}

	float mapRcInput(uint16_t input) {
		if (input < MIN_MOTOR_CMD || input > MAX_MOTOR_CMD ) {
			return 0;
		}

		return fmap(input, MIN_MOTOR_CMD, MAX_MOTOR_CMD, -1, 1);
	}

	// Main control loop. Runs at 1000hz Must finish in less than 1ms otherwise controller will freeze.
	void processUpdate(const MpuUpdate& update) {
		imu_.compute(update);
		State current_state = state_.update();

		switch (current_state) {
		case State::Stopped:
			motor1_.reset();
			motor2_.reset();

			status_led_.setState(0);
			beeper_.setState(0);
			break;

		case State::FirstIteration:
			motor1_.reset();
			motor2_.reset();

			fwd_lpf_.reset();

			pitch_balancer_.reset();
			status_led_.setState(1);
			// intentional fall through
		case State::Starting:
		case State::Running:

			// float fwdTargetAngle = mapRcInput(rxVals[1]) * 5;
			// float rightTargetAngle = mapRcInput(rxVals[0]) * 5;
			
			// //float yaw = yaw_pid_controler_.compute(update.gyro[2])  * state_.start_progress();
			// float yaw = mapRcInput(rxVals[3]) * 1500;

			float fwdTargetAngle = 0;


			if (current_state == State::Starting){
				fwd = pitch_balancer_.computeStarting(imu_.angles[1] - fwdTargetAngle, update.gyro[1], state_.start_progress());;
			}
			else {
				fwd = pitch_balancer_.compute(imu_.angles[1] - fwdTargetAngle, update.gyro[1]);
			}

			fwd *= settings_->balance_settings.max_current;



			motor1_.set(fwd);
			motor2_.set(fwd);

			break;
		}
	}


public:
	float fwd;
	float right;

	Config* settings_;
	IMU& imu_;
	StateTracker state_;
	BalanceController pitch_balancer_;


	StepperOut out[2] = {0, 1};
	ConstrainedOut motor1_;
	ConstrainedOut motor2_;
	GenericOut& status_led_;
	GenericOut& beeper_;

	GenericOut& green_led_;

	// These lpfs compensate for body inertia.
	BiQuadLpf fwd_lpf_;

	VescComm* vesc_;
	int vesc_update_cycle_ctr_ = 0;
};
