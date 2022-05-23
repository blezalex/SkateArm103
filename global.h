#pragma once

#include <stdint.h>
#include <math.h>

#include "cmsis_boot/stm32f10x.h"
#include "arduino.h"

//#define REV5

//***** Apply board orientation transform *** //
// IMU is using right hand coordinate system, it tracks gravity vector and calculates angle between gravity vector and the board frame.
// Normal orientation - X forward, Y right Z down


// ****** start mode settings *********/
#define START_MAX_POWER 300 // [-START_MAX_POWER: +START_MAX_POWER] out of MOTOR_CMD_RANGE
#define START_DURATION 1500 // in ms. Time to bring board from tilted to balanced.

#define START_ANGLE_DRIVE 12
#define START_ANGLE_DRIVE_FULL 6
#define START_D_MAX_MULTIPLIER 2

#define MIN_MOTOR_CMD 990
#define MAX_MOTOR_CMD 2010
#define NEUTRAL_MOTOR_CMD 1500

// Configure brake current on invalid pulse with 100ms timeout
#define BRAKE_MOTOR_CMD 0

#define MOTOR_CMD_RANGE (MAX_MOTOR_CMD - NEUTRAL_MOTOR_CMD)

#define ANGLE_DRIVE 1
#define ANGLE_STEER 0


#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
  TypeName(const TypeName&);   \
  void operator=(const TypeName&)


//inline int sgn(float val) { return (0 < val) - (val < 0); }
inline int sgn(float val) { return val >= 0 ? 1 : -1; } // returns 1 for 0 and up, -1 for less than zero.

inline float applyExpoReal(float x, float k) { return sgn(x) * powf(fabs(x), 1+k); }

constexpr float E =  2.71828;

inline float applyExpoNatural(float x, float k) {
	float absx = fabs(x);
	return sgn(x) * (powf(E, k*absx) - 1) / (powf(E, k) - 1) ;
}

inline float applyExpoPoly(float x, float k) {
	float absx = fabs(x);
	return sgn(x) * absx/(1+k*(1-absx));
}

constexpr inline float deg_to_rad(float angle) {
	return angle * M_PI / 180;
}
