#pragma once
#include <stdint.h>
#include "global.h"

class PwmOut {
public:
	PwmOut(uint16_t idx) : idx_(idx) {}

	void set(uint16_t val);
	uint16_t get();

	static void InitAll();

private:
	DISALLOW_COPY_AND_ASSIGN(PwmOut);

	uint16_t idx_;
};
