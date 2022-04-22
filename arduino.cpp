#include <stdint.h>

#include "cmsis_boot/stm32f10x.h"
#include "stm_lib/inc/stm32f10x_tim.h"
#include "stm_lib/inc/stm32f10x_rcc.h"


#define MILLIS_TIMER TIM2
#define MILLIS_TIMER_PERIPH RCC_APB1Periph_TIM2

void initMicrosTimer() {
	/***************** TIM1 ****************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseInitTypeDef TimerBaseInit;
	TIM_TimeBaseStructInit(&TimerBaseInit);

	TimerBaseInit.TIM_Prescaler =  SystemCoreClock / 1000000 - 1; // 1us tick ;
	TimerBaseInit.TIM_Period = 0xFFFF;
	TimerBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerBaseInit.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1,&TimerBaseInit);
	TIM_Cmd(TIM1, ENABLE);
}

void initHalfMillisTimer() {
	RCC_APB1PeriphClockCmd(MILLIS_TIMER_PERIPH, ENABLE);

	TIM_TimeBaseInitTypeDef TimerBaseInit;
	TIM_TimeBaseStructInit(&TimerBaseInit);

	TimerBaseInit.TIM_Prescaler =  SystemCoreClock / 2 / 1000 - 1; // 1ms tick ;
	TimerBaseInit.TIM_Period = 0xFFFF;
	TimerBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerBaseInit.TIM_ClockDivision = TIM_CKD_DIV2;
	TIM_TimeBaseInit(MILLIS_TIMER,&TimerBaseInit);

	TIM_Cmd(MILLIS_TIMER, ENABLE);
}

void initArduino() {
	initMicrosTimer();
	initHalfMillisTimer();
}

uint16_t millis() {
	return MILLIS_TIMER->CNT;
}

uint16_t micros() {
	return TIM1->CNT;
}

void delay(uint16_t time) {
	uint16_t start_time = millis();

	while ((uint16_t)(millis() - start_time) < time);
}
