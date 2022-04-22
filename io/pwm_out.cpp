#include "pwm_out.hpp"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

void PwmOut::InitAll() {
	/***************** TIM1 ****************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseInitTypeDef TimerBaseInit;
	TIM_TimeBaseStructInit(&TimerBaseInit);

	TimerBaseInit.TIM_Prescaler =  SystemCoreClock / 1000000 - 1; // 1us tick ;
	TimerBaseInit.TIM_Period = 2500; // 400hz
	TimerBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerBaseInit.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1,&TimerBaseInit);
	TIM_Cmd(TIM1, ENABLE);

	TIM_OCInitTypeDef OC_Config;
	TIM_OCStructInit(&OC_Config);
	OC_Config.TIM_OCMode = TIM_OCMode_PWM1;
	OC_Config.TIM_Pulse = 1500;
	OC_Config.TIM_OutputState = TIM_OutputState_Enable;
	OC_Config.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM1, &OC_Config);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OC4Init(TIM1, &OC_Config);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);


	/**************** TIM4 ****************/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	TIM_TimeBaseStructInit(&TimerBaseInit);
	TimerBaseInit.TIM_Prescaler =  SystemCoreClock / 1000000 - 1; // 1us tick ;
	TimerBaseInit.TIM_Period = 2500; // 400hz
	TimerBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerBaseInit.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4,&TimerBaseInit);
	TIM_Cmd(TIM4, ENABLE);

	TIM_OCStructInit(&OC_Config);
	OC_Config.TIM_OCMode = TIM_OCMode_PWM1;
	OC_Config.TIM_Pulse = 1500;
	OC_Config.TIM_OutputState = TIM_OutputState_Enable;
	OC_Config.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM4, &OC_Config);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OC2Init(TIM4, &OC_Config);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

void PwmOut::set(uint16_t val) {
	switch (idx_) {
	case 1:
		TIM1->CCR1 = val;
		break;
	case 2:
		TIM1->CCR4 = val;
		break;
	case 3:
		TIM4->CCR1 = val;
		break;
	default: while(1);
	}
}

uint16_t PwmOut::get() {
	switch (idx_) {
	case 1:
		return TIM1->CCR1;
		break;
	case 2:
		return TIM1->CCR4;
		break;
	case 3:
		return TIM4->CCR1;
		break;
	default: while(1);
	}
}
