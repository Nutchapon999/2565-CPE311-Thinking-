/*-------------------------------------------------------------------------------*/
/*----------------------LIBRERIA SENSOR COLOR TCS3200----------------------------*/
/*------------------------Proyecto Sistemas Embebidos----------------------------*/
/*-----------------Diseñada y programada por Jaime Laborda-----------------------*/
/*------------------------------Diciembre de 2016--------------------------------*/
/*----------------------------------tcs3200.c------------------------------------*/
/*-------------------------------------------------------------------------------*/

#include <stdbool.h>
#include "stm32l1xx.h"
#include "tcs3200.h"
#include "stm32l1xx_ll_gpio.h"
#include "stm32l1xx_ll_tim.h"
#include "stm32l1xx_ll_bus.h"

#include "stm32l1xx_ll_utils.h"

bool IC_ColorMode = false;
uint8_t calibrate_number;

uint16_t TimeColor_H=0, TimeColor_L=0;
uint16_t TimeColor;
uint16_t FreqColor = 0;

void Captura_TCS3200_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStructure;
	LL_TIM_InitTypeDef TIM_TimeBaseInitStructure;
	LL_TIM_IC_InitTypeDef TIM_ICInitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
	
	/* GPIOB clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	/*-------------------------- GPIO Configuration ----------------------------*/
	/* GPIOB Configuration: PB0 as input for capture */
	GPIO_InitStructure.Pin = LL_GPIO_PIN_3;
	GPIO_InitStructure.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;

	GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStructure.Alternate = LL_GPIO_AF_3;
	LL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Connect TIM4 pins to AF2 */
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3); //TIM3 CC3 -> PB0

	/*Activo Clock para el periferico del timer*/
  	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM9);
	// RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	/*Configuro la base de tiempos del timer*/
	TIM_TimeBaseInitStructure.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_TimeBaseInitStructure.Autoreload = 0xFFFF;
  TIM_TimeBaseInitStructure.Prescaler = 32-1; //Resolución de 0.001ms = 1	us
	// TIM_TimeBaseInitStructure.ClockDivision = 0;
	
	LL_TIM_Init(TIM9, &TIM_TimeBaseInitStructure);
	
	TIM_ICInitStructure.ICActiveInput = LL_TIM_ACTIVEINPUT_DIRECTTI;   
	TIM_ICInitStructure.ICFilter = LL_TIM_IC_FILTER_FDIV1_N2;   

	//CHANNEL 3 -> SUBIDA
	//TIM_ICInitStructure.Channel = TIM_Channel_3;
	TIM_ICInitStructure.ICPolarity = LL_TIM_IC_POLARITY_RISING;
	//TIM_ICInitStructure.ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.ICPrescaler = LL_TIM_ICPSC_DIV2;
	//TIM_ICInitStructure.ICFilter = 5;
   TIM_ICInitStructure.ICFilter = LL_TIM_IC_FILTER_FDIV1_N2;   
	
	LL_TIM_IC_Init(TIM9,LL_TIM_CHANNEL_CH2,&TIM_ICInitStructure);
	
	LL_TIM_EnableIT_CC2(TIM9);
 
	//Configuro interrupcion en el TIM3 CC3
	//TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE);
	
	NVIC_SetPriority(TIM9_IRQn, 0);	
	NVIC_EnableIRQ(TIM9_IRQn);
	 LL_TIM_CC_EnableChannel(TIM9, LL_TIM_CHANNEL_CH2);
	//LL_TIM_EnableIT_CC2(TIM3);
	LL_TIM_EnableCounter(TIM9);
	/*
	TIM_Cmd(TIM3, ENABLE);	
	
	//Configurar interrupción del Channel 4 (BAJADA) del TIM3 -> NVIC
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	
	NVIC_Init(&NVIC_InitStructure);*/
}

void TCS3200_Config(void)
{
	//Configuración de los pines de entrada y salida para configurar el filtro y el preescaler
	//GPIOB
	//S0 -> GPIO_Pin_1 10
	//S1 -> GPIO_Pin_2 11
	//S2 -> GPIO_Pin_3 12
	//S3 -> GPIO_Pin_4 13
	
	LL_GPIO_InitTypeDef GPIO_InitStructure;
	
	/* GPIOB clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
	/*-------------------------- GPIO Configuration ----------------------------*/
	/* GPIOB Configuration: PB0 como entrada para captura */
	GPIO_InitStructure.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11 | LL_GPIO_PIN_12 | LL_GPIO_PIN_13;
	GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStructure.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStructure.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	LL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Set_Filter (uint8_t mode) //Mode es de tipo enum Filtro
{
	switch (mode){
		case(Red):
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12 | LL_GPIO_PIN_13);
			break;
		case(Blue):
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12);
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_13);
			break;
		case(Clear):
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_13);
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);
			break;
		case(Green):
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12 | LL_GPIO_PIN_13);
			break;
	}
}

void Set_Scaling (uint8_t mode) //Mode es de tipo enum Filtro
{
	switch (mode){
		case(Scl0):
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10 | LL_GPIO_PIN_11);
			break;
		case(Scl2):
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_10);
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11);
			break;
		case(Scl20):
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11);
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10);
			break;
		case(Scl100):
			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11 | LL_GPIO_PIN_11);
			break;
	}
}
	
int Output_Color;

int GetColor(int set_color) //Funcion que Devuelve RGB de color Rojo (0-255)
{
	//char Output_Color;
	
	calibrate_number=0;

	TimeColor_H=0;
	TimeColor_L=0;
	TimeColor=0;
	
	LL_TIM_EnableIT_CC2(TIM9);
	//TIM_ITConfig(TIM3, TIM_IT_CC3, ENABLE); //Enable interruption
	
	Set_Filter(set_color); //Set filter to Color
	//DelayMillis(100);
	LL_mDelay(100);
	IC_ColorMode = true;
	
	while(IC_ColorMode == true); //Wait until value is get on the interrupt routine
	
	Set_Filter(Clear); //Set filter to default
	
	
	//TIM_ITConfig(TIM3, TIM_IT_CC3, DISABLE); //Disable interruption
	LL_TIM_DisableIT_CC2(TIM9);
	
	/*Timing calculation -> Get Color Value*/
	if(TimeColor_H > TimeColor_L) //Avoid overflow
		TimeColor = TimeColor_H - TimeColor_L;
	else
		TimeColor = 0xFFFF - TimeColor_L + TimeColor_H;
	
	
	//FreqColor = SystemCoreClock/(TimeColor*84); //Frequency conversion by means of SystemCoreClock 
	FreqColor= 10000*SystemCoreClock/(TimeColor*84); //168
		  
	//Freq to Color -> Depending of the filter
	switch (set_color){
		case Red:
			Output_Color = 50*(255.0/(16400.0-500.0))*(FreqColor-500.0); //MAPEAR FUNCIÓN
			break;
		
		case Green:
			Output_Color = 50*(255.0/(11000.0-700.0))*(FreqColor-700.0);  //MAPEAR FUNCIÓN
			break;
		
		case Blue: 
			Output_Color = 50*(255.0/(10000.0-600.0))*(FreqColor-600.0);  //MAPEAR FUNCIÓN
			break;
	}
	
	//Constrain Value to MaxRange
	if (Output_Color > 255) Output_Color = 255;
	if (Output_Color < 0) Output_Color = 0;
	
	return Output_Color	; //Mapeo y retorno valor
}

void TIM9_IRQHandler(void)
{
	//Manejador de interrupción del TIM3 
	if(LL_TIM_IsActiveFlag_CC2(TIM9) == SET)
	{	
		LL_TIM_ClearFlag_CC2(TIM9);
		if((IC_ColorMode == true) && (calibrate_number == 0))
		{
			TimeColor_L = LL_TIM_IC_GetCaptureCH2(TIM9);
			//TimeColor_L = TIM_GetCapture3(TIM3);
			calibrate_number = 1;
		}
		else if((IC_ColorMode == true) && (calibrate_number == 1))
		{
			TimeColor_H = LL_TIM_IC_GetCaptureCH2(TIM9);
			//TimeColor_H = TIM_GetCapture3(TIM3);
			IC_ColorMode = false;
			calibrate_number = 0;
		}
	
	/*if (TIM_GetITStatus(TIM3,TIM_IT_CC3)!= RESET){
		//Clear flag is the first statement
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC3);
		
		if((IC_ColorMode == true) && (calibrate_number == 0))
		{
			TimeColor_L = TIM_GetCapture3(TIM3);
			calibrate_number = 1;
		}
		else if((IC_ColorMode == true) && (calibrate_number == 1))
		{
			TimeColor_H = TIM_GetCapture3(TIM3);
			IC_ColorMode = false;
			calibrate_number = 0;
		}*/
	}
	
}
