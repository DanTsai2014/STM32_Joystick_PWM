#include "FreeRTOS.h"
#include "PWM.h"
#include "timers.h"
#include "EPW_behavior.h"

void init_Wheels() {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //portA enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); // portC enable

    GPIO_InitStructure.GPIO_Pin = Wheels | EnableWheels;  // PA8,9,10,11 | PC6,7
    //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  //alternate function
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //push/pull configuration
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //  pull up resistor
   
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);   

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1); //PA8(TIM1_Ch1)
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1); //PA9(TIM1_CH2)
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1); //PA10(TIM1_CH3)
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1); //PA11(TIM1_CH4)

     GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3); //PC6(TIM3_CH1)
     GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); //PC7(TIM3_Ch2)
}

void reset_Wheels() {
    TIM_SetCompare1(TIM1, 0);
    TIM_SetCompare2(TIM1, 0);
    TIM_SetCompare3(TIM1, 0);
    TIM_SetCompare4(TIM1, 0);
    TIM_SetCompare1(TIM3, 0);
    TIM_SetCompare2(TIM3, 0);
}

void init_Timer() {
    /* TIM5.TIM3 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //see stm32f4xx_tim.c, TIM1 must use APB2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //see stm32f4xx_tim.c, TIM3 must use APB1
    /* Compute the prescaler value */
    u32 PrescalerValue = 250 - 1; //SystemCoreClock(SYSCLK) = 168000000Hz (see in system_stm32f4xx.c)

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 256 - 1; //週期period(TIM_ARR)//the more the voltage on LED less
    TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
}

void init_PWM() {
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

//---------set pwm channel in timer4-------------
    /* PWM1 Mode configuration: Channel1 (GPIOA Pin 8)*/
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //channel 1
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel4 (GPIOA Pin 9)*/
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //channel 2
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    
    /* PWM1 Mode configuration: Channel3 (GPIOA Pin 10)*/
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);  //channel 3
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    
    /* PWM1 Mode configuration: Channel1 (GPIOA Pin 11)*/
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //channel 4
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
//---------------------------

    TIM_Cmd(TIM1, ENABLE);


//---------set pwm channel in timer3-------------
    /* PWM1 Mode configuration: Channel1 (GPIOD Pin 6)*/
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);  //channel 1
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

    /* PWM1 Mode configuration: Channel2 (GPIOD Pin 7)*/
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);  //channel 2
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
//---------------------------

    TIM_Cmd(TIM3, ENABLE);
}