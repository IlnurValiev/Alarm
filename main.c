#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_cortex.h"
#include "main.h"

int hour = 0;
int min = 0;
int sec = 0;

int alarm_hour = 0;
int alarm_min = 0;

int set_clock = -1; 

int counter = 0;

int counter_dp = 0;

int counter_alarm = 0;

int alarm_on = 0;
int alarm_trigger = 0;

static void rcc_config()
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1);

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                                LL_RCC_PLL_MUL_12);

    LL_RCC_PLL_Enable();
    while (LL_RCC_PLL_IsReady() != 1);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    SystemCoreClock = 48000000;
}

static void buzz()
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_3, LL_GPIO_AF_2);

    
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_TIM_SetPrescaler(TIM2, 479);
    
    LL_TIM_SetAutoReload(TIM2, 999);

    LL_TIM_OC_SetCompareCH2(TIM2, 500);
    
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetMode(TIM2, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
    LL_TIM_SetCounterMode(TIM2, LL_TIM_COUNTERMODE_UP);
    LL_TIM_EnableIT_CC2(TIM2);
    LL_TIM_EnableCounter(TIM2);
    
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 0);
    return;
}


static void gpio_config(void)
{
    
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);

    
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_DOWN);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_1, LL_GPIO_PULL_DOWN);

    return;
}

static void exti_config() {
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);

    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE1);
    
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);
    
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_1);

    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_1);

    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 0);
}

static void timers_config_set_clock(void)
{

    
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_5, LL_GPIO_AF_2);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_UP);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    LL_TIM_SetPrescaler(TIM2, 47999);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV16_N5);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1,
                          LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1,
                             LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableIT_CC1(TIM2);
    LL_TIM_EnableCounter(TIM2);
    LL_TIM_GenerateEvent_UPDATE(TIM2);
   
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 0);

    return;
}

static void systick_config() {
    LL_InitTick(48000000, 1000);
    LL_SYSTICK_EnableIT();
    NVIC_SetPriority(SysTick_IRQn, 0);
}

static void timer_alarm_config(void)
{
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_0, LL_GPIO_AF_1);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
    LL_TIM_SetPrescaler(TIM3, 47999);
    LL_TIM_IC_SetFilter(TIM3, LL_TIM_CHANNEL_CH3, LL_TIM_IC_FILTER_FDIV16_N5);
    LL_TIM_IC_SetPolarity(TIM3, LL_TIM_CHANNEL_CH3,
                          LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM3, LL_TIM_CHANNEL_CH3,
                             LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM3, LL_TIM_CHANNEL_CH3, LL_TIM_ICPSC_DIV1);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH3);
    LL_TIM_EnableIT_CC3(TIM3);
    LL_TIM_EnableCounter(TIM3);
    LL_TIM_GenerateEvent_UPDATE(TIM3);

    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 0);
    return;

}

void TIM2_IRQHandler() {

    switch(set_clock) {
    case 0:
        LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_9);
        set_clock = -1;
        break;
    case 1:
        LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
        LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_9);
        set_clock = 0;
        break;
    default:
        LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
        set_clock = 1;
        break;
    }
    LL_TIM_ClearFlag_CC1(TIM2);
    LL_TIM_ClearFlag_CC2(TIM2);
   
}

void EXTI0_1_IRQHandler() {

    static int states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

    int cur_enc = 0;
    
    static int old_new_enc = 0;

    static int state_enc = 0;

    cur_enc = (LL_GPIO_ReadInputPort(GPIOA) & 1) | (LL_GPIO_ReadInputPort(GPIOB) & 2); 
    old_new_enc = ((3 & old_new_enc) << 2) | cur_enc;
    state_enc += states[old_new_enc];

    switch(set_clock) {
    case 3:
         if(state_enc == 4) {
            alarm_hour++;
            state_enc = 0;
        }
        if(state_enc == -4) {
            if(alarm_hour < 0) {
                alarm_hour += 24;
                state_enc = 0;
            }
            else {
                alarm_hour--;
                state_enc = 0;
            }
        }
        break;
    case 2:
         if(state_enc == 4) {
            alarm_min++;
            state_enc = 0;
        }
        if(state_enc == -4) {
            if(alarm_min < 0) {
                alarm_min += 60;
                state_enc = 0;
            }
            else {
                alarm_min--;
                state_enc = 0;
            }
        }
        break;
    case 1:
        if(state_enc == 2) {
            sec += 60;
            state_enc = 0;
        }
        if(state_enc == -2) {
            if(sec < 0) {
                sec += 60*60;
                state_enc = 0;
            }
            else {
                sec -= 60;
                state_enc = 0;
            }
        }
        break;
    case 0:
        if(state_enc == 2) {
            sec += 3600;
            state_enc = 0;
        }
        if(state_enc == -2) {
            if(sec < 0) {
                sec += 3600*24;
                state_enc = 0;
            }
            else {
                sec -= 3600;
                state_enc = 0;
            }
        }
        break;
    default:
        break;
    }
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);

}

static void ind_time(int min, int hour) {
    static int dig_num = 0;

    switch (dig_num) {
    case 0:
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_2);
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_3);
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_4);
        LL_GPIO_ResetOutputPin(LED_OUT_PORT, LED_DIG_1);
        ind_dig(min%10);
        break;
    case 1:
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_1);
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_3);
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_4);
        LL_GPIO_ResetOutputPin(LED_OUT_PORT, LED_DIG_2);
        ind_dig(min/10%10);
        break;
    case 2:
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_2);
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_1);
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_4);
        LL_GPIO_ResetOutputPin(LED_OUT_PORT, LED_DIG_3);
        ind_dig(hour%10);
        break;
    case 3:
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_3);
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_2);
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_1);
        LL_GPIO_ResetOutputPin(LED_OUT_PORT, LED_DIG_4);
        ind_dig(hour/10%10);
        break;
    case 4:
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_2);
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_1);
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_4);
        LL_GPIO_ResetOutputPin(LED_OUT_PORT, LED_DIG_3);
        if(counter_dp < 1000) on_dp();
        else off_dp();
        break;
    case 5:
        if(alarm_on) {
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_3);
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_2);
        LL_GPIO_SetOutputPin(LED_OUT_PORT, LED_DIG_4);
        LL_GPIO_ResetOutputPin(LED_OUT_PORT, LED_DIG_1);
        on_dp();
        break;
        }
    default:
        break;
    }
    dig_num = (dig_num+1)%6;
}

void SysTick_Handler() {

    counter = (counter+1)%1000;
    counter_dp = (counter_dp+1)%2000;
    counter_alarm = (counter_alarm+1)%500;

    if(!counter) {
        sec++;
        min = sec/60%60;
        hour = sec/3600%24;       
    }

    if(hour == alarm_hour && min == alarm_min && alarm_on)
        alarm_trigger = 1;

    if(alarm_trigger) {
        buzz();
    }

    if(set_clock == -1 || set_clock == 0 || set_clock == 1)
        ind_time(min, hour);
    
    if(set_clock == 2 || set_clock == 3) {
        ind_time(alarm_min%60, alarm_hour%24);

        if(set_clock == 2 && !counter_alarm) {
            LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9);
            LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_8);
        }
        else if(!counter_alarm) {
            LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_8);
            LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_9);
        }
    }
}

void TIM3_IRQHandler() {
    switch(set_clock) {
    case 2:
        set_clock = 3;
        break;
    case 3:
        set_clock = 2;
        break;
    default:
        set_clock = 2;
        break;
    }

    LL_TIM_ClearFlag_CC3(TIM3);
}

static void alarm_key_config(void)
{
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_0_7(GPIOA, LL_GPIO_PIN_7, LL_GPIO_AF_4);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_7, LL_GPIO_PULL_UP);

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM14);
    LL_TIM_SetPrescaler(TIM14, 47999);
    LL_TIM_IC_SetFilter(TIM14, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV16_N5);
    LL_TIM_IC_SetPolarity(TIM14, LL_TIM_CHANNEL_CH1,
                          LL_TIM_IC_POLARITY_RISING);
    LL_TIM_IC_SetActiveInput(TIM14, LL_TIM_CHANNEL_CH1,
                             LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM14, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_CC_EnableChannel(TIM14, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableIT_CC1(TIM14);
    LL_TIM_EnableCounter(TIM14);

    NVIC_EnableIRQ(TIM14_IRQn);
    NVIC_SetPriority(TIM14_IRQn, 0);
    return;

}

void TIM14_IRQHandler() {
    alarm_on = !alarm_on;
    alarm_trigger = 0;
    timers_config_set_clock();
    LL_TIM_SetAutoReload(TIM2, 0);
    LL_TIM_ClearFlag_CC1(TIM14);
}

int main(void)
{
    exti_config();
    rcc_config();
    gpio_config();
    timers_config_set_clock();
    timer_alarm_config();
    gpio_config_leds();
    systick_config();
    alarm_key_config();

}
