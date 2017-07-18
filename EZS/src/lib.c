#include "lib.h"

int initRotaryEncoderTimer(uint32_t tim, uint32_t portA, uint16_t pinA, uint8_t afA, uint32_t portB, uint16_t pinB, uint8_t afB) {

    enum rcc_periph_clken rccTim;
    switch (tim) {
	case TIM2: rccTim = RCC_TIM2; break;
	case TIM3: rccTim = RCC_TIM3; break;
	case TIM4: rccTim = RCC_TIM4; break;
	case TIM5: rccTim = RCC_TIM5; break;
	default: return 1;
    }
    enum rcc_periph_clken rccA;
    switch (portA) {
	case GPIOA: rccA = RCC_GPIOA; break;
	case GPIOB: rccA = RCC_GPIOB; break;
	case GPIOC: rccA = RCC_GPIOC; break;
	case GPIOD: rccA = RCC_GPIOD; break;
	default: return 2;
    }
    enum rcc_periph_clken rccB;
    switch (portB) {
	case GPIOA: rccB = RCC_GPIOA; break;
	case GPIOB: rccB = RCC_GPIOB; break;
	case GPIOC: rccB = RCC_GPIOC; break;
	case GPIOD: rccB = RCC_GPIOD; break;
	default: return 2;
    }

    gpio_mode_setup(portA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, pinA);
    gpio_set_af(portA, afA, pinA);

    gpio_mode_setup(portB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, pinB);
    gpio_set_af(portB, afB, pinB);

    rcc_periph_clock_enable(rccA);
    rcc_periph_clock_enable(rccB);
    rcc_periph_clock_enable(rccTim);

    timer_reset(tim);

    timer_set_mode(tim, TIM_CR1_CKD_CK_INT, //For dead time and filter sampling, not important for now.
	    TIM_CR1_CMS_EDGE,	//TIM_CR1_CMS_EDGE
	    //TIM_CR1_CMS_CENTER_1
	    //TIM_CR1_CMS_CENTER_2
	    //TIM_CR1_CMS_CENTER_3 la frequencia del pwm se divide a la mitad.
	    TIM_CR1_DIR_UP);

    timer_set_prescaler(tim, 0);
    timer_set_repetition_counter(tim, 0);
    timer_enable_preload(tim);
    timer_continuous_mode(tim);

    timer_slave_set_mode(tim, TIM_SMCR_SMS_EM3); // encoder

    timer_set_oc_polarity_high(tim, TIM_OC1);
    timer_set_oc_polarity_high(tim, TIM_OC2);

    timer_ic_disable(tim, TIM_IC1);
    timer_ic_disable(tim, TIM_IC2);

    timer_ic_set_input(tim, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_input(tim, TIM_IC2, TIM_IC_IN_TI1);

    timer_disable_oc_output(tim, TIM_OC1);
    timer_disable_oc_output(tim, TIM_OC2);

    timer_disable_preload_complementry_enable_bits(tim);

    // TODOa adjust for 16 bit timer
    timer_set_period(tim, 0xFFFFFFFF);

    timer_enable_counter(tim);

    return 0;
}
