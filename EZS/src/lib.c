#include "lib.h"

void initRCC(void) {
        rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
	// It might be neccessary to wait some cycles after enabling the rcc
	// See http://www.st.com/content/ccc/resource/technical/document/errata_sheet/81/45/af/ac/2b/e1/4d/07/DM00137034.pdf/files/DM00137034.pdf/jcr:content/translations/en.DM00137034.pdf
	// "Delay after an RCC peripheral clock enabling"
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_TIM1);
	rcc_periph_clock_enable(RCC_TIM2);
	rcc_periph_clock_enable(RCC_TIM3);
	rcc_periph_clock_enable(RCC_TIM4);
	rcc_periph_clock_enable(RCC_TIM5);
        rcc_periph_clock_enable(RCC_ADC1);
}

int initRotaryEncoderTimer(uint32_t tim, uint32_t portA, uint16_t pinA, uint8_t afA, uint32_t portB, uint16_t pinB, uint8_t afB) {
    gpio_mode_setup(portA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, pinA);
    gpio_set_af(portA, afA, pinA);

    gpio_mode_setup(portB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, pinB);
    gpio_set_af(portB, afB, pinB);

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
    timer_set_period(tim, 0xFFFFFFFF);

    timer_enable_counter(tim);

    return 0;
}

int initTimer(uint32_t tim) {
	timer_set_mode(tim, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	timer_continuous_mode(tim);
	timer_set_period(tim, 0xFFFFFFFF);
	timer_set_prescaler(tim, 0);
	timer_enable_preload(tim);

	timer_enable_counter(tim);

        return 0;
}

void initPinOutput(uint32_t port, uint16_t pin) {
    gpio_mode_setup(port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin);
}

void setPinHigh(uint32_t port, uint16_t pin) {
    gpio_set(port, pin);
}
void setPinLow(uint32_t port, uint16_t pin) {
    gpio_clear(port, pin);
}

/*
 * Possible pins:
 * 	Pin	| Channel
 * 	A0 	| 0
 * 	A1	| 1
 * 	A2	| 2
 * 	A3	| 3
 * 	A4	| 4
 * 	A5	| 5
 * 	A6	| 6
 * 	A7	| 7
 *
 * 	B0	| 8
 * 	B1	| 9
 *
 * 	C0	| 10
 * 	C1	| 11
 * 	C2	| 12
 * 	C3	| 13
 * 	C4	| 14
 * 	C5	| 15
 */

static uint8_t channel;

void initADC(uint32_t port, uint16_t pin)
{
        gpio_mode_setup(port, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, pin);

	switch (port) {
		case GPIOA:
			switch(pin) {
				case GPIO0: channel = 0; break;
				case GPIO1: channel = 1; break;
				case GPIO2: channel = 2; break;
				case GPIO3: channel = 3; break;
				case GPIO4: channel = 4; break;
				case GPIO5: channel = 5; break;
				case GPIO6: channel = 6; break;
				case GPIO7: channel = 7; break;
			}
			break;
		case GPIOB:
			switch(pin) {
				case GPIO0: channel = 8; break;
				case GPIO1: channel = 9; break;
			}
			break;
		case GPIOC:
			switch(pin) {
				case GPIO0: channel = 10; break;
				case GPIO1: channel = 11; break;
				case GPIO2: channel = 12; break;
				case GPIO3: channel = 13; break;
				case GPIO4: channel = 14; break;
				case GPIO5: channel = 15; break;
			}
			break;
	}

        adc_power_off(ADC1);
        adc_disable_scan_mode(ADC1);
        adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_15CYC);
        adc_power_on(ADC1);

        uint8_t channel_array[16];
        channel_array[0] = channel;
        adc_set_regular_sequence(ADC1, 1, channel_array);
}

uint16_t readADCBlocking()
{
        adc_start_conversion_regular(ADC1);
        while (!adc_eoc(ADC1));
        uint16_t reg16 = adc_read_regular(ADC1);
        return reg16;
}
