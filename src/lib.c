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

void initPinInput(uint32_t port, uint16_t pin, uint8_t pull) {
    gpio_mode_setup(port, GPIO_MODE_INPUT, pull, pin);
}

bool readPin(uint32_t port, uint16_t pin) {
    return gpio_get(port, pin);
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

void pwm_init(uint32_t timer, uint8_t channel, uint32_t period, uint32_t port, uint16_t pin, uint16_t af) {
// function stolen from somewhere on stackoverflow

  // Convert channel number to internal rep
  enum tim_oc_id chan;
  switch (channel) {
    case 1:   chan = TIM_OC1; break;
    case 2:   chan = TIM_OC2; break;
    case 3:   chan = TIM_OC3; break;
    case 4:   chan = TIM_OC4; break;
    default:  chan = TIM_OC4; break;
  }

  // Timer Base Configuration
  // timer_reset(timer);
  timer_set_mode(timer, TIM_CR1_CKD_CK_INT, // clock division
                        TIM_CR1_CMS_EDGE,   // Center-aligned mode selection
                        TIM_CR1_DIR_UP);    // TIMx_CR1 DIR: Direction
  timer_continuous_mode(timer);             // Disables TIM_CR1_OPM (One pulse mode)
  timer_set_period(timer, period);                    // Sets TIMx_ARR
  timer_set_prescaler(timer, 1);               // Adjusts speed of timer
  timer_set_clock_division(timer, 0);            // Adjusts speed of timer
  timer_set_master_mode(timer, TIM_CR2_MMS_UPDATE);   // Master Mode Selection
  timer_enable_preload(timer);                        // Set ARPE bit in TIMx_CR1

  // Channel-specific settings
  timer_set_oc_value(timer, chan, 0);             // sets TIMx_CCRx
   //In PWM mode 1 channel is active as long as TIMx_CNT<TIMx_CCR1, else inactive
  timer_set_oc_mode(timer, chan, TIM_OCM_PWM1);   // Sets PWM Mode 1
  timer_enable_oc_preload(timer, chan);           // Sets OCxPE in TIMx_CCMRx
  timer_set_oc_polarity_high(timer, chan);        // set desired polarity in TIMx_CCER
  timer_enable_oc_output(timer, chan);             // set CCxE bit in TIMx_CCER  (enable output)

  // Initialize all counters in the register
  switch (timer) {
    case TIM1:  TIM1_EGR |= TIM_EGR_UG; break;
    case TIM2:  TIM2_EGR |= TIM_EGR_UG; break;
    case TIM3:  TIM3_EGR |= TIM_EGR_UG; break;
    case TIM4:  TIM4_EGR |= TIM_EGR_UG; break;
    case TIM5:  TIM5_EGR |= TIM_EGR_UG; break;
    case TIM6:  TIM6_EGR |= TIM_EGR_UG; break;
    case TIM7:  TIM7_EGR |= TIM_EGR_UG; break;
    case TIM8:  TIM8_EGR |= TIM_EGR_UG; break;
    }

  gpio_mode_setup(port, GPIO_MODE_AF, GPIO_PUPD_NONE, pin);
  // AF2 = TIM4_CH1..4
  gpio_set_af(port, af, pin);
  timer_enable_counter(timer);
}

void stopMotor(void) {
        setPinLow(GPIOA, GPIO15);
}

void startMotor(void) {
        setPinHigh(GPIOA, GPIO15);
}

void driveLeft(float speed) {
        timer_set_oc_value(TIM4, TIM_OC1, PWM_PERIOD*(1.0f-speed));
        timer_set_oc_value(TIM4, TIM_OC2, PWM_PERIOD);
}
void driveLeftFast(void) {
        timer_set_oc_value(TIM4, TIM_OC1, 0);
        timer_set_oc_value(TIM4, TIM_OC2, PWM_PERIOD);
}

void breakMotor(void) {
        timer_set_oc_value(TIM4, TIM_OC1, PWM_PERIOD);
        timer_set_oc_value(TIM4, TIM_OC2, PWM_PERIOD);
}

void driveRight(float speed) {
        timer_set_oc_value(TIM4, TIM_OC1, PWM_PERIOD);
        timer_set_oc_value(TIM4, TIM_OC2, PWM_PERIOD*(1.0f-speed));
}
void driveRightFast(void) {
        timer_set_oc_value(TIM4, TIM_OC1, PWM_PERIOD);
        timer_set_oc_value(TIM4, TIM_OC2, 0);
}

void initHardware(void) {
        /**
         * Initialize hardware components
         * TIM2 is position of waggon (32bit)
         *      pins
         *              A1 GREEN
         *              A5 WHITE
         *      39393
         *      39253
         *      39260
         * TIM3 is position of pendulum (16bit)
         *      pins
         *              C6 WHITE
         *              C7 GREEN
         *
         * TIM4 is pwm
         *      Channel 1 is IN 1
         *              pins B6
         *      Channel 2 is IN 2
         *              pins B7
         *
         * EN A15
         *
         * TIM5 is time (32bit)
         *
         * Taster links D1
         * Taster rechts D2
         *
         * ADC B0
         */
        initRCC();
        // Position Waggon
        initRotaryEncoderTimer(TIM2, GPIOA, GPIO5, GPIO_AF1, GPIOA, GPIO1, GPIO_AF1);
        // Position Pendulum
        initRotaryEncoderTimer(TIM3, GPIOC, GPIO6, GPIO_AF2, GPIOC, GPIO7, GPIO_AF2);
        timer_disable_counter(TIM3);
        timer_set_period(TIM3, 2399);
        timer_enable_counter(TIM3);
        // IN1
        pwm_init(TIM4, 1, PWM_PERIOD, GPIOB, GPIO6, GPIO_AF2);
        // IN2
        pwm_init(TIM4, 2, PWM_PERIOD, GPIOB, GPIO7, GPIO_AF2);
        // Time
        initTimer(TIM5);
        // EN
        initPinOutput(GPIOA, GPIO15);
        // Motor
        startMotor();
        // Taster links
        initPinInput(GPIOD, GPIO1, GPIO_PUPD_PULLUP);
        // Taster rechts
        initPinInput(GPIOD, GPIO2, GPIO_PUPD_PULLUP);
        // ADC
        initADC(GPIOB, GPIO0);
}

int convertPendulumAngleToUpper(int16_t positionPendulum) {
        return 1200 - positionPendulum;
}
int convertPendulumAngleToLower(int16_t positionPendulum) {
        if (positionPendulum > 1200) {
                return positionPendulum - 2400;
        } else {
                return positionPendulum;
        }
}
double fabs(double a) {
        if (a < 0) return -a;
        else return a;
}
double fmin(double a, double b) {
        if (a < b) return a;
        else return b;
}
