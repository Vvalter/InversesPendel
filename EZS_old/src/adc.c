#include <adc.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

void clock_setup()
{
        rcc_clock_setup_hse_3v3(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
        /* Enable GPIOD clock for LED & USARTs. */
        //rcc_periph_clock_enable(RCC_GPIOD);
        //rcc_periph_clock_enable(RCC_GPIOA);

        /* Enable clocks for USART2 and dac */
        //rcc_periph_clock_enable(RCC_USART2);
        //rcc_periph_clock_enable(RCC_DAC);

        /* And ADC*/
        rcc_periph_clock_enable(RCC_ADC1);
}

void adc_setup()
{
        //gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
        //gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
        gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);


        adc_power_off(ADC1);
        adc_disable_scan_mode(ADC1);
        adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_8CYC);
        adc_power_on(ADC1);
}

uint16_t read_adc_naiive(uint8_t channel)
{
        uint8_t channel_array[16];
        channel_array[0] = channel;
        adc_set_regular_sequence(ADC1, 1, channel_array);
        adc_start_conversion_regular(ADC1);
        while (!adc_eoc(ADC1));
        uint16_t reg16 = adc_read_regular(ADC1);
        return reg16;
}
