#include "adControl.h"

adControl::adControl(uint8_t ADC_CHANNEL_SELECT, rcc_periph_clken RCC_GPSelect, 
                     uint32_t GPIO_PortSelect, uint32_t GPIO_PinSelect)
{
    RCC_GPIO_PORT_SELECT = RCC_GPSelect, GPIO_PORT_SELECT = GPIO_PortSelect;
    GPIO_PIN_SELECT = GPIO_PIN_SELECT;

    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOA); 
    rcc_periph_clock_enable(RCC_ADC1);
    gpio_set_mode(GPIO_PORT_SELECT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO_PIN_SELECT);

    rcc_peripheral_enable_clock(&RCC_APB2ENR,RCC_APB2ENR_ADC1EN);
    adc_power_off(ADC1);
    rcc_peripheral_reset(&RCC_APB2RSTR,RCC_APB2RSTR_ADC1RST);
    rcc_peripheral_clear_reset(&RCC_APB2RSTR,RCC_APB2RSTR_ADC1RST);
    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);
    adc_set_dual_mode(ADC_CR1_DUALMOD_IND);
    adc_disable_scan_mode(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_single_conversion_mode(ADC1);

    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC);
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate_async(ADC1);
    while(adc_is_calibrating(ADC1));
    
}

uint16_t adControl::adc_read()
{
    adc_set_regular_sequence(ADC1, 1, &ADC_CHANNEL_SELECT);
    adc_start_conversion_regular(ADC1);
    while(!adc_eoc(ADC1));

    return adc_read_regular(ADC1);    
}