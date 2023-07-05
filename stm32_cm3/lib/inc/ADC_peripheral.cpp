#include "ADC_peripheral.h"

ADC_peripheral::ADC_peripheral(uint32_t ADC_PSelect, rcc_periph_clken RCC_ADSelect,
                                rcc_periph_clken RCC_GPSelect, uint32_t GPIO_PortSelect)
{
    RCC_GPIO_PORT_SELECT = RCC_GPSelect, RCC_ADC_SELECT = RCC_ADSelect;
    GPIO_PORT_SELECT = GPIO_PortSelect, ADC_PERIPHERAL_SELECT = ADC_PSelect;


    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIO_PORT_SELECT); 
    rcc_periph_clock_enable(RCC_GPIO_PORT_SELECT);

    if(ADC_PERIPHERAL_SELECT == ADC1)
    {
        rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC1EN);
        adc_power_off(ADC_PERIPHERAL_SELECT);
        rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC1RST);
        rcc_peripheral_clear_reset(&RCC_APB2RSTR,RCC_APB2RSTR_ADC1RST);

    }
    else if(ADC_PERIPHERAL_SELECT == ADC2)
    {
        rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_ADC2EN);
        adc_power_off(ADC_PERIPHERAL_SELECT);
        rcc_peripheral_reset(&RCC_APB2RSTR, RCC_APB2RSTR_ADC2RST);
        rcc_peripheral_clear_reset(&RCC_APB2RSTR,RCC_APB2RSTR_ADC2RST);
    }

    rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);
    adc_set_dual_mode(ADC_CR1_DUALMOD_IND);
    adc_disable_scan_mode(ADC_PERIPHERAL_SELECT);
    adc_set_right_aligned(ADC_PERIPHERAL_SELECT);
    adc_set_single_conversion_mode(ADC_PERIPHERAL_SELECT);

    adc_set_sample_time_on_all_channels(ADC_PERIPHERAL_SELECT, ADC_SMPR_SMP_239DOT5CYC);
    adc_power_on(ADC_PERIPHERAL_SELECT);
    adc_reset_calibration(ADC_PERIPHERAL_SELECT);
    adc_calibrate_async(ADC_PERIPHERAL_SELECT);
    while(adc_is_calibrating(ADC_PERIPHERAL_SELECT));
    
}

void ADC_peripheral::gpioSetup(uint16_t GPIO_PIN_SELECT)
{
    gpio_set_mode(GPIO_PORT_SELECT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO_PIN_SELECT);
}

uint16_t ADC_peripheral::adc_read(uint8_t ADC_CHANNEL_SELECT)
{
    adc_set_regular_sequence(ADC_PERIPHERAL_SELECT, 1, &ADC_CHANNEL_SELECT);
    adc_start_conversion_direct(ADC_PERIPHERAL_SELECT);
    while(!adc_eoc(ADC_PERIPHERAL_SELECT));

    return adc_read_regular(ADC_PERIPHERAL_SELECT);    
}