/**
 * @file ADC_peripheral.cpp
 * @author Isaac Lima (isaac.lima.sousa61@aluno.ifce.edu.br)
 * @brief Source file for the ADC peripheral class drive.
 * @version 0.1
 * @date 2023-07-02
 * 
 */

#include "ADC_peripheral.hpp"

/**
 * @brief ADC_peripheral constructor
 * @param ADC_P_Select Selects which ADC PORT will be used
 * @param RCC_ADSelect Selects which ADC peripheral will be activated
 * @param RCC_GPSelect Selects which GPIO PORT will be activated
 * @param GPIO_PortSelect Selects which GPIO PORT will be used
 */

ADC_peripheral::ADC_peripheral(uint32_t ADC_PSelect, rcc_periph_clken RCC_ADSelect,
                                rcc_periph_clken RCC_GPSelect, uint32_t GPIO_PortSelect, uint8_t continuous)
{
    RCC_GPIO_PORT_SELECT = RCC_GPSelect, RCC_ADC_SELECT = RCC_ADSelect;
    GPIO_PORT_SELECT = GPIO_PortSelect, ADC_PERIPHERAL_SELECT = ADC_PSelect;

    rcc_periph_clock_enable(RCC_GPIO_PORT_SELECT);
    rcc_periph_clock_enable(RCC_ADC_SELECT);
    rcc_peripheral_enable_clock(&RCC_APB2ENR,RCC_APB2ENR_ADC1EN);


    adc_power_off(ADC_PERIPHERAL_SELECT);
    rcc_periph_reset_pulse(RST_ADC);
    rcc_set_rtcpre(RCC_CFGR_MCOPRE_DIV_2);
    adc_disable_scan_mode(ADC_PERIPHERAL_SELECT);
    adc_set_right_aligned(ADC_PERIPHERAL_SELECT);
    
    if(continuous)
    {
        adc_set_continuous_conversion_mode(ADC_PERIPHERAL_SELECT);
    }
    else
    {
        adc_set_single_conversion_mode(ADC_PERIPHERAL_SELECT);
    }
    
    adc_set_sample_time_on_all_channels(ADC_PERIPHERAL_SELECT, ADC_SMPR_SMP_56CYC);
    adc_power_on(ADC_PERIPHERAL_SELECT);
    // Enable end of conversion interrupt
    adc_enable_eoc_interrupt(ADC_PERIPHERAL_SELECT);
    adc_enable_dma(ADC1);    
}

void ADC_peripheral::ADC_initialization(uint32_t ADC_PSelect, rcc_periph_clken RCC_ADSelect,
                                rcc_periph_clken RCC_GPSelect, uint32_t GPIO_PortSelect, uint8_t continuous)
{
    RCC_GPIO_PORT_SELECT = RCC_GPSelect, RCC_ADC_SELECT = RCC_ADSelect;
    GPIO_PORT_SELECT = GPIO_PortSelect, ADC_PERIPHERAL_SELECT = ADC_PSelect;

    rcc_periph_clock_enable(RCC_GPIO_PORT_SELECT);
    rcc_periph_clock_enable(RCC_ADC_SELECT);
    rcc_peripheral_enable_clock(&RCC_APB2ENR,RCC_APB2ENR_ADC1EN);

    adc_power_off(ADC_PERIPHERAL_SELECT);
    rcc_periph_reset_pulse(RST_ADC);
    rcc_set_rtcpre(RCC_CFGR_MCOPRE_DIV_2);
    adc_disable_scan_mode(ADC_PERIPHERAL_SELECT);
    adc_set_right_aligned(ADC_PERIPHERAL_SELECT);
    
    
    if(continuous)
    {
        adc_set_continuous_conversion_mode(ADC_PERIPHERAL_SELECT);
    }
    else
    {
        adc_set_single_conversion_mode(ADC_PERIPHERAL_SELECT);
    }

    uint8_t channels[] = {ADC_CHANNEL0, ADC_CHANNEL1};
    adc_enable_scan_mode(ADC_PERIPHERAL_SELECT);
    adc_set_regular_sequence(ADC_PERIPHERAL_SELECT, 2, channels);
    adc_set_sample_time_on_all_channels(ADC_PERIPHERAL_SELECT, ADC_SMPR_SMP_28CYC);

    // Enable end of conversion interrupt
    //adc_enable_eoc_interrupt(ADC_PERIPHERAL_SELECT);
    adc_enable_dma(ADC_PERIPHERAL_SELECT);
    adc_power_on(ADC_PERIPHERAL_SELECT);
    adc_set_dma_continue(ADC_PERIPHERAL_SELECT);
    adc_start_conversion_regular(ADC_PERIPHERAL_SELECT);
}

/**
 * @brief ADC_peripheral gpioSetup
 * Activates the selected pin for ADC reading.
 * @param GPIO_PIN_SELECT Selects which GPIO PIN will be used.
 */

void ADC_peripheral::gpioSetup(uint16_t GPIO_PIN_SELECT)
{
    gpio_mode_setup(GPIO_PORT_SELECT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_SELECT);
}

/**
 * @brief ADC_peripheral adc_read
 * single read value for the selected channel
 * @param ADC_CHANNEL_SELECT Selects which channel will be read.
 */

uint16_t ADC_peripheral::adc_read(uint8_t ADC_CHANNEL_SELECT)
{
    //adc_set_sample_time(ADC_PERIPHERAL_SELECT, ADC_CHANNEL_SELECT, ADC_SMPR_SMP_56CYC);
    //adc_set_regular_sequence(ADC_PERIPHERAL_SELECT, 1 ,&ADC_CHANNEL_SELECT);
    //adc_start_conversion_regular(ADC_PERIPHERAL_SELECT);
    //while(!adc_eoc(ADC_PERIPHERAL_SELECT));

    return adc_read_regular(ADC_PERIPHERAL_SELECT);    
}