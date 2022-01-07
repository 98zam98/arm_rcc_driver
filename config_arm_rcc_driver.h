/******************************************************************************
 * Module: 
 * File Name: Config_arm_rcc_driver.h
 * Version: 1
 * Date: 31/12/2021
 * Description: user edits
 * Author: Zyad Ahmed Mackawy (ZAM)
 *******************************************************************************/

#ifndef __CONFIG_ARM_RCC_DRIVER_H__
#define __CONFIG_ARM_RCC_DRIVER_H__

/* user edits*/
#define SYSTEM_CLOCK_USER_CONFIG HSE_selected_as_system_clock_U1_VALUE_SYSTEM_CLOCK
/*
options:
    HSI_selected_as_system_clock_U1_VALUE_SYSTEM
    HSE_selected_as_system_clock_U1_VALUE_SYSTEM
*/

#define HSE_SOURCE_USER_CONFIG crystal_ceramic_resonator_U1_VALUE_HSE_SOURCE_SYSTEM_CLOCK
/*
options:
    crystal_ceramic_resonator_U1_VALUE_HSE_SOURCE_SYSTEM_CLOCK,
    user_external_clock_U1_VALUE_HSE_SOURCE_SYSTEM_CLOCK

*/


#define PLL_USAGE_USER_CONFIG using_PLL_u1_value_PLL_USAGE_USER_CONFIG
/*
options:
    not_used_u1_value_PLL_USAGE_USER_CONFIG,
    using_PLL_u1_value_PLL_USAGE_USER_CONFIG
*/
#define PLL_entry_clock_source_USER_CONFIG HSE_u1_value_RCC_CFGR_PLLSRC_PLL_entry_clock_source
/*
options:
    HSI_div_2_u1_value_RCC_CFGR_PLLSRC_PLL_entry_clock_source,
    HSE_u1_value_RCC_CFGR_PLLSRC_PLL_entry_clock_source
*/

#define HSE_divider_for_PLL_entry_USER_CONFIG HSE_divided_2_u1_value_RCC_CFGR_PLLXTPRE_HSE_divider_for_PLL_entry
// divide HSE before PLL entry
/*
options:
    HSE_not_divided_u1_value_RCC_CFGR_PLLXTPRE_HSE_divider_for_PLL_entry,
    HSE_divided_2_u1_value_RCC_CFGR_PLLXTPRE_HSE_divider_for_PLL_entry
*/

#define PLL_multiplication_factor_USER_CONFIG x2_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor
// The PLL output frequency must not exceed 72 MHz
/*
options:
    x2_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x3_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x4_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x5_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x6_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x7_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x8_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x9_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x10_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x11_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x12_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x13_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x14_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x15_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
    x16_PLL_input_clock_RCC_CFGR_PLLMUL_PLL_multiplication_factor,
*/

#define USB_USAGE_USER_CONFIG not_used_u1_value_USB_USAGE_USER_CONFIG
/*
options:
    not_used_u1_value_USB_USAGE_USER_CONFIG,
    using_usb_u1_value_USB_USAGE_USER_CONFIG
*/
#define USB_PLL_USER_CONFIG PLL_not_divided_U1_VALUE_USB_prescaler_RCC_CFGR
// The PLL output frequency must not exceed 48 MHz
/*
options:
    PLL_divided_1_5_U1_VALUE_USB_prescaler_RCC_CFGR,
    PLL_not_divided_U1_VALUE_USB_prescaler_RCC_CFGR
*/

//--------------------------------------

#endif /* __CONFIG_ARM_RCC_DRIVER_H__ */