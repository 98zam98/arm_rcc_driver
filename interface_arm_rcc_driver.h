/******************************************************************************
 * Module: 
 * File Name: Interface_arm_rcc_driver.h
 * Version: 1
 * Date: 31/12/2021
 * Description: function prototypes and variables and macro functions that can be called by user 
 * Author: Zyad Ahmed Mackawy (ZAM)
 *******************************************************************************/

#ifndef __INTERFACE_ARM_RCC_DRIVER_H__
#define __INTERFACE_ARM_RCC_DRIVER_H__

/* function prototypes*/

//--------------------------------------	

/* variables */

/*
Three different clock sources can be used to drive the system clock (SYSCLK):
• HSI oscillator clock
• HSE oscillator clock
• PLL clock
*/

enum U1_VALUE_SYSTEM_CLOCK
{
    HSI_selected_as_system_clock_U1_VALUE_SYSTEM_CLOCK,
    HSE_selected_as_system_clock_U1_VALUE_SYSTEM_CLOCK,
};


/*
The high speed external clock signal (HSE) can be generated from two possible clock
sources:
• HSE external crystal/ceramic resonator
• HSE user external clock
*/

enum U1_VALUE_HSE_SOURCE_SYSTEM_CLOCK
{
    crystal_ceramic_resonator_U1_VALUE_HSE_SOURCE_SYSTEM_CLOCK,
    user_external_clock_U1_VALUE_HSE_SOURCE_SYSTEM_CLOCK
};


/* 
RCC_CFGR_USBPRE_USB_prescaler
Bit 22 USBPRE: USB prescaler
Set and cleared by software to generate 48 MHz USB clock. This bit must be valid before
enabling the USB clock in the RCC_APB1ENR register. This bit can’t be reset if the USB
clock is enabled.
0: PLL clock is divided by 1.5
1: PLL clock is not divided
*/
enum U1_VALUE_USB_prescaler_RCC_CFGR
{
    PLL_divided_1_5_U1_VALUE_USB_prescaler_RCC_CFGR,
    PLL_not_divided_U1_VALUE_USB_prescaler_RCC_CFGR
};




enum u3_value_RCC_CFGR_MCO_Microcontroller_clock_output
{
    no_clk_RCC_CFGR_MCO_Microcontroller_clock_output,
    System_clock_SYSCLK_selected_RCC_CFGR_MCO_Microcontroller_clock_output = 0b100,
    HSI_clock_selected_RCC_CFGR_MCO_Microcontroller_clock_output,
    HSE_clock_selected_RCC_CFGR_MCO_Microcontroller_clock_output,
    PLL_clock_divided_by_2_selected_RCC_CFGR_MCO_Microcontroller_clock_output,
};
//enum u8_value_RCC_CFGR_MCO_Microcontroller_clock_output x; // declaring an enum variable


enum u4_value_RCC_CFGR_PLLMUL_PLL_multiplication_factor
{
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
};



enum u1_value_RCC_CFGR_PLLXTPRE_HSE_divider_for_PLL_entry
{
    HSE_not_divided_u1_value_RCC_CFGR_PLLXTPRE_HSE_divider_for_PLL_entry,
    HSE_divided_2_u1_value_RCC_CFGR_PLLXTPRE_HSE_divider_for_PLL_entry
};


enum u1_value_RCC_CFGR_PLLSRC_PLL_entry_clock_source
{
    HSI_div_2_u1_value_RCC_CFGR_PLLSRC_PLL_entry_clock_source,
    HSE_u1_value_RCC_CFGR_PLLSRC_PLL_entry_clock_source
};



enum u1_value_PLL_USAGE_USER_CONFIG
{
    not_used_u1_value_PLL_USAGE_USER_CONFIG,
    using_PLL_u1_value_PLL_USAGE_USER_CONFIG
};





enum u1_value_USB_USAGE_USER_CONFIG
{
    not_used_u1_value_USB_USAGE_USER_CONFIG,
    using_usb_u1_value_USB_USAGE_USER_CONFIG
};



enum u2_value_RCC_CFGR_ADC
{
    PCLK2_divided_2_RCC_CFGR_ADC,
    PCLK2_divided_4_RCC_CFGR_ADC,
    PCLK2_divided_6_RCC_CFGR_ADC,
    PCLK2_divided_8_RCC_CFGR_ADC,
};

enum u3_value_RCC_CFGR_PPRE2_APB_high_speed_prescaler
{
    HCLK_not_divided_RCC_CFGR_PPRE2_APB_high_speed_prescaler,
    HCLK_divided_2_RCC_CFGR_PPRE2_APB_high_speed_prescaler=0b100,
    HCLK_divided_4_RCC_CFGR_PPRE2_APB_high_speed_prescaler,
    HCLK_divided_8_RCC_CFGR_PPRE2_APB_high_speed_prescaler,
    HCLK_divided_16_RCC_CFGR_PPRE2_APB_high_speed_prescaler,
};




enum u3_value_RCC_CFGR_PPRE1_APB_low_speed_prescaler
{
    HCLK_not_divided_RCC_CFGR_PPRE1_APB_low_speed_prescaler,
    HCLK_divided_2_RCC_CFGR_PPRE1_APB_low_speed_prescaler=0b100,
    HCLK_divided_4_RCC_CFGR_PPRE1_APB_low_speed_prescaler,
    HCLK_divided_8_RCC_CFGR_PPRE1_APB_low_speed_prescaler,
    HCLK_divided_16_RCC_CFGR_PPRE1_APB_low_speed_prescaler,
};




enum u4_value_RCC_CFGR_HPRE_AHB_prescaler
{
    SYSCLK_not_divided_RCC_CFGR_HPRE_AHB_prescaler,
    SYSCLK_divided_2_RCC_CFGR_HPRE_AHB_prescaler=0b1000,
    SYSCLK_divided_4_RCC_CFGR_HPRE_AHB_prescaler,
    SYSCLK_divided_8_RCC_CFGR_HPRE_AHB_prescaler,
    SYSCLK_divided_16_RCC_CFGR_HPRE_AHB_prescaler,
    SYSCLK_divided_64_RCC_CFGR_HPRE_AHB_prescaler,
    SYSCLK_divided_128_RCC_CFGR_HPRE_AHB_prescaler,
    SYSCLK_divided_256_RCC_CFGR_HPRE_AHB_prescaler,
    SYSCLK_divided_512_RCC_CFGR_HPRE_AHB_prescaler,
};



enum u2_value_RCC_CFGR_SW_System_clock_switch
{
    HSI_selected_as_system_clock_RCC_CFGR_SW_System_clock_switch,
    HSE_selected_as_system_clock_RCC_CFGR_SW_System_clock_switch,
    PLL_selected_as_system_clock_RCC_CFGR_SW_System_clock_switch,
};




enum u2_value_RCC_BDCR_RTCSEL_RTC_clock_source_selection
{
    No_clock_RCC_BDCR_RTCSEL_RTC_clock_source_selection,
    LSE_oscillator_clock_used_as_RTC_clock_RCC_BDCR_RTCSEL_RTC_clock_source_selection,
    LSI_oscillator_clock_used_as_RTC_clock_RCC_BDCR_RTCSEL_RTC_clock_source_selection,
    HSE_oscillator_clock_divided_by_128_used_as_RTC_clock_RCC_BDCR_RTCSEL_RTC_clock_source_selection,
};



//--------------------------------------	


/* macro function*/

#define MACRO_CONFIG_RCC_CFGR_PLLMUL_PLL_multiplication_factor(val)         RCC_CFGR_PLLMUL_PLL_multiplication_factor_REG=(val<<2)  // CONFIG BEFORE LOWER AND HIGHER BITS
#define MACRO_CONFIG_RCC_CFGR_HPRE_AHB_prescaler(val)                       RCC_CFGR_HPRE_AHB_prescaler_REG=val

#define MACRO_ENABLE_PLL_RCC_CR_PLLON_PLL_enable()                          SETBIT(RCC_CR_REG,RCC_CR_PLLON_PLL_enable)
#define MACRO_CHECK_PLL_READY_RCC_CR_PLLRDY_PLL_clock_ready_flag()          GETBIT(RCC_CR_REG,RCC_CR_PLLRDY_PLL_clock_ready_flag)



#define MACRO_ENABLE_PERPH_RCC(BUS_OFFSET,PERPH_BIT)         SETBIT(  (*(volatile u32 *)(RCC_BASE + BUS_OFFSET)), PERPH_BIT)
#define MACRO_DISABLE_PERPH_RCC(BUS_OFFSET,PERPH_BIT)        CLEARBIT((*(volatile u32 *)(RCC_BASE + BUS_OFFSET)), PERPH_BIT)

//--------------------------------------	


#endif /* __INTERFACE_ARM_RCC_DRIVER_H__ */