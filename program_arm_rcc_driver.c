/******************************************************************************
 * Module: 
 * File Name: Program_arm_rcc_driver.c
 * Version: 1
 * Date: 31/12/2021
 * Description: function prototypes and variables and macro functions that can be called by user 
 * Author: Zyad Ahmed Mackawy (ZAM)
 *******************************************************************************/
#include "../../macro.h"
#include "private_arm_rcc_driver.h"
#include "interface_arm_rcc_driver.h"
#include "hardware_arm_rcc_driver.h"
#include "config_arm_rcc_driver.h"
/* Private */

//--------------------------------------

/* Interface*/
void void_init_sysclk_rcc()
{
#if SYSTEM_CLOCK_USER_CONFIG == HSE_selected_as_system_clock_U2_VALUE_SYSTEM_CLOCK && HSE_SOURCE_USER_CONFIG == crystal_ceramic_resonator_U1_VALUE_HSE_SOURCE_SYSTEM_CLOCK
/*
    External crystal/ceramic resonator (HSE crystal)
    The 4 to 16 MHz external oscillator has the advantage of producing a very accurate rate on
    the main clock.
    The associated hardware configuration is shown in Figure 9. Refer to the electrical
    characteristics section of the datasheet for more details.
    The HSERDY flag in the Clock control register (RCC_CR) indicates if the high-speed
    external oscillator is stable or not. At startup, the clock is not released until this bit is set by
    hardware. An interrupt can be generated if enabled in the Clock interrupt register
    (RCC_CIR).
    The HSE Crystal can be switched on and off using the HSEON bit in the Clock control
    register (RCC_CR).
    ------------------------
    ------------------------
    ------------------------
    RCC_CR -> HSEON , Crystal can be switched on and off
    ------------------------
    RCC_CR -> HSERDY , At startup, the clock is not released until this bit is set by hardware
    */
   
    //RCC_CR -> HSEON , Crystal can be switched on and off
    SETBIT(RCC_CR_REG,RCC_CR_HSEON_HSE_clock_enable);
    //RCC_CR -> HSERDY , At startup, the clock is not released until this bit is set by hardware
    while (1)
    {
        if (GETBIT(RCC_CR_REG,RCC_CR_HSERDY_External_high_speed_clock_ready_flag))
        {
            break;
        }
    }
#elif ((SYSTEM_CLOCK_USER_CONFIG == HSE_selected_as_system_clock_U2_VALUE_SYSTEM_CLOCK) && (HSE_SOURCE_USER_CONFIG == user_external_clock_U1_VALUE_HSE_SOURCE_SYSTEM_CLOCK))
    /*
    External source (HSE bypass)
    In this mode, an external clock source must be provided. It can have a frequency of up to
    25 MHz. You select this mode by setting the HSEBYP and HSEON bits in the Clock control
    register (RCC_CR). The external clock signal (square, sinus or triangle) with ~50% duty
    cycle has to drive the OSC_IN pin while the OSC_OUT pin should be left hi-Z
    ------------------------
    ------------------------
    ------------------------
    RCC_CR -> HSEON,HSEBYP , select this mode by setting
    */
    SETBIT(RCC_CR_REG,RCC_CR_HSEON_HSE_clock_enable);
    SETBIT(RCC_CR_REG,RCC_CR_HSEBYP_External_high_speed_clock_bypass);

#else
//HSI_selected_as_system_clock_U2_VALUE_SYSTEM_CLOCK
/*
    The HSI clock signal is generated from an internal 8 MHz RC Oscillator and can be used
    directly as a system clock or divided by 2 to be used as PLL input

    RC oscillator frequencies can vary from one chip to another due to manufacturing process
    variations, this is why each device is factory calibrated by ST for 1% accuracy at TA=25°C.
    After reset, the factory calibration value is loaded in the HSICAL[7:0] bits in the Clock control
    register (RCC_CR).
    If the application is subject to voltage or temperature variations this may affect the RC
    oscillator speed. You can trim the HSI frequency in the application using the HSITRIM[4:0]
    bits in the Clock control register (RCC_CR).
    The HSIRDY flag in the Clock control register (RCC_CR) indicates if the HSI RC is stable or
    not. At startup, the HSI RC output clock is not released until this bit is set by hardware.
    The HSI RC can be switched on and off using the HSION bit in the Clock control register
    (RCC_CR).
    The HSI signal can also be used as a backup source (Auxiliary clock) if the HSE crystal
    oscillator fails - Clock security system (CSS).
    ------------------------
    ------------------------
    ------------------------
    RCC_CR -> HSION, switched on and off
    ------------------------
    RCC_CR -> HSIRDY, stable or not. At startup, the HSI RC output clock is not released until this bit is set by hardware
    ------------------------
    RCC_CR -> HSITRIM[4:0], trim the HSI frequency in the application
    ------------------------
    RCC_CR -> CSS, backup source (Auxiliary clock) if the HSE crystal oscillator fails
    */
   
    //RCC_CR -> HSION, switched on and off
    
    //RCC_CR -> HSIRDY, stable or not. At startup, the HSI RC output clock is not released until this bit is set by hardware
    //RCC_CR -> HSITRIM[4:0], trim the HSI frequency in the application
    
    //RCC_CR -> CSS, backup source (Auxiliary clock) if the HSE crystal oscillator fails
#endif

#if ((PLL_USAGE_USER_CONFIG == using_PLL_u1_value_PLL_USAGE_USER_CONFIG) && (USB_USAGE_USER_CONFIG == not_used_u1_value_USB_USAGE_USER_CONFIG))
    /*
    The internal PLL can be used to multiply the HSI RC output or HSE crystal output clock
    frequency. 
    The PLL configuration (selection of HSI oscillator divided by 2 or HSE oscillator for PLL
    input clock, and multiplication factor) must be done before enabling the PLL. Once the PLL
    enabled, these parameters cannot be changed.
    An interrupt can be generated when the PLL is ready if enabled in the Clock interrupt
    register (RCC_CIR).
    If the USB interface is used in the application, the PLL must be programmed to output 48 or
    72 MHz. This is needed to provide a 48 MHz USBCLK.
    ------------------------
    ------------------------
    ------------------------
    PLL configuration must be done before enabling the PLL
    PLL configuration : (HSI oscillator divided by 2, HSE oscillator) for PLL input clock, and multiplication factor
    Once the PLL enabled, these parameters cannot be changed
    If the USB interface is used in the application, the PLL must be programmed to output 48 or
    72 MHz. This is needed to provide a 48 MHz USBCLK.
    ------------------------
    RCC_CFGR , (HSI oscillator divided by 2, HSE oscillator) for PLL input clock, and multiplication factor
    ------------------------
    RCC_CFGR -> PLLSRC, input clock
    ------------------------
    RCC_CFGR -> PLLXTPRE, HSE divider
    ------------------------
    RCC_CFGR -> PLLMUL, multiplication factor
    ------------------------
    RCC_CR -> RCC_CR_PLLON_PLL_enable, on
    ------------------------
    RCC_CR -> RCC_CR_PLLRDY_PLL_clock_ready_flag, ready
    ------------------------
    RCC_CFGR -> RCC_CFGR_USBPRE_USB, PLL must be programmed to output 48 or 72 MHz. This is needed to provide a 48 MHz USBCLK.
    
    */
    //RCC_CFGR -> PLLMUL, multiplication factor
    MACRO_CONFIG_RCC_CFGR_PLLMUL_PLL_multiplication_factor(PLL_multiplication_factor_USER_CONFIG);

    // RCC_CFGR -> PLLSRC, input clock
    if (PLL_entry_clock_source_USER_CONFIG)
    {
        SETBIT(RCC_CFGR_REG, RCC_CFGR_PLLSRC_PLL_entry_clock_source);
    }
    else
    {
        CLEARBIT(RCC_CFGR_REG, RCC_CFGR_PLLSRC_PLL_entry_clock_source);
    }

    // RCC_CFGR -> PLLXTPRE, HSE divider
    if (HSE_divider_for_PLL_entry_USER_CONFIG)
    {
        SETBIT(RCC_CFGR_REG, RCC_CFGR_PLLXTPRE_HSE_divider_for_PLL_entry);
    }
    else
    {
        CLEARBIT(RCC_CFGR_REG, RCC_CFGR_PLLXTPRE_HSE_divider_for_PLL_entry);
    }

    //RCC_CR -> RCC_CR_PLLON_PLL_enable, on
    MACRO_ENABLE_PLL_RCC_CR_PLLON_PLL_enable();

    //RCC_CR -> RCC_CR_PLLRDY_PLL_clock_ready_flag, ready
    while (1)
    {
        if (MACRO_CHECK_PLL_READY_RCC_CR_PLLRDY_PLL_clock_ready_flag())
        {
            break;
        }
    }

#endif
}
//--------------------------------------
