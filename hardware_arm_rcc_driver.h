/******************************************************************************
 * Module: 
 * File Name: Hardware_arm_rcc_driver.h
 * Version: 1
 * Date: 31/12/2021
 * Description: registers or connection with their macros and data types
 * Author: Zyad Ahmed Mackawy (ZAM)
 *******************************************************************************/
#ifndef __HARDWARE_ARM_RCC_DRIVER_H__
#define __HARDWARE_ARM_RCC_DRIVER_H__

/* RCC */

#define RCC_BASE 0x40021000


/* RCC_CR start */

#define RCC_CR_OFFSET 0x00
#define RCC_CR_BASE (RCC_BASE+RCC_CR_OFFSET)
#define RCC_CR_REG (*(volatile u32 *)RCC_CR_BASE)

#define RCC_CR_PLLRDY_PLL_clock_ready_flag  25
/* PLL clock ready flag
Set by hardware to indicate that the PLL is locked.
0: PLL unlocked
1: PLL locked 
*/
#define RCC_CR_PLLON_PLL_enable 24

/*
Bit 24 PLLON: PLL enable
Set and cleared by software to enable PLL.
Cleared by hardware when entering Stop or Standby mode. This bit can not be reset if the
PLL clock is used as system clock or is selected to become the system clock. Software
must disable the USB OTG FS clock before clearing this bit.
0: PLL OFF
1: PLL ON
*/

// Bits 23:20 Reserved, must be kept at reset value.
#define RCC_CR_CSSON_Clock_security_system_enable 19
/*
Bit 19 CSSON: Clock security system enable
Set and cleared by software to enable the clock security system. When CSSON is set, the
clock detector is enabled by hardware when the HSE oscillator is ready, and disabled by
hardware if a HSE clock failure is detected.
0: Clock detector OFF
1: Clock detector ON (Clock detector ON if the HSE oscillator is ready, OFF if not)
*/
#define RCC_CR_HSEBYP_External_high_speed_clock_bypass 18
/*
Bit 18 HSEBYP: External high-speed clock bypass
Set and cleared by software to bypass the oscillator with an external clock. The external
clock must be enabled with the HSEON bit set, to be used by the device. The HSEBYP bit
can be written only if the HSE oscillator is disabled.
0: external 3-25 MHz oscillator not bypassed
1: external 3-25 MHz oscillator bypassed with external clock
*/
#define RCC_CR_HSERDY_External_high_speed_clock_ready_flag 17
/*
Bit 17 HSERDY: External high-speed clock ready flag
Set by hardware to indicate that the HSE oscillator is stable. This bit needs 6 cycles of the
HSE oscillator clock to fall down after HSEON reset.
0: HSE oscillator not ready
1: HSE oscillator ready
*/
#define RCC_CR_HSEON_HSE_clock_enable 16
/*
Bit 16 HSEON: HSE clock enable
Set and cleared by software.
Cleared by hardware to stop the HSE oscillator when entering Stop or Standby mode. This
bit cannot be reset if the HSE oscillator is used directly or indirectly as the system clock.
0: HSE oscillator OFF
1: HSE oscillator ON
*/
#define RCC_CR_HSICAL_Internal_high_speed_clock_calibration_OFFSET 0x01
#define RCC_CR_HSICAL_Internal_high_speed_clock_calibration_BASE (RCC_CR_BASE+RCC_CR_HSICAL_Internal_high_speed_clock_calibration_OFFSET)
#define RCC_CR_HSICAL_Internal_high_speed_clock_calibration_REG (*(volatile u8 *)RCC_CR_HSICAL_Internal_high_speed_clock_calibration_BASE) 

/*
Bits 15:8 HSICAL[7:0]: Internal high-speed clock calibration
These bits are initialized automatically at startup. 

*/
#define RCC_CR_HSITRIM_Internal_high_speed_clock_trimming_OFFSET 0x00
#define RCC_CR_HSITRIM_Internal_high_speed_clock_trimming_BASE (RCC_CR_BASE+RCC_CR_HSITRIM_Internal_high_speed_clock_trimming_OFFSET)
#define RCC_CR_HSITRIM_Internal_high_speed_clock_trimming_REG (*(volatile u8 *)RCC_CR_HSITRIM_Internal_high_speed_clock_trimming_BASE) // SHIFT 3 -- before setting RCC_CR_HSIRDY & RCC_CR_HSION 
/* 
Bits 7:3 HSITRIM[4:0]: Internal high-speed clock trimming
These bits provide an additional user-programmable trimming value that is added to the
HSICAL[7:0] bits. It can be programmed to adjust to variations in voltage and temperature
that influence the frequency of the internal HSI RC.
The default value is 16, which, when added to the HSICAL value, should trim the HSI to 8
MHz ± 1%. The trimming step (Fhsitrim) is around 40 kHz between two consecutive HSICAL
steps.
*/

//Bit 2 Reserved, must be kept at reset value.
#define RCC_CR_HSIRDY_Internal_high_speed_clock_ready_flag 1 // after setting RCC_CR_HSITRIM
/*
Bit 1 HSIRDY: Internal high-speed clock ready flag
Set by hardware to indicate that internal 8 MHz RC oscillator is stable. After the HSION bit
is cleared, HSIRDY goes low after 6 internal 8 MHz RC oscillator clock cycles.
0: Internal 8 MHz RC oscillator not ready
1: Internal 8 MHz RC oscillator ready
*/
#define RCC_CR_HSION_Internal_high_speed_clock_enable 0 // after setting RCC_CR_HSITRIM
/*
Bit 0 HSION: Internal high-speed clock enable
*/

/* RCC_CR end */
//--------------------------------------

/* RCC_CFGR start */

#define RCC_CFGR_OFFSET 0x04
#define RCC_CFGR_BASE (RCC_BASE+RCC_CFGR_OFFSET)
#define RCC_CFGR_REG (*(volatile u32 *)RCC_CFGR_BASE)
/* 
Bits 31:27 Reserved, must be kept at reset value.
*/
#define RCC_CFGR_MCO_Microcontroller_clock_output_OFFSET 0x03
#define RCC_CFGR_MCO_Microcontroller_clock_output_BASE (RCC_CFGR_BASE+RCC_CFGR_MCO_Microcontroller_clock_output_OFFSET)
#define RCC_CFGR_MCO_Microcontroller_clock_output_REG (*(volatile u8 *)RCC_CFGR_MCO_Microcontroller_clock_output_BASE) 
/* 
Bits 26:24 MCO: Microcontroller clock output
Set and cleared by software.
0xx: No clock
100: System clock (SYSCLK) selected
101: HSI clock selected
110: HSE clock selected
111: PLL clock divided by 2 selected
Note: This clock output may have some truncated cycles at startup or during MCO clock
source switching.
When the System Clock is selected to output to the MCO pin, make sure that this clock
does not exceed 50 MHz (the maximum IO speed).
*/
// 23 RES
#define RCC_CFGR_USBPRE_USB_prescaler   22
/* 
Bit 22 USBPRE: USB prescaler
Set and cleared by software to generate 48 MHz USB clock. This bit must be valid before
enabling the USB clock in the RCC_APB1ENR register. This bit can’t be reset if the USB
clock is enabled.
0: PLL clock is divided by 1.5
1: PLL clock is not divided

*/

#define RCC_CFGR_PLLMUL_PLL_multiplication_factor_OFFSET 0x02
#define RCC_CFGR_PLLMUL_PLL_multiplication_factor_BASE (RCC_CFGR_BASE+RCC_CFGR_PLLMUL_PLL_multiplication_factor_OFFSET)
#define RCC_CFGR_PLLMUL_PLL_multiplication_factor_REG (*(volatile u8 *)RCC_CFGR_PLLMUL_PLL_multiplication_factor_BASE)  // NEED SHIFT 2 - CONFIG BEFORE LOWER AND HIGHER BITS

/* 
Bits 21:18 PLLMUL: PLL multiplication factor
These bits are written by software to define the PLL multiplication factor. These bits can be
written only when PLL is disabled.
Caution: The PLL output frequency must not exceed 72 MHz.
0000: PLL input clock x 2
0001: PLL input clock x 3
0010: PLL input clock x 4
0011: PLL input clock x 5
0100: PLL input clock x 6
0101: PLL input clock x 7
0110: PLL input clock x 8
0111: PLL input clock x 9
1000: PLL input clock x 10
1001: PLL input clock x 11
1010: PLL input clock x 12
1011: PLL input clock x 13
1100: PLL input clock x 14
1101: PLL input clock x 15
1110: PLL input clock x 16
1111: PLL input clock x 16
*/
#define RCC_CFGR_PLLXTPRE_HSE_divider_for_PLL_entry   17
/* 
Bit 17 PLLXTPRE: HSE divider for PLL entry
Set and cleared by software to divide HSE before PLL entry. This bit can be written only
when PLL is disabled.
0: HSE clock not divided
1: HSE clock divided by 2
*/
#define RCC_CFGR_PLLSRC_PLL_entry_clock_source   16
/* 
Bit 16 PLLSRC: PLL entry clock source
Set and cleared by software to select PLL clock source. This bit can be written only when
PLL is disabled.
0: HSI oscillator clock / 2 selected as PLL input clock
1: HSE oscillator clock selected as PLL input clock
*/
#define RCC_CFGR_ADC_BIT_1      15
#define RCC_CFGR_ADC_BIT_0      14
/*
Bits 15:14 ADCPRE: ADC prescaler
Set and cleared by software to select the frequency of the clock to the ADCs.
00: PCLK2 divided by 2
01: PCLK2 divided by 4
10: PCLK2 divided by 6
11: PCLK2 divided by 8
*/
#define RCC_CFGR_PPRE2_APB_high_speed_prescaler_BIT_2       13
#define RCC_CFGR_PPRE2_APB_high_speed_prescaler_BIT_1       12
#define RCC_CFGR_PPRE2_APB_high_speed_prescaler_BIT_0       11
/* 
Bits 13:11 PPRE2: APB high-speed prescaler (APB2)
Set and cleared by software to control the division factor of the APB high-speed clock
(PCLK2).
0xx: HCLK not divided
100: HCLK divided by 2
101: HCLK divided by 4
110: HCLK divided by 8
111: HCLK divided by 16
*/
#define RCC_CFGR_PPRE1_APB_low_speed_prescaler_BIT_2        10
#define RCC_CFGR_PPRE1_APB_low_speed_prescaler_BIT_1        9
#define RCC_CFGR_PPRE1_APB_low_speed_prescaler_BIT_0        8
/* 
Bits 10:8 PPRE1: APB low-speed prescaler (APB1)
Set and cleared by software to control the division factor of the APB low-speed clock
(PCLK1).
Warning: the software has to set correctly these bits to not exceed 36 MHz on this domain.
0xx: HCLK not divided
100: HCLK divided by 2
101: HCLK divided by 4
110: HCLK divided by 8
111: HCLK divided by 16
*/

#define RCC_CFGR_HPRE_AHB_prescaler_OFFSET 0x00
#define RCC_CFGR_HPRE_AHB_prescaler_BASE (RCC_CFGR_BASE+RCC_CFGR_HPRE_AHB_prescaler_OFFSET)
#define RCC_CFGR_HPRE_AHB_prescaler_REG (*(volatile u8 *)RCC_CFGR_HPRE_AHB_prescaler_BASE)  // SET BEFORE LOWER BITS SWS SW
/* 

Bits 7:4 HPRE: AHB prescaler
Set and cleared by software to control the division factor of the AHB clock.
0xxx: SYSCLK not divided
1000: SYSCLK divided by 2
1001: SYSCLK divided by 4
1010: SYSCLK divided by 8
1011: SYSCLK divided by 16
1100: SYSCLK divided by 64
1101: SYSCLK divided by 128
1110: SYSCLK divided by 256
1111: SYSCLK divided by 512
Note: The prefetch buffer must be kept on when using a prescaler different from 1 on the
AHB clock. Refer to Reading the Flash memory section for more details.

*/
#define RCC_CFGR_SWS_System_clock_switch_status_BIT_1   3
#define RCC_CFGR_SWS_System_clock_switch_status_BIT_0   2
/* 
Bits 3:2 SWS: System clock switch status
Set and cleared by hardware to indicate which clock source is used as system clock.
00: HSI oscillator used as system clock
01: HSE oscillator used as system clock
10: PLL used as system clock
11: not applicable

*/
#define RCC_CFGR_SW_System_clock_switch_BIT_1   1
#define RCC_CFGR_SW_System_clock_switch_BIT_0   0
/* 
Bits 1:0 SW: System clock switch
Set and cleared by software to select SYSCLK source.
Set by hardware to force HSI selection when leaving Stop and Standby mode or in case of
failure of the HSE oscillator used directly or indirectly as system clock (if the Clock Security
System is enabled).
00: HSI selected as system clock
01: HSE selected as system clock
10: PLL selected as system clock
11: not allowed
*/
/* 

*/
/* 

*/
/* RCC_CFGR end */
//--------------------------------------




/* RCC_CIR start */

#define RCC_CIR_OFFSET 0x08
#define RCC_CIR_BASE (RCC_BASE+RCC_CIR_OFFSET)
#define RCC_CIR_REG (*(volatile u32 *)RCC_CIR_BASE)


/* 
Bits 31:24 Reserved, must be kept at reset value.

*/
#define RCC_CIR_CSSC_Clock_security_system_interrupt_clear        23
/* 
Bit 23 CSSC: Clock security system interrupt clear
This bit is set by software to clear the CSSF flag.
0: No effect
1: Clear CSSF flag

*/

/* 
Bits 22:21 Reserved, must be kept at reset value.

*/
#define RCC_CIR_PLLRDYC_PLL_ready_interrupt_clear         20
/* 
Bit 20 PLLRDYC: PLL ready interrupt clear
This bit is set by software to clear the PLLRDYF flag.
0: No effect
1: PLLRDYF cleared

*/
#define RCC_CIR_HSERDYC_HSE_ready_interrupt_clear         19
/* 
Bit 19 HSERDYC: HSE ready interrupt clear
This bit is set by software to clear the HSERDYF flag.
0: No effect
1: HSERDYF cleared

*/
#define RCC_CIR_HSIRDYC_HSI_ready_interrupt_clear     18
/* 
Bit 18 HSIRDYC: HSI ready interrupt clear
This bit is set software to clear the HSIRDYF flag.
0: No effect
1: HSIRDYF cleared

*/
#define RCC_CIR_LSERDYC_LSE_ready_interrupt_clear     17
/* 
Bit 17 LSERDYC: LSE ready interrupt clear
This bit is set by software to clear the LSERDYF flag.
0: No effect
1: LSERDYF cleared

*/
#define RCC_CIR_LSIRDYC_LSI_ready_interrupt_clear         16
/* 
Bit 16 LSIRDYC: LSI ready interrupt clear
This bit is set by software to clear the LSIRDYF flag.
0: No effect
1: LSIRDYF cleared

*/

/* 
Bits 15:13 Reserved, must be kept at reset value.

*/
#define RCC_CIR_PLLRDYIE_PLL_ready_interrupt_enable        12
/* 
Bit 12 PLLRDYIE: PLL ready interrupt enable
Set and cleared by software to enable/disable interrupt caused by PLL lock.
0: PLL lock interrupt disabled
1: PLL lock interrupt enabled
*/
#define RCC_CIR_HSERDYIE_HSE_ready_interrupt_enable        11
/* 
Bit 11 HSERDYIE: HSE ready interrupt enable
Set and cleared by software to enable/disable interrupt caused by the external 4-16 MHz
oscillator stabilization.
0: HSE ready interrupt disabled
1: HSE ready interrupt enabled
*/
#define RCC_CIR_HSIRDYIE_HSI_ready_interrupt_enable    10
/* 
Bit 10 HSIRDYIE: HSI ready interrupt enable
Set and cleared by software to enable/disable interrupt caused by the internal 8 MHz RC
oscillator stabilization.
0: HSI ready interrupt disabled
1: HSI ready interrupt enabled

*/
#define RCC_CIR_LSERDYIE_LSE_ready_interrupt_enable    9
/* 
Bit 9 LSERDYIE: LSE ready interrupt enable
Set and cleared by software to enable/disable interrupt caused by the external 32 kHz
oscillator stabilization.
0: LSE ready interrupt disabled
1: LSE ready interrupt enabled

*/
#define RCC_CIR_LSIRDYIE_LSI_ready_interrupt_enable    8
/* 
Bit 8 LSIRDYIE: LSI ready interrupt enable
Set and cleared by software to enable/disable interrupt caused by internal RC 40 kHz
oscillator stabilization.
0: LSI ready interrupt disabled
1: LSI ready interrupt enabled

*/
#define RCC_CIR_CSSF_Clock_security_system_interrupt_flag    7
/* 
Bit 7 CSSF: Clock security system interrupt flag
Set by hardware when a failure is detected in the external 4-16 MHz oscillator.
Cleared by software setting the CSSC bit.
0: No clock security interrupt caused by HSE clock failure
1: Clock security interrupt caused by HSE clock failure

*/

/* 
Bits 6:5 Reserved, must be kept at reset value.

*/
#define RCC_CIR_PLLRDYF_PLL_ready_interrupt_flag     4
/* 
Bit 4 PLLRDYF: PLL ready interrupt flag
Set by hardware when the PLL locks and PLLRDYDIE is set.
Cleared by software setting the PLLRDYC bit.
0: No clock ready interrupt caused by PLL lock
1: Clock ready interrupt caused by PLL lock

*/
#define RCC_CIR_HSERDYF_HSE_ready_interrupt_flag     3
/* 
Bit3 HSERDYF: HSE ready interrupt flag
Set by hardware when External High Speed clock becomes stable and HSERDYDIE is set.
Cleared by software setting the HSERDYC bit.
0: No clock ready interrupt caused by the external 4-16 MHz oscillator
1: Clock ready interrupt caused by the external 4-16 MHz oscillator
*/
#define RCC_CIR_HSIRDYF_HSI_ready_interrupt_flag     2
/* 
Bit 2 HSIRDYF: HSI ready interrupt flag
Set by hardware when the Internal High Speed clock becomes stable and HSIRDYDIE is
set.
Cleared by software setting the HSIRDYC bit.
0: No clock ready interrupt caused by the internal 8 MHz RC oscillator
1: Clock ready interrupt caused by the internal 8 MHz RC oscillator

*/
#define RCC_CIR_LSERDYF_LSE_ready_interrupt_flag        1
/* 
Bit 1 LSERDYF: LSE ready interrupt flag
Set by hardware when the External Low Speed clock becomes stable and LSERDYDIE is
set.
Cleared by software setting the LSERDYC bit.
0: No clock ready interrupt caused by the external 32 kHz oscillator
1: Clock ready interrupt caused by the external 32 kHz oscillator

*/

#define RCC_CIR_LSIRDYF_LSI_ready_interrupt_flag     0
/* 
Bit 0 LSIRDYF: LSI ready interrupt flag
Set by hardware when the internal low speed clock becomes stable and LSIRDYDIE is set.
Cleared by software setting the LSIRDYC bit.
0: No clock ready interrupt caused by the internal RC 40 kHz oscillator
1: Clock ready interrupt caused by the internal RC 40 kHz oscillator
*/
/* RCC_CIR end */
//--------------------------------------




/* RCC_APB2RSTR start */

#define RCC_APB2RSTR_OFFSET 0x0C
#define RCC_APB2RSTR_BASE (RCC_BASE+RCC_APB2RSTR_OFFSET)
#define RCC_APB2RSTR_REG (*(volatile u32 *)RCC_APB2RSTR_BASE)

/* 
Bits 31:22 Reserved, must be kept at reset value.
*/
#define RCC_APB2RSTR_TIM11RST_TIM11_timer_reset 21

/* Bit 21 TIM11RST: TIM11 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM11 timer
*/

#define RCC_APB2RSTR_TIM10RST_TIM10_timer_reset 20
/* 

Bit 20 TIM10RST: TIM10 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM10 timer
*/

#define RCC_APB2RSTR_TIM9RST_TIM9_timer_reset 19
/* 

Bit 19 TIM9RST: TIM9 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM9 timer
*/
/* 

Bits 18:16 Reserved, always read as 0
*/

#define RCC_APB2RSTR_ADC3RST_ADC3_interface_reset    15
/* 
Bit 15 ADC3RST: ADC3 interface reset
Set and cleared by software.
0: No effect
1: Reset ADC3 interface
*/

#define RCC_APB2RSTR_USART1RST_USART1_reset  14
/* 
Bit 14 USART1RST: USART1 reset
Set and cleared by software.
0: No effect
1: Reset USART1
*/

#define RCC_APB2RSTR_TIM8RST_TIM8_timer_reset    13
/* 
Bit 13 TIM8RST: TIM8 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM8 timer
*/

#define RCC_APB2RSTR_SPI1RST_SPI1_reset    12
/* 
Bit 12 SPI1RST: SPI1 reset
Set and cleared by software.
0: No effect
1: Reset SPI1
*/

#define RCC_APB2RSTR_TIM1RST_TIM1_timer_reset    11
/* 
Bit 11 TIM1RST: TIM1 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM1 timer
*/

#define RCC_APB2RSTR_ADC2RST_ADC_2_interface_reset    10
/* 
Bit 10 ADC2RST: ADC 2 interface reset
Set and cleared by software.
0: No effect
1: Reset ADC 2 interface
*/

#define RCC_APB2RSTR_ADC1RST_ADC_1_interface_reset    9
/* 
Bit 9 ADC1RST: ADC 1 interface reset
Set and cleared by software.
0: No effect
1: Reset ADC 1 interface
*/

#define RCC_APB2RSTR_IOPGRST_IO_port_G_reset    8
/* 
Bit 8 IOPGRST: IO port G reset
Set and cleared by software.
0: No effect
1: Reset IO port G
*/

#define RCC_APB2RSTR_IOPFRST_IO_port_F_reset    7
/* 
Bit 7 IOPFRST: IO port F reset
Set and cleared by software.
0: No effect
1: Reset IO port F
*/

#define RCC_APB2RSTR_IOPERST_IO_port_E_reset    6
/* 
Bit 6 IOPERST: IO port E reset
Set and cleared by software.
0: No effect
1: Reset IO port E
*/

#define RCC_APB2RSTR_IOPDRST_IO_port_D_reset    5
/* 
Bit 5 IOPDRST: IO port D reset
Set and cleared by software.
0: No effect
1: Reset IO port D
*/

#define RCC_APB2RSTR_IOPCRST_IO_port_C_reset    4
/* 
Bit 4 IOPCRST: IO port C reset
Set and cleared by software.
0: No effect
1: Reset IO port C
*/

#define RCC_APB2RSTR_IOPBRST_IO_port_B_reset    3
/* 
Bit 3 IOPBRST: IO port B reset
Set and cleared by software.
0: No effect
1: Reset IO port B
*/

#define RCC_APB2RSTR_IOPARST_IO_port_A_reset    2
/* 
Bit 2 IOPARST: IO port A reset
Set and cleared by software.
0: No effect
1: Reset IO port A
*/

/* 
Bit 1 Reserved, must be kept at reset value.
*/

#define RCC_APB2RSTR_AFIORST_Alternate_function_IO_reset    0
/* 
Bit 0 AFIORST: Alternate function IO reset
Set and cleared by software.
0: No effect
1: Reset Alternate Function
*/
/* RCC_APB2RSTR end */
//--------------------------------------




/* RCC_APB1RSTR start */

#define RCC_APB1RSTR_OFFSET 0x010
#define RCC_APB1RSTR_BASE (RCC_BASE+RCC_APB1RSTR_OFFSET)
#define RCC_APB1RSTR_REG (*(volatile u32 *)RCC_APB1RSTR_BASE)

/* 
Bits 31:30 Reserved, must be kept at reset value.
*/
#define RCC_APB1RSTR_DACRST_DAC_interface_reset  29
/* 

Bit 29 DACRST: DAC interface reset
Set and cleared by software.
0: No effect
1: Reset DAC interface
*/
#define RCC_APB1RSTR_PWRRST_Power_interface_reset       28
/* 

Bit 28 PWRRST: Power interface reset
Set and cleared by software.
0: No effect
1: Reset power interface
*/
#define RCC_APB1RSTR_BKPRST_Backup_interface_reset      27
/* 

Bit 27 BKPRST: Backup interface reset
Set and cleared by software.
0: No effect
1: Reset backup interface
*/
/* 

Bit 26 Reserved, must be kept at reset value.
*/
#define RCC_APB1RSTR_CANRST_CAN_reset       25
/* 

Bit 25 CANRST: CAN reset
Set and cleared by software.
0: No effect
1: Reset CAN
*/
/* 

Bit 24 Reserved, always read as 0.
*/
#define RCC_APB1RSTR_USBRST_USB_reset       23
/* 

Bit 23 USBRST: USB reset
Set and cleared by software.
0: No effect
1: Reset USB

*/
#define RCC_APB1RSTR_I2C2RST_I2C2_reset     22
/* 
Bit 22 I2C2RST: I2C2 reset
Set and cleared by software.
0: No effect
1: Reset I2C2

*/
#define RCC_APB1RSTR_I2C1RST_I2C1_reset     21
/* 
Bit 21 I2C1RST: I2C1 reset
Set and cleared by software.
0: No effect
1: Reset I2C1
*/
#define RCC_APB1RSTR_UART5RST_USART5 reset       20
/* 
Bit 20 UART5RST: USART5 reset
Set and cleared by software.
0: No effect
1: Reset USART5

*/
#define RCC_APB1RSTR_UART4RST_USART4 reset       19
/* 
Bit 19 UART4RST: USART4 reset
Set and cleared by software.
0: No effect
1: Reset USART4

*/
#define RCC_APB1RSTR_USART3RST_USART3_reset       18
/* 
Bit 18 USART3RST: USART3 reset
Set and cleared by software.
0: No effect
1: Reset USART3

*/
#define RCC_APB1RSTR_USART2RST_USART2_reset       17
/* 
Bit 17 USART2RST: USART2 reset
Set and cleared by software.
0: No effect
1: Reset USART2

*/
/* 
Bit 16 Reserved, must be kept at reset value.

*/
#define RCC_APB1RSTR_SPI3RST_SPI3_reset     15
/* 
Bit 15 SPI3RST: SPI3 reset
Set and cleared by software.
0: No effect
1: Reset SPI3

*/
#define RCC_APB1RSTR_SPI2RST_SPI2_reset     14
/* 
Bit 14 SPI2RST: SPI2 reset
Set and cleared by software.
0: No effect
1: Reset SPI2

*/

/* 
Bits 13:12 Reserved, must be kept at reset value.

*/
#define RCC_APB1RSTR_WWDGRST_Window_watchdog_reset      11
/* 
Bit 11 WWDGRST: Window watchdog reset
Set and cleared by software.
0: No effect
1: Reset window watchdog

*/
/* 
Bits 10:9 Reserved, must be kept at reset value.

*/
#define RCC_APB1RSTR_TIM14RST_TIM14_timer_reset     8
/* 
Bit 8 TIM14RST: TIM14 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM14

*/
#define RCC_APB1RSTR_TIM13RST_TIM13_timer_reset     7
/* 
Bit 7 TIM13RST: TIM13 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM13

*/
#define RCC_APB1RSTR_TIM12RST_TIM12_timer_reset     6
/* 
Bit 6 TIM12RST: TIM12 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM12
*/
#define RCC_APB1RSTR_TIM7RST_TIM7_timer_reset       5
/* 
Bit 5 TIM7RST: TIM7 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM7

*/
#define RCC_APB1RSTR_TIM6RST_TIM6_timer_reset       4
/* 
Bit 4 TIM6RST: TIM6 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM6

*/
#define RCC_APB1RSTR_TIM5RST_TIM5_timer_reset       3
/* 
Bit 3 TIM5RST: TIM5 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM5

*/
#define RCC_APB1RSTR_TIM4RST_TIM4_timer_reset       2
/* 
Bit 2 TIM4RST: TIM4 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM4

*/
#define RCC_APB1RSTR_TIM3RST_TIM3_timer_reset       1
/* 
Bit 1 TIM3RST: TIM3 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM3

*/
#define RCC_APB1RSTR_TIM2RST_TIM2_timer_reset       0
/* 
Bit 0 TIM2RST: TIM2 timer reset
Set and cleared by software.
0: No effect
1: Reset TIM2
*/

/* RCC_APB1RSTR end */
//--------------------------------------




/* RCC_AHBENR start */

#define RCC_AHBENR_OFFSET 0x14
#define RCC_AHBENR_BASE (RCC_BASE+RCC_AHBENR_OFFSET)
#define RCC_AHBENR_REG (*(volatile u32 *)RCC_AHBENR_BASE)

/* 
Bits 31:11 Reserved, must be kept at reset value.
*/
#define RCC_AHBENR_SDIOEN_SDIO_clock_enable     10
/* 

Bit 10 SDIOEN: SDIO clock enable
Set and cleared by software.
0: SDIO clock disabled
1: SDIO clock enabled

*/
/* 
Bits 9 Reserved, always read as 0
*/
#define RCC_AHBENR_FSMCEN_FSMC_clock_enable     8
/* 
Bit 8 FSMCEN: FSMC clock enable
Set and cleared by software.
0: FSMC clock disabled
1: FSMC clock enabled

*/
/* 
Bit 7 Reserved, always read as 0.

*/
#define RCC_AHBENR_CRCEN_CRC_clock_enable       6
/* 
Bit 6 CRCEN: CRC clock enable
Set and cleared by software.
0: CRC clock disabled
1: CRC clock enabled

*/
/* 
Bit 5 Reserved, must be kept at reset value.

*/
#define RCC_AHBENR_FLITFEN_FLITF_clock_enable       4
/* 
Bit 4 FLITFEN: FLITF clock enable
Set and cleared by software to disable/enable FLITF clock during Sleep mode.
0: FLITF clock disabled during Sleep mode
1: FLITF clock enabled during Sleep mode

*/
/* 
Bit 3 Reserved, must be kept at reset value.

*/
#define RCC_AHBENR_SRAMEN_SRAM_interface_clock_enable       2
/* 
Bit 2 SRAMEN: SRAM interface clock enable
Set and cleared by software to disable/enable SRAM interface clock during Sleep mode.
0: SRAM interface clock disabled during Sleep mode.
1: SRAM interface clock enabled during Sleep mode

*/
#define RCC_AHBENR_DMA2EN_DMA2_clock_enable     1
/* 
Bit 1 DMA2EN: DMA2 clock enable
Set and cleared by software.
0: DMA2 clock disabled
1: DMA2 clock enabled

*/
#define RCC_AHBENR_DMA1EN_DMA1_clock_enable     0
/* 
Bit 0 DMA1EN: DMA1 clock enable
Set and cleared by software.
0: DMA1 clock disabled
1: DMA1 clock enabled
*/
/* RCC_AHBENR end */
//--------------------------------------




/* RCC_APB2ENR start */

#define RCC_APB2ENR_OFFSET 0x18
#define RCC_APB2ENR_BASE (RCC_BASE+RCC_APB2ENR_OFFSET)
#define RCC_APB2ENR_REG (*(volatile u32 *)RCC_APB2ENR_BASE)

/* 
Bits 31:22 Reserved, must be kept at reset value.

*/
#define RCC_APB2ENR_TIM11EN_TIM11_timer_clock_enable        21
/* 

Bit 21 TIM11EN: TIM11 timer clock enable
Set and cleared by software.
0: TIM11 timer clock disabled
1: TIM11 timer clock enabled

*/
#define RCC_APB2ENR_TIM10EN_TIM10_timer_clock_enable        20
/* 

Bit 20 TIM10EN: TIM10 timer clock enable
Set and cleared by software.
0: TIM10 timer clock disabled
1: TIM10 timer clock enabled

*/
#define RCC_APB2ENR_TIM9EN_TIM9_timer_clock_enable          19
/* 

Bit 19 TIM9EN: TIM9 timer clock enable
Set and cleared by software.
0: TIM9 timer clock disabled
1: TIM9 timer clock enabled

*/
/* 

Bits 18:16 Reserved, always read as 0.

*/
#define RCC_APB2ENR_ADC3EN_ADC3_interface_clock_enable      15
/* 

Bit 15 ADC3EN: ADC3 interface clock enable
Set and cleared by software.
0: ADC3 interface clock disabled
1: ADC3 interface clock enabled

*/
#define RCC_APB2ENR_USART1EN_USART1_clock_enable            14
/* 

Bit 14 USART1EN: USART1 clock enable
Set and cleared by software.
0: USART1 clock disabled
1: USART1 clock enabled

*/
#define RCC_APB2ENR_TIM8EN_TIM8_Timer_clock_enable      13
/* 

Bit 13 TIM8EN: TIM8 Timer clock enable
Set and cleared by software.
0: TIM8 timer clock disabled
1: TIM8 timer clock enabled

*/
#define RCC_APB2ENR_SPI1EN_SPI1_clock_enable            12
/* 

Bit 12 SPI1EN: SPI1 clock enable
Set and cleared by software.
0: SPI1 clock disabled
1: SPI1 clock enabled

*/
#define RCC_APB2ENR_TIM1EN_TIM1_timer_clock_enable          11
/* 

Bit 11 TIM1EN: TIM1 timer clock enable
Set and cleared by software.
0: TIM1 timer clock disabled
1: TIM1 timer clock enabled

*/
#define RCC_APB2ENR_ADC2EN_ADC_2_interface_clock_enable        10
/* 

Bit 10 ADC2EN: ADC 2 interface clock enable
Set and cleared by software.
0: ADC 2 interface clock disabled
1: ADC 2 interface clock enabled
*/
#define RCC_APB2ENR_ADC1EN_ADC_1_interface_clock_enable     9
/* 
Bit 9 ADC1EN: ADC 1 interface clock enable
Set and cleared by software.
0: ADC 1 interface disabled
1: ADC 1 interface clock enabled

*/
#define RCC_APB2ENR_IOPGEN_IO_port_G_clock_enable       8
/* 

Bit 8 IOPGEN: IO port G clock enable
Set and cleared by software.
0: IO port G clock disabled
1: IO port G clock enabled

*/
#define RCC_APB2ENR_IOPFEN_IO_port_F_clock_enable       7
/* 

Bit 7 IOPFEN: IO port F clock enable
Set and cleared by software.
0: IO port F clock disabled
1: IO port F clock enabled

*/
#define RCC_APB2ENR_IOPEEN_IO_port_E_clock_enable       6
/* 

Bit 6 IOPEEN: IO port E clock enable
Set and cleared by software.
0: IO port E clock disabled
1: IO port E clock enabled

*/
#define RCC_APB2ENR_IOPDEN_IO_port_D_clock_enable       5
/* 

Bit 5 IOPDEN: IO port D clock enable
Set and cleared by software.
0: IO port D clock disabled
1: IO port D clock enabled

*/
/* 

Bit 4 IOPCEN: IO port C clock enable
Set and cleared by software.
0: IO port C clock disabled
1: IO port C clock enabled

*/
#define RCC_APB2ENR_IOPBEN_IO_port_B_clock_enable       3
/* 

Bit 3 IOPBEN: IO port B clock enable
Set and cleared by software.
0: IO port B clock disabled
1: IO port B clock enabled

*/
#define RCC_APB2ENR_IOPAEN_IO_port_A_clock_enable       2
/* 

Bit 2 IOPAEN: IO port A clock enable
Set and cleared by software.
0: IO port A clock disabled
1: IO port A clock enabled

*/
/* 

Bit 1 Reserved, must be kept at reset value.

*/
#define RCC_APB2ENR_AFIOEN_Alternate_function_IO_clock_enable       0
/* 

Bit 0 AFIOEN: Alternate function IO clock enable
Set and cleared by software.
0: Alternate Function IO clock disabled
1: Alternate Function IO clock enabled
*/
/* RCC_APB2ENR end */
//--------------------------------------




/* RCC_APB1ENR start */

#define RCC_APB1ENR_OFFSET 0x1C
#define RCC_APB1ENR_BASE (RCC_BASE+RCC_APB1ENR_OFFSET)
#define RCC_APB1ENR_REG (*(volatile u32 *)RCC_APB1ENR_BASE)

/* 
Bits 31:30 Reserved, must be kept at reset value.

*/
#define RCC_APB1ENR_DACEN_DAC_interface_clock_enable            29
/* 

Bit 29 DACEN: DAC interface clock enable
Set and cleared by software.
0: DAC interface clock disabled
1: DAC interface clock enable

*/
#define RCC_APB1ENR_PWREN_Power_interface_clock_enable            28
/* 

Bit 28 PWREN: Power interface clock enable
Set and cleared by software.
0: Power interface clock disabled
1: Power interface clock enable

*/
#define RCC_APB1ENR_BKPEN_Backup_interface_clock_enable            27
/* 

Bit 27 BKPEN: Backup interface clock enable
Set and cleared by software.
0: Backup interface clock disabled
1: Backup interface clock enabled

*/
/* 

Bit 26 Reserved, must be kept at reset value.

*/
#define RCC_APB1ENR_CANEN_CAN_clock_enable                25
/* 

Bit 25 CANEN: CAN clock enable
Set and cleared by software.
0: CAN clock disabled
1: CAN clock enabled

*/
/* 

Bit 24 Reserved, always read as 0.

*/
#define RCC_APB1ENR_USBEN_USB_clock_enable            23
/* 

Bit 23 USBEN: USB clock enable
Set and cleared by software.
0: USB clock disabled
1: USB clock enabled
*/
#define RCC_APB1ENR_I2C2EN_I2C2_clock_enable                        22
/* 
Bit 22 I2C2EN: I2C2 clock enable
Set and cleared by software.
0: I2C2 clock disabled
1: I2C2 clock enabled

*/
#define RCC_APB1ENR_I2C1EN_I2C1_clock_enable                    21
/* 

Bit 21 I2C1EN: I2C1 clock enable
Set and cleared by software.
0: I2C1 clock disabled
1: I2C1 clock enabled

*/
#define RCC_APB1ENR_UART5EN_USART5_clock_enable                20
/* 

Bit 20 UART5EN: USART5 clock enable
Set and cleared by software.
0: USART5 clock disabled
1: USART5 clock enabled

*/
#define RCC_APB1ENR_UART4EN_USART4_clock_enable                    19
/* 

Bit 19 UART4EN: USART4 clock enable
Set and cleared by software.
0: USART4 clock disabled
1: USART4 clock enabled

*/
#define RCC_APB1ENR_USART3EN_USART3_clock_enable                18
/* 

Bit 18 USART3EN: USART3 clock enable
Set and cleared by software.
0: USART3 clock disabled
1: USART3 clock enabled

*/
#define RCC_APB1ENR_USART2EN_USART2_clock_enable                        17
/* 

Bit 17 USART2EN: USART2 clock enable
Set and cleared by software.
0: USART2 clock disabled
1: USART2 clock enabled

*/
/* 

Bits 16 Reserved, always read as 0.

*/
#define RCC_APB1ENR_SPI3EN_SPI_3_clock_enable                    15
/* 

Bit 15 SPI3EN: SPI 3 clock enable
Set and cleared by software.
0: SPI 3 clock disabled
1: SPI 3 clock enabled

*/
#define RCC_APB1ENR_SPI2EN_SPI2_clock_enable                    14
/* 

Bit 14 SPI2EN: SPI2 clock enable
Set and cleared by software.
0: SPI2 clock disabled
1: SPI2 clock enabled

*/
/* 

Bits 13:12 Reserved, must be kept at reset value.

*/
#define RCC_APB1ENR_WWDGEN_Window_watchdog_clock_enable                11
/* 

Bit 11 WWDGEN: Window watchdog clock enable
Set and cleared by software.
0: Window watchdog clock disabled
1: Window watchdog clock enabled

*/
/* 

Bits 10:9 Reserved, must be kept at reset value.

*/
#define RCC_APB1ENR_TIM14EN_TIM14_timer_clock_enable                8
/* 

Bit 8 TIM14EN: TIM14 timer clock enable
Set and cleared by software.
0: TIM14 clock disabled
1: TIM14 clock enabled
*/
#define RCC_APB1ENR_TIM13EN_TIM13_timer_clock_enable                7
/* 
Bit 7 TIM13EN: TIM13 timer clock enable
Set and cleared by software.
0: TIM13 clock disabled
1: TIM13 clock enabled

*/
#define RCC_APB1ENR_TIM12EN_TIM12_timer_clock_enable                    6
/* 

Bit 6 TIM12EN: TIM12 timer clock enable
Set and cleared by software.
0: TIM12 clock disabled
1: TIM12 clock enabled

*/
#define RCC_APB1ENR_TIM7EN_TIM7_timer_clock_enable                    5
/* 

Bit 5 TIM7EN: TIM7 timer clock enable
Set and cleared by software.
0: TIM7 clock disabled
1: TIM7 clock enabled

*/
#define RCC_APB1ENR_TIM6EN_TIM6_timer_clock_enable                4
/* 

Bit 4 TIM6EN: TIM6 timer clock enable
Set and cleared by software.
0: TIM6 clock disabled
1: TIM6 clock enabled

*/
#define RCC_APB1ENR_TIM5EN_TIM5_timer_clock_enable                    3
/* 

Bit 3 TIM5EN: TIM5 timer clock enable
Set and cleared by software.
0: TIM5 clock disabled
1: TIM5 clock enabled

*/
#define RCC_APB1ENR_TIM4EN_TIM4_timer_clock_enable                2
/* 

Bit 2 TIM4EN: TIM4 timer clock enable
Set and cleared by software.
0: TIM4 clock disabled
1: TIM4 clock enabled

*/
#define RCC_APB1ENR_TIM3EN_TIM3_timer_clock_enable                1
/* 

Bit 1 TIM3EN: TIM3 timer clock enable
Set and cleared by software.
0: TIM3 clock disabled
1: TIM3 clock enabled

*/
#define RCC_APB1ENR_TIM2EN_TIM2_timer_clock_enable                    0
/* 

Bit 0 TIM2EN: TIM2 timer clock enable
Set and cleared by software.
0: TIM2 clock disabled
1: TIM2 clock enabled
*/

/* RCC_APB1ENR end */
//--------------------------------------




/* RCC_BDCR start */

#define RCC_BDCR_OFFSET 0x20
#define RCC_BDCR_BASE (RCC_BASE+RCC_BDCR_OFFSET)
#define RCC_BDCR_REG (*(volatile u32 *)RCC_BDCR_BASE)

/* 
Bits 31:17 Reserved, must be kept at reset value.

*/
#define RCC_BDCR_BDRST_Backup_domain_software_reset         16
/* 

Bit 16 BDRST: Backup domain software reset
Set and cleared by software.
0: Reset not activated
1: Resets the entire Backup domain

*/
#define RCC_BDCR_RTCEN_RTC_clock_enable             15
/* 

Bit 15 RTCEN: RTC clock enable
Set and cleared by software.
0: RTC clock disabled
1: RTC clock enabled

*/
/* 

Bits 14:10 Reserved, must be kept at reset value.

*/
#define RCC_BDCR_RTCSEL_RTC_clock_source_selection_BIT1             9
#define RCC_BDCR_RTCSEL_RTC_clock_source_selection_BIT0             8
/* 

Bits 9:8 RTCSEL[1:0]: RTC clock source selection
Set by software to select the clock source for the RTC. Once the RTC clock source has been
selected, it cannot be changed anymore unless the Backup domain is reset. The BDRST bit
can be used to reset them.
00: No clock
01: LSE oscillator clock used as RTC clock
10: LSI oscillator clock used as RTC clock
11: HSE oscillator clock divided by 128 used as RTC clock

*/
/* 

Bits 7:3 Reserved, must be kept at reset value
*/
#define RCC_BDCR_LSEBYP_External_low_speed_oscillator_bypass            2
/* 
Bit 2 LSEBYP: External low-speed oscillator bypass
Set and cleared by software to bypass oscillator in debug mode. This bit can be written only
when the external 32 kHz oscillator is disabled.
0: LSE oscillator not bypassed
1: LSE oscillator bypassed

*/
#define RCC_BDCR_LSERDY_External_low_speed_oscillator_ready             1
/* 

Bit 1 LSERDY: External low-speed oscillator ready
Set and cleared by hardware to indicate when the external 32 kHz oscillator is stable. After
the LSEON bit is cleared, LSERDY goes low after 6 external low-speed oscillator clock
cycles.
0: External 32 kHz oscillator not ready
1: External 32 kHz oscillator ready

*/
#define RCC_BDCR_LSEON_External_low_speed_oscillator_enable             0
/* 

Bit 0 LSEON: External low-speed oscillator enable
Set and cleared by software.
0: External 32 kHz oscillator OFF
1: External 32 kHz oscillator ON
*/

/* RCC_BDCR end */
//--------------------------------------




/* RCC_CSR start */

#define RCC_CSR_OFFSET 0x24
#define RCC_CSR_BASE (RCC_BASE+RCC_CSR_OFFSET)
#define RCC_CSR_REG (*(volatile u32 *)RCC_CSR_BASE)

#define RCC_CSR_LPWRRSTF_Low_power_reset_flag               31
/* 
Bit 31 LPWRRSTF: Low-power reset flag
Set by hardware when a Low-power management reset occurs.
Cleared by writing to the RMVF bit.
0: No Low-power management reset occurred
1: Low-power management reset occurred
For further information on Low-power management reset, refer to Low-power management
reset.

*/
#define RCC_CSR_WWDGRSTF_Window_watchdog_reset_flag                 30
/* 

Bit 30 WWDGRSTF: Window watchdog reset flag
Set by hardware when a window watchdog reset occurs.
Cleared by writing to the RMVF bit.
0: No window watchdog reset occurred
1: Window watchdog reset occurred

*/
#define RCC_CSR_IWDGRSTF_Independent_watchdog_reset_flag                29
/* 

Bit 29 IWDGRSTF: Independent watchdog reset flag
Set by hardware when an independent watchdog reset from VDD domain occurs.
Cleared by writing to the RMVF bit.
0: No watchdog reset occurred
1: Watchdog reset occurred

*/
#define RCC_CSR_SFTRSTF_Software_reset_flag         28
/* 

Bit 28 SFTRSTF: Software reset flag
Set by hardware when a software reset occurs.
Cleared by writing to the RMVF bit.
0: No software reset occurred
1: Software reset occurred

*/
#define RCC_CSR_PORRSTF_POR_PDR_reset_flag              27
/* 

Bit 27 PORRSTF: POR/PDR reset flag
Set by hardware when a POR/PDR reset occurs.
Cleared by writing to the RMVF bit.
0: No POR/PDR reset occurred
1: POR/PDR reset occurred

*/
#define RCC_CSR_PINRSTF_PIN_reset_flag              26
/* 

Bit 26 PINRSTF: PIN reset flag
Set by hardware when a reset from the NRST pin occurs.
Cleared by writing to the RMVF bit.
0: No reset from NRST pin occurred
1: Reset from NRST pin occurred

*/
/* 

Bit 25 Reserved, must be kept at reset value.

*/
#define RCC_CSR_RMVF_Remove_reset_flag                      24
/* 

Bit 24 RMVF: Remove reset flag
Set by software to clear the reset flags.
0: No effect
1: Clear the reset flags

*/
/* 

Bits 23:2 Reserved, must be kept at reset value.

*/
#define RCC_CSR_LSIRDY_Internal_low_speed_oscillator_ready                  1
/* 
Bit 1 LSIRDY: Internal low-speed oscillator ready
Set and cleared by hardware to indicate when the internal RC 40 kHz oscillator is stable.
After the LSION bit is cleared, LSIRDY goes low after 3 internal RC 40 kHz oscillator clock
cycles.
0: Internal RC 40 kHz oscillator not ready
1: Internal RC 40 kHz oscillator ready

*/
#define RCC_CSR_LSION_Internal_low_speed_oscillator_enable          0
/* 
Bit 0 LSION: Internal low-speed oscillator enable
Set and cleared by software.
0: Internal RC 40 kHz oscillator OFF
1: Internal RC 40 kHz oscillator ON
*/
/* RCC_CSR end */
//--------------------------------------


//--------------------------------------

#endif /* __HARDWARE_ARM_RCC_DRIVER_H__ */