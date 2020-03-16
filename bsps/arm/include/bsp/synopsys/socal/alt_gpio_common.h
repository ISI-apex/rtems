/**
 * @file
 *
 * @ingroup RTEMSBSPsARMCycVContrib
 */

/*******************************************************************************
*                                                                              *
* Copyright 2013 Altera Corporation. All Rights Reserved.                      *
*                                                                              *
* Redistribution and use in source and binary forms, with or without           *
* modification, are permitted provided that the following conditions are met:  *
*                                                                              *
* 1. Redistributions of source code must retain the above copyright notice,    *
*    this list of conditions and the following disclaimer.                     *
*                                                                              *
* 2. Redistributions in binary form must reproduce the above copyright notice, *
*    this list of conditions and the following disclaimer in the documentation *
*    and/or other materials provided with the distribution.                    *
*                                                                              *
* 3. The name of the author may not be used to endorse or promote products     *
*    derived from this software without specific prior written permission.     *
*                                                                              *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR *
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, ARE DISCLAIMED. IN NO  *
* EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,       *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, *
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;  *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,     *
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR      *
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF       *
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                   *
*                                                                              *
*******************************************************************************/

/* Altera - ALT_GPIO */

#ifndef __ALTERA_ALT_GPIO_COMMON_H__
#define __ALTERA_ALT_GPIO_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */

#include <bsp/hwinfo.h>
#include <bspopts.h>


#define ALT_HPS_ADDR        0

/* The byte offset of the ALT_GPIO_SWPORTA_DR register from the beginning of the component. */
#define ALT_GPIO_SWPORTA_DR_OFST        0x0
/* The address of the ALT_GPIO_SWPORTA_DR register. */
#define ALT_GPIO_SWPORTA_DR_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTA_DR_OFST))

/* The byte offset of the ALT_GPIO_SWPORTB_DR register from the beginning of the component. */
#define ALT_GPIO_SWPORTB_DR_OFST        0x0c
/* The address of the ALT_GPIO_SWPORTB_DR register. */
#define ALT_GPIO_SWPORTB_DR_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTB_DR_OFST))

/* The byte offset of the ALT_GPIO_SWPORTC_DR register from the beginning of the component. */
#define ALT_GPIO_SWPORTC_DR_OFST        0x18
/* The address of the ALT_GPIO_SWPORTC_DR register. */
#define ALT_GPIO_SWPORTC_DR_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTC_DR_OFST))

/* The byte offset of the ALT_GPIO_SWPORTD_DR register from the beginning of the component. */
#define ALT_GPIO_SWPORTD_DR_OFST        0x24
/* The address of the ALT_GPIO_SWPORTD_DR register. */
#define ALT_GPIO_SWPORTD_DR_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTD_DR_OFST))

/*
 * Register : Port A Data Direction Register - gpio_swporta_ddr
 *
 * This register establishes the direction of each corresponding GPIO Data Field
 * Bit.
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_A
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:----------------------------
 *  [x:0]   | RW     | z     | Port A Data Direction Field
 *  [31:y]  | ???    | 0x0   | *Reserved - read as zero*
 *
 * z = GPIO_DFLT_DIR_D==1 ? 1:0
 */
/*
 * Field : Port A Data Direction Field - gpio_swporta_ddr
 *
 * Values written to this register independently control the direction of the
 * corresponding data bit in the Port A Data Register.
 *
 * Field Enumeration Values:
 *
 *  Enum                                        | Value | Description
 * :--------------------------------------------|:------|:-----------------
 *  ALT_GPIO_SWPORTA_DDR_GPIO_SWPORTA_DDR_E_IN  | 0x0   | Input Direction
 *  ALT_GPIO_SWPORTA_DDR_GPIO_SWPORTA_DDR_E_OUT | 0x1   | Output Direction
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_SWPORTA_DDR_GPIO_SWPORTA_DDR
 *
 * Input Direction
 */
#define ALT_GPIO_SWPORTA_DDR_GPIO_SWPORTA_DDR_E_IN  0x0
/*
 * Enumerated value for register field ALT_GPIO_SWPORTA_DDR_GPIO_SWPORTA_DDR
 *
 * Output Direction
 */
#define ALT_GPIO_SWPORTA_DDR_GPIO_SWPORTA_DDR_E_OUT 0x1

/* The reset value of the ALT_GPIO_SWPORTA_DR_GPIO_SWPORTA_DR register field. */
#define ALT_GPIO_SWPORTA_DDR_GPIO_SWPORTA_DDR_RESET      0x0

/* The byte offset of the ALT_GPIO_SWPORTA_DDR register from the beginning of the component. */
#define ALT_GPIO_SWPORTA_DDR_OFST        0x4
/* The address of the ALT_GPIO_SWPORTA_DDR register. */
#define ALT_GPIO_SWPORTA_DDR_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTA_DDR_OFST))

/*
 * Register : Port A Data Source Register - gpio_swporta_ctl
 *
 * This register establishes the data and control source of each corresponding GPIO Data Field
 * Bit.
 *
 * The data and control source for a signal can come from either software or hardware;
 * this bit selects between them. The default source is configurable through the
 * GPIO_DFLT_DIR_A configuration parameter. If GPIO_PORTA_SINGLE_CTL = 0, the register will contain
 * one bit for each bit of the signal. Upon reset in this case, the value of GPIO_DFLT_SRC_A
 * is replicated across all bits of the signal so that all bits power up with the same operating mode.
 * Furthermore, the default source of each bit of the signal can subsequently be changed by writing
 * to the corresponding bit of this register. This register is not available unless GPIO_HW_PORTA = 1.
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_A
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:----------------------------
 *  [x:0]   | RW     | z     | Port A Data Source Control Field
 *  [31:y]  | ???    | 0x0   | *Reserved - read as zero*
 *
 * z = If GPIO_PORTA_SINGLE_CTL = 1, then the reset value is GPIO_DFLT_SRC_A.
 * If GPIO_PORTA_SINGLE_CTL = 0, then the reset value is {GPIO_PWIDTH_A{GPIO_DFLT_SRC_A in each bit}}.
 */
/*
 * Field : Port A Data Source Field - gpio_swporta_ctl
 *
 * Values written to this register independently control the data and control source of the
 * corresponding data bit in the Port A Data Register.
 *
 * Field Enumeration Values:
 *
 *  Enum                                              | Value | Description
 * :--------------------------------------------------|:------|:-----------------
 *  ALT_GPIO_SWPORTA_CTL_GPIO_SWPORTA_CTL_E_SOFTWARE  | 0x0   | Software Mode
 *  ALT_GPIO_SWPORTA_CTL_GPIO_SWPORTA_CTL_E_HARDWARE  | 0x1   | Hardware Mode
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_SWPORTA_CTL_GPIO_SWPORTA_CTL
 *
 * Software Mode
 */
#define ALT_GPIO_SWPORTA_CTL_GPIO_SWPORTA_CTL_E_SOFTWARE  0x0
/*
 * Enumerated value for register field ALT_GPIO_SWPORTA_CTL_GPIO_SWPORTA_CTL
 *
 * Hardware Mode
 */
#define ALT_GPIO_SWPORTA_CTL_GPIO_SWPORTA_CTL_E_HARDWARE 0x1

/* The byte offset of the ALT_GPIO_SWPORTA_CTL register from the beginning of the component. */
#define ALT_GPIO_SWPORTA_CTL_OFST        0x08
/* The address of the ALT_GPIO_SWPORTA_CTL register. */
#define ALT_GPIO_SWPORTA_CTL_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTA_CTL_OFST))

/* The byte offset of the ALT_GPIO_SWPORTB_DR register from the beginning of the component. */
#define ALT_GPIO_SWPORTB_DR_OFST        0x0c
/* The address of the ALT_GPIO_SWPORTB_DR register. */
#define ALT_GPIO_SWPORTB_DR_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTB_DR_OFST))

/* The byte offset of the ALT_GPIO_SWPORTB_DDR register from the beginning of the component. */
#define ALT_GPIO_SWPORTB_DDR_OFST        0x10
/* The address of the ALT_GPIO_SWPORTB_DDR register. */
#define ALT_GPIO_SWPORTB_DDR_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTB_DDR_OFST))

/* Enumerated values for register field ALT_GPIO_SWPORTB_CTL_GPIO_SWPORTB_CTL */
#define ALT_GPIO_SWPORTB_CTL_GPIO_SWPORTB_CTL_E_SOFTWARE  0x0
#define ALT_GPIO_SWPORTB_CTL_GPIO_SWPORTB_CTL_E_HARDWARE 0x1
/* The byte offset of the ALT_GPIO_SWPORTB_CTL register from the beginning of the component. */
#define ALT_GPIO_SWPORTB_CTL_OFST        0x14
/* The address of the ALT_GPIO_SWPORTB_CTL register. */
#define ALT_GPIO_SWPORTB_CTL_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTB_CTL_OFST))

/* The byte offset of the ALT_GPIO_SWPORTC_DR register from the beginning of the component. */
#define ALT_GPIO_SWPORTC_DR_OFST        0x18
/* The address of the ALT_GPIO_SWPORTC_DR register. */
#define ALT_GPIO_SWPORTC_DR_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTC_DR_OFST))

/* The byte offset of the ALT_GPIO_SWPORTC_DDR register from the beginning of the component. */
#define ALT_GPIO_SWPORTC_DDR_OFST        0x1c
/* The address of the ALT_GPIO_SWPORTC_DDR register. */
#define ALT_GPIO_SWPORTC_DDR_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTC_DDR_OFST))


/* Enumerated values for register field ALT_GPIO_SWPORTC_CTL_GPIO_SWPORTC_CTL */
#define ALT_GPIO_SWPORTC_CTL_GPIO_SWPORTC_CTL_E_SOFTWARE  0x0
#define ALT_GPIO_SWPORTC_CTL_GPIO_SWPORTC_CTL_E_HARDWARE 0x1
/* The byte offset of the ALT_GPIO_SWPORTC_CTL register from the beginning of the component. */
#define ALT_GPIO_SWPORTC_CTL_OFST        0x20
/* The address of the ALT_GPIO_SWPORTC_CTL register. */
#define ALT_GPIO_SWPORTC_CTL_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTC_CTL_OFST))

/* The byte offset of the ALT_GPIO_SWPORTD_DR register from the beginning of the component. */
#define ALT_GPIO_SWPORTD_DR_OFST        0x24
/* The address of the ALT_GPIO_SWPORTD_DR register. */
#define ALT_GPIO_SWPORTD_DR_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTD_DR_OFST))

/* The byte offset of the ALT_GPIO_SWPORTD_DDR register from the beginning of the component. */
#define ALT_GPIO_SWPORTD_DDR_OFST        0x28
/* The address of the ALT_GPIO_SWPORTD_DDR register. */
#define ALT_GPIO_SWPORTD_DDR_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTD_DDR_OFST))

/* Enumerated values for register field ALT_GPIO_SWPORTD_CTL_GPIO_SWPORTD_CTL */
#define ALT_GPIO_SWPORTD_CTL_GPIO_SWPORTD_CTL_E_SOFTWARE  0x0
#define ALT_GPIO_SWPORTD_CTL_GPIO_SWPORTD_CTL_E_HARDWARE 0x1
/* The byte offset of the ALT_GPIO_SWPORTD_CTL register from the beginning of the component. */
#define ALT_GPIO_SWPORTD_CTL_OFST        0x2c
/* The address of the ALT_GPIO_SWPORTD_CTL register. */
#define ALT_GPIO_SWPORTD_CTL_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_SWPORTD_CTL_OFST))

/*
 * Register : Interrupt Enable Register - gpio_inten
 *
 * The Interrupt enable register allows interrupts for each bit of the Port A data
 * register.
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_A
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:-----------------------
 *  [x:0]   | RW     | 0x0   | Interrupt Enable Field
 *  [31:y]  | R      | 0x0   | *Reserved - read as zero*
 *
 */
/*
 * Field : Interrupt Enable Field - gpio_inten
 *
 * Allows each bit of Port A Data Register to be configured for interrupt
 * capability. Interrupts are disabled on the corresponding bits of Port A Data
 * Register if the corresponding data direction register is set to Output.
 *
 * Field Enumeration Values:
 *
 *  Enum                            | Value | Description
 * :--------------------------------|:------|:----------------------------
 *  ALT_GPIO_INTEN_GPIO_INTEN_E_DIS | 0x0   | Disable Interrupt on Port A
 *  ALT_GPIO_INTEN_GPIO_INTEN_E_EN  | 0x1   | Enable Interrupt on Port A
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_INTEN_GPIO_INTEN
 *
 * Disable Interrupt on Port A
 */
#define ALT_GPIO_INTEN_GPIO_INTEN_E_DIS 0x0
/*
 * Enumerated value for register field ALT_GPIO_INTEN_GPIO_INTEN
 *
 * Enable Interrupt on Port A
 */
#define ALT_GPIO_INTEN_GPIO_INTEN_E_EN  0x1
/* The reset value of the ALT_GPIO_INTEN_GPIO_INTEN register field. */
#define ALT_GPIO_INTEN_GPIO_INTEN_RESET      0x0
/* The byte offset of the ALT_GPIO_INTEN register from the beginning of the component. */
#define ALT_GPIO_INTEN_OFST        0x30
/* The address of the ALT_GPIO_INTEN register. */
#define ALT_GPIO_INTEN_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_INTEN_OFST))

/*
 * Field : Interrupt Mask Field - gpio_intmask
 *
 * Controls whether an interrupt on Port A Data Register can generate an interrupt
 * to the interrupt controller by not masking it. The unmasked status can be read
 * as well as the resultant status after masking.
 *
 * Field Enumeration Values:
 *
 *  Enum                              | Value | Description
 * :----------------------------------|:------|:----------------------------
 *  ALT_GPIO_INTMSK_GPIO_INTMSK_E_DIS | 0x0   | Interrupt bits are unmasked
 *  ALT_GPIO_INTMSK_GPIO_INTMSK_E_EN  | 0x1   | Mask Interrupt
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_INTMSK_GPIO_INTMSK
 *
 * Interrupt bits are unmasked
 */
#define ALT_GPIO_INTMSK_GPIO_INTMSK_E_DIS   0x0
/*
 * Enumerated value for register field ALT_GPIO_INTMSK_GPIO_INTMSK
 *
 * Mask Interrupt
 */
#define ALT_GPIO_INTMSK_GPIO_INTMSK_E_EN    0x1

/* The reset value of the ALT_GPIO_INTMSK_GPIO_INTMSK register field. */
#define ALT_GPIO_INTMSK_GPIO_INTMSK_RESET      0x0

/* The byte offset of the ALT_GPIO_INTMSK register from the beginning of the component. */
#define ALT_GPIO_INTMSK_OFST        0x34
/*
 * Field : Interrupt Level Field - gpio_inttype_level
 *
 * This field controls the type of interrupt that can occur on the Port A Data
 * Register.
 *
 * Field Enumeration Values:
 *
 *  Enum                                              | Value | Description
 * :--------------------------------------------------|:------|:----------------
 *  ALT_GPIO_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_E_LEVEL | 0x0   | Level-sensitive
 *  ALT_GPIO_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_E_EDGE  | 0x1   | Edge-sensitive
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL
 *
 * Level-sensitive
 */
#define ALT_GPIO_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_E_LEVEL   0x0
/*
 * Enumerated value for register field ALT_GPIO_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL
 *
 * Edge-sensitive
 */
#define ALT_GPIO_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_E_EDGE    0x1
/* The reset value of the ALT_GPIO_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL register field. */
#define ALT_GPIO_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_RESET      0x0

/* The byte offset of the ALT_GPIO_INTTYPE_LEVEL register from the beginning of the component. */
#define ALT_GPIO_INTTYPE_LEVEL_OFST        0x38

/*
 * Field : Polarity Control Field - gpio_int_polarity
 *
 * Controls the polarity of edge or level sensitivity that can occur on input of
 * Port A Data Register.
 *
 * Field Enumeration Values:
 *
 *  Enum                                    | Value | Description
 * :----------------------------------------|:------|:------------
 *  ALT_GPIO_INT_POL_GPIO_INT_POL_E_ACTLOW  | 0x0   | Active low
 *  ALT_GPIO_INT_POL_GPIO_INT_POL_E_ACTHIGH | 0x1   | Active high
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_INT_POL_GPIO_INT_POL
 *
 * Active low
 */
#define ALT_GPIO_INT_POL_GPIO_INT_POL_E_ACTLOW  0x0
/*
 * Enumerated value for register field ALT_GPIO_INT_POL_GPIO_INT_POL
 *
 * Active high
 */
#define ALT_GPIO_INT_POL_GPIO_INT_POL_E_ACTHIGH 0x1
/* The reset value of the ALT_GPIO_INT_POL_GPIO_INT_POL register field. */
#define ALT_GPIO_INT_POL_GPIO_INT_POL_RESET      0x0

/* The byte offset of the ALT_GPIO_INT_POL register from the beginning of the component. */
#define ALT_GPIO_INT_POL_OFST        0x3c

/*
 * Field : Interrupt Status Field - gpio_intstatus
 *
 * Interrupt status of Port A Data Register.
 *
 * Field Enumeration Values:
 *
 *  Enum                                  | Value | Description
 * :--------------------------------------|:------|:------------
 *  ALT_GPIO_INTSTAT_GPIO_INTSTAT_E_INACT | 0x0   | Inactive
 *  ALT_GPIO_INTSTAT_GPIO_INTSTAT_E_ACT   | 0x1   | Active
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_INTSTAT_GPIO_INTSTAT
 *
 * Inactive
 */
#define ALT_GPIO_INTSTAT_GPIO_INTSTAT_E_INACT   0x0
/*
 * Enumerated value for register field ALT_GPIO_INTSTAT_GPIO_INTSTAT
 *
 * Active
 */
#define ALT_GPIO_INTSTAT_GPIO_INTSTAT_E_ACT     0x1
/* The reset value of the ALT_GPIO_INTSTAT_GPIO_INTSTAT register field. */
#define ALT_GPIO_INTSTAT_GPIO_INTSTAT_RESET      0x0

/* The byte offset of the ALT_GPIO_INTSTAT register from the beginning of the component. */
#define ALT_GPIO_INTSTAT_OFST        0x40
/*
 * Field : Raw Interrupt Status Field - gpio_raw_intstatus
 *
 * Raw interrupt of status of Port A Data Register (premasking bits)
 *
 * Field Enumeration Values:
 *
 *  Enum                                          | Value | Description
 * :----------------------------------------------|:------|:------------
 *  ALT_GPIO_RAW_INTSTAT_GPIO_RAW_INTSTAT_E_INACT | 0x0   | Inactive
 *  ALT_GPIO_RAW_INTSTAT_GPIO_RAW_INTSTAT_E_ACT   | 0x1   | Active
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_RAW_INTSTAT_GPIO_RAW_INTSTAT
 *
 * Inactive
 */
#define ALT_GPIO_RAW_INTSTAT_GPIO_RAW_INTSTAT_E_INACT   0x0
/*
 * Enumerated value for register field ALT_GPIO_RAW_INTSTAT_GPIO_RAW_INTSTAT
 *
 * Active
 */
#define ALT_GPIO_RAW_INTSTAT_GPIO_RAW_INTSTAT_E_ACT     0x1
/* The reset value of the ALT_GPIO_RAW_INTSTAT_GPIO_RAW_INTSTAT register field. */
#define ALT_GPIO_RAW_INTSTAT_GPIO_RAW_INTSTAT_RESET      0x0

/* The byte offset of the ALT_GPIO_RAW_INTSTAT register from the beginning of the component. */
#define ALT_GPIO_RAW_INTSTAT_OFST        0x44
/*
 * Field : gpio_debounce
 *
 * Controls whether an external signal that is the source of an interrupt needs to
 * be debounced to remove any spurious glitches. A signal must be valid for two
 * periods of an external clock (gpio_db_clk) before it is internally processed.
 *
 * Field Enumeration Values:
 *
 *  Enum                                  | Value | Description
 * :--------------------------------------|:------|:----------------
 *  ALT_GPIO_DEBOUNCE_GPIO_DEBOUNCE_E_DIS | 0x0   | No debounce
 *  ALT_GPIO_DEBOUNCE_GPIO_DEBOUNCE_E_EN  | 0x1   | Enable debounce
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_DEBOUNCE_GPIO_DEBOUNCE
 *
 * No debounce
 */
#define ALT_GPIO_DEBOUNCE_GPIO_DEBOUNCE_E_DIS   0x0
/*
 * Enumerated value for register field ALT_GPIO_DEBOUNCE_GPIO_DEBOUNCE
 *
 * Enable debounce
 */
#define ALT_GPIO_DEBOUNCE_GPIO_DEBOUNCE_E_EN    0x1
/* The reset value of the ALT_GPIO_DEBOUNCE_GPIO_DEBOUNCE register field. */
#define ALT_GPIO_DEBOUNCE_GPIO_DEBOUNCE_RESET      0x0

/* The byte offset of the ALT_GPIO_DEBOUNCE register from the beginning of the component. */
#define ALT_GPIO_DEBOUNCE_OFST        0x48
/*
 * Field : Clears Edge Interrupts Field - gpio_porta_eoi
 *
 * Controls the clearing of edge type interrupts from the Port A Data Register.
 *
 * Field Enumeration Values:
 *
 *  Enum                                      | Value | Description
 * :------------------------------------------|:------|:-------------------
 *  ALT_GPIO_PORTA_EOI_GPIO_PORTA_EOI_E_NOCLR | 0x0   | No interrupt clear
 *  ALT_GPIO_PORTA_EOI_GPIO_PORTA_EOI_E_CLR   | 0x1   | Clear interrupt
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_PORTA_EOI_GPIO_PORTA_EOI
 *
 * No interrupt clear
 */
#define ALT_GPIO_PORTA_EOI_GPIO_PORTA_EOI_E_NOCLR   0x0
/*
 * Enumerated value for register field ALT_GPIO_PORTA_EOI_GPIO_PORTA_EOI
 *
 * Clear interrupt
 */
#define ALT_GPIO_PORTA_EOI_GPIO_PORTA_EOI_E_CLR     0x1
/* The reset value of the ALT_GPIO_PORTA_EOI_GPIO_PORTA_EOI register field. */
#define ALT_GPIO_PORTA_EOI_GPIO_PORTA_EOI_RESET      0x0

/* The byte offset of the ALT_GPIO_PORTA_EOI register from the beginning of the component. */
#define ALT_GPIO_PORTA_EOI_OFST        0x4c
/* The reset value of the ALT_GPIO_EXT_PORTA_GPIO_EXT_PORTA register field. */
#define ALT_GPIO_EXT_PORTA_GPIO_EXT_PORTA_RESET      0x0

/* The byte offset of the ALT_GPIO_EXT_PORTA register from the beginning of the component. */
#define ALT_GPIO_EXT_PORTA_OFST        0x50
/* The byte offset of the ALT_GPIO_EXT_PORTB register from the beginning of the component. */
#define ALT_GPIO_EXT_PORTB_OFST        0x54
/* The byte offset of the ALT_GPIO_EXT_PORTC register from the beginning of the component. */
#define ALT_GPIO_EXT_PORTC_OFST        0x58
/* The byte offset of the ALT_GPIO_EXT_PORTD register from the beginning of the component. */
#define ALT_GPIO_EXT_PORTD_OFST        0x5c
/*
 * Field : Synchronization Level Field - gpio_ls_sync
 *
 * The level-sensitive interrupts is synchronized to pclk_intr.
 *
 * Field Enumeration Values:
 *
 *  Enum                                   | Value | Description
 * :---------------------------------------|:------|:--------------------------------
 *  ALT_GPIO_LS_SYNC_GPIO_LS_SYNC_E_NOSYNC | 0x0   | No synchronization to pclk_intr
 *  ALT_GPIO_LS_SYNC_GPIO_LS_SYNC_E_SYNC   | 0x1   | Synchronize to pclk_intr
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_LS_SYNC_GPIO_LS_SYNC
 *
 * No synchronization to pclk_intr
 */
#define ALT_GPIO_LS_SYNC_GPIO_LS_SYNC_E_NOSYNC  0x0
/*
 * Enumerated value for register field ALT_GPIO_LS_SYNC_GPIO_LS_SYNC
 *
 * Synchronize to pclk_intr
 */
#define ALT_GPIO_LS_SYNC_GPIO_LS_SYNC_E_SYNC    0x1

/* The byte offset of the ALT_GPIO_LS_SYNC register from the beginning of the component. */
#define ALT_GPIO_LS_SYNC_OFST        0x60
/* The reset value of the ALT_GPIO_ID_CODE_GPIO_ID_CODE register field. */
#define ALT_GPIO_ID_CODE_GPIO_ID_CODE_RESET      0x0

/* The byte offset of the ALT_GPIO_ID_CODE register from the beginning of the component. */
#define ALT_GPIO_ID_CODE_OFST        0x64

/*
 * Register : GPIO Version Register - gpio_ver_id_code
 *
 * GPIO Component Version
 *
 * Register Layout
 *
 *  Bits   | Access | Reset      | Description
 * :-------|:-------|:-----------|:------------------------------
 *  [31:0] | R      | 0x3230382a | ASCII Component Version Field
 *
 */
/*
 * Field : ASCII Component Version Field - gpio_ver_id_code
 *
 * ASCII value for each number in the version, followed by *. For example.
 * 32_30_31_2A represents the version 2.01
 *
 * Field Access Macros:
 *
 */
/* The Least Significant Bit (LSB) position of the ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE register field. */
#define ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE register field. */
#define ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE_MSB        31
/* The width in bits of the ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE register field. */
#define ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE_WIDTH      32
/* The mask used to set the ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE register field value. */
#define ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE_SET_MSK    0xffffffff
/* The mask used to clear the ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE register field value. */
#define ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE_CLR_MSK    0x00000000
/* The reset value of the ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE register field. */
#define ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE_RESET      0x3230382a
/* Extracts the ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE field value from a register. */
#define ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE_GET(value) (((value) & 0xffffffff) >> 0)
/* Produces a ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE register field value suitable for setting the register. */
#define ALT_GPIO_VER_ID_CODE_GPIO_VER_ID_CODE_SET(value) (((value) << 0) & 0xffffffff)

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO_VER_ID_CODE.
 */
struct ALT_GPIO_VER_ID_CODE_s
{
    const uint32_t  gpio_ver_id_code : 32;  /* ASCII Component Version Field */
};

/* The typedef declaration for register ALT_GPIO_VER_ID_CODE. */
typedef volatile struct ALT_GPIO_VER_ID_CODE_s  ALT_GPIO_VER_ID_CODE_t;
#endif  /* __ASSEMBLY__ */

/* The byte offset of the ALT_GPIO_VER_ID_CODE register from the beginning of the component. */
#define ALT_GPIO_VER_ID_CODE_OFST        0x6c
/* The address of the ALT_GPIO_VER_ID_CODE register. */
#define ALT_GPIO_VER_ID_CODE_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_VER_ID_CODE_OFST))

/*
 * Register : Configuration Register 2 - gpio_config_reg2
 *
 * Specifies the bit width of ports A-D.
 *   0x0 = 8 bits
 *   0x1 = 16 bits
 *   0x2 = 32 bits
 *   0x3 = Reserved
 *
 * This register is a read-only register that is present when the configuration parameter
 * GPIO_ADD_ENCODED_PARAMS is set to True.
 * If this configuration is set to False, then this register reads back 0.
 *
 * Register Layout
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:----------------------
 *  [4:0]   | R      | 0x?   | Port A Width
 *  [9:5]   | R      | 0x?   | Port B Width
 *  [14:10] | R      | 0x?   | Port C Width
 *  [19:15] | R      | 0x?   | Port D Width
 *  [31:20] | R      | 0x0   | *Reserved - read as zero*
 *
 */
/*
 * Field : Port A Width - encoded_id_pwidth_a
 *
 * Specifies the width of GPIO Port A.
 *   0x0 = 8 bits
 *   0x1 = 16 bits
 *   0x2 = 32 bits
 *   0x3 = Reserved
 *
 * Field Access Macros:
 *
 */
/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A_MSB        4
/* The width in bits of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A_WIDTH      5
/* The mask used to set the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A register field value. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A_SET_MSK    0x0000001f
/* The mask used to clear the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A register field value. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A_CLR_MSK    0xffffffe0
/* Extracts the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A field value from a register. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A_GET(value) (((value) & 0x0000001f) >> 0)
/* Produces a ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_A_SET(value) (((value) << 0) & 0x0000001f)
/*
 * Field : Port B Width - encoded_id_pwidth_b
 *
 * Specifies the width of GPIO Port B.
 *   0x0 = 8 bits
 *   0x1 = 16 bits
 *   0x2 = 32 bits
 *   0x3 = Reserved
 *
 * Field Access Macros:
 *
 */

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B_LSB        5
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B_MSB        9
/* The width in bits of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B_WIDTH      5
/* The mask used to set the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B register field value. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B_SET_MSK    0x000003e0
/* The mask used to clear the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B register field value. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B_CLR_MSK    0xfffffc1f
/* The reset value of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B_RESET      0x7
/* Extracts the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B field value from a register. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B_GET(value) (((value) & 0x000003e0) >> 5)
/* Produces a ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_B_SET(value) (((value) << 5) & 0x000003e0)
/*
 * Field : Port C Width - encoded_id_pwidth_c
 *
 * Specifies the width of GPIO Port C.
 *   0x0 = 8 bits
 *   0x1 = 16 bits
 *   0x2 = 32 bits
 *   0x3 = Reserved
 *
 *
 * Field Access Macros:
 *
 */

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C_LSB        10
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C_MSB        14
/* The width in bits of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C_WIDTH      5
/* The mask used to set the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C register field value. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C_SET_MSK    0x00007c00
/* The mask used to clear the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C register field value. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C_CLR_MSK    0xffff83ff
/* The reset value of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C_RESET      0x7
/* Extracts the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C field value from a register. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C_GET(value) (((value) & 0x00007c00) >> 10)
/* Produces a ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_C_SET(value) (((value) << 10) & 0x00007c00)
/*
 * Field : Port D Width - encoded_id_pwidth_d
 *
 * Specifies the width of GPIO Port D.
 *   0x0 = 8 bits
 *   0x1 = 16 bits
 *   0x2 = 32 bits
 *   0x3 = Reserved
 *
 * Field Access Macros:
 *
 */

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D_LSB        15
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D_MSB        19
/* The width in bits of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D_WIDTH      5
/* The mask used to set the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D register field value. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D_SET_MSK    0x000f8000
/* The mask used to clear the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D register field value. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D_CLR_MSK    0xfff07fff
/* The reset value of the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D register field. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D_RESET      0x7
/* Extracts the ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D field value from a register. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D_GET(value) (((value) & 0x000f8000) >> 15)
/* Produces a ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG2_ENC_ID_PWIDTH_D_SET(value) (((value) << 15) & 0x000f8000)

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO_CFG_REG2.
 */
struct ALT_GPIO_CFG_REG2_s
{
    const uint32_t  encoded_id_pwidth_a :  5;  /* Port A Width (less 1) */
    const uint32_t  encoded_id_pwidth_b :  5;  /* Port B Width (less 1) */
    const uint32_t  encoded_id_pwidth_c :  5;  /* Port C Width (less 1) */
    const uint32_t  encoded_id_pwidth_d :  5;  /* Port D Width (less 1) */
    uint32_t                            : 12;  /* *UNDEFINED* */
};

/* The typedef declaration for register ALT_GPIO_CFG_REG2. */
typedef volatile struct ALT_GPIO_CFG_REG2_s  ALT_GPIO_CFG_REG2_t;
#endif  /* __ASSEMBLY__ */

/* The byte offset of the ALT_GPIO_CFG_REG2 register from the beginning of the component. */
#define ALT_GPIO_CFG_REG2_OFST        0x70
/* The address of the ALT_GPIO_CFG_REG2 register. */
#define ALT_GPIO_CFG_REG2_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_CFG_REG2_OFST))


/*
 * Register : Configuration Register 1 - gpio_config_reg1
 *
 * Reports settings of various GPIO configuration parameters
 *
 * Register Layout
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:----------------------------------
 *  [1:0]   | R      | 0x2   | APB DATA WIDTH
 *  [3:2]   | R      | 0x0   | NUM PORTS
 *  [4]     | R      | 0x1   | PORT A SINGLE CTL
 *  [5]     | R      | 0x1   | PORT B SINGLE CTL
 *  [6]     | R      | 0x1   | PORT C SINGLE CTL
 *  [7]     | R      | 0x1   | PORT D SINGLE CTL
 *  [8]     | R      | 0x0   | HW PORTA
 *  [11:9]  | ???    | 0x0   | *UNDEFINED*
 *  [12]    | R      | 0x1   | Port A Interrupt Field
 *  [13]    | R      | 0x1   | Debounce Field
 *  [14]    | R      | 0x1   | Encoded GPIO Parameters Available
 *  [15]    | R      | 0x1   | ID Field
 *  [20:16] | R      | 0x1f  | Encoded ID Width Field
 *  [31:21] | ???    | 0x0   | *UNDEFINED*
 *
 */
/*
 * Field : APB DATA WIDTH - apb_data_width
 *
 * Fixed to support an ABP data bus width of 32-bits.
 *
 * Field Enumeration Values:
 *
 *  Enum                                           | Value | Description
 * :-----------------------------------------------|:------|:-------------------------
 *  ALT_GPIO_CFG_REG1_APB_DATA_WIDTH_E_WIDTH32BITS | 0x2   | APB Data Width = 32-bits
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_APB_DATA_WIDTH
 *
 * APB Data Width = 32-bits
 */
#define ALT_GPIO_CFG_REG1_APB_DATA_WIDTH_E_WIDTH32BITS  0x2

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_APB_DATA_WIDTH register field. */
#define ALT_GPIO_CFG_REG1_APB_DATA_WIDTH_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_APB_DATA_WIDTH register field. */
#define ALT_GPIO_CFG_REG1_APB_DATA_WIDTH_MSB        1
/* The width in bits of the ALT_GPIO_CFG_REG1_APB_DATA_WIDTH register field. */
#define ALT_GPIO_CFG_REG1_APB_DATA_WIDTH_WIDTH      2
/* The mask used to set the ALT_GPIO_CFG_REG1_APB_DATA_WIDTH register field value. */
#define ALT_GPIO_CFG_REG1_APB_DATA_WIDTH_SET_MSK    0x00000003
/* The mask used to clear the ALT_GPIO_CFG_REG1_APB_DATA_WIDTH register field value. */
#define ALT_GPIO_CFG_REG1_APB_DATA_WIDTH_CLR_MSK    0xfffffffc
/* The reset value of the ALT_GPIO_CFG_REG1_APB_DATA_WIDTH register field. */
#define ALT_GPIO_CFG_REG1_APB_DATA_WIDTH_RESET      0x2
/* Extracts the ALT_GPIO_CFG_REG1_APB_DATA_WIDTH field value from a register. */
#define ALT_GPIO_CFG_REG1_APB_DATA_WIDTH_GET(value) (((value) & 0x00000003) >> 0)
/* Produces a ALT_GPIO_CFG_REG1_APB_DATA_WIDTH register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_APB_DATA_WIDTH_SET(value) (((value) << 0) & 0x00000003)

/*
 * Field : NUM PORTS - num_ports
 *
 * Field Enumeration Values:
 *
 *  Enum                                     | Value | Description
 * :-----------------------------------------|:------|:-------------------------
 *  ALT_GPIO_CFG_REG1_NUM_PORTS_E_ONEPORTA   | 0x0   | Number of GPIO Ports = 1
 *  ALT_GPIO_CFG_REG1_NUM_PORTS_E_TWOPORTA   | 0x1   | Number of GPIO Ports = 2
 *  ALT_GPIO_CFG_REG1_NUM_PORTS_E_THREEPORTA | 0x2   | Number of GPIO Ports = 3
 *  ALT_GPIO_CFG_REG1_NUM_PORTS_E_FOURPORTA  | 0x3   | Number of GPIO Ports = 4
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_NUM_PORTS
 *
 */
#define ALT_GPIO_CFG_REG1_NUM_PORTS_E_ONEPORTA    0x0
#define ALT_GPIO_CFG_REG1_NUM_PORTS_E_TWOPORTA    0x1
#define ALT_GPIO_CFG_REG1_NUM_PORTS_E_THREEPORTA  0x2
#define ALT_GPIO_CFG_REG1_NUM_PORTS_E_FOURPORTA   0x3

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_NUM_PORTS register field. */
#define ALT_GPIO_CFG_REG1_NUM_PORTS_LSB        2
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_NUM_PORTS register field. */
#define ALT_GPIO_CFG_REG1_NUM_PORTS_MSB        3
/* The width in bits of the ALT_GPIO_CFG_REG1_NUM_PORTS register field. */
#define ALT_GPIO_CFG_REG1_NUM_PORTS_WIDTH      2
/* The mask used to set the ALT_GPIO_CFG_REG1_NUM_PORTS register field value. */
#define ALT_GPIO_CFG_REG1_NUM_PORTS_SET_MSK    0x0000000c
/* The mask used to clear the ALT_GPIO_CFG_REG1_NUM_PORTS register field value. */
#define ALT_GPIO_CFG_REG1_NUM_PORTS_CLR_MSK    0xfffffff3
/* The reset value of the ALT_GPIO_CFG_REG1_NUM_PORTS register field. */
#define ALT_GPIO_CFG_REG1_NUM_PORTS_RESET      0x0
/* Extracts the ALT_GPIO_CFG_REG1_NUM_PORTS field value from a register. */
#define ALT_GPIO_CFG_REG1_NUM_PORTS_GET(value) (((value) & 0x0000000c) >> 2)
/* Produces a ALT_GPIO_CFG_REG1_NUM_PORTS register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_NUM_PORTS_SET(value) (((value) << 2) & 0x0000000c)

/*
 * Field : PORT A SINGLE CTL - porta_single_ctl
 *
 * Indicates the mode of operation of Port A to be software controlled only.
 *
 * Field Enumeration Values:
 *
 *  Enum                                             | Value | Description
 * :-------------------------------------------------|:------|:-----------------------------------------
 *  ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL_E_SOFTCTLONLY | 0x1   | Software Enabled Individual Port Control
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL
 *
 * Software Enabled Individual Port Control
 */
#define ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL_E_SOFTCTLONLY    0x1

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL_LSB        4
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL_MSB        4
/* The width in bits of the ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL_WIDTH      1
/* The mask used to set the ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL register field value. */
#define ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL_SET_MSK    0x00000010
/* The mask used to clear the ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL register field value. */
#define ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL_CLR_MSK    0xffffffef
/* The reset value of the ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL_RESET      0x1
/* Extracts the ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL field value from a register. */
#define ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL_GET(value) (((value) & 0x00000010) >> 4)
/* Produces a ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_PORTA_SINGLE_CTL_SET(value) (((value) << 4) & 0x00000010)
/*
 * Field : PORT B SINGLE CTL - portb_single_ctl
 *
 * Indicates the mode of operation of Port B to be software controlled only.
 *
 * Field Enumeration Values:
 *
 *  Enum                                             | Value | Description
 * :-------------------------------------------------|:------|:-----------------------------------------
 *  ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL_E_SOFTCTLONLY | 0x1   | Software Enabled Individual Port Control
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL
 *
 * Software Enabled Individual Port Control
 */
#define ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL_E_SOFTCTLONLY    0x1

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL_LSB        5
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL_MSB        5
/* The width in bits of the ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL_WIDTH      1
/* The mask used to set the ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL register field value. */
#define ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL_SET_MSK    0x00000020
/* The mask used to clear the ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL register field value. */
#define ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL_CLR_MSK    0xffffffdf
/* The reset value of the ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL_RESET      0x1
/* Extracts the ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL field value from a register. */
#define ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL_GET(value) (((value) & 0x00000020) >> 5)
/* Produces a ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_PORTB_SINGLE_CTL_SET(value) (((value) << 5) & 0x00000020)
/*
 * Field : PORT C SINGLE CTL - portc_single_ctl
 *
 * Indicates the mode of operation of Port C to be software controlled only.
 *
 * Field Enumeration Values:
 *
 *  Enum                                             | Value | Description
 * :-------------------------------------------------|:------|:-----------------------------------------
 *  ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL_E_SOFTCTLONLY | 0x1   | Software Enabled Individual Port Control
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL
 *
 * Software Enabled Individual Port Control
 */
#define ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL_E_SOFTCTLONLY    0x1

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL_LSB        6
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL_MSB        6
/* The width in bits of the ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL_WIDTH      1
/* The mask used to set the ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL register field value. */
#define ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL_SET_MSK    0x00000040
/* The mask used to clear the ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL register field value. */
#define ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL_CLR_MSK    0xffffffbf
/* The reset value of the ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL_RESET      0x1
/* Extracts the ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL field value from a register. */
#define ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL_GET(value) (((value) & 0x00000040) >> 6)
/* Produces a ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_PORTC_SINGLE_CTL_SET(value) (((value) << 6) & 0x00000040)
/*
 * Field : PORT D SINGLE CTL - portd_single_ctl
 *
 * Indicates the mode of operation of Port D to be software controlled only.
 *
 * Field Enumeration Values:
 *
 *  Enum                                             | Value | Description
 * :-------------------------------------------------|:------|:-----------------------------------------
 *  ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL_E_SOFTCTLONLY | 0x1   | Software Enabled Individual Port Control
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL
 *
 * Software Enabled Individual Port Control
 */
#define ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL_E_SOFTCTLONLY    0x1

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL_LSB        7
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL_MSB        7
/* The width in bits of the ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL_WIDTH      1
/* The mask used to set the ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL register field value. */
#define ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL_SET_MSK    0x00000080
/* The mask used to clear the ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL register field value. */
#define ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL_CLR_MSK    0xffffff7f
/* The reset value of the ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL register field. */
#define ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL_RESET      0x1
/* Extracts the ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL field value from a register. */
#define ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL_GET(value) (((value) & 0x00000080) >> 7)
/* Produces a ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_PORTD_SINGLE_CTL_SET(value) (((value) << 7) & 0x00000080)

/*
 * Field : HW PORTA - hw_porta
 *
 * The value is fixed to enable Port A configuration to be controlled by software
 * only.
 *
 * Field Enumeration Values:
 *
 *  Enum                                     | Value | Description
 * :-----------------------------------------|:------|:---------------------------------------
 *  ALT_GPIO_CFG_REG1_HW_PORTA_E_PORTANOHARD | 0x0   | Software Configuration Control Enabled
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_HW_PORTA
 *
 * Software Configuration Control Enabled
 */
#define ALT_GPIO_CFG_REG1_HW_PORTA_E_PORTANOHARD    0x0

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_HW_PORTA register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTA_LSB        8
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_HW_PORTA register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTA_MSB        8
/* The width in bits of the ALT_GPIO_CFG_REG1_HW_PORTA register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTA_WIDTH      1
/* The mask used to set the ALT_GPIO_CFG_REG1_HW_PORTA register field value. */
#define ALT_GPIO_CFG_REG1_HW_PORTA_SET_MSK    0x00000100
/* The mask used to clear the ALT_GPIO_CFG_REG1_HW_PORTA register field value. */
#define ALT_GPIO_CFG_REG1_HW_PORTA_CLR_MSK    0xfffffeff
/* The reset value of the ALT_GPIO_CFG_REG1_HW_PORTA register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTA_RESET      0x0
/* Extracts the ALT_GPIO_CFG_REG1_HW_PORTA field value from a register. */
#define ALT_GPIO_CFG_REG1_HW_PORTA_GET(value) (((value) & 0x00000100) >> 8)
/* Produces a ALT_GPIO_CFG_REG1_HW_PORTA register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_HW_PORTA_SET(value) (((value) << 8) & 0x00000100)

/*
 * Field : HW PORTB - hw_portb
 *
 * The value is fixed to enable Port B configuration to be controlled by software
 * only.
 *
 * Field Enumeration Values:
 *
 *  Enum                                     | Value | Description
 * :-----------------------------------------|:------|:---------------------------------------
 *  ALT_GPIO_CFG_REG1_HW_PORTB_E_PORTBNOHARD | 0x0   | Software Configuration Control Enabled
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_HW_PORTB
 *
 * Software Configuration Control Enabled
 */
#define ALT_GPIO_CFG_REG1_HW_PORTB_E_PORTANOHARD    0x0

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_HW_PORTB register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTB_LSB        9
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_HW_PORTB register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTB_MSB        9
/* The width in bits of the ALT_GPIO_CFG_REG1_HW_PORTB register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTB_WIDTH      1
/* The mask used to set the ALT_GPIO_CFG_REG1_HW_PORTB register field value. */
#define ALT_GPIO_CFG_REG1_HW_PORTB_SET_MSK    0x00000200
/* The mask used to clear the ALT_GPIO_CFG_REG1_HW_PORTB register field value. */
#define ALT_GPIO_CFG_REG1_HW_PORTB_CLR_MSK    0xfffffdff
/* The reset value of the ALT_GPIO_CFG_REG1_HW_PORTB register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTB_RESET      0x0
/* Extracts the ALT_GPIO_CFG_REG1_HW_PORTB field value from a register. */
#define ALT_GPIO_CFG_REG1_HW_PORTB_GET(value) (((value) & 0x00000100) >> 9)
/* Produces a ALT_GPIO_CFG_REG1_HW_PORTB register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_HW_PORTB_SET(value) (((value) << 9) & 0x00000100)

/*
 * Field : HW PORTC - hw_portc
 *
 * The value is fixed to enable Port C configuration to be controlled by software
 * only.
 *
 * Field Enumeration Values:
 *
 *  Enum                                     | Value | Description
 * :-----------------------------------------|:------|:---------------------------------------
 *  ALT_GPIO_CFG_REG1_HW_PORTC_E_PORTCNOHARD | 0x0   | Software Configuration Control Enabled
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_HW_PORTC
 *
 * Software Configuration Control Enabled
 */
#define ALT_GPIO_CFG_REG1_HW_PORTC_E_PORTCNOHARD    0x0

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_HW_PORTC register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTC_LSB        10
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_HW_PORTC register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTC_MSB        10
/* The width in bits of the ALT_GPIO_CFG_REG1_HW_PORTC register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTC_WIDTH      1
/* The mask used to set the ALT_GPIO_CFG_REG1_HW_PORTC register field value. */
#define ALT_GPIO_CFG_REG1_HW_PORTC_SET_MSK    0x00000400
/* The mask used to clear the ALT_GPIO_CFG_REG1_HW_PORTC register field value. */
#define ALT_GPIO_CFG_REG1_HW_PORTC_CLR_MSK    0xfffffbff
/* The reset value of the ALT_GPIO_CFG_REG1_HW_PORTC register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTC_RESET      0x0
/* Extracts the ALT_GPIO_CFG_REG1_HW_PORTC field value from a register. */
#define ALT_GPIO_CFG_REG1_HW_PORTC_GET(value) (((value) & 0x00000100) >> 10)
/* Produces a ALT_GPIO_CFG_REG1_HW_PORTC register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_HW_PORTC_SET(value) (((value) << 10) & 0x00000100)

/*
 * Field : HW PORTD - hw_portd
 *
 * The value is fixed to enable Port C configuration to be controlled by software
 * only.
 *
 * Field Enumeration Values:
 *
 *  Enum                                     | Value | Description
 * :-----------------------------------------|:------|:---------------------------------------
 *  ALT_GPIO_CFG_REG1_HW_PORTD_E_PORTDNOHARD | 0x0   | Software Configuration Control Enabled
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_HW_PORTD
 *
 * Software Configuration Control Enabled
 */
#define ALT_GPIO_CFG_REG1_HW_PORTD_E_PORTDNOHARD    0x0

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_HW_PORTD register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTD_LSB        11
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_HW_PORTD register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTD_MSB        11
/* The width in bits of the ALT_GPIO_CFG_REG1_HW_PORTD register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTD_WIDTH      1
/* The mask used to set the ALT_GPIO_CFG_REG1_HW_PORTD register field value. */
#define ALT_GPIO_CFG_REG1_HW_PORTD_SET_MSK    0x00000800
/* The mask used to clear the ALT_GPIO_CFG_REG1_HW_PORTD register field value. */
#define ALT_GPIO_CFG_REG1_HW_PORTD_CLR_MSK    0xfffff7ff
/* The reset value of the ALT_GPIO_CFG_REG1_HW_PORTD register field. */
#define ALT_GPIO_CFG_REG1_HW_PORTD_RESET      0x0
/* Extracts the ALT_GPIO_CFG_REG1_HW_PORTD field value from a register. */
#define ALT_GPIO_CFG_REG1_HW_PORTD_GET(value) (((value) & 0x00000100) >> 11)
/* Produces a ALT_GPIO_CFG_REG1_HW_PORTD register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_HW_PORTD_SET(value) (((value) << 11) & 0x00000100)

/*
 * Field : Port A Interrupt Field - porta_intr
 *
 * The value of this field is fixed to allow interrupts on Port A.
 *
 * Field Enumeration Values:
 *
 *  Enum                                       | Value | Description
 * :-------------------------------------------|:------|:--------------------------
 *  ALT_GPIO_CFG_REG1_PORTA_INTR_E_PORTAINTERR | 0x1   | Port A Interrupts Enabled
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_PORTA_INTR
 *
 * Port A Interrupts Enabled
 */
#define ALT_GPIO_CFG_REG1_PORTA_INTR_E_PORTAINTERR  0x1

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_PORTA_INTR register field. */
#define ALT_GPIO_CFG_REG1_PORTA_INTR_LSB        12
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_PORTA_INTR register field. */
#define ALT_GPIO_CFG_REG1_PORTA_INTR_MSB        12
/* The width in bits of the ALT_GPIO_CFG_REG1_PORTA_INTR register field. */
#define ALT_GPIO_CFG_REG1_PORTA_INTR_WIDTH      1
/* The mask used to set the ALT_GPIO_CFG_REG1_PORTA_INTR register field value. */
#define ALT_GPIO_CFG_REG1_PORTA_INTR_SET_MSK    0x00001000
/* The mask used to clear the ALT_GPIO_CFG_REG1_PORTA_INTR register field value. */
#define ALT_GPIO_CFG_REG1_PORTA_INTR_CLR_MSK    0xffffefff
/* The reset value of the ALT_GPIO_CFG_REG1_PORTA_INTR register field. */
#define ALT_GPIO_CFG_REG1_PORTA_INTR_RESET      0x1
/* Extracts the ALT_GPIO_CFG_REG1_PORTA_INTR field value from a register. */
#define ALT_GPIO_CFG_REG1_PORTA_INTR_GET(value) (((value) & 0x00001000) >> 12)
/* Produces a ALT_GPIO_CFG_REG1_PORTA_INTR register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_PORTA_INTR_SET(value) (((value) << 12) & 0x00001000)

/*
 * Field : Debounce Field
 *
 * The value of this field is fixed to include debounce capability.
 *
 * Field Enumeration Values:
 *
 *  Enum                                       | Value | Description
 * :-------------------------------------------|:------|:--------------------------
 *  ALT_GPIO_CFG_REG1_DEBOUNCE_E_DEBOUNCE      | 0x1   | Debounce capability enabled
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_DEBOUNCE
 *
 * Debounce capability Enabled
 */
#define ALT_GPIO_CFG_REG1_DEBOUNCE_E_DEBOUNCE  0x1

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_DEBOUNCE register field. */
#define ALT_GPIO_CFG_REG1_DEBOUNCE_LSB        13
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_DEBOUNCE register field. */
#define ALT_GPIO_CFG_REG1_DEBOUNCE_MSB        13
/* The width in bits of the ALT_GPIO_CFG_REG1_DEBOUNCE register field. */
#define ALT_GPIO_CFG_REG1_DEBOUNCE_WIDTH      1
/* The mask used to set the ALT_GPIO_CFG_REG1_DEBOUNCE register field value. */
#define ALT_GPIO_CFG_REG1_DEBOUNCE_SET_MSK    0x00002000
/* The mask used to clear the ALT_GPIO_CFG_REG1_DEBOUNCE register field value. */
#define ALT_GPIO_CFG_REG1_DEBOUNCE_CLR_MSK    0xffffefff
/* The reset value of the ALT_GPIO_CFG_REG1_DEBOUNCE register field. */
#define ALT_GPIO_CFG_REG1_DEBOUNCE_RESET      0x1
/* Extracts the ALT_GPIO_CFG_REG1_DEBOUNCE field value from a register. */
#define ALT_GPIO_CFG_REG1_DEBOUNCE_GET(value) (((value) & 0x00001000) >> 13)
/* Produces a ALT_GPIO_CFG_REG1_DEBOUNCE register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_DEBOUNCE_SET(value) (((value) << 13) & 0x00001000)

/*
 * Field : Encoded GPIO Parameters Available - add_encoded_params
 *
 * Fixed to allow the indentification of the Designware IP component.
 *
 * Field Enumeration Values:
 *
 *  Enum                                            | Value | Description
 * :------------------------------------------------|:------|:--------------------------
 *  ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS_E_ADDENCPARAMS | 0x1   | Enable IP indentification
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS
 *
 * Enable IP indentification
 */
#define ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS_E_ADDENCPARAMS 0x1

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS register field. */
#define ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS_LSB        14
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS register field. */
#define ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS_MSB        14
/* The width in bits of the ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS register field. */
#define ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS_WIDTH      1
/* The mask used to set the ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS register field value. */
#define ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS_SET_MSK    0x00004000
/* The mask used to clear the ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS register field value. */
#define ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS_CLR_MSK    0xffffbfff
/* The reset value of the ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS register field. */
#define ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS_RESET      0x1
/* Extracts the ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS field value from a register. */
#define ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS_GET(value) (((value) & 0x00004000) >> 14)
/* Produces a ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_ADD_ENC_PARAMS_SET(value) (((value) << 14) & 0x00004000)

/*
 * Field : ID Field - gpio_id
 *
 * Provides an ID code value
 *
 * Field Enumeration Values:
 *
 *  Enum                               | Value | Description
 * :-----------------------------------|:------|:-------------
 *  ALT_GPIO_CFG_REG1_GPIO_ID_E_IDCODE | 0x1   | GPIO ID Code
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_GPIO_ID
 *
 * GPIO ID Code
 */
#define ALT_GPIO_CFG_REG1_GPIO_ID_E_IDCODE  0x1

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_GPIO_ID register field. */
#define ALT_GPIO_CFG_REG1_GPIO_ID_LSB        15
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_GPIO_ID register field. */
#define ALT_GPIO_CFG_REG1_GPIO_ID_MSB        15
/* The width in bits of the ALT_GPIO_CFG_REG1_GPIO_ID register field. */
#define ALT_GPIO_CFG_REG1_GPIO_ID_WIDTH      1
/* The mask used to set the ALT_GPIO_CFG_REG1_GPIO_ID register field value. */
#define ALT_GPIO_CFG_REG1_GPIO_ID_SET_MSK    0x00008000
/* The mask used to clear the ALT_GPIO_CFG_REG1_GPIO_ID register field value. */
#define ALT_GPIO_CFG_REG1_GPIO_ID_CLR_MSK    0xffff7fff
/* The reset value of the ALT_GPIO_CFG_REG1_GPIO_ID register field. */
#define ALT_GPIO_CFG_REG1_GPIO_ID_RESET      0x1
/* Extracts the ALT_GPIO_CFG_REG1_GPIO_ID field value from a register. */
#define ALT_GPIO_CFG_REG1_GPIO_ID_GET(value) (((value) & 0x00008000) >> 15)
/* Produces a ALT_GPIO_CFG_REG1_GPIO_ID register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_GPIO_ID_SET(value) (((value) << 15) & 0x00008000)

/*
 * Field : Encoded ID Width Field - encoded_id_width
 *
 * This value is fixed at 32 bits.
 *
 * Field Enumeration Values:
 *
 *  Enum                                        | Value | Description
 * :--------------------------------------------|:------|:------------------
 *  ALT_GPIO_CFG_REG1_ENC_ID_WIDTH_E_ENCIDWIDTH | 0x1f  | Width of ID Field
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field ALT_GPIO_CFG_REG1_ENC_ID_WIDTH
 *
 * Width of ID Field
 */
#define ALT_GPIO_CFG_REG1_ENC_ID_WIDTH_E_ENCIDWIDTH 0x1f

/* The Least Significant Bit (LSB) position of the ALT_GPIO_CFG_REG1_ENC_ID_WIDTH register field. */
#define ALT_GPIO_CFG_REG1_ENC_ID_WIDTH_LSB        16
/* The Most Significant Bit (MSB) position of the ALT_GPIO_CFG_REG1_ENC_ID_WIDTH register field. */
#define ALT_GPIO_CFG_REG1_ENC_ID_WIDTH_MSB        20
/* The width in bits of the ALT_GPIO_CFG_REG1_ENC_ID_WIDTH register field. */
#define ALT_GPIO_CFG_REG1_ENC_ID_WIDTH_WIDTH      5
/* The mask used to set the ALT_GPIO_CFG_REG1_ENC_ID_WIDTH register field value. */
#define ALT_GPIO_CFG_REG1_ENC_ID_WIDTH_SET_MSK    0x001f0000
/* The mask used to clear the ALT_GPIO_CFG_REG1_ENC_ID_WIDTH register field value. */
#define ALT_GPIO_CFG_REG1_ENC_ID_WIDTH_CLR_MSK    0xffe0ffff
/* The reset value of the ALT_GPIO_CFG_REG1_ENC_ID_WIDTH register field. */
#define ALT_GPIO_CFG_REG1_ENC_ID_WIDTH_RESET      0x1f
/* Extracts the ALT_GPIO_CFG_REG1_ENC_ID_WIDTH field value from a register. */
#define ALT_GPIO_CFG_REG1_ENC_ID_WIDTH_GET(value) (((value) & 0x001f0000) >> 16)
/* Produces a ALT_GPIO_CFG_REG1_ENC_ID_WIDTH register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_ENC_ID_WIDTH_SET(value) (((value) << 16) & 0x001f0000)

/*
 * Field : INTERRUPT_BOTH_EDGE_TYPE Field - interrupt_both_edge_type
 *
 * Field Enumeration Values:
 *
 *  Enum                                                            | Value | Description
 * :----------------------------------------------------------------|:------|:------------------
 *  ALT_GPIO_CFG_REG1_INTERRUPT_BOTH_EDGE_TYPE_E_RISING_OR_FALLING  | 0x0   | Rising or Falling
 *  ALT_GPIO_CFG_REG1_INTERRUPT_BOTH_EDGE_TYPE_E_RISING_AND_FALLING | 0x1   | Rising and Falling
 *
 * Field Access Macros:
 *
 */
/*
 * Enumerated value for register field INTERRUPT_BOTH_EDGE_TYPE
 *
 * Interrupt generation on rising or falling edge or interrupt generation on rising and falling edge
 */
#define ALT_GPIO_CFG_REG1_INTERRUPT_BOTH_EDGE_TYPE_E_RISING_OR_FALLING 0x0
#define ALT_GPIO_CFG_REG1_INTERRUPT_BOTH_EDGE_TYPE_E_RISING_AND_FALLING 0x1

/* The Least Significant Bit (LSB) position of the INTERRUPT_BOTH_EDGE_TYPE register field. */
#define ALT_GPIO_CFG_REG1_INTERRUPT_BOTH_EDGE_TYPE_LSB        21
/* The Most Significant Bit (MSB) position of the INTERRUPT_BOTH_EDGE_TYPE register field. */
#define ALT_GPIO_CFG_REG1_INTERRUPT_BOTH_EDGE_TYPE_MSB        21
/* The width in bits of the INTERRUPT_BOTH_EDGE_TYPE register field. */
#define ALT_GPIO_CFG_REG1_INTERRUPT_BOTH_EDGE_TYPE_WIDTH      1
/* The mask used to set the INTERRUPT_BOTH_EDGE_TYPE register field value. */
#define ALT_GPIO_CFG_REG1_INTERRUPT_BOTH_EDGE_TYPE_SET_MSK    0x00200000
/* The mask used to clear the INTERRUPT_BOTH_EDGE_TYPE register field value. */
#define ALT_GPIO_CFG_REG1_INTERRUPT_BOTH_EDGE_TYPE_CLR_MSK    0xffdfffff
/* Extracts the INTERRUPT_BOTH_EDGE_TYPE field value from a register. */
#define ALT_GPIO_CFG_REG1_INTERRUPT_BOTH_EDGE_TYPE_GET(value) (((value) & 0x00200000) >> 21)
/* Produces a INTERRUPT_BOTH_EDGE_TYPE register field value suitable for setting the register. */
#define ALT_GPIO_CFG_REG1_INTERRUPT_BOTH_EDGE_TYPE_SET(value) (((value) << 21) & 0x00200000)

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO_CFG_REG1.
 */
struct ALT_GPIO_CFG_REG1_s
{
    const uint32_t  apb_data_width     :  2;  /* APB DATA WIDTH */
    const uint32_t  num_ports          :  2;  /* NUM PORTS */
    const uint32_t  porta_single_ctl   :  1;  /* PORT A SINGLE CTL */
    const uint32_t  portb_single_ctl   :  1;  /* PORT B SINGLE CTL */
    const uint32_t  portc_single_ctl   :  1;  /* PORT C SINGLE CTL */
    const uint32_t  portd_single_ctl   :  1;  /* PORT D SINGLE CTL */
    const uint32_t  hw_porta           :  1;  /* HW PORTA */
    uint32_t                           :  3;  /* *UNDEFINED* */
    const uint32_t  porta_intr         :  1;  /* Port A Interrupt Field */
    const uint32_t  debounce           :  1;  /* Debounce Field */
    const uint32_t  add_encoded_params :  1;  /* Encoded GPIO Parameters Available */
    const uint32_t  gpio_id            :  1;  /* ID Field */
    const uint32_t  encoded_id_width   :  5;  /* Encoded ID Width Field */
    uint32_t                           : 11;  /* *UNDEFINED* */
};

/* The typedef declaration for register ALT_GPIO_CFG_REG1. */
typedef volatile struct ALT_GPIO_CFG_REG1_s  ALT_GPIO_CFG_REG1_t;
#endif  /* __ASSEMBLY__ */

/* The byte offset of the ALT_GPIO_CFG_REG1 register from the beginning of the component. */
#define ALT_GPIO_CFG_REG1_OFST        0x74
/* The address of the ALT_GPIO_CFG_REG1 register. */
#define ALT_GPIO_CFG_REG1_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_CFG_REG1_OFST))


#ifndef __ASSEMBLY__
/* The struct declaration for the raw register contents of register group ALT_GPIO. */
struct ALT_GPIO_raw_s
{
    volatile uint32_t  gpio_swporta_dr;     /* ALT_GPIO_SWPORTA_DR */
    volatile uint32_t  gpio_swporta_ddr;    /* ALT_GPIO_SWPORTA_DDR */
#if 0
    volatile uint32_t  _pad_0x8_0x2f[10];   /* *UNDEFINED* */
#else
    volatile uint32_t  gpio_swporta_ctl;    /* ALT_GPIO_SWPORTA_CTL */
    volatile uint32_t  gpio_swportb_dr;     /* ALT_GPIO_SWPORTB_DR */
    volatile uint32_t  gpio_swportb_ddr;    /* ALT_GPIO_SWPORTB_DDR */
    volatile uint32_t  gpio_swportb_ctl;    /* ALT_GPIO_SWPORTB_CTL */
    volatile uint32_t  gpio_swportc_dr;     /* ALT_GPIO_SWPORTC_DR */
    volatile uint32_t  gpio_swportc_ddr;    /* ALT_GPIO_SWPORTC_DDR */
    volatile uint32_t  gpio_swportc_ctl;    /* ALT_GPIO_SWPORTC_CTL */
    volatile uint32_t  gpio_swportd_dr;     /* ALT_GPIO_SWPORTD_DR */
    volatile uint32_t  gpio_swportd_ddr;    /* ALT_GPIO_SWPORTD_DDR */
    volatile uint32_t  gpio_swportd_ctl;    /* ALT_GPIO_SWPORTD_CTL */
#endif
    volatile uint32_t  gpio_inten;          /* ALT_GPIO_INTEN */
    volatile uint32_t  gpio_intmask;        /* ALT_GPIO_INTMSK */
    volatile uint32_t  gpio_inttype_level;  /* ALT_GPIO_INTTYPE_LEVEL */
    volatile uint32_t  gpio_int_polarity;   /* ALT_GPIO_INT_POL */
    volatile uint32_t  gpio_intstatus;      /* ALT_GPIO_INTSTAT */
    volatile uint32_t  gpio_raw_intstatus;  /* ALT_GPIO_RAW_INTSTAT */
    volatile uint32_t  gpio_debounce;       /* ALT_GPIO_DEBOUNCE */
    volatile uint32_t  gpio_porta_eoi;      /* ALT_GPIO_PORTA_EOI */
    volatile uint32_t  gpio_ext_porta;      /* ALT_GPIO_EXT_PORTA */
#if 0
    volatile uint32_t  _pad_0x54_0x5f[3];   /* *UNDEFINED* */
#else
    volatile uint32_t  gpio_ext_portb;      /* ALT_GPIO_EXT_PORTB */
    volatile uint32_t  gpio_ext_portc;      /* ALT_GPIO_EXT_PORTC */
    volatile uint32_t  gpio_ext_portd;      /* ALT_GPIO_EXT_PORTD */
#endif
    volatile uint32_t  gpio_ls_sync;        /* ALT_GPIO_LS_SYNC */
    volatile uint32_t  gpio_id_code;        /* ALT_GPIO_ID_CODE */
#if 0
    volatile uint32_t  _pad_0x68_0x6b;      /* *UNDEFINED* */
#else
    volatile uint32_t gpio_int_bothedge;    /* ALT_GPIO_INT_BOTHEDGE */
#endif
    volatile uint32_t  gpio_ver_id_code;    /* ALT_GPIO_VER_ID_CODE */
    volatile uint32_t  gpio_config_reg2;    /* ALT_GPIO_CFG_REG2 */
    volatile uint32_t  gpio_config_reg1;    /* ALT_GPIO_CFG_REG1 */
    volatile uint32_t  _pad_0x78_0x80[2];   /* *UNDEFINED* */
};

/* The typedef declaration for the raw register contents of register group ALT_GPIO. */
typedef volatile struct ALT_GPIO_raw_s  ALT_GPIO_raw_t;
#endif  /* __ASSEMBLY__ */

#ifdef __cplusplus
}
#endif  /* __cplusplus */
#endif  /* __ALTERA_ALT_GPIO_COMMON_H__ */

