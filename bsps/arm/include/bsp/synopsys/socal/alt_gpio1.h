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

#ifndef __ALTERA_ALT_GPIO1_H__
#define __ALTERA_ALT_GPIO1_H__

#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */

#include <bsp/hwinfo.h>
#include <bspopts.h>
#include "alt_gpio_common.h"

/*
 * Component Instance : gpio1
 *
 * Instance gpio1 of component ALT_GPIO.
 *
 *
 */
/* The address of the ALT_GPIO_SWPORTA_DR register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_SWPORTA_DR_ADDR  ALT_GPIO_SWPORTA_DR_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_SWPORTA_DDR register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_SWPORTA_DDR_ADDR  ALT_GPIO_SWPORTA_DDR_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_INTEN register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_INTEN_ADDR  ALT_GPIO_INTEN_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_INTMSK register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_INTMSK_ADDR  ALT_GPIO_INTMSK_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_INTTYPE_LEVEL register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_INTTYPE_LEVEL_ADDR  ALT_GPIO_INTTYPE_LEVEL_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_INT_POL register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_INT_POL_ADDR  ALT_GPIO_INT_POL_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_INTSTAT register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_INTSTAT_ADDR  ALT_GPIO_INTSTAT_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_RAW_INTSTAT register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_RAW_INTSTAT_ADDR  ALT_GPIO_RAW_INTSTAT_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_DEBOUNCE register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_DEBOUNCE_ADDR  ALT_GPIO_DEBOUNCE_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_PORTA_EOI register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_PORTA_EOI_ADDR  ALT_GPIO_PORTA_EOI_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_EXT_PORTA register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_EXT_PORTA_ADDR  ALT_GPIO_EXT_PORTA_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_LS_SYNC register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_LS_SYNC_ADDR  ALT_GPIO_LS_SYNC_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_ID_CODE register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_ID_CODE_ADDR  ALT_GPIO_ID_CODE_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_VER_ID_CODE register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_VER_ID_CODE_ADDR  ALT_GPIO_VER_ID_CODE_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_CFG_REG2 register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_CFG_REG2_ADDR  ALT_GPIO_CFG_REG2_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_CFG_REG1 register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_CFG_REG1_ADDR  ALT_GPIO_CFG_REG1_ADDR(ALT_GPIO1_ADDR)

/* The address of the ALT_GPIO_SWPORTB_DR register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_SWPORTB_DR_ADDR  ALT_GPIO_SWPORTB_DR_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_SWPORTB_DDR register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_SWPORTB_DDR_ADDR  ALT_GPIO_SWPORTB_DDR_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_EXT_PORTB register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_EXT_PORTB_ADDR  ALT_GPIO_EXT_PORTB_ADDR(ALT_GPIO1_ADDR)

/* The address of the ALT_GPIO_SWPORTC_DR register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_SWPORTC_DR_ADDR  ALT_GPIO_SWPORTC_DR_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_SWPORTC_DDR register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_SWPORTC_DDR_ADDR  ALT_GPIO_SWPORTC_DDR_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_EXT_PORTC register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_EXT_PORTC_ADDR  ALT_GPIO_EXT_PORTC_ADDR(ALT_GPIO1_ADDR)

/* The address of the ALT_GPIO_SWPORTD_DR register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_SWPORTD_DR_ADDR  ALT_GPIO_SWPORTD_DR_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_SWPORTD_DDR register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_SWPORTD_DDR_ADDR  ALT_GPIO_SWPORTD_DDR_ADDR(ALT_GPIO1_ADDR)
/* The address of the ALT_GPIO_EXT_PORTD register for the ALT_GPIO1 instance. */
#define ALT_GPIO1_EXT_PORTD_ADDR  ALT_GPIO_EXT_PORTD_ADDR(ALT_GPIO1_ADDR)

/* The base address byte offset for the start of the ALT_GPIO1 component. */
#define ALT_GPIO1_OFST        GPIO1_BASE
/* The start address of the ALT_GPIO1 component. */
#define ALT_GPIO1_ADDR        ALT_CAST(void *, (ALT_CAST(char *, ALT_HPS_ADDR) + ALT_GPIO1_OFST))
/* The lower bound address range of the ALT_GPIO1 component. */
#define ALT_GPIO1_LB_ADDR     ALT_GPIO1_ADDR
/* The upper bound address range of the ALT_GPIO1 component. */
#define ALT_GPIO1_UB_ADDR     ALT_CAST(void *, ((ALT_CAST(char *, ALT_GPIO1_ADDR) + 0x80) - 1))

/*
 * Component : GPIO Module - ALT_GPIO
 * GPIO Module
 *
 * Registers in the GPIO module
 *
 */
/*
 * Register : Port A Data Register - gpio_swporta_dr
 *
 * This GPIO Data register is used to input or output data
 *
 * Check the GPIO chapter in the handbook for details on how GPIO is implemented.
 *
 * Register Layout
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:------------
 *  [31:0]  | RW     | 0x0   | Port A Data
  *
 */
/*
 * Field : Port A Data - gpio_swporta_dr
 *
 * Values written to this register are output on the I/O signals of the GPIO Data
 * Register, if the corresponding data direction bits for GPIO Data Direction Field
 * are set to Output mode. The value read back is equal to the last value written
 * to this register.
 *
 * Check the GPIO chapter in the handbook for details on how GPIO is implemented.
 *
 * Field Access Macros:
 *
 */
/* The Least Significant Bit (LSB) position of the ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR register field. */
#define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR register field. */
#define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR register field. */
#define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_WIDTH      GPIO1_PWIDTH_A
#if GPIO1_PWIDTH_A == 8
  /* The mask used to set the ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR register field value. */
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR register field value. */
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR field value from a register. */
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR register field value suitable for setting the register. */
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_A == 16
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_SET_MSK    0xffff
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_CLR_MSK    0x0000
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_A == 32
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_SET_MSK    0xffffffff
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_CLR_MSK    0x00000000
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_SET_MSK    0xffffffff
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_CLR_MSK    0x00000000
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_SWPORTA_DR_GPIO_SWPORTA_DR_SET(value) (((value) << 0) & 0xffffffff)
#endif
/* The reset value of the ALT_GPIO_SWPORTA_DR_GPIO_SWPORTA_DR register field. */
#define ALT_GPIO_SWPORTA_DR_GPIO_SWPORTA_DR_RESET      0x0

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_SWPORTA_DR.
 */
struct ALT_GPIO1_SWPORTA_DR_s
{
  uint32_t gpio_swporta_dr : GPIO1_PWIDTH_A;      /* Port A Data */
  uint32_t                 : (32-GPIO1_PWIDTH_A); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_SWPORTA_DR. */
typedef volatile struct ALT_GPIO1_SWPORTA_DR_s  ALT_GPIO1_SWPORTA_DR_t;

struct ALT_GPIO1_SWPORTB_DR_s
{
  uint32_t gpio_swportb_dr : GPIO1_PWIDTH_B;      /* Port B Data */
  uint32_t                 : (32-GPIO1_PWIDTH_B); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_SWPORTB_DR. */
typedef volatile struct ALT_GPIO1_SWPORTB_DR_s  ALT_GPIO1_SWPORTB_DR_t;

struct ALT_GPIO1_SWPORTC_DR_s
{
  uint32_t gpio_swportc_dr : GPIO1_PWIDTH_C;      /* Port C Data */
  uint32_t                 : (32-GPIO1_PWIDTH_C); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_SWPORTC_DR. */
typedef volatile struct ALT_GPIO1_SWPORTC_DR_s  ALT_GPIO1_SWPORTC_DR_t;

struct ALT_GPIO1_SWPORTD_DR_s
{
  uint32_t gpio_swportd_dr : GPIO1_PWIDTH_D;      /* Port D Data */
  uint32_t                 : (32-GPIO1_PWIDTH_D); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_SWPORTD_DR. */
typedef volatile struct ALT_GPIO1_SWPORTD_DR_s  ALT_GPIO1_SWPORTD_DR_t;

#endif  /* __ASSEMBLY__ */

/* The Least Significant Bit (LSB) position of the ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR register field. */
#define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR register field. */
#define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR register field. */
#define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_WIDTH      GPIO1_PWIDTH_A
#if GPIO1_PWIDTH_A == 8
  /* The mask used to set the ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR register field value. */
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR register field value. */
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR field value from a register. */
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR register field value suitable for setting the register. */
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_A == 16
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_SET_MSK    0xffff
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_CLR_MSK    0x0000
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_A == 32
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_SET_MSK    0xffffffff
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_CLR_MSK    0x00000000
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_SET_MSK    0xffffffff
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_CLR_MSK    0x00000000
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_SWPORTA_DDR_GPIO_SWPORTA_DDR_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_SWPORTA_DDR.
 */
struct ALT_GPIO1_SWPORTA_DDR_s
{
    uint32_t gpio_swporta_ddr : GPIO1_PWIDTH_A;      /* Port A Data Direction*/
    uint32_t                  : (32-GPIO1_PWIDTH_A); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_SWPORTA_DDR. */
typedef volatile struct ALT_GPIO1_SWPORTA_DDR_s  ALT_GPIO1_SWPORTA_DDR_t;
#endif  /* __ASSEMBLY__ */

/* The Least Significant Bit (LSB) position of the ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR register field. */
#define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR register field. */
#define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_MSB        (GPIO0_PWIDTH_B-1)
/* The width in bits of the ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR register field. */
#define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_WIDTH      GPIO0_PWIDTH_B
#if GPIO0_PWIDTH_B == 8
  /* The mask used to set the ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR register field value. */
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR register field value. */
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR field value from a register. */
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR register field value suitable for setting the register. */
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_SET(value) (((value) << 0) & 0xff)
#elif GPIO0_PWIDTH_B == 16
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_SET_MSK    0xffff
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_CLR_MSK    0x0000
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_SET(value) (((value) << 0) & 0xffff)
#elif GPIO0_PWIDTH_B == 32
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_SET_MSK    0xffffffff
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_CLR_MSK    0x00000000
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_SET_MSK    0xffffffff
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_CLR_MSK    0x00000000
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_SWPORTB_DDR_GPIO_SWPORTB_DDR_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_SWPORTB_DDR.
 */
struct ALT_GPIO1_SWPORTB_DDR_s
{
    uint32_t gpio_SWPORTB_DDR : GPIO0_PWIDTH_B;      /* Port B Data Direction*/
    uint32_t                  : (32-GPIO0_PWIDTH_B); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_SWPORTB_DDR. */
typedef volatile struct ALT_GPIO1_SWPORTB_DDR_s  ALT_GPIO1_SWPORTB_DDR_t;
#endif  /* __ASSEMBLY__ */

/* The Least Significant Bit (LSB) position of the ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR register field. */
#define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR register field. */
#define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_MSB        (GPIO0_PWIDTH_C-1)
/* The width in bits of the ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR register field. */
#define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_WIDTH      GPIO0_PWIDTH_C
#if GPIO0_PWIDTH_C == 8
  /* The mask used to set the ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR register field value. */
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR register field value. */
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR field value from a register. */
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR register field value suitable for setting the register. */
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_SET(value) (((value) << 0) & 0xff)
#elif GPIO0_PWIDTH_C == 16
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_SET_MSK    0xffff
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_CLR_MSK    0x0000
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_SET(value) (((value) << 0) & 0xffff)
#elif GPIO0_PWIDTH_C == 32
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_SET_MSK    0xffffffff
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_CLR_MSK    0x00000000
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_SET_MSK    0xffffffff
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_CLR_MSK    0x00000000
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_SWPORTC_DDR_GPIO_SWPORTC_DDR_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_SWPORTC_DDR.
 */
struct ALT_GPIO1_SWPORTC_DDR_s
{
    uint32_t gpio_SWPORTC_DDR : GPIO0_PWIDTH_C;      /* Port B Data Direction*/
    uint32_t                  : (32-GPIO0_PWIDTH_C); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_SWPORTC_DDR. */
typedef volatile struct ALT_GPIO1_SWPORTC_DDR_s  ALT_GPIO1_SWPORTC_DDR_t;
#endif  /* __ASSEMBLY__ */

/* The Least Significant Bit (LSB) position of the ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR register field. */
#define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR register field. */
#define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_MSB        (GPIO0_PWIDTH_D-1)
/* The width in bits of the ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR register field. */
#define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_WIDTH      GPIO0_PWIDTH_D
#if GPIO0_PWIDTH_D == 8
  /* The mask used to set the ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR register field value. */
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR register field value. */
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR field value from a register. */
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR register field value suitable for setting the register. */
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_SET(value) (((value) << 0) & 0xff)
#elif GPIO0_PWIDTH_D == 16
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_SET_MSK    0xffff
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_CLR_MSK    0x0000
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_SET(value) (((value) << 0) & 0xffff)
#elif GPIO0_PWIDTH_D == 32
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_SET_MSK    0xffffffff
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_CLR_MSK    0x00000000
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_SET_MSK    0xffffffff
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_CLR_MSK    0x00000000
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_SWPORTD_DDR_GPIO_SWPORTD_DDR_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_SWPORTD_DDR.
 */
struct ALT_GPIO1_SWPORTD_DDR_s
{
    uint32_t gpio_SWPORTD_DDR : GPIO0_PWIDTH_D;      /* Port B Data Direction*/
    uint32_t                  : (32-GPIO0_PWIDTH_D); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_SWPORTD_DDR. */
typedef volatile struct ALT_GPIO1_SWPORTD_DDR_s  ALT_GPIO1_SWPORTD_DDR_t;
#endif  /* __ASSEMBLY__ */

/* The Least Significant Bit (LSB) position of the ALT_GPIO1_INTEN_GPIO_INTEN register field. */
#define ALT_GPIO1_INTEN_GPIO_INTEN_LSB        0
#if GPIO1_PWIDTH_A == 8
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_INTEN_GPIO_INTEN register field. */
#define ALT_GPIO1_INTEN_GPIO_INTEN_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO1_INTEN_GPIO_INTEN register field. */
#define ALT_GPIO1_INTEN_GPIO_INTEN_WIDTH      GPIO1_PWIDTH_A
/* The mask used to set the ALT_GPIO1_INTEN_GPIO_INTEN register field value. */
#define ALT_GPIO1_INTEN_GPIO_INTEN_SET_MSK    0xff
/* The mask used to clear the ALT_GPIO1_INTEN_GPIO_INTEN register field value. */
#define ALT_GPIO1_INTEN_GPIO_INTEN_CLR_MSK    0x00
/* Extracts the ALT_GPIO1_INTEN_GPIO_INTEN field value from a register. */
#define ALT_GPIO1_INTEN_GPIO_INTEN_GET(value) (((value) & 0xff) >> 0)
/* Produces a ALT_GPIO1_INTEN_GPIO_INTEN register field value suitable for setting the register. */
#define ALT_GPIO1_INTEN_GPIO_INTEN_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_A == 16
#define ALT_GPIO1_INTEN_GPIO_INTEN_MSB        (GPIO1_PWIDTH_A-1)
#define ALT_GPIO1_INTEN_GPIO_INTEN_WIDTH      GPIO1_PWIDTH_A
#define ALT_GPIO1_INTEN_GPIO_INTEN_SET_MSK    0xffff
#define ALT_GPIO1_INTEN_GPIO_INTEN_CLR_MSK    0x0000
#define ALT_GPIO1_INTEN_GPIO_INTEN_GET(value) (((value) & 0xffff) >> 0)
#define ALT_GPIO1_INTEN_GPIO_INTEN_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_A == 32
#define ALT_GPIO1_INTEN_GPIO_INTEN_MSB        (GPIO1_PWIDTH_A-1)
#define ALT_GPIO1_INTEN_GPIO_INTEN_WIDTH      GPIO1_PWIDTH_A
#define ALT_GPIO1_INTEN_GPIO_INTEN_SET_MSK    0xffffffff
#define ALT_GPIO1_INTEN_GPIO_INTEN_CLR_MSK    0x00000000
#define ALT_GPIO1_INTEN_GPIO_INTEN_GET(value) (((value) & 0xffffffff) >> 0)
#define ALT_GPIO1_INTEN_GPIO_INTEN_SET(value) (((value) << 0) & 0xffffffff)
#else
#define ALT_GPIO1_INTEN_GPIO_INTEN_MSB        (GPIO1_PWIDTH_A-1)
#define ALT_GPIO1_INTEN_GPIO_INTEN_WIDTH      GPIO1_PWIDTH_A
#define ALT_GPIO1_INTEN_GPIO_INTEN_SET_MSK    0xffffffff
#define ALT_GPIO1_INTEN_GPIO_INTEN_CLR_MSK    0x00000000
#define ALT_GPIO1_INTEN_GPIO_INTEN_GET(value) (((value) & 0xffffffff) >> 0)
#define ALT_GPIO1_INTEN_GPIO_INTEN_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO_INTEN.
 */
struct ALT_GPIO1_INTEN_s
{
  uint32_t gpio_inten : GPIO1_PWIDTH_A;      /* Interrupt Enable Field */
  uint32_t            : (32-GPIO1_PWIDTH_A); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_INTEN. */
typedef volatile struct ALT_GPIO1_INTEN_s  ALT_GPIO1_INTEN_t;
#endif  /* __ASSEMBLY__ */

/*
 * Register : Interrupt Mask Register - gpio_intmask
 *
 * Controls which pins cause interrupts on Port A Data Register inputs.
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_A
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:---------------------
 *  [x:0]   | RW     | 0x0   | Interrupt Mask Field
 *  [31:y]  | R      | 0x0   | *Reserved - read as zero*
 *
 */
/* The Least Significant Bit (LSB) position of the ALT_GPIO1_INTMSK_GPIO_INTMSK register field. */
#define ALT_GPIO1_INTMSK_GPIO_INTMSK_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO_INTMSK_GPIO_INTMSK register field. */
#define ALT_GPIO1_INTMSK_GPIO_INTMSK_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO_INTMSK_GPIO_INTMSK register field. */
#define ALT_GPIO1_INTMSK_GPIO_INTMSK_WIDTH      GPIO1_PWIDTH_A
#if GPIO1_PWIDTH_A == 8
  /* The mask used to set the ALT_GPIO1_INTMSK_GPIO_INTMSK register field value. */
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_INTMSK_GPIO_INTMSK register field value. */
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_INTMSK_GPIO_INTMSK field value from a register. */
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_INTMSK_GPIO_INTMSK register field value suitable for setting the register. */
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_A == 16
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_SET_MSK    0xffff
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_CLR_MSK    0x0000
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_A == 32
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_SET_MSK    0xffffffff
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_CLR_MSK    0x00000000
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_SET_MSK    0xffffffff
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_CLR_MSK    0x00000000
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_INTMSK_GPIO_INTMSK_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_INTMSK.
 */
struct ALT_GPIO1_INTMSK_s
{
  uint32_t gpio_intmask : GPIO1_PWIDTH_A;      /* Interrupt Mask Field */
  uint32_t              : (32-GPIO1_PWIDTH_A); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_INTMSK. */
typedef volatile struct ALT_GPIO1_INTMSK_s  ALT_GPIO1_INTMSK_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_INTMSK register. */
#define ALT_GPIO_INTMSK_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_INTMSK_OFST))

/*
 * Register : Interrupt Level Register - gpio_inttype_level
 *
 * The interrupt level register defines the type of interrupt (edge or level).
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_A
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:----------------------
 *  [x:0]   | RW     | 0x0    | Interrupt Level Field
 *  [31:y]  | R      | 0x0    | *Reserved - read as zero*
 *
 */

/* The Least Significant Bit (LSB) position of the ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL register field. */
#define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL register field. */
#define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL register field. */
#define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_WIDTH      GPIO1_PWIDTH_A
#if GPIO1_PWIDTH_A == 8
  /* The mask used to set the ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL register field value. */
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL register field value. */
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL field value from a register. */
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL register field value suitable for setting the register. */
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_A == 16
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_SET_MSK    0xffff
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_CLR_MSK    0x0000
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_A == 32
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_SET_MSK    0xffffffff
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_CLR_MSK    0x00000000
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_SET_MSK    0xffffffff
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_CLR_MSK    0x00000000
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_INTTYPE_LEVEL_GPIO_INTTYPE_LEVEL_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_INTTYPE_LEVEL.
 */
struct ALT_GPIO1_INTTYPE_LEVEL_s
{
  uint32_t gpio_inttype_level : GPIO1_PWIDTH_A;      /* Interrupt Level Field */
  uint32_t                    : (32-GPIO1_PWIDTH_A); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_INTTYPE_LEVEL. */
typedef volatile struct ALT_GPIO1_INTTYPE_LEVEL_s  ALT_GPIO1_INTTYPE_LEVEL_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_INTTYPE_LEVEL register. */
#define ALT_GPIO_INTTYPE_LEVEL_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_INTTYPE_LEVEL_OFST))

/*
 * Register : Interrupt Polarity Register - gpio_int_polarity
 *
 * Controls the Polarity of Interrupts that can occur on inputs of Port A Data
 * Register
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_A
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:-----------------------
 *  [x:0]  | RW      | 0x0   | Polarity Control Field
 *  [31:y] | R       | 0x0   | *Reserved - read as zero*
 *
 */
/* The Least Significant Bit (LSB) position of the ALT_GPIO1_INT_POL_GPIO_INT_POL register field. */
#define ALT_GPIO1_INT_POL_GPIO_INT_POL_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_INT_POL_GPIO_INT_POL register field. */
#define ALT_GPIO1_INT_POL_GPIO_INT_POL_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO1_INT_POL_GPIO_INT_POL register field. */
#define ALT_GPIO1_INT_POL_GPIO_INT_POL_WIDTH      GPIO1_PWIDTH_A
#if GPIO1_PWIDTH_A == 8
  /* The mask used to set the ALT_GPIO1_INT_POL_GPIO_INT_POL register field value. */
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_INT_POL_GPIO_INT_POL register field value. */
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_INT_POL_GPIO_INT_POL field value from a register. */
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_INT_POL_GPIO_INT_POL register field value suitable for setting the register. */
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_A == 16
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_SET_MSK    0xffff
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_CLR_MSK    0x0000
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_A == 32
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_SET_MSK    0xffffffff
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_CLR_MSK    0x00000000
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_SET_MSK    0xffffffff
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_CLR_MSK    0x00000000
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_INT_POL_GPIO_INT_POL_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_INT_POL.
 */
struct ALT_GPIO1_INT_POL_s
{
  uint32_t gpio_int_polarity : GPIO1_PWIDTH_A;      /* Polarity Control Field */
  uint32_t                   : (32-GPIO1_PWIDTH_A); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_INT_POL. */
typedef volatile struct ALT_GPIO1_INT_POL_s  ALT_GPIO1_INT_POL_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_INT_POL register. */
#define ALT_GPIO_INT_POL_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_INT_POL_OFST))

/*
 * Register : Interrupt Status Register - gpio_intstatus
 *
 * The Interrupt status is reported for all Port A Data Register Bits.
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_A
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:-----------------------
 *  [x:0]   | R      | 0x0   | Interrupt Status Field
 *  [31:y]  | R      | 0x0   | *Reserved - read as zero*
 *
 */

/* The Least Significant Bit (LSB) position of the ALT_GPIO1_INTSTAT_GPIO_INTSTAT register field. */
#define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_INTSTAT_GPIO_INTSTAT register field. */
#define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO1_INTSTAT_GPIO_INTSTAT register field. */
#define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_WIDTH      GPIO1_PWIDTH_A
#if GPIO1_PWIDTH_A == 8
  /* The mask used to set the ALT_GPIO1_INTSTAT_GPIO_INTSTAT register field value. */
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_INTSTAT_GPIO_INTSTAT register field value. */
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_INTSTAT_GPIO_INTSTAT field value from a register. */
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_INTSTAT_GPIO_INTSTAT register field value suitable for setting the register. */
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_A == 16
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_SET_MSK    0xffff
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_CLR_MSK    0x0000
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_A == 32
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_SET_MSK    0xffffffff
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_CLR_MSK    0x00000000
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_SET_MSK    0xffffffff
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_CLR_MSK    0x00000000
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_INTSTAT_GPIO_INTSTAT_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO_INTSTAT.
 */
struct ALT_GPIO1_INTSTAT_s
{
  uint32_t gpio_intstatus : GPIO1_PWIDTH_A;      /* Interrupt Status Field */
  uint32_t                : (32-GPIO1_PWIDTH_A); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_INTSTAT. */
typedef volatile struct ALT_GPIO1_INTSTAT_s  ALT_GPIO1_INTSTAT_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_INTSTAT register. */
#define ALT_GPIO_INTSTAT_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_INTSTAT_OFST))

/*
 * Register : Raw Interrupt Status Register - gpio_raw_intstatus
 *
 * This is the Raw Interrupt Status Register for Port A Data Register. It is used
 * with the Interrupt Mask Register to allow interrupts from the Port A Data
 * Register.
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_A
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:---------------------------
 *  [x:0]   | R      | 0x0   | Raw Interrupt Status Field
 *  [31:y]  | R      | 0x0   | *Reserved - read as zero*
 *
 */
/* The Least Significant Bit (LSB) position of the ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT register field. */
#define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT register field. */
#define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT register field. */
#define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_WIDTH      GPIO1_PWIDTH_A
#if GPIO1_PWIDTH_A == 8
  /* The mask used to set the ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT register field value. */
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT register field value. */
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT field value from a register. */
  #define ALT_GPIO_RAW_INTSTAT_GPIO1_RAW_INTSTAT_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT register field value suitable for setting the register. */
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_A == 16
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_SET_MSK    0xffff
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_CLR_MSK    0x0000
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_A == 32
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_SET_MSK    0xffffffff
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_CLR_MSK    0x00000000
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_SET_MSK    0xffffffff
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_CLR_MSK    0x00000000
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_RAW_INTSTAT_GPIO_RAW_INTSTAT_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO_RAW_INTSTAT.
 */
struct ALT_GPIO1_RAW_INTSTAT_s
{
  uint32_t gpio_raw_intstatus : GPIO1_PWIDTH_A;      /* Raw Interrupt Status Field */
  uint32_t                    : (32-GPIO1_PWIDTH_A); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_RAW_INTSTAT. */
typedef volatile struct ALT_GPIO1_RAW_INTSTAT_s  ALT_GPIO1_RAW_INTSTAT_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_RAW_INTSTAT register. */
#define ALT_GPIO_RAW_INTSTAT_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_RAW_INTSTAT_OFST))

/*
 * Register : Debounce Enable Register - gpio_debounce
 *
 * Debounces each IO Pin
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_A
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:--------------------------------
 *  [x:0]  | RW      | 0x0   | ALT_GPIO_DEBOUNCE_GPIO_DEBOUNCE
 *  [31:y] | R       | 0x0   | *Reserved - read as zero*
 *
 */

/* The Least Significant Bit (LSB) position of the ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE register field. */
#define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE register field. */
#define ALT_GPIO_DEBOUNCE_GPIO1_DEBOUNCE_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE register field. */
#define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_WIDTH      GPIO1_PWIDTH_A
#if GPIO1_PWIDTH_A == 8
  /* The mask used to set the ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE register field value. */
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE register field value. */
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE field value from a register. */
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE register field value suitable for setting the register. */
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_A == 16
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_SET_MSK    0xffff
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_CLR_MSK    0x0000
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_A == 32
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_SET_MSK    0xffffffff
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_CLR_MSK    0x00000000
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_SET_MSK    0xffffffff
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_CLR_MSK    0x00000000
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_DEBOUNCE_GPIO_DEBOUNCE_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO_DEBOUNCE.
 */
struct ALT_GPIO1_DEBOUNCE_s
{
  uint32_t gpio_debounce : GPIO1_PWIDTH_A;      /* ALT_GPIO_DEBOUNCE_GPIO_DEBOUNCE */
  uint32_t               : (32-GPIO1_PWIDTH_A); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_DEBOUNCE. */
typedef volatile struct ALT_GPIO1_DEBOUNCE_s  ALT_GPIO1_DEBOUNCE_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_DEBOUNCE register. */
#define ALT_GPIO_DEBOUNCE_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_DEBOUNCE_OFST))

/*
 * Register : Clear Interrupt Register - gpio_porta_eoi
 *
 * Port A Data Register interrupt handling.
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_A
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:-----------------------------
 *  [x:0]   | W      | 0x0   | Clears Edge Interrupts Field
 *  [31:y]  | W      | 0x0   | *Reserved - read as zero*
 *
 */

/* The Least Significant Bit (LSB) position of the ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI register field. */
#define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI register field. */
#define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI register field. */
#define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_WIDTH      GPIO1_PWIDTH_A
#if GPIO1_PWIDTH_A == 8
  /* The mask used to set the ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI register field value. */
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_SET_MSK    0xffffffff
  /* The mask used to clear the ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI register field value. */
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_CLR_MSK    0x00000000
  /* Extracts the ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI field value from a register. */
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_GET(value) (((value) & 0xffffffff) >> 0)
  /* Produces a ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI register field value suitable for setting the register. */
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_SET(value) (((value) << 0) & 0xffffffff)
#elif GPIO1_PWIDTH_A == 16
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_SET_MSK    0xffff
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_CLR_MSK    0x0000
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_A == 32
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_SET_MSK    0xffffffff
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_CLR_MSK    0x00000000
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_SET_MSK    0xffffffff
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_CLR_MSK    0x00000000
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_PORTA_EOI_GPIO_PORTA_EOI_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_PORTA_EOI.
 */
struct ALT_GPIO1_PORTA_EOI_s
{
  uint32_t gpio_porta_eoi : GPIO1_PWIDTH_A;      /* Clears Edge Interrupts Field */
  uint32_t                : (32-GPIO1_PWIDTH_A); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_PORTA_EOI. */
typedef volatile struct ALT_GPIO1_PORTA_EOI_s  ALT_GPIO1_PORTA_EOI_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_PORTA_EOI register. */
#define ALT_GPIO_PORTA_EOI_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_PORTA_EOI_OFST))

/*
 * Register : External Port A Register - gpio_ext_porta
 *
 * The external port register is used to input data to the metastability flops.
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_A
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:--------------------
 *  [x:0]   | R      | 0x0   | External Port Field
 *  [31:y]  | R      | 0x0   | *Reserved - read as zero*
 *
 */

/*
 * Field : External Port Field - gpio_ext_porta
 *
 * When Port A Data Register is configured as Input, then reading this location
 * reads the values on the signals. When the data direction of Port A Data Register
 * is set as Output, reading this location reads Port A Data Register
 *
 * Field Access Macros:
 *
 */
/* The Least Significant Bit (LSB) position of the ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA register field. */
#define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA register field. */
#define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA register field. */
#define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_WIDTH      GPIO1_PWIDTH_A
#if GPIO1_PWIDTH_A == 8
  /* The mask used to set the ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA register field value. */
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA register field value. */
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA field value from a register. */
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA register field value suitable for setting the register. */
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_A == 16
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_SET_MSK    0xffff
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_CLR_MSK    0x00000000
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_A == 32
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_SET_MSK    0xffffffff
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_CLR_MSK    0x00000000
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_SET_MSK    0xffffffff
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_CLR_MSK    0x00000000
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_EXT_PORTA_GPIO_EXT_PORTA_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_EXT_PORTA.
 */
struct ALT_GPIO1_EXT_PORTA_s
{
  const uint32_t gpio_ext_porta : GPIO1_PWIDTH_A;      /* External Port Field */
  uint32_t                      : (32-GPIO1_PWIDTH_A); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_EXT_PORTA. */
typedef volatile struct ALT_GPIO1_EXT_PORTA_s  ALT_GPIO1_EXT_PORTA_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_EXT_PORTA register. */
#define ALT_GPIO_EXT_PORTA_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_EXT_PORTA_OFST))

/*
 * Register : External Port B Register - gpio_ext_portb
 *
 * The external port register is used to input data to the metastability flops.
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_B
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:--------------------
 *  [x:0]   | R      | 0x0   | External Port Field
 *  [31:y]  | R      | 0x0   | *Reserved - read as zero*
 *
 */

/*
 * Field : External Port Field - gpio_ext_portb
 *
 * When Port B Data Register is configured as Input, then reading this location
 * reads the values on the signals. When the data direction of Port B Data Register
 * is set as Output, reading this location reads Port B Data Register
 *
 * Field Access Macros:
 *
 */
/* The Least Significant Bit (LSB) position of the ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB register field. */
#define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB register field. */
#define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_MSB        (GPIO1_PWIDTH_B-1)
/* The width in bits of the ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB register field. */
#define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_WIDTH      GPIO1_PWIDTH_B
#if GPIO1_PWIDTH_B == 8
  /* The mask used to set the ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB register field value. */
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB register field value. */
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB field value from a register. */
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB register field value suitable for setting the register. */
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_B == 16
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_SET_MSK    0xffff
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_CLR_MSK    0x00000000
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_B == 32
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_SET_MSK    0xffffffff
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_CLR_MSK    0x00000000
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_SET_MSK    0xffffffff
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_CLR_MSK    0x00000000
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_EXT_PORTB_GPIO_EXT_PORTB_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_EXT_PORTB.
 */
struct ALT_GPIO1_EXT_PORTB_s
{
  const uint32_t gpio_ext_portb : GPIO1_PWIDTH_B;      /* External Port Field */
  uint32_t                      : (32-GPIO1_PWIDTH_B); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_EXT_PORTB. */
typedef volatile struct ALT_GPIO1_EXT_PORTB_s  ALT_GPIO1_EXT_PORTB_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_EXT_PORTB register. */
#define ALT_GPIO_EXT_PORTB_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_EXT_PORTB_OFST))

/*
 * Register : External Port C Register - gpio_ext_portc
 *
 * The external port register is used to input data to the metastability flops.
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_C
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:--------------------
 *  [x:0]   | R      | 0x0   | External Port Field
 *  [31:y]  | R      | 0x0   | *Reserved - read as zero*
 *
 */

/*
 * Field : External Port Field - gpio_ext_portc
 *
 * When Port C Data Register is configured as Input, then reading this location
 * reads the values on the signals. When the data direction of Port C Data Register
 * is set as Output, reading this location reads Port C Data Register
 *
 * Field Access Macros:
 *
 */
/* The Least Significant Bit (LSB) position of the ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC register field. */
#define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC register field. */
#define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_MSB        (GPIO1_PWIDTH_C-1)
/* The width in bits of the ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC register field. */
#define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_WIDTH      GPIO1_PWIDTH_C
#if GPIO1_PWIDTH_C == 8
  /* The mask used to set the ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC register field value. */
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC register field value. */
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC field value from a register. */
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC register field value suitable for setting the register. */
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_C == 16
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_SET_MSK    0xffff
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_CLR_MSK    0x00000000
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_C == 32
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_SET_MSK    0xffffffff
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_CLR_MSK    0x00000000
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_SET_MSK    0xffffffff
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_CLR_MSK    0x00000000
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_EXT_PORTC_GPIO_EXT_PORTC_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_EXT_PORTC.
 */
struct ALT_GPIO1_EXT_PORTC_s
{
  const uint32_t gpio_ext_portc : GPIO1_PWIDTH_C;      /* External Port Field */
  uint32_t                      : (32-GPIO1_PWIDTH_C); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_EXT_PORTC. */
typedef volatile struct ALT_GPIO1_EXT_PORTC_s  ALT_GPIO1_EXT_PORTC_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_EXT_PORTC register. */
#define ALT_GPIO_EXT_PORTC_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_EXT_PORTC_OFST))

/*
 * Register : External Port D Register - gpio_ext_portd
 *
 * The external port register is used to input data to the metastability flops.
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_D
 *
 *  Bits    | Access | Reset | Description
 * :--------|:-------|:------|:--------------------
 *  [x:0]   | R      | 0x0   | External Port Field
 *  [31:y]  | R      | 0x0   | *Reserved - read as zero*
 *
 */

/*
 * Field : External Port Field - gpio_ext_portd
 *
 * When Port D Data Register is configured as Input, then reading this location
 * reads the values on the signals. When the data direction of Port D Data Register
 * is set as Output, reading this location reads Port D Data Register
 *
 * Field Access Macros:
 *
 */
/* The Least Significant Bit (LSB) position of the ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTC register field. */
#define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTC register field. */
#define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_MSB        (GPIO1_PWIDTH_D-1)
/* The width in bits of the ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTC register field. */
#define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_WIDTH      GPIO1_PWIDTH_D
#if GPIO1_PWIDTH_D == 8
  /* The mask used to set the ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTC register field value. */
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTC register field value. */
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTC field value from a register. */
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTC register field value suitable for setting the register. */
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_D == 16
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_SET_MSK    0xffff
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_CLR_MSK    0x00000000
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_D == 32
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_SET_MSK    0xffffffff
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_CLR_MSK    0x00000000
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_SET_MSK    0xffffffff
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_CLR_MSK    0x00000000
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_EXT_PORTD_GPIO_EXT_PORTD_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_EXT_PORTD.
 */
struct ALT_GPIO1_EXT_PORTD_s
{
  const uint32_t gpio_ext_portd : GPIO1_PWIDTH_D;      /* External Port Field */
  uint32_t                      : (32-GPIO1_PWIDTH_D); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_EXT_PORTD. */
typedef volatile struct ALT_GPIO1_EXT_PORTD_s  ALT_GPIO1_EXT_PORTD_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_EXT_PORTD register. */
#define ALT_GPIO_EXT_PORTD_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_EXT_PORTD_OFST))

/*
 * Register : Synchronization Level Register - gpio_ls_sync
 *
 * The Synchronization level register is used to synchronize input with pclk_intr
 *
 * Register Layout
 *
 *  Bits   | Access | Reset | Description
 * :-------|:-------|:------|:----------------------------
 *  [0]    | RW     | 0x0   | Synchronization Level Field
 *  [31:1] | R      | 0x0   | *Reserved - read as zero*
 *
 */

/* The Least Significant Bit (LSB) position of the ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC register field. */
#define ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC register field. */
#define ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC register field. */
#define ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC_WIDTH      1
/* The mask used to set the ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC register field value. */
#define ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC_SET_MSK    0x00000001
/* The mask used to clear the ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC register field value. */
#define ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC_CLR_MSK    0xfffffffe
/* The reset value of the ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC register field. */
#define ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC_RESET      0x0
/* Extracts the ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC field value from a register. */
#define ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC_GET(value) (((value) & 0x00000001) >> 0)
/* Produces a ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC register field value suitable for setting the register. */
#define ALT_GPIO1_LS_SYNC_GPIO_LS_SYNC_SET(value) (((value) << 0) & 0x00000001)

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_LS_SYNC.
 */
struct ALT_GPIO1_LS_SYNC_s
{
    uint32_t  gpio_ls_sync :  1;  /* Synchronization Level Field */
    uint32_t               : 31;  /* *UNDEFINED* */
};

/* The typedef declaration for register ALT_GPIO1_LS_SYNC. */
typedef volatile struct ALT_GPIO1_LS_SYNC_s  ALT_GPIO1_LS_SYNC_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_LS_SYNC register. */
#define ALT_GPIO_LS_SYNC_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_LS_SYNC_OFST))

/*
 * Register : ID Code Register - gpio_id_code
 *
 * GPIO ID code.
 *
 * Register Layout - dependent on port width x where x=GPIO_PWIDTH_A
 *
 *  Bits   | Access | Reset | Description
 * :-------|:-------|:------|:--------------
 *  [x:0]  | R      | 0x0   | ID Code Field
 *  [31:y] | R      | 0x0   | *Reserved - read as zero*
 *
 */
/*
 * Field : ID Code Field - gpio_id_code
 *
 * Chip identification
 *
 * Field Access Macros:
 *
 */
/* The Least Significant Bit (LSB) position of the ALT_GPIO1_ID_CODE_GPIO_ID_CODE register field. */
#define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_LSB        0
/* The Most Significant Bit (MSB) position of the ALT_GPIO1_ID_CODE_GPIO_ID_CODE register field. */
#define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_MSB        (GPIO1_PWIDTH_A-1)
/* The width in bits of the ALT_GPIO1_ID_CODE_GPIO_ID_CODE register field. */
#define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_WIDTH      GPIO1_PWIDTH_A
#if GPIO1_PWIDTH_A == 8
  /* The mask used to set the ALT_GPIO1_ID_CODE_GPIO_ID_CODE register field value. */
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_SET_MSK    0xff
  /* The mask used to clear the ALT_GPIO1_ID_CODE_GPIO_ID_CODE register field value. */
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_CLR_MSK    0x00
  /* Extracts the ALT_GPIO1_ID_CODE_GPIO_ID_CODE field value from a register. */
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_GET(value) (((value) & 0xff) >> 0)
  /* Produces a ALT_GPIO1_ID_CODE_GPIO_ID_CODE register field value suitable for setting the register. */
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_SET(value) (((value) << 0) & 0xff)
#elif GPIO1_PWIDTH_A == 16
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_SET_MSK    0xffff
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_CLR_MSK    0x00000000
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_GET(value) (((value) & 0xffff) >> 0)
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_SET(value) (((value) << 0) & 0xffff)
#elif GPIO1_PWIDTH_A == 32
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_SET_MSK    0xffffffff
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_CLR_MSK    0x00000000
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_SET(value) (((value) << 0) & 0xffffffff)
#else
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_SET_MSK    0xffffffff
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_CLR_MSK    0x00000000
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_GET(value) (((value) & 0xffffffff) >> 0)
  #define ALT_GPIO1_ID_CODE_GPIO_ID_CODE_SET(value) (((value) << 0) & 0xffffffff)
#endif

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register ALT_GPIO1_ID_CODE.
 */
struct ALT_GPIO1_ID_CODE_s
{
    const uint32_t  gpio_id_code : GPIO1_PWIDTH_A;      /* ID Code Field */
    const uint32_t               : (32-GPIO1_PWIDTH_A); /* Reserved - read as zero */
};

/* The typedef declaration for register ALT_GPIO1_ID_CODE. */
typedef volatile struct ALT_GPIO1_ID_CODE_s  ALT_GPIO1_ID_CODE_t;
#endif  /* __ASSEMBLY__ */

/* The address of the ALT_GPIO_ID_CODE register. */
#define ALT_GPIO_ID_CODE_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + ALT_GPIO_ID_CODE_OFST))

#ifndef __ASSEMBLY__
/*
 * WARNING: The C register and register group struct declarations are provided for
 * convenience and illustrative purposes. They should, however, be used with
 * caution as the C language standard provides no guarantees about the alignment or
 * atomicity of device memory accesses. The recommended practice for writing
 * hardware drivers is to use the SoCAL access macros and alt_read_word() and
 * alt_write_word() functions.
 *
 * The struct declaration for register group ALT_GPIO.
 */
struct ALT_GPIO1_s
{
    volatile ALT_GPIO1_SWPORTA_DR_t     gpio_swporta_dr;     /* ALT_GPIO_SWPORTA_DR */
    volatile ALT_GPIO1_SWPORTA_DDR_t    gpio_swporta_ddr;    /* ALT_GPIO_SWPORTA_DDR */
    volatile uint32_t                  _pad_0x8_0x2f[10];    /* *UNDEFINED* */
    volatile ALT_GPIO1_INTEN_t          gpio_inten;          /* ALT_GPIO_INTEN */
    volatile ALT_GPIO1_INTMSK_t         gpio_intmask;        /* ALT_GPIO_INTMSK */
    volatile ALT_GPIO1_INTTYPE_LEVEL_t  gpio_inttype_level;  /* ALT_GPIO_INTTYPE_LEVEL */
    volatile ALT_GPIO1_INT_POL_t        gpio_int_polarity;   /* ALT_GPIO_INT_POL */
    volatile ALT_GPIO1_INTSTAT_t        gpio_intstatus;      /* ALT_GPIO_INTSTAT */
    volatile ALT_GPIO1_RAW_INTSTAT_t    gpio_raw_intstatus;  /* ALT_GPIO_RAW_INTSTAT */
    volatile ALT_GPIO1_DEBOUNCE_t       gpio_debounce;       /* ALT_GPIO_DEBOUNCE */
    volatile ALT_GPIO1_PORTA_EOI_t      gpio_porta_eoi;      /* ALT_GPIO_PORTA_EOI */
    volatile ALT_GPIO1_EXT_PORTA_t      gpio_ext_porta;      /* ALT_GPIO_EXT_PORTA */
    volatile uint32_t                  _pad_0x54_0x5f[3];    /* *UNDEFINED* */
    volatile ALT_GPIO1_LS_SYNC_t        gpio_ls_sync;        /* ALT_GPIO_LS_SYNC */
    volatile ALT_GPIO1_ID_CODE_t        gpio_id_code;        /* ALT_GPIO_ID_CODE */
    volatile uint32_t                  _pad_0x68_0x6b;       /* *UNDEFINED* */
    volatile ALT_GPIO_VER_ID_CODE_t    gpio_ver_id_code;    /* ALT_GPIO_VER_ID_CODE */
    volatile ALT_GPIO_CFG_REG2_t       gpio_config_reg2;    /* ALT_GPIO_CFG_REG2 */
    volatile ALT_GPIO_CFG_REG1_t       gpio_config_reg1;    /* ALT_GPIO_CFG_REG1 */
    volatile uint32_t                  _pad_0x78_0x80[2];    /* *UNDEFINED* */
};

/* The typedef declaration for register group ALT_GPIO. */
typedef volatile struct ALT_GPIO1_s  ALT_GPIO1_t;
#endif  /* __ASSEMBLY__ */

#ifdef __cplusplus
}
#endif  /* __cplusplus */
#endif  /* __ALTERA_ALT_GPIO1_H__ */
