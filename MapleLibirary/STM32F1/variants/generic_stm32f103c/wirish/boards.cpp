/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011, 2012 LeafLabs, LLC.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file wirish/boards.cpp
 * @brief init() and board routines.
 *
 * This file is mostly interesting for the init() function, which
 * configures Flash, the core clocks, and a variety of other available
 * peripherals on the board so the rest of Wirish doesn't have to turn
 * things on before using them.
 *
 * Prior to returning, init() calls boardInit(), which allows boards
 * to perform any initialization they need to. This file includes a
 * weak no-op definition of boardInit(), so boards that don't need any
 * special initialization don't have to define their own.
 *
 * How init() works is chip-specific. See the boards_setup.cpp files
 * under e.g. wirish/stm32f1/, wirish/stmf32f2 for the details, but be
 * advised: their contents are unstable, and can/will change without
 * notice.
 */

#include <boards.h>
#include <libmaple/libmaple_types.h>
#include <libmaple/flash.h>
#include <libmaple/nvic.h>
#include <libmaple/systick.h>
#include "boards_private.h"
#include <stdint.h>
#include "elf2.h"
#include "platform.h"
#include "syscon.h"


//static void delay(uint32_t n);
static void setup_waitfor(void);
static void setup_flash(void);
static void setup_clocks(void);
static void setup_nvic(void);
static void setup_adcs(void);
static void setup_timers(void);

/*
 * Exported functions
 */
#define CFG_SR_REG   (*((__IO uint32 *)(0x4000b000)))

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
/* ToDo: add here your necessary defines for device initialization
         following is an example for different system frequencies */
#define SYSTEM_CLOCK    AHB_CLK


/*----------------------------------------------------------------------------
  System Core Clock Variable
 *----------------------------------------------------------------------------*/
/* ToDo: initialize SystemCoreClock with the system core clock frequency value
         achieved after system intitialization.
         This means system core clock frequency after call to SystemInit() */
uint32_t SystemCoreClock = SYSTEM_CLOCK;  /* System Clock Frequency (Core Clock)*/

void init(void) {
//    setup_waitfor();
//    setup_clocks();
	SystemInit();
    setup_nvic();
    #if 0
    systick_init(SYSTICK_RELOAD_VAL);
    wirish::priv::board_setup_gpio();
    setup_adcs();
    setup_timers();
    wirish::priv::board_setup_usb();
    wirish::priv::series_init();
    boardInit();
    #endif //‘›«“œ»◊¢ ÕµÙ
}

/*
 * Auxiliary routines
 */

static void setup_waitfor(void) {
    while((CFG_SR_REG & 0x04000000)!= 0x04000000);          // wait fpga start
    //delay(60000);                                           // wait clock stable
}
/* Provide a default no-op boardInit(). */
__weak void boardInit(void) {
}

/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/

void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
/* ToDo: add code to calculate the system frequency based upon the current
         register settings.
         This function can be used to retrieve the system core clock frequeny
         after user changed register sittings. */
  SystemCoreClock = SYSTEM_CLOCK;
}

static void delay(uint32_t n)
{
    uint32_t i;
    for(i=0;i<n;i++)
    {
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
    }
}

void SystemInit (void)
{
    while((CFG_SR_REG & 0x04000000)!= 0x04000000);          // wait fpga start
    delay(60000);                                           // wait clock stable
/* ToDo: add code to initialize the system
         do not use global variables because this function is called before
         reaching pre-main. RW section maybe overwritten afterwards. */
    SystemCoreClock = SYSTEM_CLOCK;

    //set sysclk (clock source)
    HAL_SYSCON_HCLK_Src_Sel(clk_sel_pll);

    //set ahb clock divider
    HAL_SYSCON_Set_HClk_Div(SYSTEM_CLK/AHB_CLK);

    //set apb clock 0 divider
    HAL_SYSCON_Set_PClk0_Div(AHB_CLK/APB_CLK0);

    //set apb clock 1 divider
    HAL_SYSCON_Set_PClk1_Div(AHB_CLK/APB_CLK1);

    //set rtc clock
    #if (RTC_SRC_XTAL == 1)
    HAL_SYSCON_Set_RTC_Div(rtcclk_from_xtal,0);
    #else
    HAL_SYSCON_Set_RTC_Div(rtcclk_from_ahb,AHB_CLK/RTC_CLK);
    #endif

    #if (RUN_ON_FLASH == 1)
    // Redirect Interrupt Vectors
	__DMB();
	SCB->VTOR = 0x10000000;
	__DSB();
    #endif

}


static void setup_clocks(void) {
	(*((__IO uint32 *)(0x40020010))) = 0x08;

}
/* You could farm this out to the files in boards/ if e.g. it takes
 * too long to test on boards with lots of pins. */
bool boardUsesPin(uint8 pin) {
    for (int i = 0; i < BOARD_NR_USED_PINS; i++) {
        if (pin == boardUsedPins[i]) {
            return true;
        }
    }
    return false;
}

static void setup_flash(void) {
//    // Turn on as many Flash "go faster" features as
//    // possible. flash_enable_features() just ignores any flags it
//    // can't support.
//    flash_enable_features(FLASH_PREFETCH | FLASH_ICACHE | FLASH_DCACHE);
//    // Configure the wait states, assuming we're operating at "close
//    // enough" to 3.3V.
//    flash_set_latency(FLASH_SAFE_WAIT_STATES);
}



/*
 * These addresses are where usercode starts when a bootloader is
 * present. If no bootloader is present, the user NVIC usually starts
 * at the Flash base address, 0x08000000.
 */
#if defined(BOOTLOADER_maple)
	#define USER_ADDR_ROM 0x08005000
#else
	#define USER_ADDR_ROM 0x08000000
#endif
#define USER_ADDR_RAM 0x20000C00
extern char __text_start__;

static void setup_nvic(void) {

	nvic_init((uint32)VECT_TAB_ADDR, 0);
//	nvic_init((uint32)0x20000000, 0);

/* Roger Clark. We now control nvic vector table in boards.txt using the build.vect paramater
#ifdef VECT_TAB_FLASH
    nvic_init(USER_ADDR_ROM, 0);
#elif defined VECT_TAB_RAM
    nvic_init(USER_ADDR_RAM, 0);
#elif defined VECT_TAB_BASE
    nvic_init((uint32)0x08000000, 0);
#elif defined VECT_TAB_ADDR
    // A numerically supplied value
    nvic_init((uint32)VECT_TAB_ADDR, 0);
#else
    // Use the __text_start__ value from the linker script; this
    // should be the start of the vector table.
    nvic_init((uint32)&__text_start__, 0);
#endif

*/
}

static void adc_default_config( adc_dev *dev) {
    adc_enable_single_swstart(dev);
    adc_set_sample_rate(dev, wirish::priv::w_adc_smp);
}

static void setup_adcs(void) {
    adc_set_prescaler(wirish::priv::w_adc_pre);
    adc_foreach(adc_default_config);
}

static void timer_default_config(timer_dev *dev) {
    timer_adv_reg_map *regs = (dev->regs).adv;
    const uint16 full_overflow = 0xFFFF;
    const uint16 half_duty = 0x8FFF;

    timer_init(dev);
    timer_pause(dev);

    regs->CR1 = TIMER_CR1_ARPE;
    regs->PSC = 1;
    regs->SR = 0;
    regs->DIER = 0;
    regs->EGR = TIMER_EGR_UG;
    switch (dev->type) {
    case TIMER_ADVANCED:
        regs->BDTR = TIMER_BDTR_MOE | TIMER_BDTR_LOCK_OFF;
        // fall-through
    case TIMER_GENERAL:
        timer_set_reload(dev, full_overflow);
        for (uint8 channel = 1; channel <= 4; channel++) {
            if (timer_has_cc_channel(dev, channel)) {
                timer_set_compare(dev, channel, half_duty);
                timer_oc_set_mode(dev, channel, TIMER_OC_MODE_PWM_1,
                                  TIMER_OC_PE);
            }
        }
        // fall-through
    case TIMER_BASIC:
        break;
    }

    timer_generate_update(dev);
    timer_resume(dev);
}

static void setup_timers(void) {
    timer_foreach(timer_default_config);
}


/************************************************************************************************************/
#include "chip.h"
#include "syscon.h"
#include "reg_sysc_cpu.h"
#include "reg_sysc_awo.h"
#include "debug.h"

void HAL_SYSCON_Clk_Enable(SYSCON_ClkGate clk_src, bool ena)
{
    uint32_t tmp;

    if((clk_src == sw_clkgate_rtc) || (clk_src == sw_clkgate_wic))
    {
        tmp = sysc_awo_sw_clkg_get();
        if(ena)
            sysc_awo_sw_clkg_set(tmp | (1<<(clk_src-15)));
        else
            sysc_awo_sw_clkg_set(tmp & (~(1<<(clk_src-15))));
    }
    else if(clk_src <= sw_clkgate_dma)
    {
        tmp = sysc_cpu_sw_clkg_get();
        if(ena)
            sysc_cpu_sw_clkg_set(tmp | (1<<clk_src));
        else
            sysc_cpu_sw_clkg_set(tmp & (~(1<<clk_src)));

    }
    else
    {
        assert_param(clk_src <= sw_clkgate_wic);
    }

}

void HAL_SYSCON_Set_HClk_Div(uint8_t ahb_divider)
{
    assert_param((ahb_divider-1)<=0xf);
    sysc_awo_hclk_div_para_m1_setf(ahb_divider-1);
    sysc_awo_div_update_set(1);
}

void HAL_SYSCON_Set_PClk0_Div(uint8_t apb0_divider)
{
    assert_param((apb0_divider-1)<=0xf);
    sysc_cpu_pclk0_div_para_m1_setf(apb0_divider-1);
    sysc_cpu_div_update_set(1);
}


void HAL_SYSCON_Set_PClk1_Div(uint8_t apb1_divider)
{
    assert_param((apb1_divider-1)<=0xf);
    sysc_awo_pclk1_div_para_m1_setf(apb1_divider-1);
    sysc_awo_div_update_set(2);
}


void HAL_SYSCON_Set_RTC_Div(SYSCON_RTCClk_Src rtc_src, uint16_t rtc_divider)
{
    assert_param((rtc_divider-1)<=0x1fff);
    assert_param(rtc_src<=rtcclk_from_ahb);

    if(rtc_src == rtcclk_from_xtal)
    {
        sysc_awo_rtc_div_para_m1_setf(0);
        sysc_awo_div_update_set(4);

    }
    else
    {
        assert_param(rtc_divider>=2);
        sysc_awo_rtc_div_para_m1_setf(rtc_divider-1);
        sysc_awo_div_update_set(4);
    }
}


void HAL_SYSCON_SW_Reset(SYSCON_SwRst module)
{
    if(module <= sw_rst_dma)
    {
        sysc_cpu_sw_rstn_set(~(1<<module));
        __NOP();
        __NOP();
        sysc_cpu_sw_rstn_set(0xffffffff);
    }
    else if(module <= sw_rst_all)
    {
        sysc_awo_sw_rstn_set(~(1<<module));
        __NOP();
        __NOP();
        sysc_awo_sw_rstn_set(0xffffffff);
    }
    else
        assert_param(module <= sw_rst_all);

}



void HAL_SYSCON_HCLK_Src_Sel(SYSCON_Clk_Sel clk_sel)
{
    sysc_awo_clk_src_sel_setf(clk_sel);
}

/**
 ****************************************************************************************
 * @brief software generate a pulse to capture I/O for retention
 ****************************************************************************************
 */
void HAL_SYSCON_Start_Cap_IO()
{
	uint32_t reg;
	//read the old state
	reg = sysc_awo_io_ret_get();
	reg |= 0x8;								//set bit[3]
	sysc_awo_io_ret_set(reg);
}

/**
 ****************************************************************************************
 * @brief enable i/o retention. Use this function with awo_start_cap_io together.
 * @param sel: sel = IO_FROM_RETENTION: enable retention. sel = IO_FROM_PINSHARE disable retention.
 ****************************************************************************************
 */
void HAL_SYSCON_IO_Ctrl_Set(SYSCON_IO_CtrlSel sel)
{
	assert_param(((sel == IO_from_pinshare) || (sel == IO_from_retention)));

	sysc_awo_io_ctrl_sel_setf(sel);
}

/**
 * @brief use this function to let pmu control io retention. Normally we use pmu to automatically control io retention.
 */
void HAL_SYSCON_Retention_Enable()
{
    sysc_awo_io_stat_ret_pmu_setf(1);
    sysc_awo_io_stat_unret_pmu_setf(1);
}



void HAL_SYSCON_GPIO_Pull_Up(uint16_t gpio_map, bool en)
{
    uint16_t tmp = sysc_awo_gpil_pu_n_getf();

    if(en)
        sysc_awo_gpil_pu_n_setf(gpio_map | tmp);
    else
        sysc_awo_gpil_pu_n_setf(tmp & (~gpio_map));

}


void HAL_SYSCON_GPIO_Pull_Down(uint16_t gpio_map, bool en)
{
    uint16_t tmp = sysc_awo_gpil_pd_getf();

    if(en)
        sysc_awo_gpil_pd_setf(gpio_map | tmp);
    else
        sysc_awo_gpil_pd_setf(tmp & (~gpio_map));

}


void HAL_SYSCON_GPIO_IO_En(bool gpio_a0_en, bool gpio_a1_en)
{
    sysc_cpu_gpioa_en_1_setf(gpio_a1_en);
    sysc_cpu_gpioa_en_0_setf(gpio_a0_en);

}

//iomux
void HAL_SYSCON_Function_IO_Set(SYSCON_Function_IO_Type io_type, SYSCON_Function_IO_Idx io_idx, uint8_t en)
{
	uint32_t enable_map;
	enable_map = sysc_cpu_func_en_get();
	if(en == 0)
    {
		enable_map &= (~(1 << io_idx));
		sysc_cpu_func_en_set(enable_map);
		return;
	}
	switch(io_idx)
    {
		case GPIO_0:
			sysc_cpu_func_io00_sel_setf(io_type);
			break;
		case GPIO_1:
			sysc_cpu_func_io01_sel_setf(io_type);
			break;
		case GPIO_2:
			sysc_cpu_func_io02_sel_setf(io_type);
			break;
		case GPIO_3:
			sysc_cpu_func_io03_sel_setf(io_type);
			break;
		case GPIO_4:
			sysc_cpu_func_io04_sel_setf(io_type);
			break;
		case GPIO_5:
			sysc_cpu_func_io05_sel_setf(io_type);
			break;
		case GPIO_6:
			sysc_cpu_func_io06_sel_setf(io_type);
			break;
		case GPIO_7:
			sysc_cpu_func_io07_sel_setf(io_type);
			break;
		case GPIO_8:
			sysc_cpu_func_io08_sel_setf(io_type);
			break;
		case GPIO_9:
			sysc_cpu_func_io09_sel_setf(io_type);
			break;
		case GPIO_10:
			sysc_cpu_func_io10_sel_setf(io_type);
			break;
		case GPIO_11:
			sysc_cpu_func_io11_sel_setf(io_type);
			break;
		case GPIO_12:
			sysc_cpu_func_io12_sel_setf(io_type);
			break;
		case GPIO_13:
			sysc_cpu_func_io13_sel_setf(io_type);
			break;
		case GPIO_14:
			sysc_cpu_func_io14_sel_setf(io_type);
			break;
		case GPIO_15:
			sysc_cpu_func_io15_sel_setf(io_type);
			break;
		default:
			break;
	}
	enable_map |= (1 << io_idx);
	sysc_cpu_func_en_set(enable_map);
}

void HAL_SYSCON_Function_IO_disable_all(void)
{
	  sysc_cpu_func_en_set(0);
}




