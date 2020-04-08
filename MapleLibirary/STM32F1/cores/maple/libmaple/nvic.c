/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2011 LeafLabs, LLC.
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
 * @file libmaple/nvic.c
 * @brief Nested vector interrupt controller support.
 */

#include <libmaple/nvic.h>
#include <libmaple/scb.h>
#include <libmaple/stm32.h>

/**
 * @brief Set interrupt priority for an interrupt line
 *
 * Note: The STM32 only implements 4 bits of priority, ignoring the
 * lower 4 bits. This means there are only 16 levels of priority.
 * Bits[3:0] read as zero and ignore writes.
 *
 * @param irqn device to set
 * @param priority Priority to set, 0 being highest priority and 15
 *                 being lowest.
 */
void nvic_irq_set_priority(nvic_irq_num irqn, uint8 priority) {
    if (irqn < 0) {
        /* This interrupt is in the system handler block */
    	((struct scb_reg_map*)SCB_BASE)->SHP[((uint32)irqn & 0xF) - 4] = (priority & 0xF) << 4;
    } else {
    	((struct nvic_reg_map*)NVIC_BASE)->IP[irqn] = (priority & 0xF) << 4;
    }
}

/**
 * @brief Initialize the NVIC, setting interrupts to a default priority.
 */
void nvic_init(uint32 address, uint32 offset) {
    uint32 i;

    nvic_set_vector_table(address, offset);

    /*
     * Lower priority level for all peripheral interrupts to lowest
     * possible.
     */
    for (i = 0; i < STM32_NR_INTERRUPTS; i++) {
        nvic_irq_set_priority((nvic_irq_num)i, 0xF);
    }

    /* Lower systick interrupt priority to lowest level */
    nvic_irq_set_priority(NVIC_SYSTICK, 0xF);
}

/**
 * @brief Set the vector table base address.
 *
 * For stand-alone products, the vector table base address is normally
 * the start of Flash (0x08000000).
 *
 * @param address Vector table base address.
 * @param offset Offset from address.  Some restrictions apply to the
 *               use of nonzero offsets; see the ARM Cortex M3
 *               Technical Reference Manual.
 */
void nvic_set_vector_table(uint32 address, uint32 offset) {
	((struct scb_reg_map*)SCB_BASE)->VTOR = address | (offset & 0x1FFFFF80);
}

/**
 * @brief Force a system reset.
 *
 * Resets all major system components, excluding debug.
 */
void nvic_sys_reset() {
    uint32 prigroup = ((struct scb_reg_map*)SCB_BASE)->AIRCR & SCB_AIRCR_PRIGROUP;
    ((struct scb_reg_map*)SCB_BASE)->AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_SYSRESETREQ | prigroup;
    asm volatile("dsb");
    while (1)
        ;
}

/***********************************************************************************************/
#include "chip.h"
#include <libmaple/gpio.h>
//#include "intc.h"
#include "debug.h"


/***********************************************************************************************/

static void clear_all_pending_irq()
{
    NVIC_ClearPendingIRQ(WDT_IRQn);
    NVIC_ClearPendingIRQ(EXT_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(DMAC_IRQn);
    NVIC_ClearPendingIRQ(QSPI_IRQn);
    NVIC_ClearPendingIRQ(ADCC0_IRQn);
    NVIC_ClearPendingIRQ(ADCC1_IRQn);
    NVIC_ClearPendingIRQ(ADCC2_IRQn);
    NVIC_ClearPendingIRQ(ADCC3_IRQn);
    NVIC_ClearPendingIRQ(ADCC4_IRQn);
    NVIC_ClearPendingIRQ(ADCC5_IRQn);
    NVIC_ClearPendingIRQ(TIMER_IRQn);
    NVIC_ClearPendingIRQ(I2C0_IRQn);
    NVIC_ClearPendingIRQ(I2C1_IRQn);
    NVIC_ClearPendingIRQ(SPIM_IRQn);
    NVIC_ClearPendingIRQ(UART_IRQn);
    NVIC_ClearPendingIRQ(GPIO_IRQn);
    NVIC_ClearPendingIRQ(I2S_IRQn);
}


void int_set_priority(IRQn_Type type, uint8_t pri)
{
    NVIC_SetPriority(type,pri);
}


void intc_init(void)
{
    clear_all_pending_irq();
//    set_irq_prio();
}


void WDT_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+WDT_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));
}

void EXT_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+EXT_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}


void RTC_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+RTC_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}


void DMAC_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+DMAC_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}


void QSPI_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+QSPI_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}


void ADCC0_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+ADCC0_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}


void ADCC1_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+ADCC1_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}


void ADCC2_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+ADCC2_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}


void ADCC3_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+ADCC3_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}


void ADCC4_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+ADCC4_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}

void ADCC5_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+ADCC5_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}

void TIMER_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+TIMER_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}

void I2C0_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+I2C0_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}

void I2C1_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+I2C1_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}

void SPIM_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+SPIM_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}

void UART_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+UART_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}

void GPIO_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+GPIO_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}

void I2S_IRQHandler(void)
{
    int_callback_t *int_call = (int_callback_t *)INT_CALLBACK_PLACE;
    typedef void (*isr_fn)(void *param);
    isr_fn *func =(isr_fn *)(int_call+I2S_CALLBACK);
    assert_param(func != NULL);
    (*func)(*(func+1));

}

/**************************************************************************************************/
static int_callback_t int_callback[DRV_CALLBACK_MAX] __attribute__((section("int_callback_area")));

static int_callback_t *int_callback_register(int_callback_index index, void * isr_func, void *param)
{
	assert_param(index < DRV_CALLBACK_MAX);
	int_callback[index].isr_func = isr_func;
	int_callback[index].param = param;
    return &int_callback[index];
}


void int_callback_unregister(int_callback_index index)
{
    assert_param(index < DRV_CALLBACK_MAX);
    int_callback[index].isr_func = NULL;
    int_callback[index].param = NULL;

}

void * int_callback_get_isr(int_callback_index index)
{
	assert_param(index < DRV_CALLBACK_MAX);
	return int_callback[index].isr_func;
}

void * int_callback_get_param(int_callback_index index)
{
	assert_param(index < DRV_CALLBACK_MAX);
	return int_callback[index].param;
}


void WDT_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(WDT_CALLBACK,isr_func,param);
}


void EXTINT_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(EXT_CALLBACK,isr_func,param);
}


void RTC_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(RTC_CALLBACK,isr_func,param);
}


void DMAC_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(DMAC_CALLBACK,isr_func,param);
}


void ADCC0_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(ADCC0_CALLBACK,isr_func,param);
}

void ADCC1_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(ADCC1_CALLBACK,isr_func,param);
}

void ADCC2_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(ADCC2_CALLBACK,isr_func,param);
}

void ADCC3_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(ADCC3_CALLBACK,isr_func,param);
}

void ADCC4_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(ADCC4_CALLBACK,isr_func,param);
}

void ADCC5_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(ADCC5_CALLBACK,isr_func,param);
}


void TIMER_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(TIMER_CALLBACK,isr_func,param);
}


void I2C0_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(I2C0_CALLBACK,isr_func,param);
}

void I2C1_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(I2C1_CALLBACK,isr_func,param);
}


void SPIM_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(SPIM_CALLBACK,isr_func,param);
}


void UART_Int_Register(void *isr_func, void *param)					//param is a uart_env_tag
{
	assert_param(isr_func != NULL);
	assert_param(param != NULL);
	int_callback_register(UART_CALLBACK,isr_func,param);
}


void GPIO_Int_Register(void * isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(GPIO_CALLBACK,isr_func,param);
}


void I2S_Int_Register(void *isr_func, void *param)							//param can be NULL
{
	assert_param(isr_func != NULL);
	int_callback_register(I2S_CALLBACK,isr_func,param);
}



