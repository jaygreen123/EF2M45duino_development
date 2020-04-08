/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 * Copyright (c) 2012 LeafLabs, LLC.
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

/*
 * STM32F1 implementations for basic GPIO functionality.
 */

#include <io.h>

#include <libmaple/gpio.h>
#include <libmaple/timer.h>

#include <boards.h>



void pinMode(uint8 pin, WiringPinMode mode) {
	GPIO_InitTypeDef gpio_mode;
	if(pin >= gpioMax){
		return;
	}

	switch(mode){
	case OUTPUT:
		gpio_mode.dir = gpio_Output;
		gpio_mode.debounce = gpio_Deb;
		gpio_mode.value = gpio_High;
		break;
	case INPUT:
		gpio_mode.dir = gpio_Input;
		gpio_mode.debounce = gpio_Deb;
		break;
	case PWM:

		break;
	default:
		ASSERT(0);
		return;
	}
	HAL_GPIO_Init((GPIO_Num)pin,gpio_mode);

}

#define ELF2_In32(Addr)  (*(volatile uint32_t *)(Addr))
#define ELF2_Out32(Addr, Value)  (*(volatile uint32_t *)((Addr)) = (Value))
#define ELF2_BRAM256_BaseAddr       0x20000000
#define size_n  8192
#define pwm_s 8100
#define pwm_n 10

void bram256kbit_init(){
	uint32_t i;
	for(i=0; i<size_n; i++)
        ELF2_Out32(ELF2_BRAM256_BaseAddr + i*4, 50);
}

void bram256kbit_write(uint32_t addr, uint32_t data){
	ELF2_Out32(ELF2_BRAM256_BaseAddr + addr * 4, data);
}

void pwmInit(){
	uint32_t i;
	for(i=pwm_s; i<pwm_s+pwm_n; i++)
	    ELF2_Out32(ELF2_BRAM256_BaseAddr + i*4, 0);
}

void pwmWrite(uint32_t addr, uint32_t data){
	ELF2_Out32(ELF2_BRAM256_BaseAddr + (pwm_s + addr) * 4, data);
}

#include "MYSerial.h"
#include "chip.h"
#include "debug.h"
#include "syscon.h"
#include "platform.h"
void begin2(uint32_t baud){
	HAL_SYSCON_Function_IO_Set(UART_TXD,GPIO_4,1);
	HAL_SYSCON_Function_IO_Set(UART_RXD,GPIO_3,1);

	UART_InitTypeDef config;
	config.UART_BaudRate = baud;
	config.UART_DataLen = Data_Length_8_bits;
	config.UART_PariTy = none_parity_check;
	config.UART_StopBits = stop_bit_1;
	HAL_UART_Init(APB_CLK0,config);

	HAL_UART_FIFO_Control(TX_Empty_Trigger_FIFO_Empty,RCVR_Trigger_One_Character);
}

#include "chip.h"
#include "reg_gpio.h"

#define GPIO_PWIDTH_A 16

typedef enum
{
    gpio_Level = 0,
    gpio_Edge
}GPIO_IrqLvl;

typedef enum
{
    gpio_Low_Falling = 0,
    gpio_High_Rising
}GPIO_Ply;



static void HAL_GPIO_SetPly(GPIO_Num n, GPIO_Ply ply)
{
    uint32_t polarity = gpio_int_polarity_getf();

    if(gpio_High_Rising == ply){
        polarity |= (1 << n);
    }else{
        polarity &= ~(1 << n);
    }

    gpio_int_polarity_setf(polarity);
}
static void HAL_GPIO_setIrqLevel(GPIO_Num n, GPIO_IrqLvl irqLvl)
{
    uint32_t irqLevel = gpio_int_level_getf();

    if(gpio_Edge== irqLvl){
        irqLevel |= (1 << n);
    }else{
        irqLevel &= ~(1 << n);
    }
    gpio_int_level_setf(irqLevel);
}


/**
 * @brief Set the GPIO trigger type.
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @param  type: the GPIO_TrigType, choosing from enum GPIO_TrigType
 * @return This function has no return value.
 */
void HAL_GPIO_TrigType(GPIO_Num n, GPIO_TrigType type)
{
	switch(type)
	{
		case gpio_Low_Level:
			HAL_GPIO_setIrqLevel(n, gpio_Level);
			HAL_GPIO_SetPly(n, gpio_Low_Falling);
			break;
		case gpio_High_Level:
			HAL_GPIO_setIrqLevel(n, gpio_Level);
			HAL_GPIO_SetPly(n, gpio_High_Rising);
			break;
		case gpio_Falling_Edge:
			HAL_GPIO_setIrqLevel(n, gpio_Edge);
			HAL_GPIO_SetPly(n, gpio_Low_Falling);
			break;
		case gpio_Rising_Edge:
			HAL_GPIO_setIrqLevel(n, gpio_Edge);
			HAL_GPIO_SetPly(n, gpio_High_Rising);
			break;
	}
}

/**
 * @brief Set the GPIO to trigger a interrupt at both edge, rising and falling.
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @param  enable: enable or disable whether a gpio trigger an interrupt at both edge.
 * @return This function has no return value.
 */
void HAL_GPIO_TrigBothEdge(GPIO_Num n, uint8_t enable)
{
	uint32_t reg = gpio_int_both_edge_getf();
	if(enable)
		gpio_int_both_edge_setf(reg | (1<<n));
	else
		gpio_int_both_edge_setf(reg & (~(1<<n)));
}

/**
 * @brief Controls whether an external signal that is the source of an interrupt needs to be debounced to remove any spurious glitches.
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @return This function has no return value.
 */
void HAL_GPIO_SetDebounce(GPIO_Num n, GPIO_Debounce debounce)
{
    uint32_t debReg = gpio_debounceenable_getf();

    if(debounce == gpio_Deb)
    {
        debReg |= (1 << n);
    }
    else
    {
        debReg &= ~(1 << n);
    }
    gpio_debounceenable_setf(debReg);
}

/**
 * @brief GPIO Initialize.
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @param  config: a struct to initialize a GPIO.
 * @return This function has no return value.
 */
void HAL_GPIO_Init(GPIO_Num n, GPIO_InitTypeDef config)
{
    HAL_GPIO_SetDir(n, config.dir);
    if(gpio_Output == config.dir)
	{
        HAL_GPIO_WritePin(n, config.value);
    }
    else
	{
		HAL_GPIO_SetDebounce(n, config.debounce);
		HAL_GPIO_TrigType(n,config.trig_type);
	}
}

/**
 * @brief Set GPIO Direction, Input or Output
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @param  dir: choose from GPIO_Direction, gpio_Input or gpio_Output
 * @return This function has no return value.
 */
void HAL_GPIO_SetDir(GPIO_Num n, GPIO_Dir dir)
{
    uint32_t dirReg;

    if(n<GPIO_PWIDTH_A){
        dirReg = gpio_portadatadirectionregister_getf();
        if(gpio_Output == dir){
            dirReg |= 1<<n;
        }else{
            dirReg &= ~(1<<n);
        }
        gpio_portadatadirectionregister_setf(dirReg);
    }else{
        dirReg = gpio_portbdatadirectionregister_getf();
        if(gpio_Output == dir){
            dirReg |= 1<<(n-GPIO_PWIDTH_A);
        }else{
            dirReg &= ~(1<<(n-GPIO_PWIDTH_A));
        }
        gpio_portbdatadirectionregister_setf(dirReg);
    }
}

/**
 * @brief Get GPIO Direction, Input or Output
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @return return the direction of the specific gpio
 */
GPIO_Dir HAL_GPIO_GetDir(GPIO_Num n)
{
	uint32_t dir;
	if(n<GPIO_PWIDTH_A){
        dir = (gpio_portadatadirectionregister_getf()>>n)&0x1;
    }else{
        dir = (gpio_portbdatadirectionregister_getf()>>(n-GPIO_PWIDTH_A))&0x1;
    }
    return (GPIO_Dir)dir;
}

/**
 * @brief Enable the interrupt of specific GPIO
 * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
 * @return This function has no return value.
 */
void HAL_GPIO_IntEnable(GPIO_Num n)
{
    uint32_t irqEnableReg = gpio_interruptenable_getf();

    irqEnableReg |= (1 << n);
    gpio_interruptenable_setf(irqEnableReg);
}

/**
 * @brief Disable the interrupt of specific GPIO
 * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
 * @return This function has no return value.
 */
void HAL_GPIO_IntDisable(GPIO_Num n)
{
    uint32_t irqEnableReg = gpio_interruptenable_getf();

    irqEnableReg &= ~(1 << n);
    gpio_interruptenable_setf(irqEnableReg);
}

/**
 * @brief When GPIO direction is output, write value to set gpio level.
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @param  value: value can be choose from GPIO_Value, gpio_Low or gpio_High
 * @return This function has no return value.
 */
void HAL_GPIO_WritePin(GPIO_Num n, GPIO_Value value)
{
    uint32_t valueReg;

    if(n<GPIO_PWIDTH_A){
        valueReg = gpio_portadataregister_getf();
        if(gpio_Low == value){
            valueReg &= ~(1 << n);
        }else{
            valueReg |= (1 << n);
        }
        gpio_portadataregister_setf(valueReg);
    }else{
        valueReg = gpio_portbdataregister_getf();
        if(gpio_Low == value){
            valueReg &= ~(1 << (n-GPIO_PWIDTH_A));
        }else{
            valueReg |= (1 << (n-GPIO_PWIDTH_A));
        }
        gpio_portbdataregister_setf(valueReg);
    }
}

/**
 * @brief When GPIO direction is input, read current gpio level.
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @return return the result of current gpio level.
 */
GPIO_Value HAL_GPIO_ReadPin(GPIO_Num n)
{
    uint32_t value;

    if(n<GPIO_PWIDTH_A){
        value = (gpio_ext_porta_getf()>>n)&0x1;
    }else{
        value = (gpio_ext_portb_getf()>>(n-GPIO_PWIDTH_A))&0x1;
    }
    return (GPIO_Value)value;
}

/**
 * @brief Toggle a gpio pin
 * @param  n: the GPIO_Num to define which GPIO to operate.
 * @return This function has no return value.
 */
void HAL_GPIO_TogglePin(GPIO_Num n)
{
    uint32_t value;
    uint32_t valueReg;
    if(n<GPIO_PWIDTH_A){
        value = (gpio_ext_porta_getf()>>n)&0x1;

        valueReg = (gpio_portadataregister_getf() & (~(1<<n))) | ((~value)<<n);

        gpio_portadataregister_setf(valueReg);
    }else{
        value = (gpio_ext_portb_getf()>>(n-GPIO_PWIDTH_A))&0x1;

        valueReg = (gpio_portbdataregister_getf() & (~(1<<(n-GPIO_PWIDTH_A)))) | ((~value)<<(n-GPIO_PWIDTH_A));

        gpio_portbdataregister_setf(valueReg);
    }
}

/**
 * @brief Mask the interrupt of specific GPIO, when the interrupt is masked, no interrupt will trigger to CPU
 * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
 * @return This function has no return value.
 */
void HAL_GPIO_MaskIrq(GPIO_Num n)
{
    uint32_t mask = gpio_int_mask_getf();

    mask |= (1 << n);

    gpio_int_mask_setf(mask);
}

/**
 * @brief Unmask the interrupt of specific GPIO, when the interrupt is unmasked, the interrupt will trigger to CPU
 * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
 * @return This function has no return value.
 */
void HAL_GPIO_UnmaskIrq(GPIO_Num n)
{
    uint32_t mask = gpio_int_mask_getf();

    mask &= ~(1 << n);

    gpio_int_mask_setf(mask);
}

/**
 * @brief Get all the interrupt status of gpio
 * @return return all the interrupt status of gpio ranging from ggpio0_0 to gpio0_15
 */
uint32_t HAL_GPIO_IntStatus(void)
{
	return gpio_int_status_getf();
}

/**
 * @brief Clear the interrupt of specific gpio
 * @param  n: the GPIO_Num to define which GPIO to operate. n can only choose from gpio0_0 to gpio0_15
 * @return This function has no return value.
 */
void HAL_GPIO_ClrIrq(GPIO_Num n)
{
    gpio_gpio_porta_eoi_set(1<<n);
}

/// @} GPIO



