/*
 * MYSerial.cpp
 *
 *  Created on: 2020Äê2ÔÂ2ÈÕ
 *      Author: Administrator
 */

#include "MYSerial.h"
#include "chip.h"
#include "debug.h"
#include "syscon.h"
#include "platform.h"
#include <libmaple/gpio.h>



MYSerial::MYSerial() {
	// TODO Auto-generated constructor stub
}

MYSerial::~MYSerial() {
	// TODO Auto-generated destructor stub
}

void MYSerial::begin(uint32_t baud) {

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

size_m MYSerial::write(char ch){
	HAL_UART_WriteChar_Polling((uint8)ch);
	return (size_m)1;
}

size_m MYSerial::write(uint8 *buf){
    if (buf == NULL) return 0;
    uint32 len=0;
    uint8 *bufptr = buf;
    for(len = 0; *bufptr != 0; len++)
    	bufptr ++;
    return write(buf,len);
}

size_m MYSerial::write(const void *buf, uint32 size){
    HAL_UART_Write_Polling((uint8 *)buf, size);
    return (size_m)size;

}

size_m MYSerial::print(char c) {
	return write(c);
}

size_m MYSerial::print(const char *buf) {
	return write((uint8*)buf);
}

size_m MYSerial::print(uint8 *buf) {
	return write(buf);
}

//size_m MYSerial::print(uint8 num, uint8 base) {
//	return print((unsigned long long)num, base);
//}
//
//size_m MYSerial::print(unsigned short num, uint8 base) {
//	return print((unsigned long long)num, base);
//}
//
//size_m MYSerial::print(unsigned int num, uint8 base) {
//	return print((unsigned long long)num, base);
//}
//
//size_m MYSerial::print(unsigned long num, uint8 base) {
//	return print((unsigned long long)num, base);
//}
//
//size_m MYSerial::print(unsigned long long num, uint8 base) {
//	return printNumber(num, base);
//}
//
//size_m MYSerial::print( short num, uint8 base) {
//	return print((long long)num, base);
//}
//
//size_m MYSerial::print( int num, uint8 base) {
//	return print((long long)num, base);
//}
//
//size_m MYSerial::print( long num, uint8 base) {
//	return print((long long)num, base);
//}
//
//size_m MYSerial::print( long long num, uint8 base) {
//	size_m n = 0;
//	if(num < 0){
//		n = print('-');
//		num = -num;
//	}
//	n += printNumber((unsigned long long)num, base);
//	return n;
//}

size_m MYSerial::print( long long num, uint8 base) {
	uint8 buf[2];
	buf[0] = '0' + num / base;
	buf[1] = '0' + num % base;
	return print(buf);
}

size_m MYSerial::println(void) {
	size_m n = print('\r');
    n += print('\n');
	return n;
}

size_m MYSerial::println(char c) {
    size_m n = print(c);
    n += println();
	return n;
}

size_m MYSerial::println(const char *buf) {
    size_m n = print(buf);
    n += println();
	return n;
}

size_m MYSerial::println(uint8 *buf) {
    size_m n = print(buf);
    n += println();
	return n;
}

//size_m MYSerial::println(uint8 num, uint8 base) {
//	size_m n = print(num, base);
//	n += println();
//	return n;
//}
//
//size_m MYSerial::println(unsigned short num, uint8 base) {
//	size_m n = print(num, base);
//	n += println();
//	return n;
//}
//
//size_m MYSerial::println(unsigned int num, uint8 base) {
//	size_m n = print(num, base);
//	n += println();
//	return n;
//}
//
//size_m MYSerial::println(unsigned long num, uint8 base) {
//	size_m n = print(num, base);
//	n += println();
//	return n;
//}
//
//size_m MYSerial::println(unsigned long long num, uint8 base) {
//	size_m n = print(num, base);
//	n += println();
//	return n;
//}
//
//size_m MYSerial::println( short num, uint8 base) {
//	size_m n = print(num, base);
//	n += println();
//	return n;
//}
//
//size_m MYSerial::println( int num, uint8 base) {
//	size_m n = print(num, base);
//	n += println();
//	return n;
//}
//
//size_m MYSerial::println( long num, uint8 base) {
//	size_m n = print(num, base);
//	n += println();
//	return n;
//}
//
//size_m MYSerial::println( long long num, uint8 base) {
//	size_m n = print(num, base);
//	n += println();
//	return n;
//}

size_m MYSerial::printNumber(unsigned long long num, uint8 base) {
    uint8 buf[100];
    uint8 i = 0;
    size_m n = 0;
    if (num == 0) {
        print('0');
        return 1;
    }

    while (num > 0) {
        buf[i++] = num % base;
        num /= base;
    }

    for (; i > 0; i--) {
        n += print((char)((buf[i - 1] < 10 )? ('0' + buf[i - 1]) : ('A' + buf[i - 1] - 10)));
    }
	return n;
}



/***************************************************************************************/
#include "reg_uart.h"

static UART_Env_Tag uart_env;

static void uart_rec_data_avail_isr(void)
{

    void (*callback) (void*,uint8_t) = NULL;
    void *data = NULL;
    uint8_t rec_char;

    while(uart_dr_getf()==data_ready)
    {
        if(uart_env.read_mode == UART_READ_FIXLEN)
        {
            // Read the received in the FIFO
            *uart_env.rx.bufptr = uart_rbr_get();
    //        LOG(LOG_LVL_INFO,"%d ",*uart_env.rx.bufptr);
            // Update RX parameters
            uart_env.rx.size--;
            uart_env.rx.bufptr++;

            // Check if all expected data have been received
            if (uart_env.rx.size == 0)
            {
                // Reset RX parameters
                uart_env.rx.bufptr = NULL;

                // Disable Rx interrupt
                uart_erbfi_setf(0);        //Received_Data_Available_Interrupt_Disabled

                // Retrieve callback pointer
                callback = uart_env.rx.callback;
                data = uart_env.rx.dummy;

                if(callback != NULL)
                {
                    // Clear callback pointer
                    uart_env.rx.callback = NULL;
                    uart_env.rx.dummy = NULL;
                    if(uart_env.errordetect==false)
                    {// Call handler
                        callback(data, STATUS_OK);
                    }else
                    {
                        uart_env.errordetect = false;
                        callback(data,STATUS_ERROR);
                    }
                }
                else
                {
                    assert_param(0);
                }

                // Exit loop
                break;
            }
        }
        else if(uart_env.read_mode == UART_READ_TILL_CHAR)
        {
            rec_char = uart_rbr_get();
            if(uart_env.echo_enable == 1)
            {
                while(uart_tfnf_getf()==0);        //Transmit_FIFO_Full
                uart_thr_set(rec_char);
            }
            // Read the received in the FIFO
            *uart_env.rx.bufptr = rec_char;
            uart_env.rx.bufptr++;
            if(rec_char == uart_env.till_char)
            {
                // Disable Rx interrupt
                uart_erbfi_setf(0);

                // Retrieve callback pointer
                callback = uart_env.rx.callback;
                data = uart_env.rx.dummy;

                if(callback != NULL)
                {
                    // Clear callback pointer
                    uart_env.rx.callback = NULL;
                    uart_env.rx.dummy = NULL;
                    if(uart_env.errordetect==false)
                    {// Call handler
                        callback(data, STATUS_OK);
                    }else
                    {
                        uart_env.errordetect = false;
                        callback(data,STATUS_ERROR);
                    }
                }
                else
                {
                    assert_param(0);
                }

                // Exit loop
                break;
            }

        }
    }

}

/**
 ****************************************************************************************
 * @brief Serves the receive data error interrupt requests. It clears the requests and
 *        executes the callback function.
 *
 ****************************************************************************************
 */

static void uart_rec_error_isr(void)
{
	void (*callback) (void*,uint8_t) = NULL;
	void *data = NULL;
	uint8_t LSR = uart_lsr_get();

	if((LSR & overrun_error)!=0)			//Overrun_Error
	{
//		LOG(LOG_LVL_ERROR,"uart overrun\n");
		while(uart_dr_getf()==data_ready)
		{
			uart_rbr_get();
		}
        if(callback != NULL)
        {
            // Clear callback pointer
            uart_env.rx.callback = NULL;
            uart_env.rx.dummy = NULL;

            // Call handler
            callback(data, STATUS_ERROR);
        }
	}
	if((LSR & framing_error)!=0)			//Framing_Error
	{
//		LOG(LOG_LVL_ERROR,"uart framing error\n");
		uart_env.errordetect = true;
	}
}

/**
 ****************************************************************************************
 * @brief Serves the transmit data fill interrupt requests. It clears the requests and
 *        executes the callback function.
 *
 * The callback function is called as soon as the last byte of the provided data is
 * put into the FIFO. The interrupt is disabled at the same time.
 ****************************************************************************************
 */
static void uart_thr_empty_isr(void)
{
	void (*callback) (void*,uint8_t) = NULL;
	void * dummy = NULL;

    // Fill TX FIFO until there is no more room inside it
    while (uart_tfnf_getf()==1)
    {
        // Put a byte in the FIFO
        uart_thr_set(*uart_env.tx.bufptr);

        // Update TX parameters
        uart_env.tx.size--;
        uart_env.tx.bufptr++;

        if (uart_env.tx.size == 0)
        {
            // Reset TX parameters
            uart_env.tx.bufptr = NULL;

            // Disable TX interrupt
			uart_etbei_setf(0);

            // Retrieve callback pointer
            callback = uart_env.tx.callback;
            dummy = uart_env.tx.dummy;

            if(callback != NULL)
            {
                // Clear callback pointer
                uart_env.tx.callback = NULL;
                uart_env.tx.dummy = NULL;

                // Call handler
                callback(dummy, STATUS_OK);
            }
            else
            {
                assert_param(0);
            }

            // Exit loop
            break;
        }
    }
}


static void HAL_UART_Set_BaudRate(uint32_t uart_src_clk, uint32_t baud)
{
	uint16_t divisor;
	uint8_t frac;
	divisor = UART_DIVISOR_INTEGER(uart_src_clk, baud);
	frac = UART_DLF(uart_src_clk,baud);
	uart_dlab_setf(1);
	uart_dll_set((uint8_t)divisor);
	uart_dlh_set((uint8_t)(divisor>>8));
	uart_dlf_set(frac);
	uart_dlab_setf(0);
}


void HAL_UART_Init(uint32_t uart_src_clk, UART_InitTypeDef config)
{
 	HAL_UART_Set_BaudRate(uart_src_clk,config.UART_BaudRate);
    if(config.UART_PariTy == none_parity_check)
    {
        uart_pen_setf(0);
    }
    else
    {
        uart_pen_setf(1);
        if(config.UART_PariTy == even_parity_check)
            uart_eps_setf(1);
        else
            uart_eps_setf(0);
    }
    uart_stop_setf(config.UART_StopBits);
	uart_dls_setf(config.UART_DataLen);
    if(config.FlowControl == hardware_flow_control)
        uart_afce_setf(1);
    else
        uart_afce_setf(0);

}


void HAL_UART_INTConfig(uint8_t irq_map)
{
    uart_ier_set(irq_map);
}

uint8_t HAL_UART_Int_Status(void)
{
    return uart_iid_getf();
}


void HAL_UART_FIFO_Control(UART_TX_TrgLvl tx_lvl, UART_RX_TrgLvl rx_lvl)
{
    uart_fcr_pack(rx_lvl,tx_lvl,1,0,0,1);
}


void HAL_UART_Flow_On(void)
{
	uart_rts_setf(1);		//RTS_Signal_Low
}


bool HAL_UART_Flow_Off(void)
{
    bool flow_off = true;
    //GLOBAL_INT_DISABLE();
    if((uart_rfne_getf()==0) && (uart_tfe_getf()==1))
    {
    	uart_rts_setf(0);		//RTS_Signal_High
    }else
    {
    	flow_off = false;
    }
    //GLOBAL_INT_RESTORE();
    return (flow_off);
}


void HAL_UART_Finish_Transfers(void)
{
	uart_rts_setf(0);		//RTS_Signal_High
	while(uart_tfe_getf()==0);
}


uint16_t HAL_UART_Get_Line_Status()
{
    return uart_lsr_get();

}


uint8_t HAL_UART_Get_Modem_Status()
{

    return uart_msr_get();

}


uint8_t HAL_UART_Normal_Status()
{
    return uart_usr_get();

}


void HAL_UART_Read(uint8_t *bufptr, uint32_t size, void (*callback) (void*,uint8_t),void* dummy)
{
	// Sanity check
    assert_param(bufptr != NULL);
    assert_param(size != 0);
    assert_param(callback != NULL);

    // Prepare RX parameters
    uart_env.rx.size = size;
    uart_env.rx.bufptr = bufptr;
    uart_env.rx.callback = callback;
    uart_env.rx.dummy = dummy;
    uart_env.read_mode= UART_READ_FIXLEN;

    uart_erbfi_setf(1);		//Received_Data_Available_Interrupt_Enabled
}



void HAL_UART_Read_Till(uint8_t *bufptr, uint8_t till_char, uint8_t echo_en, void (*callback) (void*,uint8_t),void* dummy)
{
	// Sanity check
    assert_param(bufptr != NULL);
    assert_param(callback != NULL);

    // Prepare RX parameters
    uart_env.rx.bufptr = bufptr;
    uart_env.rx.callback = callback;
    uart_env.rx.dummy = dummy;
    uart_env.read_mode= UART_READ_TILL_CHAR;
    uart_env.till_char = till_char;
    uart_env.echo_enable = echo_en;

    uart_erbfi_setf(1);		//Received_Data_Available_Interrupt_Enabled

}

void HAL_UART_Write(uint8_t *bufptr, uint32_t size, void (*callback) (void*,uint8_t), void *dummy)
{
    // Sanity check
    assert_param(bufptr != NULL);
    assert_param(size != 0);
    assert_param(callback != NULL);

    // Prepare TX parameters
    uart_env.tx.size = size;
    uart_env.tx.bufptr = bufptr;
    uart_env.tx.callback = callback;
    uart_env.tx.dummy = dummy;

	uart_etbei_setf(1);			//Transmit_Holding_Register_Empty_Interrupt_Enabled
}


void HAL_UART_WriteChar_Polling(uint8_t ch)
{
    while(uart_tfnf_getf()==0);		//Transmit_FIFO_Full
    uart_thr_set(ch);

}

void HAL_UART_Write_Polling(uint8_t *bufptr, uint32_t size)
{
    uint32_t i;
    for(i=0; i<size; i++)
    {
        HAL_UART_WriteChar_Polling(*bufptr);
        bufptr += 1;
    }
}



uint8_t HAL_UART_ReadChar(uint8_t *ch)               // not polling
{
    if((uart_lsr_get() & data_ready)!=0)
    {
        *ch = uart_rbr_get();
        return 1;
    }
    else
        return 0;
}



void HAL_UART_Isr(void)
{
    UART_Int_Identity irq_stat;
    while(1)
    {
        irq_stat = (UART_Int_Identity)uart_iid_getf();
        if(irq_stat==No_Interrupt_Pending)
            break;
        switch(irq_stat)
        {
        case Receiver_Line_Status_Interrupt:
        	uart_rec_error_isr();
        	break;
        case Received_Data_Available_Interrupt:
        	uart_rec_data_avail_isr();
        	break;
        case THR_Empty_Interrupt:
        	uart_thr_empty_isr();
        	break;
        case Character_Timeout_Interrupt:
        	uart_rec_data_avail_isr();
        	break;
        default:
        	break;
        }

    }
}

