
#ifndef __UART_H__
#define __UART_H__
//-----------------------------------------------
#include "stdint.h"
#include "usart.h"
#include "stm32f1xx_it.h"
#include "bno005.h"
#include "time.h"
#include "gps.h"
#include "crc.h"
#include "string.h"
#include "cmsis_os.h"


//-----------------------------------------------


/*DMA接收长度宏定义*/
#define UART_DMA_BUFFER_SIZE 200
#define TX_BUF_SIZE 20
#define UART3_DMA_BUFFER_SIZE 40
extern osSemaphoreId Analyse_Gps_SemHandle;
extern osSemaphoreId Uart3_Rcv_SemHandle;
//-----------------------------------------------
typedef struct _UART{
uint8_t rx_buffer[UART_DMA_BUFFER_SIZE];
uint8_t rx_buffer_cpy[UART_DMA_BUFFER_SIZE];
uint8_t tx_buffer[TX_BUF_SIZE];
uint8_t recv_end_flag;
uint8_t recv3_end_flag;
uint16_t rx_len;
uint16_t rx3_len;
uint16_t tx_len;
}ST_uart,*ST_uart_p;
//-----------------------------------------------


typedef struct _Uart_Handle{
void (*read_uart_idle)(void);
void (*uart_init)(void);
void (*uart_send_test)(void);
char* (*get_gps_buff_ptr)(void);
int8_t (*restart_uart_receive_dma)(void);	
void (*uart_rcv_gps)(uint32_t time_temp);
void (*read_uart3_idle)(void);
void (*uart3_init)(void);
void (*uart3_rcv_date)(void);
}ST_uart_handle,*ST_uart_handle_p;
extern ST_uart_handle uart_handle;
extern ST_uart_handle_p uart_handle_p;
//-----------------------------------------------

//-----------------------------------------------

#endif

