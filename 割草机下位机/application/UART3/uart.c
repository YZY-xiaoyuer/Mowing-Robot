#include "uart.h"
#include "data.h"
#include "uppercmd.h"
#include "uart3_rcv_queue.h"
#include "protocol.h"
//-----------------------------------------------------------
//#pragma pack (1)
static uint8_t  rx3_buffer[UART3_DMA_BUFFER_SIZE] = {0};	
//#pragma pack ()
//------------------------------------------------------------
//--本函数需要放到stm32f1xx_it.c的串口中断函数里面
static void Read_Uart_Idle(void);
//--串口初始化
static void Uart_Init(void);
//--GPS测试用
static void Uart_Send_Test(void);
//--获取GPS字符串
static char* Get_Gps_Buff_Ptr(void);
//--开启串口DMA接收
static int8_t Restart_Uart_Receive_Dma(void);
//--数据打包发送
static void Uart_Rcv_Gps(uint32_t time_temp);
static void Read_Uart3_Idle(void);
static void Uart3_Init(void);
static void Uart3_Rcv_Date(void);
//------------------------------------------------------------
ST_uart   uart = {0};
ST_uart_p uart_p = &uart;
//-----------------------------------------------------------
ST_uart_handle uart_handle = {
Read_Uart_Idle,
Uart_Init,
Uart_Send_Test,
Get_Gps_Buff_Ptr,
Restart_Uart_Receive_Dma,
Uart_Rcv_Gps,
Read_Uart3_Idle,
Uart3_Init,
Uart3_Rcv_Date,
};
ST_uart_handle_p uart_handle_p =&uart_handle;

//------------------------------------------------------------
//--本函数需要放到stm32f1xx_it.c的串口中断函数里面
//--hdma_usart1_rx变量无法识别。解决办法，将cube自动生成的变量定义从stm32f1xx_it.c
//--转移到stm32f1xx_it.h里面
static void Read_Uart_Idle(void)
{
   uint32_t temp_flag = 0;
	 uint32_t temp;
	
  temp_flag = __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE);//--判断空闲标志
	if(temp_flag !=RESET){
	__HAL_UART_CLEAR_IDLEFLAG(&huart2);//--清除空闲标志
		//--读取串口状态寄存器
	temp = huart2.Instance->SR;
		//--读取川久数据寄存器
	temp = huart2.Instance->DR;
	HAL_UART_DMAStop(&huart2);//--关闭DMA
	//--读取DMA剩余传输数量
	temp = hdma_usart2_rx.Instance->CNDTR;
	uart_p->rx_len = UART_DMA_BUFFER_SIZE - temp;//--计算数据长度
	uart_p->recv_end_flag = 1;
  osSemaphoreRelease(Analyse_Gps_SemHandle);//--发送信号量
	}
}
static void Read_Uart3_Idle(void)
{
   uint32_t temp_flag = 0;
	 uint32_t temp;
	
  temp_flag = __HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE);//--判断空闲标志
	if(temp_flag !=RESET){
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);//--清除空闲标志
		//--读取串口状态寄存器
	temp = huart3.Instance->SR;
		//--读取川久数据寄存器
	temp = huart3.Instance->DR;
	HAL_UART_DMAStop(&huart3);//--关闭DMA
	//--读取DMA剩余传输数量
	temp = hdma_usart3_rx.Instance->CNDTR;
	uart_p->rx3_len = UART3_DMA_BUFFER_SIZE - temp;//--计算数据长度
	uart_p->recv3_end_flag = 1;
  osSemaphoreRelease(Uart3_Rcv_SemHandle);//--发送信号量
	}
}
//--串口初始化
static void Uart_Init(void)
{
	//--开启串口空闲中断
 __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);//--gps接收
	//--开启串口DMA接收
HAL_UART_Receive_DMA(&huart2,uart_p->rx_buffer,UART_DMA_BUFFER_SIZE);
}
static void Uart3_Init(void)
{
	//--开启串口空闲中断
__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);//--上位机接收
	//--开启串口DMA接收
HAL_UART_Receive_DMA(&huart3,rx3_buffer,UART3_DMA_BUFFER_SIZE);
}
//--GPS测试用
static void Uart_Send_Test(void)
{
 if(uart_p->recv_end_flag == 1)
  {
	  memcpy(uart_p->rx_buffer_cpy,uart_p->rx_buffer,uart_p->rx_len);
	  HAL_UART_Transmit_DMA(&huart3,uart_p->rx_buffer_cpy,uart_p->rx_len);
		memset(uart_p->rx_buffer,0,UART_DMA_BUFFER_SIZE);
		uart_handle_p->restart_uart_receive_dma();	//--重启DMA	
	}
}

//--串口接收GPS数据
static void Uart_Rcv_Gps(uint32_t time_temp)
{
	 if(uart_p->recv_end_flag == 1)//--更新GPS数据数据
  {
	memcpy(uart_p->rx_buffer_cpy,uart_p->rx_buffer,uart_p->rx_len);
  gps_handle_p->gps_gnrmc_analyse(uart_handle_p->get_gps_buff_ptr());//--解析GPS数据
	Gps_Get_Time(time_temp);//--更新GPS数据时间
	uart_handle_p->restart_uart_receive_dma();	//--重启DMA	
	}

}		

//--------------------------------------------------------------
//--获取GPS字符串
static char* Get_Gps_Buff_Ptr(void)
{
  if(uart_p->recv_end_flag == 1)
  {

		return (char*)uart_p->rx_buffer_cpy;
	}
	return NULL;
}
//--开启串口DMA接收
static int8_t Restart_Uart_Receive_Dma(void)
{
  uart_p->rx_len = 0;
	uart_p->recv_end_flag = 0;
		//--开启串口DMA接收
   HAL_UART_Receive_DMA(&huart2,uart_p->rx_buffer,UART_DMA_BUFFER_SIZE);
	return 0;
}


//--串口接收上位机数据//--小端模式
static void Uart3_Rcv_Date(void)
{
	if(uart_p->recv3_end_flag == 1)//--接收到上位机数据
  {	
		/*判断包头*/
		if(rx3_buffer[0] != PROTOCOL_HEAD) goto uart3_end2;
		/*判断包长*/
		uint32_t len = 0;
		memcpy(&len,&rx3_buffer[1],4);
		if(len != uart_p->rx3_len) goto uart3_end2;
		/*判断包尾*/
		if(rx3_buffer[len-1] != PROTOCOL_TAIL) goto uart3_end2;
		/*计算CRC*/
		uint8_t crc = crc8_chk_value((unsigned char *)(rx3_buffer+1),len-3);
		/*判断CRC*/
		if (crc != rx3_buffer[len-2]) goto uart3_end2;
		/*判断包类型*/
		printf("0x%x\r\n",rx3_buffer[5]);	
		switch(rx3_buffer[5]){
			case E_PROTOCOL_NULL_CMD:				   
				break;
			case E_PROTOCOL_MOVING_CMD:	
			   {
					 if(upper_cmd_header_p->get_remote_flag() == 1)
					   {
						
					  /*设置非遥控状态*/
					    upper_cmd_header_p->set_remote_flag(0);
					    /*释放命令队列所有数据*/
					    queue_handle_p->free_moving_cmd_list();
					   }
					 if(queue_handle_p->list_is_null() == 1)//--链表是空的
					 {
							/*创建一个空节点用于保存轨迹跟踪数据*/
							queue_handle_p->insert_to_moving_cmd_list(queue_handle_p->creat_moving_cmd_node(255,0,0,0,0));					 
					 }					 
				   /*判断是否是老命令包*/
		       if(queue_handle_p->get_last_package_id() == rx3_buffer[6]) goto uart3_end1;
			     		/*解析数据*/
							int16_t v_temp;
			        memcpy(&v_temp,&rx3_buffer[8],2);
			        float w_temp;
			        memcpy(&w_temp,&rx3_buffer[10],4);
							float a_temp ;
              memcpy(&a_temp,&rx3_buffer[14],4);
						 /*更新轨迹跟踪数据*/
						 queue_handle_p->update_track_tracking_cmd(rx3_buffer[7],rx3_buffer[6],v_temp,w_temp,a_temp);
						 upper_cmd_header_p->set_update_cmd_flag(1);//--设置更新命令标志
		        /*将数据加入命令队列*/
	          // queue_handle_p->insert_to_moving_cmd_list(queue_handle_p->creat_moving_cmd_node(rx3_buffer[6],rx3_buffer[7],v_temp,w_temp,a_temp));
			    }
				break;
			case E_PROTOCOL_HEADING_SYCN:	
           upper_cmd_header_p->set_package_type((E_upper_package_type)rx3_buffer[5]);				
				break;
			case E_PROTOCOL_POSITION_SYCN:	
           upper_cmd_header_p->set_package_type((E_upper_package_type)rx3_buffer[5]);								
				break;
			case E_PROTOCOL_LOCATION_DATA:
            upper_cmd_header_p->set_package_type((E_upper_package_type)rx3_buffer[5]);	
            printf("2\r\n");			
				break;
			case E_PROTOCOL_MACHINE_STATE:
            upper_cmd_header_p->set_package_type((E_upper_package_type)rx3_buffer[5]);								
				break;
		  case E_PROTOCOL_ANSWER_CMD:			
				break;
			case E_PROTOCOL_REMOTE_CMD://--遥控
			       {
							 if(upper_cmd_header_p->get_remote_flag() == 0)//--如果上一个状态不是遥控
					     {
					       /*设置遥控状态*/
					      upper_cmd_header_p->set_remote_flag(1);
					      /*释放命令队列所有数据*/
					      queue_handle_p->free_moving_cmd_list();
					     }
				if(queue_handle_p->list_is_null() == 1)//--链表是空的
					 {
							/*创建一个空节点用于保存轨迹跟踪数据*/
							queue_handle_p->insert_to_moving_cmd_list(queue_handle_p->creat_moving_cmd_node(0,0,0,0,0));					 
					 }	
			     		/*解析数据*/
							int16_t v_temp;
			        memcpy(&v_temp ,&rx3_buffer[8],2);	 
			        float w_temp;
			        memcpy(&w_temp,&rx3_buffer[10],4);
							//float a_temp ;
              //memcpy(&a_temp,&rx3_buffer[14],4);
							/*更新遥控数据*/
							queue_handle_p->update_remote_cmd(rx3_buffer[7],v_temp,w_temp);
			      }
				break;
		  case E_PROTOCOL_SIZE:
				break;
			default:
				goto uart3_end2;
		}
		
	 uart3_end1:
	 	/*设置回应标志*/
	 if(rx3_buffer[len-3]) upper_cmd_header_p->set_reply_flag(1);
	 else                  upper_cmd_header_p->set_reply_flag(0);
   uart3_end2: 
	 /*清除接受缓存和接受标志*/
	 uart_p->rx3_len = 0;
	 uart_p->recv3_end_flag = 0;
	 memset(rx3_buffer,0,40);
		//--开启串口DMA接收
   HAL_UART_Receive_DMA(&huart3,rx3_buffer,UART3_DMA_BUFFER_SIZE);
	}

}	