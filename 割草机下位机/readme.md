Lawn mower lower machine  
IAP:
use usart1 to update the code 
the config of the usart1 is :

	Baud rate :115200
	word length : 8 bits
	stop bits : 1
	parity : 0
	mode : rx tx

use Ymodem to transfer code ,Group size 1024 bytes

Application:
use the freeRTOS to Scheduling module

STM32Cubemx version 5.30  
keil version 5.25.2.0
the package name and version  STM32Cube FW_F1 V1.8.0