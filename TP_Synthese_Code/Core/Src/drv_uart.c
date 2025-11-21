
#include "drv_uart.h"

#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"

extern SemaphoreHandle_t uartRxSemaphore;
extern uint8_t rxCharBuffer;

uint8_t drv_uart_receive(char * pData, uint16_t size)
{
		xSemaphoreTake(uartRxSemaphore, portMAX_DELAY);
		*pData = rxCharBuffer;

	return 0;
}

uint8_t drv_uart_transmit(const char * pData, uint16_t size)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)pData, size, HAL_MAX_DELAY);

	return 0;
}
