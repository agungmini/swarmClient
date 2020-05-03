#ifndef __ESP01_AT_H_
#define __ESP01_AT_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include <man_uart_stm32f4.h>
#include "string.h"

void esp_restart(USART_TypeDef* USARTx);
void connect_ssid(USART_TypeDef* USARTx,const char *SSID,const char *pass);
void static_ip(USART_TypeDef* USARTx,const char *own_address,const char *gateway,const char *mask);
void set_udp_connection(USART_TypeDef* USARTx,const char *IP_dest,uint8_t ID,uint16_t port);
void send_udp(USART_TypeDef* USARTx,uint8_t ID,const char *ptr);

#ifdef __cplusplus
}
#endif

#endif /* LCD_I2C_H_ */
