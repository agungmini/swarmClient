#ifndef __ESP01_AT_H_
#define __ESP01_AT_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include <man_uart_stm32f4.h>
#include "string.h"
#include "stdio.h"
#include "stdlib.h"

/*fungsi untuk mereset ESP8266
 * USARTx merupakan peripheral USART yang digunakan untuk memberikan perintah ke ESP8266*/
void esp_restart(USART_TypeDef* USARTx);

/*fungsi untuk terhubung oleh access point
 * USARTx merupakan peripheral USART yang digunakan untuk memberikan perintah ke ESP8266
 * SSID merupakan nama SSID yang hendak dihubungkan
 * pass merupakan password daro SSID*/
void connect_ssid(USART_TypeDef* USARTx,const char *SSID,const char *pass);

/*fungsi ini untuk set static ip client
 * USARTx merupakan peripheral USART yang digunakan untuk memberikan perintah ke ESP8266
 * own address merupakan static ip client
 * gateway merupakan ip address gateway
 * mask merupakan mask, default value 255.255.255.0*/
void static_ip(USART_TypeDef* USARTx,const char *own_address,const char *gateway,const char *mask);

/*fungsi ini untuk set UDP connection
 * USARTx merupakan peripheral USART yang digunakan untuk memberikan perintah ke ESP8266
 * IP dest merupakan alamat tujuan koneksi yang dibangun
 * ID merupakan ID robot
 * port merupakan port yang digunakan*/
void set_udp_connection(USART_TypeDef* USARTx,const char *IP_dest,uint8_t ID,uint16_t port);

/*fungsi ini untuk mengirim string melalui esp8266
 * USARTx merupakan peripheral USART yang digunakan untuk memberikan perintah ke ESP8266
 * ID merupakan ID robot
 * ptr merupakan string yang dikirim*/
void send_udp(USART_TypeDef* USARTx,uint8_t ID,const char *ptr);

/*fungsi ini untuk disable echo data yang dikirim
 * USARTx merupakan peripheral USART yang digunakan untuk memberikan perintah ke ESP8266*/
void disable_echo(USART_TypeDef* USARTx);

#ifdef __cplusplus
}
#endif

#endif /* LCD_I2C_H_ */
