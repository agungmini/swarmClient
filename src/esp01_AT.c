#include <esp01_AT.h>

char buff[100];

void esp_restart(USART_TypeDef* USARTx){
	sprintf(buff,"AT+RST%c%c",0x0D,0x0A);
	UART_sendStr(USARTx,buff);

	while(UART_getChar(USARTx)!='I'); //wait until OK received
	while(UART_getChar(USARTx)!='P');
	while(UART_getChar(USARTx)!= 0x0D); //wait until OK received
	while(UART_getChar(USARTx)!= 0x0A);
}

void connect_ssid(USART_TypeDef* USARTx,const char *SSID,const char *pass){
	sprintf(buff,"AT+CWMODE=1%c%c",0x0D,0x0A);
	UART_sendStr(USARTx,buff);

	while(UART_getChar(USARTx)!='O'); //wait until OK received
	while(UART_getChar(USARTx)!='K');
	while(UART_getChar(USARTx)!= 0x0D); //wait until OK received
	while(UART_getChar(USARTx)!= 0x0A);

	sprintf(buff,"AT+CWJAP=%c%s%c,%c%s%c%c%c",0x22,SSID,0x22,0x22,pass,0x22,0x0D,0x0A);
	UART_sendStr(USARTx,buff);

	while(UART_getChar(USARTx)!='O'); //wait until OK received
	while(UART_getChar(USARTx)!='K');
	while(UART_getChar(USARTx)!= 0x0D); //wait until OK received
	while(UART_getChar(USARTx)!= 0x0A);
}

void static_ip(USART_TypeDef* USARTx,const char *own_address,const char *gateway,const char *mask){
	sprintf(buff,"AT+CIPSTA=%c%s%c,%c%s%c,%c%s%c%c%c",0x22,own_address,0x22,0x22,gateway,0x22,0x22,mask,0x22,0x0D,0x0A);
	UART_sendStr(USARTx,buff);

	while(UART_getChar(USARTx)!='O'); //wait until OK received
	while(UART_getChar(USARTx)!='K');
	while(UART_getChar(USARTx)!= 0x0D); //wait until OK received
	while(UART_getChar(USARTx)!= 0x0A);
}

void set_udp_connection(USART_TypeDef* USARTx,const char *IP_dest,uint8_t ID,uint16_t port){
	sprintf(buff,"AT+CIPMUX=1%c%c",0x0D,0x0A);	//at+cipmux=1\r\n
	UART_sendStr(USARTx,buff);

	while(UART_getChar(USARTx)!='O'); //wait until OK received
	while(UART_getChar(USARTx)!='K');
	while(UART_getChar(USARTx)!= 0x0D); //wait until OK received
	while(UART_getChar(USARTx)!= 0x0A);

	sprintf(buff,"AT+CIPSTART=%d,%cUDP%c,%c%s%c,%d,1112,0%c%c",ID,0x22,0x22,0x22,IP_dest,0x22,port,0x0D,0x0A);
	UART_sendStr(USARTx,buff);

	while(UART_getChar(USARTx)!='O'); //wait until OK received
	while(UART_getChar(USARTx)!='K');
	while(UART_getChar(USARTx)!= 0x0D); //wait until OK received
	while(UART_getChar(USARTx)!= 0x0A);
}

void send_udp(USART_TypeDef* USARTx,uint8_t ID,const char *ptr){
	sprintf(buff,"AT+CIPSEND=%d,%d%c%c",ID,strlen(ptr),0x0D,0x0A);
	UART_sendStr(USARTx,buff);

	while(UART_getChar(USARTx)!='O'); //wait until OK received
	while(UART_getChar(USARTx)!='K');
	while(UART_getChar(USARTx)!='>');

	sprintf(buff,"%s",ptr);
	UART_sendStr(USARTx,buff);

	while(UART_getChar(USARTx)!='O'); //wait until OK received
	while(UART_getChar(USARTx)!='K');
}
