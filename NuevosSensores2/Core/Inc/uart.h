/*
 * uart.h
 *
 *  Created on: 15/11/2021
 *      Author: Alcides Ramos
 */

#ifndef INC_UART_H_
#define INC_UART_H_


#include "main.h"



void uartx_write(UART_HandleTypeDef *huart,uint8_t ch);
void uartx_write_text(UART_HandleTypeDef *huart, uint8_t *info);
uint8_t uartx_read(UART_HandleTypeDef *huart);
void uartx_read_text(UART_HandleTypeDef *huart,uint8_t  *info,const uint8_t final,uint8_t cuanto);

void COMANDO_AT_SEND(char Address_Lora[],char Carga_util[],char Cadena[]);
uint16_t COMANDO_AT_SEND1(const char *Address_Lora, const char *Carga_util, char *Cadena, uint16_t MaxLength);
void Comando(char Address_Lora[], char Carga_util[]);



//Modulo gps
void Format_data(float time, float lat, char lat_dir, float lon, char long_dir);
void get_location(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_UART_H_ */
