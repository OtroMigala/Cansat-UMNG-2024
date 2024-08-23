/*
 * uart.c
 *
 *  Created on: 15/11/2021
 *      Author: Alcides Ramos
 */

#include <uart.h>
#include "stdio.h"
#include "string.h"

//Variables externas en el main
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;


void uartx_write(UART_HandleTypeDef *huart,uint8_t ch)
{
HAL_UART_Transmit(huart, &ch, 1, 0xffff);

}

void uartx_write_text(UART_HandleTypeDef *huart, uint8_t *info)
{

while(*info)  uartx_write(huart,*info++);

}


uint8_t uartx_read(UART_HandleTypeDef *huart)
{
	uint8_t dato_rx_=0;
	HAL_UART_Receive(huart,&dato_rx_, 1,HAL_MAX_DELAY);
	return(dato_rx_);
}



void uartx_read_text(UART_HandleTypeDef *huart,uint8_t  *info,const uint8_t final,uint8_t cuanto)
{
uint8_t dato_rx=0;//  los datos leidos son de 8 bits
char i=0;
    memset(info,0,cuanto);//limpia el buffer
	while(dato_rx!=final)// lee hasta que llegue el final
	{
	dato_rx =uartx_read(huart);
	*info=dato_rx;//  va almacenado en el buffer
    info++;
	i++;// incrementa contador
	if (i>cuanto-1) break;// si llegan n elementosa sale
	}
}
extern uint16_t tamaño;
void COMANDO_AT_SEND(char Address_Lora[],char Carga_util[],char Cadena[])

{
	char tam[1000] = {0};
    uint8_t Tamaño_carga_util = strlen(Carga_util);
    sprintf(tam, "%d", Tamaño_carga_util);
    strcpy(Cadena, "AT+SEND=");
    strcat(Cadena, Address_Lora);
    strcat(Cadena, ",");
    strcat(Cadena, tam);
    strcat(Cadena, ",");
    strcat(Cadena, Carga_util);
    strcat(Cadena, "\r\n");
    tamaño=strlen(Cadena);

}

uint16_t COMANDO_AT_SEND1(const char *Address_Lora, const char *Carga_util, char *Cadena, uint16_t MaxLength)
{
    uint16_t Tamaño_carga_util = strlen(Carga_util);
    uint16_t Tamaño_total = snprintf(Cadena, MaxLength, "AT+SEND=%s,%d,%s\r\n", Address_Lora, Tamaño_carga_util, Carga_util);

    // Verificar si el tamaño de la cadena generada excede el límite del búfer
    if (Tamaño_total >= MaxLength)
    {
        // Truncar la cadena si excede el límite
        Cadena[MaxLength - 1] = '\0';
        Tamaño_total = MaxLength - 1;
    }

    return Tamaño_total;
}


//FUNCION NUEVA A VER SI RESULTA
void Comando(char Address_Lora[], char Carga_util[]){
//variables en funcion
	char Cadena[256]={0};




	//Para calcular el tamaño de la carga a enviar
	char tam[500] = {0};
	uint8_t Tamaño_carga_util = strlen(Carga_util);
	sprintf(tam, "%d", Tamaño_carga_util);

	//Formando AT+SEND=Address,longitud_carga,carga\r\n
	strcpy(Cadena, "AT+SEND=");
	strcat(Cadena, Address_Lora);
	strcat(Cadena, ",");  //AT+SEND=Addres,
	strcat(Cadena, tam);
	strcat(Cadena, ",");
	strcat(Cadena, Carga_util);
	strcat(Cadena, "\r\n");


	uint8_t tamaño_completo = strlen(Cadena);
	    char Cadenal123[tamaño_completo + 1]; // +1 para el carácter nulo
	    memset(Cadenal123, 0, tamaño_completo + 1); // Llenar el arreglo con ceros

	    // Copiar la cadena Cadena a Cadenal123
	    strcpy(Cadenal123, Cadena);

	//CADENA DEFINIDA CON EL ANCHO PERMITIDO

	//memcpy();



	HAL_UART_Transmit(&huart3, (uint8_t *)Cadenal123, tamaño_completo, 100);


}
void Comandofinal(char Address_Lora[], char Carga_util[])
{
    // variables en funcion
    char Cadena[256] = {0};

    // Para calcular el tamaño de la carga a enviar
    char tam[500] = {0};
    uint8_t Tamaño_carga_util = strlen(Carga_util);
    sprintf(tam, "%d", Tamaño_carga_util);

    // Formando AT+SEND=Address,longitud_carga,carga\r\n
    strcpy(Cadena, "AT+SEND=");
    strcat(Cadena, Address_Lora);
    strcat(Cadena, ","); // AT+SEND=Addres,
    strcat(Cadena, tam);
    strcat(Cadena, ",");
    strcat(Cadena, Carga_util);
    strcat(Cadena, "\r\n");

    uint8_t tamaño_completo = strlen(Cadena);
    char Cadenal123[tamaño_completo + 1];       // +1 para el carácter nulo
    memset(Cadenal123, 0, tamaño_completo +1); // Llenar el arreglo con ceros

    // Copiar la cadena Cadena a Cadenal123
    strcpy(Cadenal123, Cadena);

    HAL_UART_Transmit(&huart3, (uint8_t *)Cadenal123, tamaño_completo, 100);
}




//FUNCIONES GPS







