/*
 * modbus.cpp
 *
 *  Created on: Feb 3, 2023
 *      Author: anh
 */

#include "modbus.h"
#include "crc.h"

#include "stdlib.h"
#include "string.h"

/**
 * @fn  modbus(USART_TypeDef*)
 * @brief
 *
 * @pre
 * @post
 * @param mb_uart
 */
modbus::modbus(UART_HandleTypeDef *mb_uart){
	uart = mb_uart;
}

/**
 * @fn void set_TE(GPIO_TypeDef*, uint16_t)
 * @brief
 *
 * @pre
 * @post
 * @param te_port
 * @param te_pin
 */
void modbus::set_TE(GPIO_TypeDef *te_port, uint16_t te_pin){
	port = te_port;
	pin = te_pin;
#ifdef USE_HAL_DRIVER
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
#endif
}

/**
 * @fn void send_data(uint8_t*)
 * @brief
 *
 * @pre
 * @post
 * @param data_frame
 */
void modbus::send_data(uint8_t *data_frame){
#ifdef USE_HAL_DRIVER
	if(pin != -1) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

	HAL_UART_Transmit(uart, data_frame, TX_SIZE, 100);

	if(pin != -1) HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
#endif
}

/**
 * @fn void send_request(uint8_t, mb_func_t, uint16_t, uint16_t)
 * @brief
 *
 * @pre
 * @post
 * @param device_address
 * @param function
 * @param register_address
 * @param Quantity
 */
void modbus::send_request(uint8_t device_address, mb_func_t function, uint16_t register_address, uint16_t data_or_quantity, bool require_response){
	uint16_t data = 0;
	if(function == READ_COILS_STATUS){
		data = (uint16_t)(data_or_quantity/8);
		if((data_or_quantity % 8) != 0) data += 1;
	}
	else if(function == READ_HOLDING_REGISTERS || function == READ_INPUT_REGISTERS){
		data = data_or_quantity;
	}
	else {
		data = data_or_quantity;
	}

	txdata[0] = device_address;     // slave address.
	txdata[1] = (uint8_t)function;  // Function code.

	//The Register address.
	txdata[2] = (uint8_t)((register_address & 0xFF00) >> 8);
	txdata[3] = (uint8_t)(register_address & 0xFF);

	// Number of registers to read.
	txdata[4] = (uint8_t)((data & 0xFF00) >> 8);
	txdata[5] = (uint8_t)(data & 0xFF);

	// CRC.
	uint16_t crc = crc16(txdata, 6);
	txdata[6] = crc & 0xFF;   		// CRC LOW.
	txdata[7] = (crc >> 8) & 0xFF;  // CRC HIGH.

	if(require_response){
		if(rxdata != NULL) free(rxdata);
		response_lenght = 5U;

		if(function == READ_COILS_STATUS){
			response_lenght += (uint16_t)(data_or_quantity/8);
			if((data_or_quantity % 8) != 0) response_lenght += 1;
		}
		else if(function == READ_HOLDING_REGISTERS || function == READ_INPUT_REGISTERS){
			response_lenght += (data_or_quantity * 2);
		}
		else{

		}

		rxdata = (uint8_t *)malloc(response_lenght);
		memset(rxdata, 0, response_lenght);
#ifdef USE_HAL_DRIVER
		HAL_UARTEx_ReceiveToIdle_IT(uart, rxdata, response_lenght);
#endif
	}
	else{
		HAL_UART_AbortReceive_IT(uart);
	}
	send_data(txdata);
}

bool modbus::wait_for_response(uint16_t timeout){
	uint32_t time = HAL_GetTick();

	while(response_flag == false){
		if(HAL_GetTick() - time > timeout) return false;

		HAL_Delay(1);
	}
	response_flag = false;

	return true;
}

/**
 * @fn bool is_device(uint8_t*, uint8_t)
 * @brief
 *
 * @pre
 * @post
 * @param buffer
 * @param device_address
 * @return
 */
bool modbus::is_device(uint8_t *buffer, uint8_t device_address){
	if(buffer[0] == device_address) return true;

	return false;
}

uint8_t modbus::get_response_num_bytes(void){
	return rxdata[3];
}

uint8_t modbus::get_response_num_coils(void){

	return 0;
}

uint16_t modbus::get_response_crc(void){

	return (uint16_t)(rxdata[response_lenght - 2] << 8 | rxdata[response_lenght - 1]);
}

void modbus::get_response_datas(uint8_t **buffer){
	*buffer = rxdata;
}


void modbus::response_handler(UART_HandleTypeDef *input_uart){
	if(input_uart -> Instance == uart -> Instance)
		response_flag = true;

}




