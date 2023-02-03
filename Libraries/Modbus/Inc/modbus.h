/*
 * modbus.h
 *
 *  Created on: Feb 3, 2023
 *      Author: anh
 */

#ifndef MODBUS_H_
#define MODBUS_H_


#ifdef __cplusplus
extern "C"{
#endif

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#else
#include "your uart driver header"
#endif

typedef enum{
	READ_COILS_STATUS 		 = 0x01U,
	READ_INPUT_STATUS 		 = 0x02U,

	READ_HOLDING_REGISTERS 	 = 0x03U,
	READ_INPUT_REGISTERS     = 0x04U,

	WRITE_SINGLE_COIL  	     = 0x05U,
	WRITE_SINGLE_REGISTER    = 0x06U,

	WRITE_MULTIPLE_COILS 	 = 0x0FU,
	WRITE_MULTIPLE_REGISTERS = 0x10U,
}mb_func_t;

#define TX_SIZE 8U

class modbus{
	public:
#ifdef USE_HAL_DRIVER
		modbus(UART_HandleTypeDef *mb_uart);
#endif
		void set_TE(GPIO_TypeDef *te_port, uint16_t te_pin);

		void send_request(uint8_t device_address, mb_func_t function, uint16_t register_address, uint16_t data_or_quantity, bool require_response = false);

		bool wait_for_response(uint16_t timeout);

		bool is_device(uint8_t *buffer, uint8_t device_address);

		uint8_t get_response_num_bytes(void);
		uint8_t get_response_num_coils(void);
		uint16_t get_response_crc(void);
		void get_response_datas(uint8_t **buffer);

		void response_handler(UART_HandleTypeDef *input_uart);

	private:
#ifdef USE_HAL_DRIVER
		UART_HandleTypeDef *uart;
#endif

		GPIO_TypeDef *port;
		int16_t pin = -1;

		uint8_t txdata[TX_SIZE];
		uint8_t *rxdata = NULL;

		uint16_t response_lenght = 0;

		volatile bool response_flag = false;

		void send_data(uint8_t *);
};



#ifdef __cplusplus
}
#endif


#endif /* MODBUS_H_ */
