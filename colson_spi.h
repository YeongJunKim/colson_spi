/*
 * colson_spi.h
 *
 *  Created on: 2018. 10. 26.
 *      Author: COLSON
 */

#ifndef COLSON_SPI_H_
#define COLSON_SPI_H_
#include "stm32f0xx_hal.h"
#include "queue.h"

#define MCU_STM32F042 	0xC01502
#define MCU_STM32WB55  	0xC01501
#define MCU_ETC  	 	0xC01503
/* define YOUR MCU */

#define MCUTYPE MCU_STM32F042

#if (MCUTYPE == MCU_STM32WB55)
#define C_SPI_RX_GPIO_PORT GPIOB
#define C_SPI_TX_GPIO_PORT GPIOB
#define C_SPI_CK_GPIO_PORT GPIOB
#define C_SPI_CS_GPIO_PORT GPIOB

#define C_SPI_RX_GPIO_PIN GPIO_PIN_0
#define C_SPI_TX_GPIO_PIN GPIO_PIN_1
#define C_SPI_CK_GPIO_PIN GPIO_PIN_8
#define C_SPI_CS_GPIO_PIN GPIO_PIN_9
#endif

#if (MCUTYPE == MCU_STM32F042)
#define C_SPI_RX_GPIO_PORT GPIOA
#define C_SPI_TX_GIPO_PORT GPIOA
#define C_SPI_CK_GPIO_PORT GPIOA
#define C_SPI_CS_GPIO_PORT GPIOA

#define C_SPI_RX_GPIO_PIN GPIO_PIN_4
#define C_SPI_TX_GPIO_PIN GPIO_PIN_5
#define C_SPI_CK_GPIO_PIN GPIO_PIN_6
#define C_SPI_CS_GPIO_PIN GPIO_PIN_7
#endif

#if (MCUTYPE == MCU_ETC)
#define C_SPI_RX_GPIO_PORT /* define here */
#define C_SPI_TX_GPIO_PORT /* define here */
#define C_SPI_CK_GPIO_PORT /* define here */
#define C_SPI_CS_GPIO_PORT /* define here */

#define C_SPI_RX_GPIO_PIN /* define here */
#define C_SPI_TX_GPIO_PIN /* define here */
#define C_SPI_CK_GPIO_PIN /* define here */
#define C_SPI_CS_GPIO_PIN /* define here */
#endif


typedef struct _colson_spi{
	queue_struct q_send;
	queue_struct q_rcv;

	uint8_t start_tx;
	uint8_t start_rx;

	uint8_t rxData;
	uint8_t txData;

	uint8_t type;					/*!< slave or master type */
	uint8_t txstate;
	uint8_t rxstate;
	uint8_t	error;					/*!< error state @ref c_spi_error */

	uint8_t clockflag;				/*!< clock state @ref c_spi_clock_state */
	uint8_t clock_count;			/*!< counting clock 0 to 7 (8bit data only) */
	uint8_t clock_bit;				/*!< clock max bit 8 ~ 16 */
	uint8_t clock_step;				/*!< clock sequence 1 ~ 4 @ref c_spi_clk_step*/
}C_SPI_HandleTypeDef;

typedef enum _c_spi_tx_start{
	C_SPI_TX_STOP = 0,
	C_SPI_TX_START
}c_spi_tx_start;
typedef enum _c_spi_rx_start{
	C_SPI_RX_STOP = 0,
	C_SPI_RX_START
}c_spi_rx_start;
typedef enum _c_spi_type{
	C_SPI_TYPE_MASTER = 0,
	C_SPI_TYPE_SLAVE
}c_spi_type;

typedef enum _c_spi_clk_step{
	C_SPI_CLK_STEP_IDLE 	= 0,
	C_SPI_CLK_STEP_RISING 	= 1,
	C_SPI_CLK_STEP_HIGH 	= 2,
	C_SPI_CLK_STEP_FALLING 	= 3,
	C_SPI_CLK_STEP_LOW 		= 4
}c_spi_clk_step;

typedef enum _c_spi_send_step{
	C_SPI_SEND_STEP_0 = 0,
	C_SPI_SEND_STEP_1 = 1,
	C_SPI_SEND_STEP_2 = 2,
	C_SPI_SEND_STEP_3 = 3,
	C_SPI_SEND_STEP_4 = 4,
	C_SPI_SEND_STEP_5 = 5,
	C_SPI_SEND_STEP_6 = 6,
	C_SPI_SEND_STEP_7 = 7,
	C_SPI_SEND_STEP_8 = 8
}c_spi_send_step;

typedef enum _c_spi_rcv_step{
	C_SPI_RCV_STEP_0 = 0,
	C_SPI_RCV_STEP_1 = 1,
	C_SPI_RCV_STEP_2 = 2,
	C_SPI_RCV_STEP_3 = 3,
	C_SPI_RCV_STEP_4 = 4,
	C_SPI_RCV_STEP_5 = 5,
	C_SPI_RCV_STEP_6 = 6,
	C_SPI_RCV_STEP_7 = 7,
	C_SPI_RCV_STEP_8 = 8
}c_spi_rcv_step;

typedef enum _c_spi_clockstate{
	C_SPI_CLOCK_OFF = 0,
	C_SPI_CLOCK_ON
}c_spi_clockstate;

typedef enum _c_spi_state{
	C_SPI_STATE_READY = 0,
	C_SPI_STATE_TX_READY,
	C_SPI_STATE_RX_READY,
	C_SPI_STATE_TX_BUSY,
	C_SPI_STATE_RX_BUSY,
	C_SPI_STATE_TXRX_BUSY
}c_spi_state;

typedef enum _c_spi_clock_bit{
	C_SPI_CLK_BIT_8 = 8,
	C_SPI_CLK_BIT_9 = 9,
	C_SPI_CLK_BIT_10 = 10,
	C_SPI_CLK_BIT_11 = 11,
	C_SPI_CLK_BIT_12 = 12,
	C_SPI_CLK_BIT_13 = 13,
	C_SPI_CLK_BIT_14 = 14,
	C_SPI_CLK_BIT_15 = 15,
	C_SPI_CLK_BIT_16 = 16
}c_spi_clock_bit;

typedef enum _c_spi_error{
	C_SPI_ERROR_NONE = 0,
	C_SPI_ERROR_NO_CLOCK,
	C_SPI_ERROR_GEN_CLOCK,
}c_spi_error;

typedef enum _c_spi_gpio_state{
	C_SPI_GPIO_RESET = 0,
	C_SPI_GPIO_SET = 1
}c_spi_gpio_state;


void c_spi_init(C_SPI_HandleTypeDef* cspi, c_spi_type spi_type);

uint8_t c_spi_get_clock_step(C_SPI_HandleTypeDef* cspi);
uint8_t c_spi_get_state(C_SPI_HandleTypeDef* cspi);

void c_spi_set_send_data(C_SPI_HandleTypeDef* cspi, uint8_t length, uint8_t *data);
void c_spi_set_rcv_data(C_SPI_HandleTypeDef* cspi, uint8_t length);

void c_spi_error_handler(C_SPI_HandleTypeDef* cspi);

void c_spi_generate_clock(C_SPI_HandleTypeDef* cspi);
void c_spi_run(C_SPI_HandleTypeDef* cspi);

void c_spi_set_gpio_high(C_SPI_HandleTypeDef* cspi);
void c_spi_set_gpio_low(C_SPI_HandleTypeDef* cspi);

void c_spi_tx_complete_callback(C_SPI_HandleTypeDef* cspi);
void c_spi_rx_complete_callback(C_SPI_HandleTypeDef* cspi, uint8_t *data);

#endif /* COLSON_SPI_H_ */
