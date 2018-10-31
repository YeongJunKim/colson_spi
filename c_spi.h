/*
 * c_spi.h
 *
 *  Created on: 2018. 10. 26.
 *      Author: COLSON
 */

#ifndef C_SPI_H_
#define C_SPI_H_
#include "ctypes.h"
#include "queue.h"


#define MCU_STM32F042 	0xC01502
#define MCU_STM32WB55  	0xC01501
#define MCU_ETC  	 	0xC01503
/* define YOUR MCU */

#define MCUTYPE MCU_STM32F042

#define SEQUENCE_READY 	0x00
#define SEQUENCE_START 	0x01
#define SEQUENCE_CMD1 	0x02
#define SEQUENCE_CMD2 	0x03
#define SEQUENCE_SID1 	0x04
#define SEQUENCE_SID2 	0x05
#define SEQUENCE_DID1 	0x06
#define SEQUENCE_DID2 	0x07
#define SEQUENCE_LEN 	0x08
#define SEQUENCE_DATA1 	0x10
#define SEQUENCE_DATA2 	0x20
#define SEQUENCE_DATA3 	0x30
#define SEQUENCE_DATA4 	0x40
#define SEQUENCE_DATA5 	0x50
#define SEQUENCE_DATA6 	0x60
#define SEQUENCE_DATA7 	0x70
#define SEQUENCE_DATA8 	0x80


#if (MCUTYPE == MCU_STM32WB55)
#include "app_common.h"
#define C_SPI_TX_GPIO_PORT GPIOA
#define C_SPI_RX_GPIO_PORT GPIOA
#define C_SPI_CK_GPIO_PORT GPIOB
#define C_SPI_CS_GPIO_PORT GPIOB

#define C_SPI_TX_GPIO_PIN GPIO_PIN_4
#define C_SPI_RX_GPIO_PIN GPIO_PIN_15
#define C_SPI_CK_GPIO_PIN GPIO_PIN_1
#define C_SPI_CS_GPIO_PIN GPIO_PIN_0
#endif

#if (MCUTYPE == MCU_STM32F042)
#include "stm32f0xx_hal.h"
#define C_SPI_TX_GPIO_PORT GPIOA
#define C_SPI_RX_GPIO_PORT GPIOA
#define C_SPI_CK_GPIO_PORT GPIOA
#define C_SPI_CS_GPIO_PORT GPIOA

#define C_SPI_TX_GPIO_PIN GPIO_PIN_4
#define C_SPI_RX_GPIO_PIN GPIO_PIN_5
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

	C_dataFrame tempFrame;			/**/
	uint8_t sequence_state;			/**/

	uint8_t tx_frame_type;			/**/
	uint8_t rx_frame_type;			/**/

	uint8_t txstart;				/**/
	uint8_t rxstart;				/**/

	uint8_t txflag;
	uint8_t rxflag;

	uint8_t txstate;				/**/
	uint8_t rxstate;				/**/

	uint8_t txlength;				/**/
	uint8_t rxlength;				/**/

	uint8_t txCount;				/**/
	uint8_t rxCount;				/**/

	uint8_t *p_txData;				/**/
	uint8_t *p_rxData;				/**/


	uint8_t rxData;					/**/
	uint8_t txData;					/**/

	uint8_t type;					/*!< slave or master type */


	uint8_t	error;					/*!< error state @ref c_spi_error */

	uint8_t clockflag;				/*!< clock state @ref c_spi_clock_state */
	uint8_t clock_count;			/*!< counting clock 0 to 7 (8bit data only) */
	uint8_t clock_step;				/*!< clock sequence 1 ~ 4 @ref c_spi_clk_step*/

	uint8_t rising_edge_count;
	uint8_t falling_edge_count;
	uint8_t edge_count;
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

typedef enum _c_spi_error{
	C_SPI_ERROR_NONE = 0,
	C_SPI_ERROR_NO_CLOCK,
	C_SPI_ERROR_GEN_CLOCK,
}c_spi_error;

typedef enum _c_spi_gpio_state{
	C_SPI_GPIO_RESET = 0,
	C_SPI_GPIO_SET = 1
}c_spi_gpio_state;

typedef enum _c_spi_edge_type{
  C_SPI_EDGE_CS = 0,
  C_SPI_EDGE_CK = 1,
}c_spi_edge_type;

typedef enum _c_spi_cs_state{
	C_SPI_CS_LOW = 0,
	C_SPI_CS_HIGH = 1
}c_spi_cs_state;

typedef enum _c_spi_frame_type{
	C_SPI_FRAME_ARR = 0,
	C_SPI_FRAME_DATA = 1
}c_spi_frame_type;



void c_spi_init(C_SPI_HandleTypeDef* cspi, c_spi_type spi_type);

/* send & receive sequence */
void c_spi_set_send_data(C_SPI_HandleTypeDef* cspi, uint8_t length, uint8_t *data);
void c_spi_set_rcv_data(C_SPI_HandleTypeDef* cspi, uint8_t length, uint8_t *data);

/* send & receive packet sequence */
void c_spi_set_send_data_frame(C_SPI_HandleTypeDef* cspi, C_dataFrame *data);
void c_spi_set_rcv_data_frame(C_SPI_HandleTypeDef* cspi, C_dataFrame *data);

void c_spi_rcv_data_frame_sequence(C_SPI_HandleTypeDef* cspi);

/* clock generator */
void c_spi_generate_clock(C_SPI_HandleTypeDef* cspi);

/* master & slave run */
void c_spi_master_run(C_SPI_HandleTypeDef* cspi);
void c_spi_slave_run(C_SPI_HandleTypeDef* cspi, c_spi_edge_type edge_type, uint8_t state);

/* gpio high | low */
void c_spi_set_gpio_high(C_SPI_HandleTypeDef* cspi);
void c_spi_set_gpio_low(C_SPI_HandleTypeDef* cspi);

/* send & receive sequence complete callback function */
void c_spi_tx_complete_callback(C_SPI_HandleTypeDef* cspi);
void c_spi_rx_complete_callback(C_SPI_HandleTypeDef* cspi);

/* C_SPI error handler */
void c_spi_error_handler(C_SPI_HandleTypeDef* cspi);

#endif /* C_SPI_H_ */
