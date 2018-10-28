/*
 * colson_spi.c
 *
 *  Created on: 2018. 10. 26.
 *      Author: COLSON
 */
#include "colson_spi.h"

/*
 *          ____________________
 *          |                  |
 *__________|                  |__________________
 *       1:T_IN    2:T_IN    3:T_IN    4:T_IN
 *T_IN = timer interrupt
 *1: RISING
 *2: MISO OR MOSI SETTING
 *3: FALLING
 *4: MISO OR MOSI SETTING
 *   1 ~ 4 loop
 *
 *sending -> falling edge detect
 *receiving -> high low
 */

void c_spi_init(C_SPI_HandleTypeDef* cspi, c_spi_type spi_type)
{
	cspi->type = spi_type;

	cspi->error = C_SPI_ERROR_NONE;
	cspi->txstate = C_SPI_STATE_TX_READY;
	cspi->rxstate = C_SPI_STATE_RX_READY;

	cspi->start_tx = C_SPI_TX_STOP;
	cspi->start_rx = C_SPI_RX_STOP;

	cspi->clock_count = 0;
	cspi->clockflag = C_SPI_CLOCK_ON;
	cspi->clock_step = C_SPI_CLK_STEP_IDLE;

	cspi->rxData = 0;
	cspi->txData = 0;

	queue_deinit(&(cspi->q_rcv));
	queue_deinit(&(cspi->q_send));
}

void c_spi_deinit(C_SPI_HandleTypeDef* cspi)
{
	queue_deinit(&(cspi->q_rcv));
	queue_deinit(&(cspi->q_send));
}

void c_spi_start_tx(C_SPI_HandleTypeDef* cspi)
{
	if(queue_isEmpty(&cspi->q_send))
	{
		c_spi_tx_complete_callback(cspi);
		queue_deinit(&cspi->q_send);
		cspi->txstate = C_SPI_STATE_TX_READY;
		cspi->start_tx = C_SPI_TX_STOP;
	}
	else
	{
		cspi->txData = queue_deQueue(&cspi->q_send);
		cspi->start_tx = C_SPI_TX_START;
	}
}

void c_spi_start_rx(C_SPI_HandleTypeDef* cspi)
{
	if(queue_isFull(&cspi->q_rcv))
	{
		c_spi_rx_complete_callback(cspi, cspi->q_rcv.arr);
		queue_deinit(&cspi->q_rcv);
		cspi->rxstate = C_SPI_STATE_RX_READY;
		cspi->start_rx = C_SPI_RX_STOP;
	}
	else
	{
		cspi->start_rx = C_SPI_RX_START;
	}
}

void c_spi_run(C_SPI_HandleTypeDef* cspi)
{
	if(cspi->error != 0) {
		c_spi_error_handler(cspi);
		return;
	}

	c_spi_generate_clock(cspi);

	/* MISO and MOSI high or low */
	if(cspi->clock_step == C_SPI_CLK_STEP_RISING)
	{
		if(cspi->q_send.arr)
			if(cspi->clock_count == 0 && cspi->q_send.size)
				c_spi_start_tx(cspi);
		if(cspi->q_rcv.arr)
			if(cspi->clock_count == 0 && cspi->q_rcv.size)
				c_spi_start_rx(cspi);
	}

	if(cspi->start_tx)
	{
		if(c_spi_get_clock_step(cspi) == C_SPI_CLK_STEP_HIGH)
		{
			uint8_t data = cspi->txData & (0x01 << cspi->clock_count);
			if(data)
				HAL_GPIO_WritePin(C_SPI_TX_GIPO_PORT, C_SPI_TX_GPIO_PIN, GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(C_SPI_TX_GIPO_PORT, C_SPI_TX_GPIO_PIN, GPIO_PIN_RESET);
		}
	}

	if(cspi->start_rx)
	{
		if(c_spi_get_clock_step(cspi) == C_SPI_CLK_STEP_LOW)
		{
			uint8_t data = HAL_GPIO_ReadPin(C_SPI_RX_GPIO_PORT, C_SPI_RX_GPIO_PIN);
			cspi->rxData |= (data << cspi->clock_count);
		}
	}

	cspi->clock_step++;

	if(cspi->clock_step == C_SPI_CLK_STEP_LOW + 1)
	{
		cspi->clock_step = 0;
	}
	if(cspi->clock_count == 8)
	{
		queue_enQueue(&cspi->q_rcv, cspi->rxData);
		cspi->start_tx = C_SPI_TX_STOP;
		cspi->start_rx = C_SPI_RX_STOP;
		cspi->rxData = 0;
		cspi->txData = 0;
		cspi->clock_count = 0;
	}
}

uint8_t c_spi_get_clock_step(C_SPI_HandleTypeDef* cspi)
{
	return cspi->clock_step;
}

void c_spi_generate_clock(C_SPI_HandleTypeDef* cspi)
{
	if(cspi->clockflag != C_SPI_CLOCK_ON)
		return;
	if(cspi->error != C_SPI_ERROR_NONE)
		return;

	GPIO_PinState read = HAL_GPIO_ReadPin(C_SPI_CK_GPIO_PORT, C_SPI_CK_GPIO_PIN);

	switch(cspi->clock_step)
	{
	case C_SPI_CLK_STEP_RISING:
		HAL_GPIO_WritePin(C_SPI_CK_GPIO_PORT, C_SPI_CK_GPIO_PIN, GPIO_PIN_SET);
		break;

	case C_SPI_CLK_STEP_HIGH:
		if(read == GPIO_PIN_RESET) {
		cspi->error = C_SPI_ERROR_GEN_CLOCK;
		return;
		}
		break;

	case C_SPI_CLK_STEP_FALLING:
		HAL_GPIO_WritePin(C_SPI_CK_GPIO_PORT, C_SPI_CK_GPIO_PIN, GPIO_PIN_RESET);
		break;

	case C_SPI_CLK_STEP_LOW:
		if(read == GPIO_PIN_SET) {
			cspi->error = C_SPI_ERROR_GEN_CLOCK;
			return;
		}
		cspi->clock_count++;
		break;
	}
}

void c_spi_error_handler(C_SPI_HandleTypeDef* cspi)
{
	switch(cspi->error)
	{
	case C_SPI_ERROR_NO_CLOCK:
		c_spi_init(cspi, C_SPI_TYPE_MASTER);
		cspi->error = C_SPI_ERROR_NONE;
		break;
	case C_SPI_ERROR_GEN_CLOCK:
		break;
	}
}

void c_spi_set_send_data(C_SPI_HandleTypeDef* cspi, uint8_t length, uint8_t *data)
{
	if(cspi->error != C_SPI_ERROR_NONE)
		return;
	if(cspi->txstate != C_SPI_STATE_TX_READY)
		return;

	queue_init(&cspi->q_send, length);
	for(int i = 0; i < length; i++)
	{
		queue_enQueue(&cspi->q_send, data[i]);
	}
	cspi->txstate = C_SPI_STATE_TX_BUSY;
}

void c_spi_set_rcv_data(C_SPI_HandleTypeDef* cspi, uint8_t length)
{
	if(cspi->error != C_SPI_ERROR_NONE)
		return;
	if(cspi->rxstate != C_SPI_STATE_RX_READY)
		return;

	queue_init(&cspi->q_rcv, length);

	cspi->rxstate = C_SPI_STATE_RX_BUSY;
}

__weak void c_spi_set_gpio_high(C_SPI_HandleTypeDef* cspi)
{

}

__weak void c_spi_set_gpio_low(C_SPI_HandleTypeDef* cspi)
{

}

__weak void c_spi_tx_complete_callback(C_SPI_HandleTypeDef* cspi)
{

}

__weak void c_spi_rx_complete_callback(C_SPI_HandleTypeDef* cspi, uint8_t *data)
{

}

