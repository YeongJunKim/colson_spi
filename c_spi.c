/*
 * c_spi.c
 *
 *  Created on: 2018. 10. 26.
 *      Author: COLSON
 */
#include "c_spi.h"

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

	cspi->sequence_state = SEQUENCE_READY;

	cspi->error = C_SPI_ERROR_NONE;
	cspi->txstate = C_SPI_STATE_TX_READY;
	cspi->rxstate = C_SPI_STATE_RX_READY;

	cspi->txstart = C_SPI_TX_STOP;
	cspi->rxstart = C_SPI_RX_STOP;

	cspi->clock_count = 0;
	cspi->clockflag = C_SPI_CLOCK_ON;
	cspi->clock_step = C_SPI_CLK_STEP_IDLE;

	cspi->rxData = 0;
	cspi->txData = 0;
}

void c_spi_master_run(C_SPI_HandleTypeDef* cspi)
{
	if(cspi->clockflag != C_SPI_CLOCK_ON) {
		c_spi_error_handler(cspi);
		return;
	}
	if(cspi->error != 0) {
		c_spi_error_handler(cspi);
		return;
	}

	if(cspi->clock_count == 0 || cspi->clock_count == 1){
		HAL_GPIO_WritePin(C_SPI_CS_GPIO_PORT, C_SPI_CS_GPIO_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(C_SPI_TX_GPIO_PORT, C_SPI_TX_GPIO_PIN, GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(C_SPI_CS_GPIO_PORT, C_SPI_CS_GPIO_PIN, GPIO_PIN_RESET);
	}

	c_spi_generate_clock(cspi);

	switch(cspi->clock_count)
	{
	case 0:
	case 1:
		break;
	case 2:
		if(cspi->clock_step == C_SPI_CLK_STEP_RISING)
		{
			cspi->rxstart = C_SPI_RX_START;
			cspi->txstart = C_SPI_TX_START;

			if(cspi->txflag)
			{
				cspi->txData = cspi->p_txData[cspi->txCount++];

				if(cspi->txlength == cspi->txCount)
				{
					cspi->txCount = 0;
					cspi->txflag = 0;
					cspi->txstate = C_SPI_STATE_TX_READY;
					c_spi_tx_complete_callback(cspi);
				}


			}
			else
				cspi->txData = 0;
		}
	default:
		if(cspi->txstart == C_SPI_TX_START)
		{
			if(cspi->clock_step == C_SPI_CLK_STEP_RISING)
			{
				uint8_t data = cspi->txData & (0x80 >> (cspi->clock_count - 2));
				if(data)
					HAL_GPIO_WritePin(C_SPI_TX_GPIO_PORT, C_SPI_TX_GPIO_PIN, GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(C_SPI_TX_GPIO_PORT, C_SPI_TX_GPIO_PIN, GPIO_PIN_RESET);
			}
		}
		if(cspi->rxstart == C_SPI_RX_START)
		{
			if(cspi->clock_step == C_SPI_CLK_STEP_FALLING)
			{
				uint8_t data = HAL_GPIO_ReadPin(C_SPI_RX_GPIO_PORT, C_SPI_RX_GPIO_PIN);

				if(data)
					cspi->rxData = cspi->rxData | (0x80 >> (cspi->clock_count - 2));
			}
		}
		break;
	}

	if(cspi->clock_step == C_SPI_CLK_STEP_RISING || cspi->clock_step == C_SPI_CLK_STEP_FALLING)
		cspi->clock_step++;

	cspi->clock_step++;


	if(cspi->clock_step == C_SPI_CLK_STEP_LOW + 1)
	{
		cspi->clock_count++;
		cspi->clock_step = 1;
	}
	if(cspi->clock_count == 10)
	{
		if(cspi->rx_frame_type == C_SPI_FRAME_ARR)
		{
			if(cspi->rxflag)
			{
				cspi->p_rxData[cspi->rxCount++] = cspi->rxData;

				if(cspi->rxCount == cspi->rxlength)
				{
					cspi->rxCount = 0;
					cspi->rxflag = 0;
					cspi->rxstate = C_SPI_STATE_RX_READY;
					c_spi_rx_complete_callback(cspi);
				}
			}

		}
		else if(cspi->rx_frame_type == C_SPI_FRAME_DATA)
		{
			c_spi_rcv_data_frame_sequence(cspi);
			cspi->rxstate = C_SPI_STATE_RX_READY;
		}

		cspi->txstart = C_SPI_TX_STOP;
		cspi->rxstart = C_SPI_RX_STOP;
		cspi->rxData = 0;
		//cspi->txData = 0;
		cspi->clock_count = 0;
	}
}
void c_spi_slave_run(C_SPI_HandleTypeDef* cspi, c_spi_edge_type edge_type, uint8_t state)
{
  if(cspi->error != 0) {
    c_spi_error_handler(cspi);
    return;
  }
  if(HAL_GPIO_ReadPin(C_SPI_CS_GPIO_PORT, C_SPI_CS_GPIO_PIN) != 0)
  {
    //TODO
    //reset all sequence
    cspi->rising_edge_count = 0;
    cspi->falling_edge_count = 0;
    cspi->rxData = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    cspi->txstart = C_SPI_TX_STOP;
    cspi->rxstart = C_SPI_RX_STOP;
    return;
  }

  if(cspi->txstart == C_SPI_TX_STOP) {
    cspi->txstart = C_SPI_TX_START;
    //set tx data
    if(cspi->txflag)
    {
      cspi->txData = cspi->p_txData[cspi->txCount++];

      if(cspi->txlength == cspi->txCount)
      {
        cspi->txCount = 0;
        cspi->txlength = 0;
        cspi->txstate = C_SPI_STATE_TX_READY;
        cspi->txflag = 0;
        c_spi_tx_complete_callback(cspi);
      }
    }
    else
      cspi->txData = 0;

  }

  if(edge_type == C_SPI_EDGE_CK)
  {
    if(state == GPIO_PIN_SET)
    {
      //TODO
      //write data
      //rising edge
      uint8_t data = cspi->txData & (0x80 >> (cspi->rising_edge_count));
      if(data){
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
      }
      else{
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
      }
      cspi->rising_edge_count++;
      if(cspi->rising_edge_count == 8)
      {
        cspi->rising_edge_count = 0;
      }
    }
    else
    {
      //TODO
      //read data
      //falling edge
      uint8_t data = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

      if(data)
        cspi->rxData = cspi->rxData | (0x80 >> cspi->falling_edge_count);

      cspi->falling_edge_count++;
      if(cspi->falling_edge_count == 8)
      {
        //set rx data
        if(cspi->rx_frame_type == C_SPI_FRAME_ARR)
        {
          if(cspi->rxlength)
          {
            if(cspi->rxflag)
              cspi->p_rxData[cspi->rxCount++] = cspi->rxData;

            if(cspi->rxlength == cspi->rxCount)
            {
              cspi->rxCount = 0;
              cspi->rxlength = 0;
              cspi->rxstate = C_SPI_STATE_RX_READY;
              cspi->rxflag = 0;
              c_spi_rx_complete_callback(cspi);
            }
          }
        }
        else if(cspi->rx_frame_type == C_SPI_FRAME_DATA)
        {
          cspi->rxstate = C_SPI_STATE_RX_READY;
          c_spi_rcv_data_frame_sequence(cspi);
        }

        cspi->falling_edge_count = 0;
        cspi->rxData = 0;
      }

    }
  }
}

void c_spi_generate_clock(C_SPI_HandleTypeDef* cspi)
{
	if(cspi->clockflag != C_SPI_CLOCK_ON)
		return;
	if(cspi->error != C_SPI_ERROR_NONE)
		return;
	if(HAL_GPIO_ReadPin(C_SPI_CS_GPIO_PORT, C_SPI_CS_GPIO_PIN) == 0)
	{
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
			break;
		}
	}
}

void c_spi_error_handler(C_SPI_HandleTypeDef* cspi)
{
	switch(cspi->error)
	{
	case C_SPI_ERROR_NO_CLOCK:
		break;
	case C_SPI_ERROR_GEN_CLOCK:
		break;
	}
}

void c_spi_set_send_data(C_SPI_HandleTypeDef* cspi, uint8_t length, uint8_t *data)
{
	if(cspi->txstate == C_SPI_STATE_TX_READY)
	{
		cspi->txflag = 1;
		cspi->tx_frame_type = C_SPI_FRAME_ARR;
		cspi->txCount = 0;
		cspi->txlength = length;
		cspi->p_txData = data;
		cspi->txstate = C_SPI_STATE_TX_BUSY;
	}
}

void c_spi_set_rcv_data(C_SPI_HandleTypeDef* cspi, uint8_t length, uint8_t *data)
{
	if(cspi->rxstate == C_SPI_STATE_RX_READY)
	{
		cspi->rxflag = 1;
		cspi->rx_frame_type = C_SPI_FRAME_ARR;
		cspi->rxCount = 0;
		cspi->rxlength = length;
		cspi->p_rxData = data;
		cspi->rxstate = C_SPI_STATE_RX_BUSY;
	}
}

void c_spi_set_send_data_frame(C_SPI_HandleTypeDef* cspi, C_dataFrame *data)
{
	if(cspi->txstate == C_SPI_STATE_TX_READY)
	{
		cspi->txflag = 1;
		cspi->tx_frame_type = C_SPI_FRAME_DATA;
		cspi->txCount = 0;
		cspi->txlength = 17 + data->len;
		cspi->p_txData = (uint8_t *)data;
		cspi->txstate = C_SPI_STATE_TX_BUSY;
	}
}

void c_spi_set_rcv_data_frame(C_SPI_HandleTypeDef* cspi, C_dataFrame *data)
{
	if(cspi->rxstate == C_SPI_STATE_RX_READY)
	{
		cspi->rxflag = 1;
		cspi->rx_frame_type = C_SPI_FRAME_DATA;
		cspi->rxCount = 0;
		cspi->rxlength = 11 + data->len;
		cspi->p_rxData = (uint8_t* )data;
		cspi->rxstate = C_SPI_STATE_RX_BUSY;
	}

}

void c_spi_rcv_data_frame_sequence(C_SPI_HandleTypeDef* cspi)
{
	switch(cspi->sequence_state)
	{
	case SEQUENCE_READY:
		if(cspi->rxData == 0xFF)
			cspi->sequence_state = SEQUENCE_START;
		break;
	case SEQUENCE_START:
		if(cspi->rxData == 0xFF)
			cspi->sequence_state = SEQUENCE_CMD1;
		else
			cspi->sequence_state = SEQUENCE_START;
		break;
	case SEQUENCE_CMD1:
		cspi->tempFrame.cmd |= ((uint16_t)cspi->rxData << 8);
		cspi->sequence_state = SEQUENCE_CMD2;
		break;
	case SEQUENCE_CMD2:
		cspi->tempFrame.cmd |= (cspi->rxData);
		cspi->sequence_state = SEQUENCE_SID1;
		break;
	case SEQUENCE_SID1:
		cspi->tempFrame.sid |= ((uint16_t)cspi->rxData << 8);
		cspi->sequence_state = SEQUENCE_SID2;
		break;
	case SEQUENCE_SID2:
		cspi->tempFrame.sid |= (cspi->rxData);
		cspi->sequence_state = SEQUENCE_DID1;
		break;
	case SEQUENCE_DID1:
		cspi->tempFrame.did |= ((uint16_t)cspi->rxData << 8);
		cspi->sequence_state = SEQUENCE_DID2;
		break;
	case SEQUENCE_DID2:
		cspi->tempFrame.did |= (cspi->rxData);
		cspi->sequence_state = SEQUENCE_LEN;
		break;
	case SEQUENCE_LEN:
		cspi->tempFrame.len = cspi->rxData;
		cspi->sequence_state = SEQUENCE_DATA1;
		break;
	case SEQUENCE_DATA1:
		cspi->tempFrame.data[0] = cspi->rxData;
		cspi->sequence_state = SEQUENCE_DATA2;
		break;
	case SEQUENCE_DATA2:
		cspi->tempFrame.data[1] = cspi->rxData;
		cspi->sequence_state = SEQUENCE_DATA3;
		break;
	case SEQUENCE_DATA3:
		cspi->tempFrame.data[2] = cspi->rxData;
		cspi->sequence_state = SEQUENCE_DATA4;
		break;
	case SEQUENCE_DATA4:
		cspi->tempFrame.data[3] = cspi->rxData;
		cspi->sequence_state = SEQUENCE_DATA5;
		break;
	case SEQUENCE_DATA5:
		cspi->tempFrame.data[4] = cspi->rxData;
		cspi->sequence_state = SEQUENCE_DATA6;
		break;
	case SEQUENCE_DATA6:
		cspi->tempFrame.data[5] = cspi->rxData;
		cspi->sequence_state = SEQUENCE_DATA7;
		break;
	case SEQUENCE_DATA7:
		cspi->tempFrame.data[6] = cspi->rxData;
		cspi->sequence_state = SEQUENCE_DATA8;
		break;
	case SEQUENCE_DATA8:
		cspi->tempFrame.data[7] = cspi->rxData;
		cspi->sequence_state = SEQUENCE_READY;
		cspi->rxflag = 0;
		c_spi_rx_complete_callback(cspi);
		c_frame_reset_message(&cspi->tempFrame);
		break;
	}
}


__weak void c_spi_tx_complete_callback(C_SPI_HandleTypeDef* cspi)
{

}

__weak void c_spi_rx_complete_callback(C_SPI_HandleTypeDef* cspi)
{

}



