
/*
 *  nrf24l01_plus.h
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 *
 *  https://github.com/mokhwasomssi/stm32_hal_nrf24l01p/tree/main
 *
 */

#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__

#include "spi.h" // header from stm32cubemx code generate
#include <stdbool.h>
#include "enums.hpp"

/* User Configurations */
#define NRF24L01P_SPI (&hspi2)

#define NRF24L01P_SPI_CS_PIN_PORT NRF_CSN_GPIO_Port
#define NRF24L01P_SPI_CS_PIN_NUMBER NRF_CSN_Pin

#define NRF24L01P_CE_PIN_PORT NRF_CE_GPIO_Port
#define NRF24L01P_CE_PIN_NUMBER NRF_CE_Pin

#define NRF24L01P_IRQ_PIN_PORT NRF_IRQ_GPIO_Port
#define NRF24L01P_IRQ_PIN_NUMBER NRF_IRQ_Pin

#define NRF24L01P_PAYLOAD_LENGTH 4 // 1 - 32bytes


/* nRF24L01+ Commands */
#define NRF24L01P_CMD_R_REGISTER 0b00000000
#define NRF24L01P_CMD_W_REGISTER 0b00100000
#define NRF24L01P_CMD_R_RX_PAYLOAD 0b01100001
#define NRF24L01P_CMD_W_TX_PAYLOAD 0b10100000
#define NRF24L01P_CMD_FLUSH_TX 0b11100001
#define NRF24L01P_CMD_FLUSH_RX 0b11100010
#define NRF24L01P_CMD_REUSE_TX_PL 0b11100011
#define NRF24L01P_CMD_R_RX_PL_WID 0b01100000
#define NRF24L01P_CMD_W_ACK_PAYLOAD 0b10101000
#define NRF24L01P_CMD_W_TX_PAYLOAD_NOACK 0b10110000
#define NRF24L01P_CMD_NOP 0b11111111

/* nRF24L01+ Registers */
#define NRF24L01P_REG_CONFIG 0x00
#define NRF24L01P_REG_EN_AA 0x01
#define NRF24L01P_REG_EN_RXADDR 0x02
#define NRF24L01P_REG_SETUP_AW 0x03
#define NRF24L01P_REG_SETUP_RETR 0x04
#define NRF24L01P_REG_RF_CH 0x05
#define NRF24L01P_REG_RF_SETUP 0x06
#define NRF24L01P_REG_STATUS 0x07
#define NRF24L01P_REG_OBSERVE_TX 0x08 // Read-Only
#define NRF24L01P_REG_RPD 0x09        // Read-Only
#define NRF24L01P_REG_RX_ADDR_P0 0x0A
#define NRF24L01P_REG_RX_ADDR_P1 0x0B
#define NRF24L01P_REG_RX_ADDR_P2 0x0C
#define NRF24L01P_REG_RX_ADDR_P3 0x0D
#define NRF24L01P_REG_RX_ADDR_P4 0x0E
#define NRF24L01P_REG_RX_ADDR_P5 0x0F
#define NRF24L01P_REG_TX_ADDR 0x10
#define NRF24L01P_REG_RX_PW_P0 0x11
#define NRF24L01P_REG_RX_PW_P1 0x12
#define NRF24L01P_REG_RX_PW_P2 0x13
#define NRF24L01P_REG_RX_PW_P3 0x14
#define NRF24L01P_REG_RX_PW_P4 0x15
#define NRF24L01P_REG_RX_PW_P5 0x16
#define NRF24L01P_REG_FIFO_STATUS 0x17
#define NRF24L01P_REG_DYNPD 0x1C
#define NRF24L01P_REG_FEATURE 0x1D


/* nRF24L01+ typedefs */
typedef uint8_t count;
typedef uint8_t widths;
typedef uint8_t length;
typedef uint16_t delay;
typedef uint16_t channel;

typedef enum
{
    _250kbps = 2,
    _1Mbps = 0,
    _2Mbps = 1
} air_data_rate;

typedef enum
{
    _0dBm = 3,
    _6dBm = 2,
    _12dBm = 1,
    _18dBm = 0
} output_power;

class nrf24l01p
{
public:
    /* Main Functions */
    void rx_init(channel MHz, air_data_rate bps);
    void tx_init(channel MHz, air_data_rate bps);

    ErrorCode rx_receive(uint8_t *rx_payload);
    ErrorCode tx_transmit(uint8_t *tx_payload);

    // Check tx_ds or max_rt
    void tx_irq();

    /* Sub Functions */
    void reset();

    void prx_mode();
    void ptx_mode();

    void power_up();
    void power_down();

    uint8_t get_status();
    uint8_t get_fifo_status();

    // Static payload lengths
    void rx_set_payload_widths(widths bytes);

private:
    uint8_t read_rx_fifo(uint8_t *rx_payload);
    uint8_t write_tx_fifo(uint8_t *tx_payload);

    void flush_rx_fifo();
    void flush_tx_fifo();

    // Clear IRQ pin. Change LOW to HIGH
    void clear_rx_dr();
    void clear_tx_ds();
    void clear_max_rt();

    void set_rf_channel(channel MHz);
    void set_rf_tx_output_power(output_power dBm);
    void set_rf_air_data_rate(air_data_rate bps);

    void set_crc_length(length bytes);
    void set_address_widths(widths bytes);
    void auto_retransmit_count(count cnt);
    void auto_retransmit_delay(delay us);
};

#endif /* __NRF24L01P_H__ */
