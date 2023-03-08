/**
 * @file lora.h
 * @author Laštůvka Lukáš
 *
 * @version 0.1
 * @date 2023-02-28
 *
 * @copyright
 * MIT License
 *
 * Copyright (c) 2023 Laštůvka Lukáš
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @ref https://github.com/LastuvkaLukas/espidf-lora
 */

#pragma once
#ifndef __ESP_LORA_H__
#define __ESP_LORA_H__

#include <esp_err.h>
#include "lora_reg.h"

/**
 * Writes a value to a LoRa register.
 *
 * @param reg The register to write.
 * @param val The value to write to the register.
 */
void lora_write_reg(lora_reg_t reg, uint8_t val);

/**
 * Reads a register from the LoRa module.
 *
 * @param reg The register to read.
 *
 * @returns The value of the register.
 */
uint8_t lora_read_reg(lora_reg_t reg);

/**
 * Initializes the LoRa module.
 *
 * @returns ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t lora_init(void);

/**
 * Sets the LoRa module to sleep mode.
 */
void lora_sleep(void);

/**
 * Returns the RSSI of the last received packet.
 *
 * @returns The RSSI of the last received packet.
 */
int16_t lora_packet_rssi(void);

/**
 * Returns the SNR of the last received packet.
 *
 * @returns The SNR of the last received packet.
 */
float lora_packet_snr(void);

/**
 * Sets the LoRa module to transmit mode.
 */
void lora_begin_tx(void);

/**
 * Writes data to the LoRa TX FIFO.
 *
 * @param buf The buffer containing the data to be written.
 * @param size The size of the buffer.
 *
 * @returns ESP_OK if successful, ESP_ERR_INVALID_SIZE if the size is invalid.
 */
esp_err_t lora_write_tx(uint8_t *buf, uint8_t size);

/**
 * Ends a transmission.
 */
esp_err_t lora_end_tx(void);

/**
 * Starts the LoRa module in receive mode.
 *
 * @returns ESP_OK if the operation was successful, ESP_FAIL otherwise.
 */
esp_err_t lora_begin_rx(uint8_t *len);

/**
 * Reads data from the LoRa module's RX FIFO.
 *
 * @param buf A pointer to the buffer to store the data.
 * @param size The number of bytes to read.
 *
 * @returns ESP_OK if the read was successful, ESP_ERR_INVALID_SIZE if the
 *          size is invalid, or ESP_FAIL if the read failed.
 */
esp_err_t lora_read_rx(uint8_t *buf, uint8_t size);

/**
 * End of income.
 */
void lora_end_rx(void);

/**
 * Waits for a CAD to be detected.
 */
void lora_waiting_cad(void);

/**
 * Test a CAD to be detected.
 */
bool lora_is_cad_detected(void);

/**
 * Starts the LoRa module in receive mode after CAD detection.
 *
 * @returns ESP_OK if the operation was successful, ESP_FAIL otherwise.
 */
esp_err_t lora_waiting_cad_rx(uint8_t *len);

#ifdef CONFIG_CONFIG_LORA_DIO_ON_GPIO

/**
 * Sets the DIO mode for a DIO pin.
 *
 * @param num The DIO pin number.
 * @param mode The DIO mode.
 * @param val The value to set the DIO pin to.
 */
void lora_dio_write(lora_reg_dio_num_t num, lora_reg_dio_mode_t mode, uint8_t val);

#endif

#endif /* __ESP_LORA_H__ */
