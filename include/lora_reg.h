/**
 * @file lora_reg.h
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
#ifndef __LORA_REG_H__
#define __LORA_REG_H__

typedef enum lora_reg_t
{
    LORA_REG_FIFO = 0x00,
    LORA_REG_OP_MODE = 0x01,
    LORA_REG_FRF_MSB = 0x06,
    LORA_REG_FRF_MID = 0x07,
    LORA_REG_FRF_LSB = 0x08,
    LORA_REG_PA_CONFIG = 0x09,
    LORA_REG_LNA = 0x0c,
    LORA_REG_FIFO_ADDR_PTR = 0x0d,
    LORA_REG_FIFO_TX_BASE_ADDR = 0x0e,
    LORA_REG_FIFO_RX_BASE_ADDR = 0x0f,
    LORA_REG_FIFO_RX_CURRENT_ADDR = 0x10,
    LORA_REG_IRQ_FLAGS_MASK = 0x11,
    LORA_REG_IRQ_FLAGS = 0x12,
    LORA_REG_RX_NB_BYTES = 0x13,
    LORA_REG_PKT_SNR_VALUE = 0x19,
    LORA_REG_PKT_RSSI_VALUE = 0x1a,
    LORA_REG_MODEM_CONFIG_1 = 0x1d,
    LORA_REG_MODEM_CONFIG_2 = 0x1e,
    LORA_REG_PREAMBLE_MSB = 0x20,
    LORA_REG_PREAMBLE_LSB = 0x21,
    LORA_REG_PAYLOAD_LENGTH = 0x22,
    LORA_REG_MODEM_CONFIG_3 = 0x26,
    LORA_REG_RSSI_WIDEBAND = 0x2c,
    LORA_REG_DETECTION_OPTIMIZE = 0x31,
    LORA_REG_DETECTION_THRESHOLD = 0x37,
    LORA_REG_SYNC_WORD = 0x39,
    LORA_REG_DIO_MAPPING_1 = 0x40,
    LORA_REG_DIO_MAPPING_2 = 0x41,
    LORA_REG_VERSION = 0x42,
} lora_reg_t;

typedef enum lora_reg_op_mode_t
{
    LORA_OP_MODE_SLEEP = 0x00,
    LORA_OP_MODE_STDBY = 0x01,
    LORA_OP_MODE_FSTX = 0x02,
    LORA_OP_MODE_TX = 0x03,
    LORA_OP_MODE_FSRX = 0x04,
    LORA_OP_MODE_RX_CONTINUOUS = 0x05,
    LORA_OP_MODE_RX_SINGLE = 0x06,
    LORA_OP_MODE_CAD = 0x07,
    LORA_OP_MODE_LOW_FREQ_ON = 0x08,
    LORA_OP_MODE_LONG_RANGE_MODE = 0x80,
} lora_reg_op_mode_t;

typedef enum lora_reg_pa_config_t
{
    LORA_PA_CONFIG_PA_SELECT_RFO = 0x00,
    LORA_PA_CONFIG_PA_SELECT_PA_BOOST = 0x80,
} lora_reg_pa_config_t;

typedef enum lora_reg_irq_flags_t
{
    LORA_IRQ_FLAGS_CAD_DETECTED = 0x01,
    LORA_IRQ_FLAGS_FHSS_CHANGE_CHANNEL = 0x02,
    LORA_IRQ_FLAGS_CAD_DONE = 0x04,
    LORA_IRQ_FLAGS_TX_DONE = 0x08,
    LORA_IRQ_FLAGS_VALID_HEADER = 0x10,
    LORA_IRQ_FLAGS_PAYLOAD_CRC_ERROR = 0x20,
    LORA_IRQ_FLAGS_RX_DONE = 0x40,
    LORA_IRQ_FLAGS_RX_TIMEOUT = 0x80,
} lora_reg_irq_flags_t;

#define LORA_LNA_BOOST_HF 0x03
#define LORA_MODEM_CONFIG2_RX_PAYLOAD_CRC_ON 0x04
#define LORA_MODEM_CONFIG3_AGC_AUTO_ON 0x04

#ifdef CONFIG_LORA_DIO_ON_GPIO

typedef enum lora_reg_dio_num_t
{
    LORA_DIO_NUM_0 = 0,
    LORA_DIO_NUM_1 = 1,
    LORA_DIO_NUM_2 = 2,
    LORA_DIO_NUM_3 = 3,
    LORA_DIO_NUM_4 = 4,
    LORA_DIO_NUM_5 = 5,
} lora_reg_dio_num_t;

typedef enum lora_reg_dio_mode_t
{
    LORA_DIO_SET = 0,
    LORA_DIO_CLEAR = 0,
} lora_reg_dio_mode_t;

#endif

#endif // __LORA_REG_H__