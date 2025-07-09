/**
 * @file lora.c
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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/gpio.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>

#include "lora.h"

#ifdef CONFIG_LORA_HSPI_ON
#define LORA_SPI HSPI_HOST
#elif CONFIG_LORA_VSPI_ON
#define LORA_SPI VSPI_HOST
#elif CONFIG_LORA_SPI2_ON
#define LORA_SPI SPI2_HOST
#elif CONFIG_LORA_SPI3_ON
#define LORA_SPI SPI3_HOST
#else
#error "No selected LORA_SPI"
#define LORA_SPI
#endif

static const char TAG[] = "lora";

/**
 * Initializes the SPI device.
 */
static spi_device_handle_t __spi;

/**
 * Clears the IRQ flags.
 */
static void lora_clear_irq(void)
{
   lora_write_reg(LORA_REG_IRQ_FLAGS, 0xff);
}

/**
 * Sets the LoRa mode.
 *
 * @param mode The LoRa mode.
 */
static void lora_set_mode(lora_reg_op_mode_t mode)
{
   ESP_LOGV(TAG, "Set mode num: %i", mode);
   lora_clear_irq();
   lora_write_reg(LORA_REG_OP_MODE, LORA_OP_MODE_LONG_RANGE_MODE | mode);
}

/**
 * Resets the LoRa module.
 */
static void lora_reset(void)
{
   ESP_LOGD(TAG, "Reset module");
   gpio_set_level(CONFIG_LORA_RST_GPIO, 0);
   vTaskDelay(pdMS_TO_TICKS(1));
   gpio_set_level(CONFIG_LORA_RST_GPIO, 1);
   vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Sets the output power of the LoRa transceiver.
 *
 * @param level The output power level.
 */
static void lora_set_tx_power(uint8_t level)
{
   ESP_LOGD(TAG, "Set TX power level: %i", level);
   if (level < 2)
      level = 2;
   else if (level > 17)
      level = 17;
   lora_write_reg(LORA_REG_PA_CONFIG, LORA_PA_CONFIG_PA_SELECT_PA_BOOST | (level - 2));
}

/**
 * Sets the frequency of the LoRa transceiver.
 *
 * @param frequency The frequency in Hz.
 */
static void lora_set_frequency(uint64_t frequency)
{
   ESP_LOGD(TAG, "Set frequency to: %lli", frequency);
   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   lora_write_reg(LORA_REG_FRF_MSB, (uint8_t)(frf >> 16));
   lora_write_reg(LORA_REG_FRF_MID, (uint8_t)(frf >> 8));
   lora_write_reg(LORA_REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Sets the preamble length for LoRa.
 *
 * @param len The length of the preamble in symbols.
 */
static void lora_set_preamble_length(uint16_t len)
{
   ESP_LOGD(TAG, "Set preamble length: %i", len);
   lora_write_reg(LORA_REG_PREAMBLE_MSB, (uint8_t)(len >> 8));
   lora_write_reg(LORA_REG_PREAMBLE_LSB, (uint8_t)(len >> 0));
}

/**
 * Sets the LoRa sync word.
 *
 * @param word The sync word to set.
 */
static void lora_set_sync_word(uint8_t word)
{
   ESP_LOGI(TAG, "Set Network ID: %#02x", word);
   lora_write_reg(LORA_REG_SYNC_WORD, word);
}

/**
 * Enables the CRC check on received packets.
 */
static void lora_enable_crc(void)
{
   ESP_LOGD(TAG, "Enable CRC in payload");
   lora_write_reg(LORA_REG_MODEM_CONFIG_2, lora_read_reg(LORA_REG_MODEM_CONFIG_2) | LORA_MODEM_CONFIG2_RX_PAYLOAD_CRC_ON);
}

/**
 * Writes a value to a LoRa register.
 *
 * @param reg The register to write.
 * @param val The value to write to the register.
 */
void lora_write_reg(lora_reg_t reg, uint8_t val)
{
   ESP_LOGV(TAG, "Write %#02x to %#02x register", val, reg);
   uint8_t tx[2] = {0x80 | reg, val};
   uint8_t rx[2];

   spi_transaction_t t = {
       .flags = 0,
       .length = 8 * sizeof(tx),
       .tx_buffer = tx,
       .rx_buffer = rx};

#ifdef CONFIG_LORA_CS_ON_GPIO
   gpio_set_level(CONFIG_LORA_CS_GPIO, 0);
#endif
   spi_device_transmit(__spi, &t);
#ifdef CONFIG_LORA_CS_ON_GPIO
   gpio_set_level(CONFIG_LORA_CS_GPIO, 1);
#endif
}

/**
 * Reads a register from the LoRa module.
 *
 * @param reg The register to read.
 *
 * @returns The value of the register.
 */
uint8_t lora_read_reg(lora_reg_t reg)
{
   ESP_LOGV(TAG, "Read from %#02x register", reg);
   uint8_t tx[2] = {reg, 0xff};
   uint8_t rx[2];

   spi_transaction_t t = {
       .flags = 0,
       .length = 8 * sizeof(tx),
       .tx_buffer = tx,
       .rx_buffer = rx};

#ifdef CONFIG_LORA_CS_ON_GPIO
   gpio_set_level(CONFIG_LORA_CS_GPIO, 0);
#endif
   spi_device_transmit(__spi, &t);
#ifdef CONFIG_LORA_CS_ON_GPIO
   gpio_set_level(CONFIG_LORA_CS_GPIO, 1);
#endif

   return rx[1];
}

/**
 * Initializes the LoRa module.
 *
 * @returns ESP_OK if successful, ESP_FAIL otherwise.
 */
esp_err_t lora_init(void)
{
   ESP_LOGI(TAG, "Initialize LoRa module");
   esp_err_t err = ESP_OK;

   gpio_pad_select_gpio(CONFIG_LORA_RST_GPIO);
   gpio_set_direction(CONFIG_LORA_RST_GPIO, GPIO_MODE_OUTPUT);

#ifdef CONFIG_LORA_CS_ON_GPIO
   gpio_pad_select_gpio(CONFIG_LORA_CS_GPIO);
   gpio_set_direction(CONFIG_LORA_CS_GPIO, GPIO_MODE_OUTPUT);
#endif

#ifdef CONFIG_LORA_DIO0_ON_GPIO
   gpio_pad_select_gpio(CONFIG_LORA_DIO0_GPIO);
   gpio_set_direction(CONFIG_LORA_DIO0_GPIO, GPIO_MODE_INPUT);
#endif

#ifdef CONFIG_LORA_DIO1_ON_GPIO
   gpio_pad_select_gpio(CONFIG_LORA_DIO1_GPIO);
   gpio_set_direction(CONFIG_LORA_DIO1_GPIO, GPIO_MODE_INPUT);
#endif

#ifdef CONFIG_LORA_DIO2_ON_GPIO
   gpio_pad_select_gpio(CONFIG_LORA_DIO2_GPIO);
   gpio_set_direction(CONFIG_LORA_DIO2_GPIO, GPIO_MODE_INPUT);
#endif

#ifdef CONFIG_LORA_DIO3_ON_GPIO
   gpio_pad_select_gpio(CONFIG_LORA_DIO3_GPIO);
   gpio_set_direction(CONFIG_LORA_DIO3_GPIO, GPIO_MODE_INPUT);
#endif

#ifdef CONFIG_LORA_DIO4_ON_GPIO
   gpio_pad_select_gpio(CONFIG_LORA_DIO4_GPIO);
   gpio_set_direction(CONFIG_LORA_DIO4_GPIO, GPIO_MODE_INPUT);
#endif

#ifdef CONFIG_LORA_DIO5_ON_GPIO
   gpio_pad_select_gpio(CONFIG_LORA_DIO5_GPIO);
   gpio_set_direction(CONFIG_LORA_DIO5_GPIO, GPIO_MODE_INPUT);
#endif

   spi_bus_config_t bus = {
       .miso_io_num = CONFIG_LORA_MISO_GPIO,
       .mosi_io_num = CONFIG_LORA_MOSI_GPIO,
       .sclk_io_num = CONFIG_LORA_SCK_GPIO,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1,
       .max_transfer_sz = 0};

   err = spi_bus_initialize(LORA_SPI, &bus, 0);
   if (err != ESP_OK)
      return err;

   spi_device_interface_config_t dev = {
       .clock_speed_hz = 9000000,
       .mode = 0,
       .spics_io_num = -1,
       .queue_size = 1,
       .flags = 0,
       .pre_cb = NULL};
   err = spi_bus_add_device(LORA_SPI, &dev, &__spi);
   if (err != ESP_OK)
      return err;

   lora_reset();
   lora_set_mode(LORA_OP_MODE_SLEEP);

   uint8_t i = 0;
   while (i++ < CONFIG_LORA_INIT_TIMEOUT)
   {
      uint8_t version = lora_read_reg(LORA_REG_VERSION);
      if (version == 0x12)
         break;
      vTaskDelay(2);
   }
   if (i >= CONFIG_LORA_INIT_TIMEOUT + 1)
      return ESP_ERR_TIMEOUT;

   lora_write_reg(LORA_REG_FIFO_RX_BASE_ADDR, 0);
   lora_write_reg(LORA_REG_FIFO_TX_BASE_ADDR, 0);

   lora_write_reg(LORA_REG_LNA, lora_read_reg(LORA_REG_LNA) | LORA_LNA_BOOST_HF);
   lora_write_reg(LORA_REG_MODEM_CONFIG_3, LORA_MODEM_CONFIG3_AGC_AUTO_ON);

   lora_set_tx_power(CONFIG_LORA_TX_POWER);
   lora_set_frequency(CONFIG_LORA_FREQ);
   lora_set_preamble_length(CONFIG_LORA_PREAMBLE_LEN);
   lora_set_sync_word(CONFIG_LORA_SYNC_WORD);
   lora_enable_crc();

   lora_set_mode(LORA_OP_MODE_STDBY);
   return err;
}

/**
 * Sets the LoRa module to sleep mode.
 */
void lora_sleep(void)
{
   ESP_LOGI(TAG, "Go sleep LoRa module");
   lora_set_mode(LORA_OP_MODE_SLEEP);
}

/**
 * Returns the RSSI of the last received packet.
 *
 * @returns The RSSI of the last received packet.
 */
int16_t lora_packet_rssi(void)
{
   return (lora_read_reg(LORA_REG_PKT_RSSI_VALUE) - (CONFIG_LORA_FREQ < 868E6 ? 164 : 157));
}

/**
 * Returns the SNR of the last received packet.
 *
 * @returns The SNR of the last received packet.
 */
float lora_packet_snr(void)
{
   return ((int8_t)lora_read_reg(LORA_REG_PKT_SNR_VALUE)) * 0.25;
}

/**
 * Sets the LoRa module to transmit mode.
 */
void lora_begin_tx(void)
{
   lora_set_mode(LORA_OP_MODE_STDBY);
   lora_write_reg(LORA_REG_FIFO_ADDR_PTR, 0);
}

/**
 * Writes data to the LoRa TX FIFO.
 *
 * @param buf The buffer containing the data to be written.
 * @param size The size of the buffer.
 *
 * @returns ESP_OK if successful ESP_ERR_INVALID_SIZE if the size is invalid.
 */
esp_err_t lora_write_tx(uint8_t *buf, uint8_t size)
{
   if (size == 0 || (uint16_t)((lora_read_reg(LORA_REG_FIFO_ADDR_PTR) + size)) > 0xff)
      return ESP_ERR_INVALID_SIZE;

   for (int i = 0; i < size; i++)
      lora_write_reg(LORA_REG_FIFO, *buf++);

   return ESP_OK;
}

/**
 * Ends a transmission.
 */
esp_err_t lora_end_tx(void)
{
   uint8_t len = lora_read_reg(LORA_REG_FIFO_ADDR_PTR);
   if (len == 0)
      return ESP_ERR_INVALID_SIZE;

   lora_write_reg(LORA_REG_PAYLOAD_LENGTH, len);

   lora_set_mode(LORA_OP_MODE_TX);
   uint8_t read;
   do
   {
      read = lora_read_reg(LORA_REG_IRQ_FLAGS);
      vTaskDelay(2);
   } while ((read & LORA_IRQ_FLAGS_TX_DONE) == 0);

   ESP_LOGI(TAG, "Transmit payload with length: %i", len);
   lora_write_reg(LORA_REG_IRQ_FLAGS, LORA_IRQ_FLAGS_TX_DONE);

   return ESP_OK;
}

/**
 * Starts the LoRa module in receive mode.
 *
 * @returns ESP_OK if the operation was successful, ESP_FAIL otherwise.
 */
esp_err_t lora_begin_rx(uint8_t *len)
{
   *len = 0;
   uint8_t read;

   lora_set_mode(LORA_OP_MODE_RX_SINGLE);

   for (;;)
   {
      read = lora_read_reg(LORA_REG_IRQ_FLAGS);

      if ((read & LORA_IRQ_FLAGS_RX_DONE))
         break;

      if (read & LORA_IRQ_FLAGS_RX_TIMEOUT)
         return ESP_ERR_TIMEOUT;

      vTaskDelay(2);
   }

   if (lora_read_reg(LORA_REG_IRQ_FLAGS) & LORA_IRQ_FLAGS_PAYLOAD_CRC_ERROR)
      return ESP_ERR_INVALID_CRC;

   *len = lora_read_reg(LORA_REG_RX_NB_BYTES);

   ESP_LOGI(TAG, "Recieved payload with length: %i", *len);
   lora_write_reg(LORA_REG_FIFO_ADDR_PTR, lora_read_reg(LORA_REG_FIFO_RX_CURRENT_ADDR));

   return ESP_OK;
}

/**
 * Reads data from the LoRa module's RX FIFO.
 *
 * @param buf A pointer to the buffer to store the data.
 * @param size The number of bytes to read.
 *
 * @returns ESP_OK if the read was successful, ESP_ERR_INVALID_SIZE if the
 *          size is invalid, or ESP_FAIL if the read failed.
 */
esp_err_t lora_read_rx(uint8_t *buf, uint8_t size)
{

   if (size == 0 || (uint16_t)((lora_read_reg(LORA_REG_FIFO_ADDR_PTR) + size)) > 0xff)
      return ESP_ERR_INVALID_SIZE;

   for (int i = 0; i < size; i++)
      *buf++ = lora_read_reg(LORA_REG_FIFO);

   return ESP_OK;
}

/**
 * End of income.
 */
void lora_end_rx(void)
{
   lora_set_mode(LORA_OP_MODE_STDBY);
}

/**
 * Waits for a CAD to be detected.
 */
void lora_waiting_cad(void)
{
   uint8_t read;

   lora_set_mode(LORA_OP_MODE_CAD);
   for (;;)
   {
      read = lora_read_reg(LORA_REG_IRQ_FLAGS);

      if (read & LORA_IRQ_FLAGS_CAD_DONE)
      {
         if (read & LORA_IRQ_FLAGS_CAD_DETECTED)
            break;
         lora_set_mode(LORA_OP_MODE_CAD);
      }

      vTaskDelay(2);
   }
   ESP_LOGD(TAG, "CAD detected");
}

/**
 * Test a CAD to be detected.
 */
bool lora_is_cad_detected(void)
{
   lora_set_mode(LORA_OP_MODE_CAD);

   uint8_t read;

   for (;;)
   {
      read = lora_read_reg(LORA_REG_IRQ_FLAGS);
      if (read & LORA_IRQ_FLAGS_CAD_DONE)
      {
         if (read & LORA_IRQ_FLAGS_CAD_DETECTED)
         {
            ESP_LOGD(TAG, "CAD detected");
            return true;
         }
         return false;
      }
   }
}

/**
 * Starts the LoRa module in receive mode after CAD detection.
 *
 * @returns ESP_OK if the operation was successful, ESP_FAIL otherwise.
 */
esp_err_t lora_waiting_cad_rx(uint8_t *len)
{
   lora_waiting_cad();
   return lora_begin_rx(len);
}

#ifdef CONFIG_CONFIG_LORA_DIO_ON_GPIO

/**
 * Sets the DIO mode for a DIO pin.
 *
 * @param num The DIO pin number.
 * @param mode The DIO mode.
 * @param val The value to set the DIO pin to.
 */
void lora_dio_write(lora_reg_dio_num_t num, lora_reg_dio_mode_t mode, uint8_t val)
{
   lora_reg_t dio_reg = ((num == LORA_DIO_NUM_4) || (num == LORA_DIO_NUM_5)) ? LORA_REG_DIO_MAPPING_2 : LORA_REG_DIO_MAPPING_1;
   uint8_t mask = 0x00;

   switch (num)
   {
   case LORA_DIO_NUM_0:
   case LORA_DIO_NUM_4:
      mask = (val << 0x06);
      break;

   case LORA_DIO_NUM_1:
   case LORA_DIO_NUM_5:
      mask = (val << 0x04);
      break;

   case LORA_DIO_NUM_2:
      mask = (val << 0x02);
      break;

   case LORA_DIO_NUM_3:
      mask = val;
      break;
   }

   if (mode == LORA_DIO_SET)
   {
      lora_write_reg(dio_reg, lora_read_reg(dio_reg) | mask);
      return;
   }
   lora_write_reg(dio_reg, lora_read_reg(dio_reg) & (~mask));
}

#endif
