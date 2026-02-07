/**
 * @project Kionix KX132 Zephyr driver
 *
 * @license SPDX-License-Identifier: Apache-2.0
 *
 * @note Based on:
 *    ST Microelectronics IIS2DH 3-axis accelerometer driver
 *    Copyright (c) 2020 STMicroelectronics
 *    SPDX-License-Identifier: Apache-2.0
 *    Datasheet:  https://www.st.com/resource/en/datasheet/iis2dh.pdf
 */

#define DT_DRV_COMPAT kionix_kx132

//----------------------------------------------------------------------
// - SECTION - includes
//----------------------------------------------------------------------

#include <string.h>
#include "kx132-1211.h"
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(KX132, CONFIG_SENSOR_LOG_LEVEL);

// For development only:
#include <stdio.h>

//----------------------------------------------------------------------
// - SECTION - defines
//----------------------------------------------------------------------

//#define KX132_SPI_READM   (3 << 6) /* 0xC0  . . . set bit 7 to read, set bit 6 to auto-increment peripheral register addr */
#define KX132_SPI_READM   (2 << 6) /* 0x80  . . . set bit 7 to read, unset bit 6 to hold peripheral register addr steady*/
#define KX132_SPI_WRITEM  (1 << 6) /* 0x40  . . . set bit 6 to auto-increment peripheral register addr */

//----------------------------------------------------------------------
// - SECTION - defines development
//----------------------------------------------------------------------

// Following define is very handy to confirm multi-register config sequences:
//#define DEV__KX_DRIVER_DEV_1202__LOW_LEVEL_SPI_WRITE

//#define DEV__KX_DRIVER_DEV_1120__LOW_LEVEL_SPI_READ



//----------------------------------------------------------------------
// - SECTION - routines
//----------------------------------------------------------------------

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

static int kx132_spi_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct kx132_device_config *config = dev->config;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#ifdef DEV__KX_DRIVER_DEV_1120__LOW_LEVEL_SPI_READ
// #warning "--- DEV 1120 --- compiling kx132_spi_read() function . . ."
    char lbuf[240];
    snprintf(lbuf, sizeof(lbuf), "- KX132 driver - via SPI called to read reg 0x%02X, requesting %u bytes . . .\n",
      reg, len);
    printk("%s", lbuf);
#endif // DEV__KX_DRIVER_DEV_1120__LOW_LEVEL_SPI_READ
#endif

	uint8_t buffer_tx[2] = { reg | KX132_SPI_READM, 0 };
	const struct spi_buf tx_buf = {
			.buf = buffer_tx,
			.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = data,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};

	if (spi_transceive_dt(&config->spi, &tx, &rx)) {
		return -EIO;
	}

	return 0;
}



static int kx132_spi_write(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct kx132_device_config *config = dev->config;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
// #warning "--- DEV 1120 --- compiling kx132_spi_write() function . . ."
#endif

//	uint8_t buffer_tx[1] = { reg | KX132_SPI_WRITEM };  // <-- this OR'ing of 0b01000000 may break comm's with KX132 sensor
	uint8_t buffer_tx[1] = { reg };
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 1,
		},
		{
			.buf = data,
			.len = len,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};

#ifdef DEV__KX_DRIVER_DEV_1202__LOW_LEVEL_SPI_WRITE
    char lbuf[240];
    snprintf(lbuf, sizeof(lbuf), "- DEV 1202 - SPI write writing reg 0x%02x with %u (asked to write %u bytes)\n",
       buffer_tx[0], data[0], len);
    printk("%s", lbuf);
#endif

	if (spi_write_dt(&config->spi, &tx)) {
		return -EIO;
	}

	return 0;
}

kionix_ctx_t kx132_spi_ctx = {
	.read_reg = (kionix_read_ptr) kx132_spi_read,
	.write_reg = (kionix_write_ptr) kx132_spi_write,
};

int kx132_spi_init(const struct device *dev)
{
	struct kx132_device_data *data = dev->data;
	const struct kx132_device_config *config = dev->config;

	if (!spi_is_ready(&config->spi)) {
		LOG_ERR("Bus device in KX132 driver, SPI part is not ready");
		return -ENODEV;
	}

	data->ctx = &kx132_spi_ctx;
	data->ctx->handle = (void *)dev;

	return 0;
}

#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */




#if 0

// diag 1
.........................
snprintf(lbuf, sizeof(lbuf), "- DEV 1120 - about to read kx132 internal register 0x%02x, requesting %u bytes . . .\n",
  reg_addr, len);
printk("%s", lbuf);
.........................


// diag 2
.........................
#warning "--- DEV 1120 --- compiling kx132_12c_read() function . . ."
snprintf(lbuf, sizeof(lbuf), "- DEV 1120 - in KX132 driver, I2C part got first byte %u out of %u bytes read\n",
  value[0], len);
printk("%s", lbuf);
.........................

#endif // 0
