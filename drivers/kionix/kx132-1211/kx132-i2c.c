/*
 *  Kionix KX132 Zephyr driver
 */

#define DT_DRV_COMPAT kionix_kx132

#include <string.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "kx132-1211.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

LOG_MODULE_DECLARE(KX132, CONFIG_SENSOR_LOG_LEVEL);

static int kx132_i2c_read(const struct device *dev, uint8_t reg_addr, uint8_t *value, uint16_t len)
{
    int rstatus = 0;
    int sensor_reg_addr = reg_addr;
    int *reg_addr_ptr = &sensor_reg_addr;
    const struct kx132_device_config *config = dev->config;
    rstatus = i2c_write_read_dt(&config->bus_cfg.i2c, reg_addr_ptr, 1, value, len);
    return rstatus;
}

// REF https://github.com/zephyrproject-rtos/zephyr/blob/main/include/zephyr/drivers/i2c.h#L77  <-- i2c_dt_spec definition here

// NOTE:  with KX132 sensor on NXP lpcxpresso55s69 developemnt board,
//  i2c burst write API call produces erroneous data.  Zephyr API
//  `i2c_write_dt()` produces correct data, but requires the peripheral
//  register address to be first element in the buffer of data to
//  write to the given peripheral.  At compile time 'len' or length of
//  data to write is not known, so we would need to call calloc()
//  or similar to allocate memory at run time, greatly complicating
//  what should be a short and fast running bus transaction routine.
//
//  The parameter list here as found in STMicro IIS2DH Zephry driver,
//  is chosen in a way to support calls to both I2C and SPI Zephyr
//  APIs.  This helps decouple sensor level driver code from the
//  communications bus of choice.  But the KX132 sensor compels us to
//  bend the otherwise I2C + SPI interchangeable parameter list.
//
// 2022-12-05 - Given that writes to sensor registers are usually
//  low in byte count, often one byte, we may have good reason to
//  implement a fixed size array for data to write, and copy peripheral
//  register to its first element, and data from *value parameter
//  to successive elements, so that calls to this functions `ctx`
//  structure of function pointers can be of one form - TMH

static int kx132_i2c_write(const struct device *dev, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    int rstatus = 0;
    const struct kx132_device_config *config = dev->config;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// NOTE 2022-11-22, trouble getting Zephyr I2C burst reads and writes
//  to work.  Alternate I2C API selection here unfortunately compells a
//  change in how calling code sets up and passes sensor internal
//  register address.
//
//  The change around the first week of 2022 December, up to Dec' 6
//  is that this routine ignored parameter two 'reg_addr' and requires
//  calling code to put given I2C peripheral register address as first
//  data byte in buffer of data bytes to write.
//
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// - DEV 1206 - 
// Given our driver is for KX132-1211 sensor only, and we know from
// sensor datasheet that we'll only ever write one byte at a time,
// we'll declare a small, local write buffer to build an I2C data
// packet whose first byte is sensor's internal register address.
// This packet is how API function i2c_write_dt() expects to receive
// data to write to given sensor or peripheral - TMH

    uint8_t data_to_write[CONFIG_KX132_I2C_WRITE_BUFFER_SIZE];

    if (len >= CONFIG_KX132_I2C_WRITE_BUFFER_SIZE)
    {
        return -EINVAL;
    }

    data_to_write[0] = reg_addr;
    for ( uint32_t i = 1; i <= len; i++ )
    {
        data_to_write[i] = data[(i - 1)];
    }

// Following Zephyr i2c burst write API implemented in STMicro IIS2DH driver,
// fails with either Kionix KX132 sensor or NXP LPC55S69 flexcomm serial
// peripheral:
//    return i2c_burst_write_dt(&config->i2c, reg_addr | 0x80, value, len);
    rstatus = i2c_write_dt(&config->bus_cfg.i2c, data_to_write, len);

    return rstatus;
}

kionix_ctx_t kx132_i2c_ctx = {
	.read_reg = (kionix_read_ptr) kx132_i2c_read,
	.write_reg = (kionix_write_ptr) kx132_i2c_write,
};

int kx132_i2c_init(const struct device *dev)
{
	struct kx132_device_data *data = dev->data;
	const struct kx132_device_config *config = dev->config;

	if (!device_is_ready(config->bus_cfg.i2c.bus)) {
		LOG_ERR("Bus device in KX132 driver, I2C part is not ready");
		return -ENODEV;
	}

	data->ctx = &kx132_i2c_ctx;
	data->ctx->handle = (void *)dev;
	return 0;
}
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
