/**
 * @project Kionix Sensor Drivers
 * @file kx132-1211.c
 * @author Ted Havelka
 * @license Apache 2.0 licensed.
 */

#define DT_DRV_COMPAT kionix_kx132

#include <stdio.h>
#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/devicetree.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(KX132, CONFIG_SENSOR_LOG_LEVEL);

#include "kx132-1211.h"
#include "kx132-registers.h"

#define DEFAULT_OPTION_ZERO (0U)

//----------------------------------------------------------------------
// - SECTION - routines
//----------------------------------------------------------------------

static int kx132_1211_attr_get(const struct device *dev,
                               enum sensor_channel chan,
                               enum sensor_attribute attr,
                               struct sensor_value *value)
{
    int rstatus = 0;

    // TODO [ ] There is a sensor attribute enum value `SENSOR_ATTR_CONFIGURATION`
    //           defined at
    //           https://github.com/zephyrproject-rtos/zephyr/blob/main/include/zephyr/drivers/sensor.h#L376
    //           Test for this value and when found.
    //
    // TODO [ ] amend the use of 'value' parameter to convey KX132-specific
    //           attribute to change in .val1 followed by flags or given value 
    //           to set passed in .val2.  This will overcome the 'not in enum'
    //           build time warnings from our non-Zephyr sensor attribute
    //           enum extending values.

    // Note 2026-20-23 dom - out-of-tree-driver-demo does not call `sensor_attr_get`
    //  thus changes here will not be visible until that supporting demo
    //  app is updated.

    switch (attr)
    {
        case SENSOR_ATTR_CONFIGURATION:
        switch (value->val1)
	{
            case SENSOR_ATTR_KIONIX__STATUS_REG_INS2: // TODO rename ==> KX132_ATTRIBUTE_REG_INS2
                rstatus = kx132_get_attr__return_interrupt_statae_2(dev, value);
                break;

            case SENSOR_ATTR_KIONIX__STATUS_REG_ODCNTL: // TODO rename ==> KX132_ATTRIBUTE_REG_ODCNTL
                rstatus = kx132_get_attr__output_data_rate(dev, value);
                break;

            case SENSOR_ATTR_KIONIX__CONFIG_REG_BUF_CNTL1: // TODO rename ==> KX132_ATTRIBUTE_REG_CNTL1
                rstatus = kx132_attr_sample_threshold_setting_get(dev, value);
                break;

            default:
		LOG_ERR("Failed to get KX132 configuration encoded as %d, unsupported config",
			value->val1);
                rstatus = -ENOTSUP;
                break;
	}
        default:
            LOG_ERR("Failed to get KX132 attribute %d, unsupported attribute", attr);
            rstatus = -EINVAL;
            break;
    }

    return rstatus;
}

//----------------------------------------------------------------------
// @brief  Routine to update KX132 sensor attributes, also called
//        configuration values.  The somewhat misnamed `value` parameter
//        . . .
//
// Example set up and call, sets up readings with sensor actuated hardware
// interrupt, and sets an accelerometer output data rate of 3200 Hz:
//
//      requested_config.val1 = KX132_ENABLE_SYNC_READINGS_WITH_HW_INTERRUPT;
//      requested_config.val2 = KX132_ODR_3200_HZ;
//
//      sensor_api_status = sensor_attr_set(
//       dev_kx132_1,
//       SENSOR_CHAN_ALL,
//       SENSOR_ATTR_PRIV_START,
//       &requested_config
//      );
//
// Notes:
//
//   +  REF https://docs.zephyrproject.org/2.6.0/reference/peripherals/sensor.html#c.sensor_attribute
//      case SENSOR_ATTR_CONFIGURATION:  <- this enumerator available in
//      Zephyr RTOS 2.7.99 but not 2.6.0 - TMH
//
//   +  For compatibility with Zephyr 2.6.0, Zephyr sensor channel
//      enum member `SENSOR_CHAN_ALL` is paired with the more recently
//      available and more fitting sensor attribute enum member
//      `SENSOR_ATTR_CONFIGURATION` in a nested switch construct of
//      this routine.
//----------------------------------------------------------------------

static int kx132_1211_attr_set(const struct device *dev,
                               enum sensor_channel chan,
                               enum sensor_attribute attr,
                               const struct sensor_value *val)
{
    uint32_t rstatus = 0;
    uint32_t sensor_config_requested = val->val1;

    switch (attr)
    {

// Note:  Zephyr standard sensor attribute enumeration values will be
//  added here, at top of SWITCH construct.  None so far implemented as
//  of 2022-11-28.  See `zephyr/include/zephyr/drivers/sensor.h` for
//  full enumeration of Zephyr defined sensor attributes.  - TMH

        case SENSOR_ATTR_PRIV_START:
        {
            switch (chan)
            {
                case SENSOR_CHAN_ALL:
                case SENSOR_ATTR_CONFIGURATION:
                {
                    switch (sensor_config_requested)
                    {

                        case KX132_PERMFORM_SOFTWARE_RESET:
                            rstatus = kx132_software_reset(dev);
                            break;

                        case KX132_ENTER_STANDBY_MODE:
                            kx132_enter_standby_mode(dev);
                            break;

                        case KX132_DISABLE_SAMPLE_BUFFER:
                            kx132_disable_sample_buffer(dev);
                            break;

                        case KX132_CLEAR_SAMPLE_BUFFER:
                            kx132_update_reg__buf_clear(dev);
                            break;

                        case KX132_ENABLE_ASYNC_READINGS:
                            kx132_enable_asynchronous_readings(dev);
                            break;

                        case KX132_SET_OUTPUT_DATA_RATE:
                            kx132_update_reg__odcntl__output_data_rate(dev,
                              (const enum kx132_1211_output_data_rates)val->val2);
                            break;

                        case KX132_SET_WMI_SAMPLE_THRESHOLD:
                            kx132_update_reg__sample_threshold_buf_cntl1(dev,
                              (const uint8_t)val->val2);
                            break;

// Shadow register updates:
                        case KX132_SET_SHADOW_REG__WMI_SAMPLE_THRESHOLD:
                            kx132_update_shadow_reg__sample_threshold(dev,
                              (const uint8_t)val->val2);
                            break;

//----------------------------------------------------------------------
// Multi-register updates, configuration routines:
//----------------------------------------------------------------------

// NEED 2023-01-13 to review whether needed parameters are missing here`
                        case KX132_ENABLE_SYNC_READINGS_WITH_HW_INTERRUPT:
                            rstatus = kx132_enable_synchronous_reading_with_hw_interrupt(dev);
                            break;

#ifdef CONFIG_KX132_TRIGGER
                        case KX132_REINITIALIZE_DRDY_GPIO_PORT:
                            rstatus = kx132_reinitialize_interrupt_port(dev, DEFAULT_OPTION_ZERO);
                            break;
#endif
                        case KX132_ENABLE_WATERMARK_INTERRUPT:
                            rstatus = kx132_enable_watermark_interrupt(dev);
                            break;

                        default:
                            rstatus = -ENOTSUP;
                            break;
                    }
                }
                    break;

                default:
                {
                    rstatus = -EINVAL;
                    break;
                }

            }
        }
            break;

        default: // ...default action to take with unrecognized sensor attribute
        {
            rstatus = -EINVAL;
            break;
        }
    }

    return rstatus;

}

static int kx132_1211_sample_fetch(const struct device *dev, enum sensor_channel channel)
{
    int rstatus = 0;

    switch (channel)
    {
        case SENSOR_CHAN_KIONIX_MANUFACTURER_ID:  // a four byte value
            kx132_fetch_device_id(dev);
            break;

        case SENSOR_CHAN_KIONIX_PART_ID:          // a two byte value
            kx132_fetch_part_id(dev);
            break;

        case SENSOR_CHAN_ACCEL_X:                 // one or two byte value, depending on KX132-1211 configuration
            kx132_fetch_acceleration_x_axis(dev);
            break;

        case SENSOR_CHAN_ACCEL_Y:                 // one or two byte value, depending on KX132-1211 configuration
            kx132_fetch_acceleration_y_axis(dev);
            break;

        case SENSOR_CHAN_ACCEL_Z:                 // one or two byte value, depending on KX132-1211 configuration
            kx132_fetch_acceleration_z_axis(dev);
            break;

        case SENSOR_CHAN_ACCEL_XYZ:               // read of prior three pairs of registers in sequence
            kx132_fetch_acceleration_xyz_axis(dev);
            break;

        case SENSOR_CHAN_KIONIX_INTERRUPT_LATCH_RELEASE:
            kx132_fetch_interrupt_latch_release(dev);
            break;

        case SENSOR_CHAN_KIONIX_BUF_READ:
            kx132_fetch_readings_from_buf_read(dev);
            break;

        default:
            rstatus = -EINVAL;
    }

    return rstatus;

}

static int kx132_1211_channel_get(const struct device *dev,
                                  enum sensor_channel chan,
                                  struct sensor_value *val)
{
// 2021-08-31 - function implementation in progress, TMH.

    int routine_status = 0;
    struct kx132_device_data *data = dev->data;

    memset(val, 0, sizeof(*val));

    switch (chan)
    {
        case SENSOR_CHAN_KIONIX_MANUFACTURER_ID:
            val->val1 = data->manufacturer_id.as_32_bit_integer;
            val->val2 = 0;
            break;

        case SENSOR_CHAN_KIONIX_PART_ID:
            val->val1 = data->part_id.as_16_bit_integer;
            val->val2 = 0;
            break;

        case SENSOR_CHAN_ACCEL_X:
            val->val1 = ( ( data->accel_axis_x[1] << 8 ) | ( data->accel_axis_x[0] ) );
            val->val2 = 0;
            break;

        case SENSOR_CHAN_ACCEL_Y:
            val->val1 = ( ( data->accel_axis_y[1] << 8 ) | ( data->accel_axis_y[0] ) );
            val->val2 = 0;
            break;

        case SENSOR_CHAN_ACCEL_Z:
            val->val1 = ( ( data->accel_axis_z[1] << 8 ) | ( data->accel_axis_z[0] ) );
            val->val2 = 0;
            break;

        case SENSOR_CHAN_ACCEL_XYZ:
// Here encode X, Y, Z two-byte accelerometer readings in struct sensor_value as follows:
//
//          sensor_value.val1 <-- [ Y_MSB ][ Y_LSB ][ X_MSB ][ X_LSB ]
//          sensor_value.val2 <-- [   0   ][   0   ][ Z_MSB ][ Z_LSB ]

            val->val1 = ( ( data->accel_axis_x[1] <<  8 ) | ( data->accel_axis_x[0] <<  0 )
                        | ( data->accel_axis_y[1] << 24 ) | ( data->accel_axis_y[0] << 16 ) );

            val->val2 = ( ( data->accel_axis_z[1] <<  8 ) | ( data->accel_axis_z[0] <<  0 ) );
            break;

// NOTE, KX132 sample buffer readings not to be confused with values
// read from XOUT_L, XOUT_H, YOUT_L, YOUT_H, etc:
        case SENSOR_CHAN_KIONIX_BUF_READ:
            val->val1 = ((data->shadow_reg_buf_read[0] <<  0) | (data->shadow_reg_buf_read[1] <<  8) | // x | xlsb, xmsb
                         (data->shadow_reg_buf_read[2] << 16) | (data->shadow_reg_buf_read[3] << 24)   // y | ylsb, ymsb
                        );
            val->val2 = ((data->shadow_reg_buf_read[4] <<  0) | (data->shadow_reg_buf_read[5] <<  8)   // z | zlsb, zmsb
                        );
            break;

        case SENSOR_CHAN_KIONIX_INTERRUPT_LATCH_RELEASE:
            val->val1 = data->shadow_reg_int_rel;
            val->val2 = 0;
            break;

        case SENSOR_CHAN_KIONIX_INS2:
            val->val1 = data->shadow_reg_ins2;
            val->val2 = 0;
            break;

        default:
            routine_status = -EINVAL;
    }

    return routine_status;

}

static int kx132_pm_action(const struct device *dev,
                            enum pm_device_action action)
{
	uint32_t a = 0x55;
	int32_t rc = 0;

	switch(action) {
	case PM_DEVICE_ACTION_TURN_ON:
		a = 0xAA;
		break;
	case PM_DEVICE_ACTION_RESUME:
		a = 0x00;
		break;
	case PM_DEVICE_ACTION_SUSPEND:
	case PM_DEVICE_ACTION_TURN_OFF:
		rc = -ENOTSUP;
		break;
	default:
		rc = -ENOTSUP;
    }

	return rc;
}

//----------------------------------------------------------------------
// - SECTION - init routines
//----------------------------------------------------------------------

// Following design pattern of Zephyr 3.2.0 zephyr/drivers/sensor/iis2dh/iis2dh.c:

static int kx132_init_interface(const struct device *dev)
{
        int rstatus;

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
        rstatus = kx132_spi_init(dev);
        if (rstatus) {
                return rstatus;
        }
#elif DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
        rstatus = kx132_i2c_init(dev);
        if (rstatus) {
                return rstatus;
        }
#else
#warning "No device with compat " DT_DRV_COMPAT " found on I2C or SPI bus"
#endif
        return 0;
}

// FEATURE - initializating function in KX132-1211 driver:
// # REF https://github.com/zephyrproject-rtos/zephyr/blob/main/drivers/sensor/iis2dh/iis2dh.c#L253

static int kx132_init(const struct device *dev)
{
LOG_ERR("M1");

#ifdef CONFIG_KX132_TRIGGER
    const struct kx132_device_config *cfg = dev->config;
#endif
    struct kx132_device_data *data = dev->data;
    uint32_t rc = 0;
    kx132_init_interface(dev);

    // TODO [ ] move software reset function call to `kx132_pm_action()`
    rc = kx132_software_reset(dev);

LOG_ERR("M2");

#ifdef CONFIG_KX132_TRIGGER
        if (cfg->int_gpio.port) {
            if (kx132_init_interrupt(dev) < 0) {
                LOG_ERR("kx132-1211.c - failed to initialize interrupts");
                return -EIO;
            }
        }
#endif // CONFIG_KX132_TRIGGER

#ifdef CONFIG_KX132_TRIGGER_OWN_THREAD
#warning "KX132 driver compiled with dedicated thread support"
#endif

    rc = pm_device_driver_init(dev, kx132_pm_action);
    return rc;
}

// Zephyr's notions of 'fetch', 'get', sensor channels and related terms
// detailed here:
// https://docs.zephyrproject.org/latest/reference/peripherals/sensor.html

static const struct sensor_driver_api kx132_driver_api =
{
    .attr_get = &kx132_1211_attr_get,
    .attr_set = &kx132_1211_attr_set,
#if CONFIG_KX132_TRIGGER
    .trigger_set = kx132_trigger_set,
#endif
    .sample_fetch = &kx132_1211_sample_fetch,
    .channel_get = &kx132_1211_channel_get
};

//----------------------------------------------------------------------
// - SECTION - driver macros
//----------------------------------------------------------------------

// Part 2:

/*
 * Device creation macro, shared by KX132_DEFINE_SPI() and
 * KX132_DEFINE_I2C().
 */

#define KX132_DEVICE_INIT(inst)                                        \
        PM_DEVICE_DT_INST_DEFINE(inst, kx132_pm_action);               \
        SENSOR_DEVICE_DT_INST_DEFINE(inst,                              \
                            kx132_init,                                \
                            PM_DEVICE_DT_INST_GET(inst),                \
                            &kx132_data_##inst,                        \
                            &kx132_config_##inst,                      \
                            POST_KERNEL,                                \
                            CONFIG_SENSOR_INIT_PRIORITY,                \
                            &kx132_driver_api);

#ifdef CONFIG_KX132_TRIGGER
#define GPIO_DT_SPEC_INST_GET_BY_IDX_COND(id, prop, idx)                \
        COND_CODE_1(DT_INST_PROP_HAS_IDX(id, prop, idx),                \
                    (GPIO_DT_SPEC_INST_GET_BY_IDX(id, prop, idx)),      \
                    ({.port = NULL, .pin = 0, .dt_flags = 0}))

#define KX132_CFG_INT(inst)                            \
        .gpio_drdy =                                                    \
            COND_CODE_1(ANYM_ON_INT1(inst),             \
                ({.port = NULL, .pin = 0, .dt_flags = 0}),                  \
                (GPIO_DT_SPEC_INST_GET_BY_IDX_COND(inst, irq_gpios, 0))),       \
        .gpio_int =                                                             \
            COND_CODE_1(ANYM_ON_INT1(inst),             \
                (GPIO_DT_SPEC_INST_GET_BY_IDX_COND(inst, irq_gpios, 0)),        \
                (GPIO_DT_SPEC_INST_GET_BY_IDX_COND(inst, irq_gpios, 1))),       \
        .int1_mode = DT_INST_PROP(inst, int1_gpio_config),                      \
        .int2_mode = DT_INST_PROP(inst, int2_gpio_config),
#else
#define KX132_CFG_INT(inst)
#endif /* CONFIG_KX132_TRIGGER */

#define KX132_CONFIG_SPI(inst)                                         \
        {                                                              \
                .bus_init = kx132_spi_init,                            \
                .bus_cfg = { .spi = SPI_DT_SPEC_INST_GET(inst,         \
                                        SPI_WORD_SET(8) |              \
                                        SPI_OP_MODE_MASTER |           \
                                        SPI_MODE_CPOL |                \
                                        SPI_MODE_CPHA,                 \
                                        0) },                          \
                KX132_CFG_INT(inst)                                    \
        }

// Part 1:

#define KX132_DEFINE_SPI(inst)                                         \
        static struct kx132_device_data kx132_data_##inst;             \
        static const struct kx132_device_config kx132_config_##inst =  \
                KX132_CONFIG_SPI(inst);                                \
        KX132_DEVICE_INIT(inst)
    
/*  
 * Instantiation macros used when a device is on an I2C bus.
 */ 
        
#define KX132_CONFIG_I2C(inst)                                         \
        {                                                              \
                .bus_init = kx132_i2c_init,                            \
                .bus_cfg = { .i2c = I2C_DT_SPEC_INST_GET(inst), },     \
                KX132_CFG_INT(inst)                                    \
        }

#define KX132_DEFINE_I2C(inst)                                         \
        static struct kx132_device_data kx132_data_##inst;             \
        static const struct kx132_device_config kx132_config_##inst =  \
                KX132_CONFIG_I2C(inst);                                \
        KX132_DEVICE_INIT(inst)

/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define KX132_DEFINE(inst)                                             \
        COND_CODE_1(DT_INST_ON_BUS(inst, spi),                         \
                    (KX132_DEFINE_SPI(inst)),                          \
                    (KX132_DEFINE_I2C(inst)))

DT_INST_FOREACH_STATUS_OKAY(KX132_DEFINE)
