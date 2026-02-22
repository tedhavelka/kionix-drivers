/**
 * Kionix KX132 Zephyr driver
 */

//----------------------------------------------------------------------
// - SECTION - notes
//----------------------------------------------------------------------

// 2022-11-21:  Encountered issue on 11-21, finding that with Kionix
//  accelerometer I2C "burst" read API call returns mostly garbled
//  data.  Tried i2c_write_read_dt() and this routine returns expected
//  data, so switched to this one.  Contributor Ted suspect that I2C
//  burst write API call in Zephyr 3.2.0 also fails with Kionix KX132
//  sensor.  The alternative API calls, however, have a different
//  enough implementation that calling them requires sending the sensor
//  internal register address as part of the data buffer to write,
//  in other words, all calls from this driver to the I2C write
//  routine must be adapted to include sensor internal register address
//  as first byte in the data buffer of data to write.
//
//  Contributor Ted working to find a Zephyr I2C API call which accepts
//  parameters and data buffers in the same way I2C write burst API
//  requires its parameters to be passed.
//
//  Note also per AN092-Getting_Started.pdf, page 3 of 27, KX132
//  "data ready" or drdy interrupt will be cleared automatically when
//  x,y,z acceleration readings registers are read out via an I2C
//  burst read operation.

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(KX132, CONFIG_SENSOR_LOG_LEVEL);

#include "kx132-1211.h"
#include "kx132-registers.h"

//----------------------------------------------------------------------
// - SECTION - register read and write wrapper functions
//----------------------------------------------------------------------

/**
  * @defgroup  KX132_Interfaces_Functions
  * @brief This section provide a set of functions used to read and write
  *   a generic register of the device.
  */

//**********************************************************************
//
// Notes on register read, write wrappers . . .
//
//                                      I2C ctrlr, regaddr, data buffer, length data to write
//                                           |       |          |                |
// 112 typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
// 113 typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);
//                                           |       |          |                |
//                                      I2C ctrlr, regaddr, data buffer, length data to read
//
// IIS2dh example call
//
//    ret = iis2dh_read_reg(ctx, IIS2DH_WHO_AM_I, buff, 1);
//
//**********************************************************************

/**
  * @brief  Read generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to read
  * @param  data  pointer to buffer that store the data read(ptr)
  * @param  len   number of consecutive register to read
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */

int32_t kx132_read_reg(kionix_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
  int32_t rc;

  rc = ctx->read_reg(ctx->handle, reg, data, len);

  return rc;
}

/**
  * @brief  Write generic device register
  *
  * @param  ctx   read / write interface definitions(ptr)
  * @param  reg   register to write
  * @param  data  pointer to data to write in register reg(ptr)
  * @param  len   number of consecutive register to write
  * @retval          interface status (MANDATORY: return 0 -> no Error)
  *
  */

int32_t kx132_write_reg(kionix_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len)
{
  int32_t rc;

  rc = ctx->write_reg(ctx->handle, reg, data, len);

  return rc;
}

//----------------------------------------------------------------------
// - SECTION - KX132 multi-register config routines
//----------------------------------------------------------------------

// Per software reset description in Kionix TN027-Power-On-Procedure.pdf:

int kx132_software_reset(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
// TODO [ ] review size of read buffer here in KX132 software reset routine:
    uint8_t reg_val_to_read[] = {0, 0, 0, 0};
    uint8_t *read_buffer = reg_val_to_read;

    uint32_t len = 2;
    int rc = 0;

    k_msleep(PERIOD_TO_POWER_UP_IN_MS);

    reg_val_to_write = 0x00U;
    rc = kx132_write_reg(data->ctx, KX132_UNNAMED_SW_RESET_REG_0x7F, write_buffer, len);

    reg_val_to_write = 0x00U;
    rc += kx132_write_reg(data->ctx, KX132_CNTL2, write_buffer, len);

    reg_val_to_write = 0x80U;
    rc += kx132_write_reg(data->ctx, KX132_CNTL2, write_buffer, len);

    k_msleep(PERIOD_TO_PERFORM_SW_RESET_IN_MS);

    rc += kx132_read_reg(data->ctx, KX132_WHO_AM_I, read_buffer, SIZE_KX132_REGISTER_VALUE);
    data->shadow_reg_who_am_i = read_buffer[0];

    rc += kx132_read_reg(data->ctx, KX132_COTR, read_buffer, SIZE_KX132_REGISTER_VALUE);
    data->shadow_reg_cotr = read_buffer[0];

// 2023-01-18
// NEED to amend return values to capture bus transaction errors,
// and yet support unexpected 'who am i' and 'cotr' values - TMH

    if ( data->shadow_reg_who_am_i != KX132_WHO_AM_I_EXPECTED_VALUE ) {
        rc = -ENOMSG;
    }

    if ( data->shadow_reg_cotr != KX132_COTR_EXPECTED_VALUE ) {
        rc = -ENOMSG;
    }

    return rc;
}

int kx132_enable_asynchronous_readings(const struct device *dev)
{
// Register sequence this routine chosen per AN092-Getting-Started.pdf
// from Kionix, page 2 of 27:

    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 2;
    int rc = 0;

    reg_val_to_write = 0x00U;
    rc  = kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

    reg_val_to_write = 0x06U;
    rc |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, len);

    reg_val_to_write = 0xC0U;
    rc |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

    return rc;
}

int kx132_enable_synchronous_reading_with_hw_interrupt(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 2;
    int rc = 0;

// From AN092-Getting-Started.pdf:
//
//   CNTL1  0x00   . . . put sensor into stand-by mode
//   INC1   0x30   . . . reg Interrupt Control 1, enable INT1, set active high, and for latched operation
//   INC4   0x10   . . . reg Interrupt Control 4, set interrupt event to "data ready"
// ( ODCNTL 0x06   . . . reg to control output data rate . . . Optional!  Default rate is 50 Hz )
//   CNTL1  0xE0   . . . put sensor into active readings mode

    reg_val_to_write = 0x00U;
    rc |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

    reg_val_to_write = 0x30U;
    rc |= kx132_write_reg(data->ctx, KX132_INC1, write_buffer, len);

    reg_val_to_write = 0x10U;
    rc |= kx132_write_reg(data->ctx, KX132_INC4, write_buffer, len);

// NEED to review setting of ODCNTL register in this routine:   - TMH
    reg_val_to_write = 0x06U;
    rc |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, len);

    reg_val_to_write = 0xE0U;
    rc |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

    return rc;
}

int kx132_enable_watermark_interrupt(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 2;
    int rc = 0;

// From AN092-Getting-Started.pdf, section 3.4.2 "Watermark Interrupt (WMI)":
//
//   CNTL1     0x00   . . . put sensor into stand-by mode
// ( ODCNTL    0x06   . . . Optional!  Set Output Data Rate via writes to this register )
//   INC1      0x30   . . . reg Interrupt Control 1, enable INT1, set active high, and for latched operation
//   INC4      0x20   . . . reg Interrupt Control 4, set interrupt event to "watermark / sample threshold reached"
//   BUF_CNTL1 0x2B   . . . buffer control reg 1 to set sample threshold or watermark interrupt "level"
//   BUF_CNTL2 0xE0   . . . buffer control reg 2,
//   CNTL1     0xE0   . . . put sensor into active readings mode

    reg_val_to_write = 0x00U;
    rc |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

#ifdef KX132_DRIVER__SET_ODR_OF_50_HZ_IN_ENABLE_WATERMARK_SEQUENCE
// For reg ODCNTL and OSA3..OSA1 bits, see page 26 of 75 of KX132-1211-Technical-Reference-Manual-Rev-5.0.pdf:
    reg_val_to_write = KX132_ODR_50_HZ;
    rc |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, len);
#endif

    reg_val_to_write = 0x30U;
    rc |= kx132_write_reg(data->ctx, KX132_INC1, write_buffer, len);

    reg_val_to_write = 0x20U;
    rc |= kx132_write_reg(data->ctx, KX132_INC4, write_buffer, len);

// NEED TO REVIEW whether there is a better way to manage / support watermark
// threshold setting, which following stanzas configure for a sample
// threshold of ten.  That may not be what end application requires . . .
//    reg_val_to_write = 0x0AU;
    reg_val_to_write = data->shadow_reg_buf_cntl1;
    rc |= kx132_write_reg(data->ctx, KX132_BUF_CNTL1, write_buffer, len);

    reg_val_to_write = 0xE0U;
    rc |= kx132_write_reg(data->ctx, KX132_BUF_CNTL2, write_buffer, len);

    reg_val_to_write = 0xE0U;
    rc |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

    return rc;
}

int kx132_enter_standby_mode(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;

    uint8_t reg_val_to_read[] = {0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    uint32_t len = 2;
    int rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_CNTL1, read_buffer, SIZE_KX132_REGISTER_VALUE);
    data->shadow_reg_cntl1 = read_buffer[0];
    data->shadow_reg_cntl1 &= ~(KX132_CNTL1_BIT_FLAG_PC1);

    rc |= kx132_write_reg(data->ctx, KX132_CNTL1, &(data->shadow_reg_cntl1), len);

    return rc;
}

int kx132_disable_sample_buffer(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;

    uint8_t reg_val_to_read[] = {0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    uint32_t len = 2;
    int rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_BUF_CNTL2, read_buffer, SIZE_KX132_REGISTER_VALUE);
    data->shadow_reg_buf_cntl2 = read_buffer[0];
    data->shadow_reg_buf_cntl2 &= (uint8_t)~(KX132_BUF_CNTL2_BIT_FLAG_BUFE);
    rc |= kx132_write_reg(data->ctx, KX132_BUF_CNTL2, &(data->shadow_reg_buf_cntl2), len);

    return rc;
}

//----------------------------------------------------------------------
// - SECTION - update register routines
//----------------------------------------------------------------------

/**
 * @brief Routine to update KX132 Output Data Rate control register.
 *
 * @note Kionix KX132 register ODCNTL can only be updated when sensor
 *       is in standby mode, hence the changes to control register
 *       CNTL1 in this routine.
 */

int kx132_update_reg__odcntl__output_data_rate(const struct device *dev,
                                          const enum kx132_1211_output_data_rates new_odr)
// NEED to review KX132 datasheet(s) to see whether there are multiple
// output data rates independtly settable by end users, in registers
// beyond KX132_ODCNTL - TMH

// NEED TO LOGICALLY 'OR' THE PASSED OUTPUT DATA RATE (ODR) BITS!!!

// NOTE - NEED to refactor or replace ODR routine near top of kx132-1211.c - TMH
{
    struct kx132_device_data *data = dev->data;

    uint8_t reg_val_to_write = 0x00U;
    uint8_t *write_buffer = &reg_val_to_write;
    uint8_t reg_val_to_read[] = {0, 0, 0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    uint32_t len = 2;

    uint8_t as_found_reg_cntl1 = 0;
    uint8_t as_found_reg_odcntl = 0;

    uint32_t rc = 0;

#ifdef DEV__ODCNTL_UPDATE_DIAG
    LOG_INF("- kx132-registers.c - ODCNTL update begin:");
    LOG_INF("- kx132-registers.c - caller requests ODR setting of %u,", new_odr);
#endif
// grab the present CNTL1 value:
    rc = kx132_read_reg(data->ctx, KX132_CNTL1, read_buffer, len);
    as_found_reg_cntl1 = read_buffer[0];
#ifdef DEV__ODCNTL_UPDATE_DIAG
    LOG_DBG("- kx132-registers.c - register CNTL1 as found value is %u,", as_found_reg_cntl1);
#endif

// mask out the PC1 (power control?) bit:
    reg_val_to_write = (as_found_reg_cntl1 & ~KX132_CNTL1_BIT_FLAG_PC1);
    rc |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);

// grab the present ODCNTL value:
    rc = kx132_read_reg(data->ctx, KX132_ODCNTL, read_buffer, len);
    as_found_reg_odcntl = read_buffer[0];
#ifdef DEV__ODCNTL_UPDATE_DIAG
    LOG_DBG("- kx132-registers.c - register ODCNTL as found value is %u,", as_found_reg_odcntl);
#endif

// mask, update and write new output data rate to ODCNTL reg:
    reg_val_to_write = (as_found_reg_odcntl & ~KX132_OSA_BITS_MASK);
    reg_val_to_write |= new_odr;
    rc |= kx132_write_reg(data->ctx, KX132_ODCNTL, write_buffer, len);
#ifdef DEV__ODCNTL_UPDATE_DIAG
    LOG_DBG("- kx132-registers.c - updated reg ODCNTL to %u,", reg_val_to_write);
#endif

// restore the CNTL1 register value to as found:
    reg_val_to_write = as_found_reg_cntl1;
    rc |= kx132_write_reg(data->ctx, KX132_CNTL1, write_buffer, len);
#ifdef DEV__ODCNTL_UPDATE_DIAG
    LOG_DBG("- kx132-registers.c - ODCNTL update end.\n");
#endif

    if ( rc == 0 ) { }
    return rc;
}

/**
 * @brief Routine to update KX132 sample threshold value, which sets
 *        count of x,y,z samples needed to trigger a "watermark reached"
 *        interrupt, also called WMI in Kionix technical references.
 */

int kx132_update_reg__sample_threshold_buf_cntl1(const struct device *dev, const uint8_t new_sample_threshold)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = new_sample_threshold;
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 2;

    uint32_t rc = kx132_write_reg(data->ctx, KX132_BUF_CNTL1, write_buffer, len);

//    if ( rc == 0 ) { }
    return rc;
}

/**
 * @brief Routine to write a value to KX132 "clear sample buffer"
 *        register.
 *
 * @note  Kionix technical manual gives no specific value
 *        required to write, only says that any write operation clears
 *        the sample buffer, and also the level based WMI (watermark)
 *        and BFI (buffer full) interrupts.  Those interrupts cannot
 *        be cleared by other means, such as a read of register INT_REL.
 */

int kx132_update_reg__buf_clear(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_write = 0x01U;  // value of one chosen arbitrarily, tech' manual specs no particular write value necesary.
    uint8_t *write_buffer = &reg_val_to_write;
    uint32_t len = 2;

    uint32_t rc = kx132_write_reg(data->ctx, KX132_BUF_CLEAR, write_buffer, len);

    if ( rc == 0 ) { }
    return rc;
}

/**
 * @brief Routing to allow for app code to set a FIFO sample watermark
 *        or threshold level, which is referenced in the multi-register
 *        config sequence to enable watermark driven interrupt sampling.
 */

int kx132_update_shadow_reg__sample_threshold(const struct device *dev, const uint8_t new_sample_threshold)
{
    struct kx132_device_data *data = dev->data;
    data->shadow_reg_buf_cntl1 = new_sample_threshold;
    return 0;
}

//----------------------------------------------------------------------
// - SECTION - fetch and get register value routines
//----------------------------------------------------------------------

/**
 * @ note Zephyr RTOS sensor API presents notions of fetching, and
 *   getting sensor readings and register values.  In Zephyr context
 *   to fetch a value means a driver reads a value from a sensor and
 *   stores this value in some variable, such as a "shadow" or copy
 *   register variable, which is itself part of the driver.
 *
 *   In Zephyr context to get a sensor reading or register value means
 *   that the driver returns that value to application code, whether
 *   reading directly from a sensor register or returning the
 *   shadowed copy of the most recent reading.
 *
 *   KX132 driver routines in this section either fetch register
 *   values, or they fetch and return values to application code.
 */

/** 
 * @brief This routine fetches Kionix KX132-1211 Manufacturer ID string.
 */

int kx132_fetch_device_id(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0, 0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int i = 0;
    int rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_MAN_ID, read_buffer, KX132_MAN_ID_SIZE);

    if ( rc != 0 )
    {
        LOG_WRN("Unable to read manufacturer ID string. Err: %i", rc);
        return rc;
    }

    for ( i = 0; i < SIZE_MANUFACT_ID_STRING; i++ )
    {
        data->manufacturer_id.as_bytes[i] = read_buffer[i];
    }

// Diag 1 here - 2022-12-05

    return rc;
}

int kx132_fetch_part_id(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int i = 0;
    int rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_PART_ID, read_buffer, KX132_PART_ID_SIZE);

    if ( rc != 0 )
    {
        LOG_WRN("Unable to read numeric part ID . Err: %i", rc);
        return rc;
    }

    for ( i = 0; i < SIZE_PART_ID_STRING; i++ )
    {
        data->part_id.as_bytes[i] = read_buffer[i];
    }

    return rc;
}
 
int kx132_get_attr__return_interrupt_statae_2(const struct device *dev, struct sensor_value *val)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0};  uint8_t *read_buffer = reg_val_to_read;
    uint32_t rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_INS2, read_buffer, SIZE_KX132_REGISTER_VALUE);
    data->shadow_reg_ins2 = read_buffer[0];
    val->val1 = data->shadow_reg_ins2;
    val->val2 = 0;

    return rc;
}

int kx132_get_attr__output_data_rate(const struct device *dev, struct sensor_value *val)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0};  uint8_t *read_buffer = reg_val_to_read;
    uint32_t rc = 0;

//    rc = kx132_read_reg(data->ctx, KX132_ODCNTL, read_buffer, 2);  // NEED we read two bytes, e.g. SIZE_KX132_REGISTER_VALUE?
    rc = kx132_read_reg(data->ctx, KX132_ODCNTL, read_buffer, SIZE_KX132_REGISTER_VALUE);  // NEED we read two bytes, e.g. SIZE_KX132_REGISTER_VALUE?
    data->shadow_reg_odcntl = read_buffer[0];
    val->val1 = (data->shadow_reg_odcntl & KX132_OSA_BITS_MASK);
    val->val2 = 0;

    return rc;
}

// IN PROGRESS - routine to return BUF_CNTL1 sample threshold value:

int kx132_get_attr__buf_cntl1__sample_threshold_setting(const struct device *dev, struct sensor_value *value)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0};  uint8_t *read_buffer = reg_val_to_read;
    uint32_t rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_BUF_CNTL1, read_buffer, SIZE_KX132_REGISTER_VALUE);
    data->shadow_reg_buf_cntl1 = read_buffer[0];
    value->val1 = data->shadow_reg_buf_cntl1;
    value->val2 = 0;

    return rc;
}

//
// 2023-01-23 - IN PROGRESS routine to return BUF_READ six bytes for
//  latest sample, directly to calling code:
//----------------------------------------------------------------------

/**
 * @brief Routine to read high resolution (16-bit) x,y,z acc sample
 *        triplet and return this sample directly to calling code.
 *
 * @note  This direct value returning is in contrast to Zephyr's sensor
 *        API practice, where Zephyr drivers conventionally "fetch"
 *        a reading from a sensor and store the reading in driver side
 *        memory, and a second routine call to the driver is then
 *        required to "get" the fetched data and return it to
 *        application code.
 */

int kx132_get_attr__buf_read__sample_as_attribute(const struct device *dev, struct sensor_value *value)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0, 0, 0, 0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_BUF_READ, read_buffer, KX132_READINGS_TRIPLET_HI_RES_BYTE_COUNT);

    if ( rc != 0 )
    {
        LOG_WRN("Unable to read acceleration sample buffer BUF_READ.  Error: %i", rc);
        return rc;
    }

    value->val1 = (
                   ( read_buffer[0] <<  0 ) +  // XOUT_L
                   ( read_buffer[1] <<  8 ) +  // XOUT_H
                   ( read_buffer[2] << 16 ) +  // YOUT_L
                   ( read_buffer[3] << 24 )    // YOUT_H
                  );
    value->val2 = (
                   ( read_buffer[4] <<  0 ) +  // ZOUT_L
                   ( read_buffer[5] <<  8 )    // ZOUT_H
                  );
    return rc;
}

//----------------------------------------------------------------------
// - SECTION - fetching routines
//----------------------------------------------------------------------

// Per Zephyr sensor API convetion these routines read sensor data and
// store this data in driver variables, but these routines do not
// themsevles return sensor data to application code.

int kx132_fetch_acceleration_x_axis(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int i = 0;
    int rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_XOUT_L, read_buffer, 2);

    if ( rc != 0 )
    {
        LOG_WRN("Unable to read X axis acceleration.  Error: %i", rc);
        return rc;
    }

    for ( i = 0; i < KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT; i++ )
    {
        data->accel_axis_x[i] = read_buffer[i];
    }

    return rc;
}

int kx132_fetch_acceleration_y_axis(const struct device *dev)
{
    struct kx132_device_data *data = (struct kx132_device_data*)dev->data;
    uint8_t reg_val_to_read[] = {0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int i = 0;
    int rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_YOUT_L, read_buffer, 2);

    if ( rc != 0 )
    {
        LOG_WRN("Unable to read Y axis acceleration.  Error: %i", rc);
        return rc;
    }

    for ( i = 0; i < KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT; i++ )
    {
        data->accel_axis_y[i] = read_buffer[i];
    }

    return rc;
}

int kx132_fetch_acceleration_z_axis(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int i = 0;
    int rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_ZOUT_L, read_buffer, 2);

    if ( rc != 0 )
    {
        LOG_WRN("Unable to read Z axis acceleration.  Error: %i", rc);
        return rc;
    }

    for ( i = 0; i < KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT; i++ )
    {
        data->accel_axis_z[i] = read_buffer[i];
    }

    return rc;
}

int kx132_fetch_acceleration_xyz_axis(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0, 0, 0, 0, 0, 0};
    uint8_t *read_buffer = reg_val_to_read;
    int i = 0;
    int rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_XOUT_L, read_buffer, 6);

    if (rc != 0)
    {
        LOG_ERR("Failed to read X,Y,Z accelerometer data, err %d", rc);
        return rc;
    }

    for (i = 0; i < (KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT + 0); i++) {
        data->accel_axis_x[i - 0] = read_buffer[i];
    }
    for (i = 2; i < (KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT + 2); i++) {
        data->accel_axis_y[i - 2] = read_buffer[i];
    }
    for (i = 4; i < (KX132_ACC_READING_SINGLE_AXIS_BYTE_COUNT + 4); i++) {
        data->accel_axis_z[i - 4] = read_buffer[i];
    }

    LOG_INF("- DEV - X, Y and Z accelerations are %d, %d, %d   - DEV -\n",
      ( ( data->accel_axis_x[1] << 8 ) + data->accel_axis_x[0] ),
      ( ( data->accel_axis_y[1] << 8 ) + data->accel_axis_y[0] ),
      ( ( data->accel_axis_z[1] << 8 ) + data->accel_axis_z[0] )
    );
    LOG_INF("- DEV - (requested %d bytes of data from sensor reg addr %d) - DEV -\n",
      sizeof(reg_val_to_read), KX132_XOUT_L);

    return rc;
}

int kx132_fetch_interrupt_latch_release(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0};
    uint8_t *read_buffer = reg_val_to_read;
    int rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_INT_REL, read_buffer, SIZE_KX132_REGISTER_VALUE);

    if ( rc != 0 )
        { LOG_WRN("Unable to read interrupt latch release register.  Err: %i", rc); }
    else
        { data->shadow_reg_int_rel = read_buffer[0]; }

    return rc;
}

int kx132_fetch_interrupt_source_2(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[] = {0};
    uint8_t *read_buffer = reg_val_to_read;
    int rc = 0;

    rc = kx132_read_reg(data->ctx, KX132_INS2, read_buffer, SIZE_KX132_REGISTER_VALUE);

    if ( rc != 0 )
        { LOG_WRN("Unable to read INS2 register.  Err: %i", rc); }
    else
        { data->shadow_reg_ins2 = read_buffer[0]; }

    return rc;
}

//
// Following routine attempts to read from KX132 readings buffer 'watermark'
// watermark count of bytes, where watermark value is set in
//
// This routine also checks for selected readings resolution, which
// defaults to 16-bit width, and may also be 8-bit width in other
// modes related to low power.
//
//----------------------------------------------------------------------

/**
 * @brief routine to read one x,y,z acceleration reading set from
 *        KX132 sample buffer.
 *
 * @param const struct device *dev A valid Zephyr device pointer to a
 *        Kionix KX132 sensor.
 *
 * @return 0 if successful, negative errno code if failure.
 *
 * @note per Kionix KX132-1211-Technical-Reference-Manual-Rev-5.0.pdf,
 *       page 45 of 75, "To prevent any data loss, data must be read on
 *       a single byte basis or in complete data sets."
 *
 *       High resolution x,y,z data sets reside in six bytes of buffer
 *       space.  Low resolution acceleration data sets take up three
 *       bytes.  This routine reads either six bytes of sample buffer
 *       data or three bytes, based on present readings resolution
 *       setting in the sensor.
 */

int kx132_fetch_readings_from_buf_read(const struct device *dev)
{
    struct kx132_device_data *data = dev->data;
    uint8_t reg_val_to_read[KX132_READINGS_TRIPLET_HI_RES_BYTE_COUNT];
    uint8_t *read_buffer = reg_val_to_read;
    uint16_t needed_sample_byte_count = 0;
    int rc = 0;

// Clear read buffer, which will be used first to obtain sample resolution setting:
    memset(read_buffer, 0, KX132_READINGS_TRIPLET_HI_RES_BYTE_COUNT);
// Read BUF_CNTL2 to determine sensor readings bit width, "low res" or "high res":
    rc = kx132_read_reg(data->ctx, KX132_BUF_CNTL2, read_buffer, SIZE_KX132_REGISTER_VALUE);

#ifdef DEV_0118
// - DIAG BEGIN -
    LOG_INF("readings resolution bit flag set to %u,",
      ( read_buffer[0] & KX132_CNTL2_BIT_FLAG_BRES ? 1 : 0));
// - DIAG END -
#endif

    if ( ( read_buffer[0] & KX132_CNTL2_BIT_FLAG_BRES ) == KX132_CNTL2_BIT_FLAG_BRES )
        { needed_sample_byte_count = 6; }
    else
        { needed_sample_byte_count = 3; }

    memset(reg_val_to_read, 0, KX132_READINGS_TRIPLET_HI_RES_BYTE_COUNT);

    rc = kx132_read_reg(data->ctx, KX132_BUF_READ, read_buffer, needed_sample_byte_count);

    memcpy(data->shadow_reg_buf_read, read_buffer, KX132_READINGS_TRIPLET_HI_RES_BYTE_COUNT);

#ifdef DEV_0118
    LOG_INF("first six bytes from BUF_READ sample buffer:");
    LOG_INF("0x%04X, 0x%04X, 0x%04X,",
            (read_buffer[0] + (read_buffer[1] << 8 )),
            (read_buffer[2] + (read_buffer[3] << 8 )),
            (read_buffer[4] + (read_buffer[5] << 8 ))
      );
    LOG_INF("shadowed BUF_READ now holds:");
    LOG_INF("0x%04X, 0x%04X, 0x%04X,",
        (data->shadow_reg_buf_read[0] + (data->shadow_reg_buf_read[1] << 8 )),
        (data->shadow_reg_buf_read[2] + (data->shadow_reg_buf_read[3] << 8 )),
        (data->shadow_reg_buf_read[4] + (data->shadow_reg_buf_read[5] << 8 ))
      );
#endif

    return rc;
}
