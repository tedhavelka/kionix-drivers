/**
 * @project Kionix Sensor Drivers
 * @author Ted Havelka
 * @license Apache 2.0 licensed.
 */

#ifndef KX132_LOW_LEVEL_BUS_INTERFACE_H
#define KX132_LOW_LEVEL_BUS_INTERFACE_H



//
//----------------------------------------------------------------------
// Following code borrowed from Zephyr 3.2.0, STMicro IIS2DH driver file:
// REF modules/hal/st/sensor/stmemsc/iis2dh_STdC/driver/iis2dh_reg.h
//
// Note, following "context" data structure kionix_ctx_t, a copy of 
// stmdev_ctx_t, is key to providing flexibly factored driver code which
// can from application developer's standpoint switch seamlessly between
// I2C and SPI bus interfaces:
//----------------------------------------------------------------------
//

/** @addtogroup  Interfaces_Functions
  * @brief       This section provide a set of functions used to read and
  *              write a generic register of the device.
  *              MANDATORY: return 0 -> no Error.
  * @{
  *
  */

typedef int32_t (*kionix_write_ptr)(void *, uint8_t, const uint8_t *, uint16_t);
typedef int32_t (*kionix_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

typedef struct
{
  /** Component mandatory fields **/
  kionix_write_ptr  write_reg;
  kionix_read_ptr   read_reg;
  /** Customizable optional pointer **/
  void *handle;
} kionix_ctx_t;

/**
  * @}
  *
  */




#endif // KX132_LOW_LEVEL_BUS_INTERFACE_H
