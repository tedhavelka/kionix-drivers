/**
 * @project Kionix Sensor Drivers
 * @file kx132-registers.h
 * @author Ted Havelka
 * @license Apache 2.0 licensed.
 */

#ifndef KX132_REGISTERS_H
#define KX132_REGISTERS_H

//----------------------------------------------------------------------
// - SECTION - includes
//----------------------------------------------------------------------

#include "kx132-bus-iface.h"

#define KX132_WHO_AM_I_EXPECTED_VALUE (0x3D)
#define KX134_WHO_AM_I_EXPECTED_VALUE (0x46)
#define KX132_COTR_EXPECTED_VALUE (0x55)

//----------------------------------------------------------------------
// - SECTION - enums and non-register-address defines
//----------------------------------------------------------------------

// REF Kionix document KX132-1211-Technical-Reference-Manual-Rev-3.0.pdf
// ODCNTL (0x21) control register, pages 25..26 of 75. 

enum kx132_1211_output_data_rates
{
    KX132_ODR_0P781_HZ,
    KX132_ODR_1P563_HZ,
    KX132_ODR_3P125_HZ,
    KX132_ODR_6P25_HZ,

    KX132_ODR_12P5_HZ,  // 4
    KX132_ODR_25_HZ,
    KX132_ODR_50_HZ,
    KX132_ODR_100_HZ,

    KX132_ODR_200_HZ,   // 8
    KX132_ODR_400_HZ,
    KX132_ODR_800_HZ,
    KX132_ODR_1600_HZ,

    KX132_ODR_3200_HZ,  // 12
    KX132_ODR_6400_HZ,
    KX132_ODR_12800_HZ,
    KX132_ODR_25600_HZ,
};

enum kx132_acceleration_ranges
{
    KX132_ACCEL_RANGES_BEGIN,
    KX132_RANGE_PLUS_MINUS_2G,
    KX132_RANGE_PLUS_MINUS_4G,
    KX132_RANGE_PLUS_MINUS_8G,
    KX132_RANGE_PLUS_MINUS_16G,
    KX132_ACCEL_RANGES_END
};

enum kx132_acceleration_resolutions
{
    KX132_ACCEL_RESOLUTION_HIGH = 1,
    KX132_ACCEL_RESOLUTION_LOW
};



//----------------------------------------------------------------------
// - SECTION - defined register addresses
//----------------------------------------------------------------------

// For following defines see Kionix document
// KX132-1211-Technical-Reference-Manual-Rev-3.0.pdf

#define KX132_MAN_ID    (0x00)
#define KX132_MAN_ID_SIZE  (4)
#define KX132_PART_ID   (0x01)
#define KX132_PART_ID_SIZE (2)

#define KX132_XOUT_L    (0x08)
#define KX132_XOUT_H    (0x09)
#define KX132_YOUT_L    (0x0A)
#define KX132_YOUT_H    (0x0B)
#define KX132_ZOUT_L    (0x0C)
#define KX132_ZOUT_H    (0x0D)

//----------------------------------------------------------------------
// 
//----------------------------------------------------------------------
#define SIZE_KX132_REGISTER_VALUE (1)


#define KX132_COTR      (0x12)
#define KX132_WHO_AM_I  (0x13)

// Tap, Double-Tap axis specific interrupts:
#define KX132_INS1      (0x16)
//#define KX132_INS1_BIT7_RESERVED
//#define KX132_INS1_BIT6_RESERVED
//#define KX132_INS1_BIT_FLAG_TLE
//#define KX132_INS1_BIT_FLAG_TRI
//#define KX132_INS1_BIT_FLAG_TDO
//#define KX132_INS1_BIT_FLAG_TUP
//#define KX132_INS1_BIT_FLAG_TFD
//#define KX132_INS1_BIT_FLAG_TFU

// Status register to indicate source of or event triggering interrupt:
#define KX132_INS2      (0x17)
#define KX132_INS2_BIT_FLAG_FREE_FALL_STATUS        (1 << 7)
#define KX132_INS2_BIT_FLAG_BUFFER_FULL             (1 << 6)
#define KX132_INS2_BIT_FLAG_WATERMARK_EXCEEDED      (1 << 5)
#define KX132_INS2_BIT_FLAG_DATA_READY              (1 << 4)
#define KX132_INS2_BIT_FLAG_TAP_DOUBLE_TAP_STATUS_1 (1 << 3)
#define KX132_INS2_BIT_FLAG_TAP_DOUBLE_TAP_STATUS_0 (1 << 2)
#define KX132_INS2_BIT_FLAG_BIT1_RESERVED
#define KX132_INS2_BIT_FLAG_TILT_POSITION_STATUS    (1 << 0)

// Wake Up From Sleep, Back To Sleep, directions of motion causing interrupts:
#define KX132_INS3      (0x18)
//#define KX132_INS3_BIT_FLAG_WUFS
//#define KX132_INS3_BIT_FLAG_BTS
//#define KX132_INS3_BIT_FLAG_XNWU
//#define KX132_INS3_BIT_FLAG_XPWU
//#define KX132_INS3_BIT_FLAG_YNWU
//#define KX132_INS3_BIT_FLAG_YPWU
//#define KX132_INS3_BIT_FLAG_ZNWU
//#define KX132_INS3_BIT_FLAG_ZPWU

// KX132-1211-Technical-Reference-Manual-Rev-5.0.pdf p 14 of 75:
// interrtup latch release register INT_REL
#define KX132_INT_REL   (0x1A)


#define KX132_CNTL1     (0x1B)
#define KX132_CNTL1_BIT_FLAG_PC1                    (1 << 7)  // "Controls the operating mode" per tech' reference manual
#define KX132_CNTL1_BIT_FLAG_RES                    (1 << 6)  // acc reading resolution, 8-bit versus 16-bit
#define KX132_CNTL1_BIT_FLAG_DRDYE                  (1 << 5)  // Data Ready Engine enable
#define KX132_CNTL1_BIT_FLAG_GSEL1                  (1 << 4)  // G range selection bit most significant
#define KX132_CNTL1_BIT_FLAG_GSEL0                  (1 << 3)  // G range selection bit least significant
#define KX132_CNTL1_BIT_FLAG_TDTE                   (1 << 2)  // Tap, Double-Tap enable
#define KX132_CNTL1_BIT_FLAG_BIT1_RESERVED
#define KX132_CNTL1_BIT_FLAG_TPE                    (1 << 0)  // Tile Position Engine enable

#define KX132_CNTL2     (0x1C)
#define KX132_CNTL2_BIT_FLAG_BRES                   (1 << 6)

#define KX132_ODCNTL    (0x21)
#define KX132_BIT_OSA3                              (1 << 3)
#define KX132_BIT_OSA2                              (1 << 2)
#define KX132_BIT_OSA1                              (1 << 1)
#define KX132_BIT_OSA0                              (1 << 0)
#define KX132_OSA_BITS_MASK (KX132_BIT_OSA3 | KX132_BIT_OSA2 | KX132_BIT_OSA1 | KX132_BIT_OSA0)

#define KX132_INC1      (0x22)
#define KX132_INC2      (0x23)
#define KX132_INC3      (0x24)
#define KX132_INC4      (0x25)


// Register whose bits express sample count threshold, a.k.a. watermark
// at which to raise interrupt for application to read samples:
#define KX132_BUF_CNTL1 (0x5E)                                // BUF_CNTL1 is an "on the fly" register
// SMP_TH7..SMP_TH0, these are sample threshold bits which indicate
//  count of x,y,z samples at which KX132 raises its level-based
//  watermark interrupt (WMI).

#define KX132_BUF_CNTL2 (0x5F)                                // BUF_CNTL2 is an "on the fly" register
#define KX132_BUF_CNTL2_BIT_FLAG_BUFE               (1 << 7)  // enable sample buffer BUF_READ
#define KX132_BUF_CNTL2_BIT_FLAG_BRES               (1 << 6)  // determines resolution of samples collected by buffer
#define KX132_BUF_CNTL2_BIT_FLAG_BFIE               (1 << 5)  // buffer full interrupt enable
#define KX132_BUF_CNTL2_BIT_FLAG_BIT4_RESERVED      (1 << 4)
#define KX132_BUF_CNTL2_BIT_FLAG_BIT3_RESERVED      (1 << 3)
#define KX132_BUF_CNTL2_BIT_FLAG_BIT2_RESERVED      (1 << 2)
#define KX132_BUF_CNTL2_BIT_FLAG_BM1                (1 << 1)  // buffer operating mode bit
#define KX132_BUF_CNTL2_BIT_FLAG_BM0                (1 << 0)  // buffer operating mode bit
// buffer operating modes, KX132 technical reference manual page 43 of 75:
//         BM1  BM0
// FIFO     0    0
// STREAM   0    1
// TRIGGER  1    0


#define KX132_BUF_STATUS_1 (0x60)                             // read only register
// SMP_LEV7..SMP_LEV0

#define KX132_BUF_STATUS_2 (0x61)                             // read only register
// BUF_TRIG RESERVED RESERVED RESERVED RESERVED RESERVED SMP_LEV9 SMP_LEV8

// Per manual latched buffer status info and sample buffer data are
// cleared whenever are data written to this register:
#define KX132_BUF_CLEAR (0x62)                                // BUF_CLEAR is an "on the fly" register

#define KX132_BUF_READ  (0x63)


// See TN027-Power-On-Procedure.pdf for following register use:
#define KX132_UNNAMED_SW_RESET_REG_0x7F (0x7F)



//----------------------------------------------------------------------
// - SECTION - prototypes
//----------------------------------------------------------------------

// Important wrapper functions to read registers, write registers:

int32_t kx132_read_reg(kionix_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);
int32_t kx132_write_reg(kionix_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len);


// As of 2023 Q1 software reset routine treated as a fetching routine, as it reads two
// registers as part of final status check:
int kx132_software_reset(const struct device *dev);

int kx132_enable_watermark_interrupt(const struct device *dev);

int kx132_enable_asynchronous_readings(const struct device *dev);

int kx132_enable_synchronous_reading_with_hw_interrupt(const struct device *dev);

// - DEV 0116 -
int kx132_enter_standby_mode(const struct device *dev);

int kx132_disable_sample_buffer(const struct device *dev);
// - DEV 0116 -


//----------------------------------------------------------------------
// - SECTION - sensor fetch routines
//----------------------------------------------------------------------

int kx132_fetch_acceleration_x_axis(const struct device *dev);

int kx132_fetch_acceleration_y_axis(const struct device *dev);

int kx132_fetch_acceleration_z_axis(const struct device *dev);

int kx132_fetch_acceleration_xyz_axis(const struct device *dev);

int kx132_fetch_readings_from_buf_read(const struct device *dev);


/**
 * @note Sensor fetch routines, but for sensor configuration and
 *       identifying values as opposed to readings of external
 *       quantifiables.
 *
 * @note Zephyr's "fetch" and "get" model is not always helpful in
 *       supporting sensors which may involve high data acquisition
 *       rates, and therefore the following routines may by modified
 *       and moved into the "get attribute" type routines group.
 *----------------------------------------------------------------------
 */

int kx132_fetch_device_id(const struct device *dev);

int kx132_fetch_part_id(const struct device *dev);

int kx132_fetch_interrupt_latch_release(const struct device *dev);

int kx132_fetch_interrupt_source_2(const struct device *dev);

// - GROUP - "get attribute" type routines which immediately return sensor register values:

int kx132_get_attr__return_interrupt_statae_2(const struct device *dev, struct sensor_value *val);

int kx132_get_attr__output_data_rate(const struct device *dev, struct sensor_value *val);

int kx132_attr_sample_threshold_setting_get(const struct device *dev, struct sensor_value *val);

// - GROUP - routines which write values to sensor registers:

int kx132_update_reg__sample_threshold_buf_cntl1(const struct device *dev, const uint8_t new_sample_threshold);

int kx132_update_reg__buf_clear(const struct device *dev);

int kx132_update_reg__odcntl__output_data_rate(
                                               const struct device *dev,
                                               enum kx132_1211_output_data_rates new_odr
                                              );

int kx132_update_shadow_reg__sample_threshold(const struct device *dev, const uint8_t new_sample_threshold);


#endif // KX132_REGISTERS_H
