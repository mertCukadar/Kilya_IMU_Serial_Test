/*
 * LSM6DSOX 6-axis IMU driver for ESP-IDF (i2c_master driver, IDF >= 5.2)
 *
 * All public functions are thread-safe (a recursive mutex guards each device).
 * Units: acceleration in g, angular rate in dps, temperature in degC.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "lsm6dsox_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct lsm6dsox_dev_t *lsm6dsox_handle_t;

/* ---------------- Configuration enums ---------------- */

typedef enum {
    LSM6DSOX_ODR_OFF    = 0,
    LSM6DSOX_ODR_12HZ5  = 1,
    LSM6DSOX_ODR_26HZ   = 2,
    LSM6DSOX_ODR_52HZ   = 3,
    LSM6DSOX_ODR_104HZ  = 4,
    LSM6DSOX_ODR_208HZ  = 5,
    LSM6DSOX_ODR_417HZ  = 6,
    LSM6DSOX_ODR_833HZ  = 7,
    LSM6DSOX_ODR_1667HZ = 8,
    LSM6DSOX_ODR_3333HZ = 9,
    LSM6DSOX_ODR_6667HZ = 10,
    LSM6DSOX_ODR_1HZ6   = 11,   /* accel low-power mode only */
} lsm6dsox_odr_t;

typedef enum {
    LSM6DSOX_XL_FS_2G  = 0,
    LSM6DSOX_XL_FS_16G = 1,
    LSM6DSOX_XL_FS_4G  = 2,
    LSM6DSOX_XL_FS_8G  = 3,
} lsm6dsox_xl_fs_t;

typedef enum {
    LSM6DSOX_GY_FS_250DPS  = 0,
    LSM6DSOX_GY_FS_500DPS  = 1,
    LSM6DSOX_GY_FS_1000DPS = 2,
    LSM6DSOX_GY_FS_2000DPS = 3,
    LSM6DSOX_GY_FS_125DPS  = 4,
} lsm6dsox_gy_fs_t;

typedef enum {
    LSM6DSOX_FIFO_BYPASS            = 0,
    LSM6DSOX_FIFO_MODE              = 1,   /* stop when full */
    LSM6DSOX_FIFO_CONT_TO_FIFO      = 3,
    LSM6DSOX_FIFO_BYPASS_TO_CONT    = 4,
    LSM6DSOX_FIFO_CONTINUOUS        = 6,   /* overwrite oldest */
    LSM6DSOX_FIFO_BYPASS_TO_FIFO    = 7,
} lsm6dsox_fifo_mode_t;

typedef enum {
    LSM6DSOX_INT1 = 1,
    LSM6DSOX_INT2 = 2,
} lsm6dsox_int_pin_t;

/* Interrupt sources that can be routed to INT1/INT2 (bit mask) */
typedef enum {
    LSM6DSOX_EVT_DRDY_XL      = (1u << 0),
    LSM6DSOX_EVT_DRDY_G       = (1u << 1),
    LSM6DSOX_EVT_DRDY_TEMP    = (1u << 2),   /* INT2 only */
    LSM6DSOX_EVT_BOOT         = (1u << 3),   /* INT1 only */
    LSM6DSOX_EVT_FIFO_TH      = (1u << 4),
    LSM6DSOX_EVT_FIFO_OVR     = (1u << 5),
    LSM6DSOX_EVT_FIFO_FULL    = (1u << 6),
    LSM6DSOX_EVT_CNT_BDR      = (1u << 7),
    LSM6DSOX_EVT_TIMESTAMP    = (1u << 8),   /* INT2 only */
    LSM6DSOX_EVT_SLEEP_CHANGE = (1u << 9),
    LSM6DSOX_EVT_SINGLE_TAP   = (1u << 10),
    LSM6DSOX_EVT_DOUBLE_TAP   = (1u << 11),
    LSM6DSOX_EVT_WAKE_UP      = (1u << 12),
    LSM6DSOX_EVT_FREE_FALL    = (1u << 13),
    LSM6DSOX_EVT_6D           = (1u << 14),
    LSM6DSOX_EVT_STEP_DET     = (1u << 15),  /* embedded */
    LSM6DSOX_EVT_TILT         = (1u << 16),  /* embedded */
    LSM6DSOX_EVT_SIG_MOTION   = (1u << 17),  /* embedded */
} lsm6dsox_event_t;

/* ---------------- Data types ---------------- */

typedef struct { int16_t x, y, z; } lsm6dsox_raw3_t;
typedef struct { float x, y, z; } lsm6dsox_vec3_t;

typedef struct {
    lsm6dsox_vec3_t accel;   /* g   */
    lsm6dsox_vec3_t gyro;    /* dps */
    float temp_c;
} lsm6dsox_sample_t;

/* Snapshot of all event source registers */
typedef struct {
    uint8_t all_int_src;     /* 0x1A */
    uint8_t wake_up_src;     /* 0x1B */
    uint8_t tap_src;         /* 0x1C */
    uint8_t d6d_src;         /* 0x1D */
    uint8_t emb_func_status; /* embedded page 0x12 (reading clears latched emb ints) */
    /* decoded */
    bool single_tap, double_tap, wake_up, free_fall, d6d, sleep_change, sleep_state;
    bool step, tilt, sig_motion;
} lsm6dsox_events_t;

typedef struct {
    uint8_t tag;             /* LSM6DSOX_TAG_xxx */
    uint8_t cnt;             /* TAG_CNT (2 bits) */
    uint8_t raw[6];
    union {
        lsm6dsox_vec3_t accel;   /* tag ACCEL (g)      */
        lsm6dsox_vec3_t gyro;    /* tag GYRO  (dps)    */
        float temp_c;            /* tag TEMP           */
        uint32_t timestamp;      /* tag TIMESTAMP (25us ticks) */
        uint16_t steps;          /* tag STEP_COUNTER   */
    };
} lsm6dsox_fifo_word_t;

typedef struct {
    uint16_t level;          /* unread words */
    bool wtm, ovr, full, cnt_bdr, ovr_latched;
} lsm6dsox_fifo_status_t;

typedef struct {
    lsm6dsox_fifo_mode_t mode;
    lsm6dsox_odr_t bdr_xl;   /* batch rate (ODR_OFF = not batched)   */
    lsm6dsox_odr_t bdr_gy;
    uint8_t temp_bdr;        /* 0=off 1=1.6Hz 2=12.5Hz 3=52Hz        */
    uint8_t ts_decimation;   /* 0=off 1=every word 2=/8 3=/32        */
    uint16_t watermark;      /* 0..511 words                          */
    bool stop_on_wtm;
} lsm6dsox_fifo_cfg_t;

typedef struct {
    bool x_en, y_en, z_en;
    uint8_t threshold;       /* 0..31, LSB = FS/32                    */
    uint8_t shock;           /* 0..3                                  */
    uint8_t quiet;           /* 0..3                                  */
    uint8_t duration;        /* 0..15 (double tap gap)                */
    bool double_tap;         /* enable single+double tap recognition  */
} lsm6dsox_tap_cfg_t;

typedef struct {
    bool accel_pass, gyro_pass;
    lsm6dsox_vec3_t accel_delta_mg;   /* |ST_on - ST_off|, limits 50..1700 mg  */
    lsm6dsox_vec3_t gyro_delta_dps;   /* limits 150..700 dps                   */
} lsm6dsox_selftest_t;

typedef struct {
    i2c_master_bus_handle_t bus;   /* already-created I2C master bus */
    uint16_t i2c_addr;             /* 0 = auto-detect 0x6A / 0x6B    */
    uint32_t scl_speed_hz;         /* 0 = 400 kHz                     */
} lsm6dsox_config_t;

typedef void (*lsm6dsox_isr_cb_t)(lsm6dsox_int_pin_t pin, void *arg);   /* ISR context! */

/* ---------------- Lifecycle ---------------- */
esp_err_t lsm6dsox_create(const lsm6dsox_config_t *cfg, lsm6dsox_handle_t *out);
esp_err_t lsm6dsox_delete(lsm6dsox_handle_t h);
uint16_t  lsm6dsox_get_addr(lsm6dsox_handle_t h);
esp_err_t lsm6dsox_get_id(lsm6dsox_handle_t h, uint8_t *id);
esp_err_t lsm6dsox_reset(lsm6dsox_handle_t h);  /* SW reset + reboot + BDU/IF_INC/I3C-off */

/* ---------------- Low level ---------------- */
esp_err_t lsm6dsox_read_regs(lsm6dsox_handle_t h, uint8_t reg, uint8_t *data, size_t len);
esp_err_t lsm6dsox_write_reg(lsm6dsox_handle_t h, uint8_t reg, uint8_t val);
esp_err_t lsm6dsox_read_reg(lsm6dsox_handle_t h, uint8_t reg, uint8_t *val);
esp_err_t lsm6dsox_update_reg(lsm6dsox_handle_t h, uint8_t reg, uint8_t mask, uint8_t val);
esp_err_t lsm6dsox_emb_read(lsm6dsox_handle_t h, uint8_t reg, uint8_t *data, size_t len);
esp_err_t lsm6dsox_emb_update(lsm6dsox_handle_t h, uint8_t reg, uint8_t mask, uint8_t val);

/* ---------------- Sensor configuration ---------------- */
esp_err_t lsm6dsox_set_accel(lsm6dsox_handle_t h, lsm6dsox_odr_t odr, lsm6dsox_xl_fs_t fs);
esp_err_t lsm6dsox_set_gyro(lsm6dsox_handle_t h, lsm6dsox_odr_t odr, lsm6dsox_gy_fs_t fs);
esp_err_t lsm6dsox_set_high_perf(lsm6dsox_handle_t h, bool xl_hp, bool gy_hp);
esp_err_t lsm6dsox_set_accel_lpf2(lsm6dsox_handle_t h, bool enable, uint8_t hpcf); /* hpcf 0..7 */
esp_err_t lsm6dsox_set_gyro_lpf1(lsm6dsox_handle_t h, bool enable, uint8_t ftype); /* ftype 0..7 */
esp_err_t lsm6dsox_set_accel_offset(lsm6dsox_handle_t h, int8_t x, int8_t y, int8_t z, bool weight_15mg);
float     lsm6dsox_accel_sens_g(lsm6dsox_handle_t h);    /* g/LSB   */
float     lsm6dsox_gyro_sens_dps(lsm6dsox_handle_t h);   /* dps/LSB */

/* ---------------- Data ---------------- */
esp_err_t lsm6dsox_get_status(lsm6dsox_handle_t h, uint8_t *status);
esp_err_t lsm6dsox_read_accel_raw(lsm6dsox_handle_t h, lsm6dsox_raw3_t *raw);
esp_err_t lsm6dsox_read_gyro_raw(lsm6dsox_handle_t h, lsm6dsox_raw3_t *raw);
esp_err_t lsm6dsox_read_accel(lsm6dsox_handle_t h, lsm6dsox_vec3_t *g);
esp_err_t lsm6dsox_read_gyro(lsm6dsox_handle_t h, lsm6dsox_vec3_t *dps);
esp_err_t lsm6dsox_read_temp(lsm6dsox_handle_t h, float *degc);
esp_err_t lsm6dsox_read_all(lsm6dsox_handle_t h, lsm6dsox_sample_t *s);   /* one 14-byte burst */

/* ---------------- Timestamp ---------------- */
esp_err_t lsm6dsox_timestamp_enable(lsm6dsox_handle_t h, bool en);
esp_err_t lsm6dsox_read_timestamp(lsm6dsox_handle_t h, uint32_t *ticks);   /* 25 us / tick */

/* ---------------- Interrupts ---------------- */
esp_err_t lsm6dsox_int_pin_mode(lsm6dsox_handle_t h, bool active_low, bool open_drain);
esp_err_t lsm6dsox_set_latched(lsm6dsox_handle_t h, bool latched);   /* basic + embedded */
esp_err_t lsm6dsox_set_drdy_pulsed(lsm6dsox_handle_t h, bool pulsed);
esp_err_t lsm6dsox_route_int(lsm6dsox_handle_t h, lsm6dsox_int_pin_t pin, uint32_t evt_mask);
esp_err_t lsm6dsox_attach_isr(lsm6dsox_handle_t h, lsm6dsox_int_pin_t pin, gpio_num_t gpio,
                              lsm6dsox_isr_cb_t cb, void *arg);
esp_err_t lsm6dsox_detach_isr(lsm6dsox_handle_t h, lsm6dsox_int_pin_t pin);
esp_err_t lsm6dsox_read_events(lsm6dsox_handle_t h, lsm6dsox_events_t *ev);

/* ---------------- Motion functions ---------------- */
esp_err_t lsm6dsox_tap_config(lsm6dsox_handle_t h, const lsm6dsox_tap_cfg_t *cfg);
esp_err_t lsm6dsox_wake_up_config(lsm6dsox_handle_t h, uint8_t ths /*0..63*/, uint8_t dur /*0..3*/);
esp_err_t lsm6dsox_free_fall_config(lsm6dsox_handle_t h, uint8_t ths /*0..7*/, uint8_t dur /*0..63*/);
esp_err_t lsm6dsox_6d_config(lsm6dsox_handle_t h, uint8_t ths /*0=80 1=70 2=60 3=50 deg*/, bool d4d);
esp_err_t lsm6dsox_activity_config(lsm6dsox_handle_t h, uint8_t inact_mode /*0..3*/, uint8_t sleep_dur /*0..15*/);

/* ---------------- Embedded functions ---------------- */
esp_err_t lsm6dsox_pedometer_enable(lsm6dsox_handle_t h, bool en);
esp_err_t lsm6dsox_step_count(lsm6dsox_handle_t h, uint16_t *steps);
esp_err_t lsm6dsox_step_reset(lsm6dsox_handle_t h);
esp_err_t lsm6dsox_tilt_enable(lsm6dsox_handle_t h, bool en);
esp_err_t lsm6dsox_sig_motion_enable(lsm6dsox_handle_t h, bool en);

/* ---------------- FIFO ---------------- */
esp_err_t lsm6dsox_fifo_config(lsm6dsox_handle_t h, const lsm6dsox_fifo_cfg_t *cfg);
esp_err_t lsm6dsox_fifo_status(lsm6dsox_handle_t h, lsm6dsox_fifo_status_t *st);
esp_err_t lsm6dsox_fifo_read(lsm6dsox_handle_t h, lsm6dsox_fifo_word_t *words, size_t max, size_t *n);

/* ---------------- Self test (restores CTRL1..CTRL10 afterwards) ---------------- */
esp_err_t lsm6dsox_self_test(lsm6dsox_handle_t h, lsm6dsox_selftest_t *res);

#ifdef __cplusplus
}
#endif
