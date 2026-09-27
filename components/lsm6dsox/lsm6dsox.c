/*
 * LSM6DSOX 6-axis IMU driver for ESP-IDF
 */
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "lsm6dsox.h"

static const char *TAG = "lsm6dsox";

#define I2C_TIMEOUT_MS  100
#define DELAY_MS(ms)    vTaskDelay(pdMS_TO_TICKS(ms) ? pdMS_TO_TICKS(ms) : 1)

typedef struct {
    lsm6dsox_handle_t dev;
    lsm6dsox_int_pin_t pin;
    gpio_num_t gpio;
    lsm6dsox_isr_cb_t cb;
    void *arg;
} isr_slot_t;

struct lsm6dsox_dev_t {
    i2c_master_dev_handle_t i2c;
    uint16_t addr;
    SemaphoreHandle_t lock;
    lsm6dsox_xl_fs_t xl_fs;
    lsm6dsox_gy_fs_t gy_fs;
    bool int_active_low;
    isr_slot_t isr[2];
};

#define LOCK(h)   xSemaphoreTakeRecursive((h)->lock, portMAX_DELAY)
#define UNLOCK(h) xSemaphoreGiveRecursive((h)->lock)

static const float XL_SENS_G[4]   = { 0.061e-3f, 0.488e-3f, 0.122e-3f, 0.244e-3f };
static const float GY_SENS_DPS[5] = { 8.75e-3f, 17.5e-3f, 35.0e-3f, 70.0e-3f, 4.375e-3f };

/* ------------------------------------------------------------------ */
/* Low level                                                           */
/* ------------------------------------------------------------------ */

esp_err_t lsm6dsox_read_regs(lsm6dsox_handle_t h, uint8_t reg, uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(h && data && len, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    LOCK(h);
    esp_err_t ret = i2c_master_transmit_receive(h->i2c, &reg, 1, data, len, I2C_TIMEOUT_MS);
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_read_reg(lsm6dsox_handle_t h, uint8_t reg, uint8_t *val)
{
    return lsm6dsox_read_regs(h, reg, val, 1);
}

esp_err_t lsm6dsox_write_reg(lsm6dsox_handle_t h, uint8_t reg, uint8_t val)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    uint8_t buf[2] = { reg, val };
    LOCK(h);
    esp_err_t ret = i2c_master_transmit(h->i2c, buf, 2, I2C_TIMEOUT_MS);
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_update_reg(lsm6dsox_handle_t h, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t v;
    LOCK(h);
    esp_err_t ret = lsm6dsox_read_reg(h, reg, &v);
    if (ret == ESP_OK) {
        v = (v & ~mask) | (val & mask);
        ret = lsm6dsox_write_reg(h, reg, v);
    }
    UNLOCK(h);
    return ret;
}

static esp_err_t emb_enter(lsm6dsox_handle_t h) { return lsm6dsox_write_reg(h, LSM6DSOX_FUNC_CFG_ACCESS, LSM6DSOX_FUNC_CFG_EMB); }
static esp_err_t emb_exit(lsm6dsox_handle_t h)  { return lsm6dsox_write_reg(h, LSM6DSOX_FUNC_CFG_ACCESS, 0x00); }

esp_err_t lsm6dsox_emb_read(lsm6dsox_handle_t h, uint8_t reg, uint8_t *data, size_t len)
{
    LOCK(h);
    esp_err_t ret = emb_enter(h);
    if (ret == ESP_OK) {
        ret = lsm6dsox_read_regs(h, reg, data, len);
    }
    esp_err_t e2 = emb_exit(h);
    UNLOCK(h);
    return ret != ESP_OK ? ret : e2;
}

esp_err_t lsm6dsox_emb_update(lsm6dsox_handle_t h, uint8_t reg, uint8_t mask, uint8_t val)
{
    LOCK(h);
    esp_err_t ret = emb_enter(h);
    if (ret == ESP_OK) {
        ret = lsm6dsox_update_reg(h, reg, mask, val);
    }
    esp_err_t e2 = emb_exit(h);
    UNLOCK(h);
    return ret != ESP_OK ? ret : e2;
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

static esp_err_t sync_fs_cache(lsm6dsox_handle_t h)
{
    uint8_t c[2];
    ESP_RETURN_ON_ERROR(lsm6dsox_read_regs(h, LSM6DSOX_CTRL1_XL, c, 2), TAG, "read ctrl");
    h->xl_fs = (c[0] >> 2) & 0x03;
    h->gy_fs = (c[1] & 0x02) ? LSM6DSOX_GY_FS_125DPS : (lsm6dsox_gy_fs_t)((c[1] >> 2) & 0x03);
    uint8_t c3;
    ESP_RETURN_ON_ERROR(lsm6dsox_read_reg(h, LSM6DSOX_CTRL3_C, &c3), TAG, "read ctrl3");
    h->int_active_low = c3 & LSM6DSOX_CTRL3_H_LACTIVE;
    return ESP_OK;
}

esp_err_t lsm6dsox_create(const lsm6dsox_config_t *cfg, lsm6dsox_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->bus && out, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    esp_err_t ret = ESP_OK;

    uint16_t addr = cfg->i2c_addr;
    if (addr == 0) {
        if (i2c_master_probe(cfg->bus, LSM6DSOX_I2C_ADDR_L, I2C_TIMEOUT_MS) == ESP_OK) {
            addr = LSM6DSOX_I2C_ADDR_L;
        } else if (i2c_master_probe(cfg->bus, LSM6DSOX_I2C_ADDR_H, I2C_TIMEOUT_MS) == ESP_OK) {
            addr = LSM6DSOX_I2C_ADDR_H;
        } else {
            ESP_LOGE(TAG, "no ACK at 0x6A or 0x6B");
            return ESP_ERR_NOT_FOUND;
        }
    }

    lsm6dsox_handle_t h = calloc(1, sizeof(*h));
    ESP_RETURN_ON_FALSE(h, ESP_ERR_NO_MEM, TAG, "no mem");
    h->addr = addr;
    h->lock = xSemaphoreCreateRecursiveMutex();
    if (!h->lock) {
        free(h);
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t dcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = cfg->scl_speed_hz ? cfg->scl_speed_hz : 400000,
    };
    ret = i2c_master_bus_add_device(cfg->bus, &dcfg, &h->i2c);
    if (ret != ESP_OK) {
        goto fail;
    }

    uint8_t id = 0;
    ret = lsm6dsox_get_id(h, &id);
    if (ret != ESP_OK) {
        goto fail;
    }
    if (id != LSM6DSOX_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I = 0x%02X (expected 0x%02X)", id, LSM6DSOX_WHO_AM_I_VAL);
        ret = ESP_ERR_INVALID_RESPONSE;
        goto fail;
    }
    ret = sync_fs_cache(h);
    if (ret != ESP_OK) {
        goto fail;
    }
    ESP_LOGI(TAG, "LSM6DSOX found at 0x%02X", addr);
    *out = h;
    return ESP_OK;

fail:
    if (h->i2c) {
        i2c_master_bus_rm_device(h->i2c);
    }
    vSemaphoreDelete(h->lock);
    free(h);
    return ret;
}

esp_err_t lsm6dsox_delete(lsm6dsox_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    lsm6dsox_detach_isr(h, LSM6DSOX_INT1);
    lsm6dsox_detach_isr(h, LSM6DSOX_INT2);
    i2c_master_bus_rm_device(h->i2c);
    vSemaphoreDelete(h->lock);
    free(h);
    return ESP_OK;
}

uint16_t lsm6dsox_get_addr(lsm6dsox_handle_t h) { return h ? h->addr : 0; }

esp_err_t lsm6dsox_get_id(lsm6dsox_handle_t h, uint8_t *id)
{
    return lsm6dsox_read_reg(h, LSM6DSOX_WHO_AM_I, id);
}

esp_err_t lsm6dsox_reset(lsm6dsox_handle_t h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    esp_err_t ret = ESP_OK;
    uint8_t v = 0;
    LOCK(h);
    /* Datasheet: put gyro/accel in power-down before SW reset */
    ESP_GOTO_ON_ERROR(emb_exit(h), out, TAG, "page");
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_CTRL2_G, 0), out, TAG, "g off");
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_CTRL1_XL, 0), out, TAG, "xl off");

    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_CTRL3_C, LSM6DSOX_CTRL3_SW_RESET), out, TAG, "sw reset");
    int tries = 0;
    do {
        DELAY_MS(1);
        ret = lsm6dsox_read_reg(h, LSM6DSOX_CTRL3_C, &v);
    } while ((ret != ESP_OK || (v & LSM6DSOX_CTRL3_SW_RESET)) && ++tries < 50);
    ESP_GOTO_ON_FALSE(tries < 50, ESP_ERR_TIMEOUT, out, TAG, "sw reset timeout");

    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_CTRL3_C, LSM6DSOX_CTRL3_BOOT | LSM6DSOX_CTRL3_IF_INC), out, TAG, "boot");
    DELAY_MS(15);
    tries = 0;
    do {
        ret = lsm6dsox_read_reg(h, LSM6DSOX_CTRL3_C, &v);
        if (ret == ESP_OK && !(v & LSM6DSOX_CTRL3_BOOT)) {
            break;
        }
        DELAY_MS(2);
    } while (++tries < 25);
    ESP_GOTO_ON_FALSE(tries < 25, ESP_ERR_TIMEOUT, out, TAG, "boot timeout");

    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_CTRL3_C, LSM6DSOX_CTRL3_BDU | LSM6DSOX_CTRL3_IF_INC), out, TAG, "ctrl3");
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, LSM6DSOX_CTRL9_XL, LSM6DSOX_CTRL9_I3C_DISABLE, LSM6DSOX_CTRL9_I3C_DISABLE), out, TAG, "i3c");

    /* SW reset does not touch the embedded-functions page: clear it explicitly */
    static const uint8_t emb_regs[] = { LSM6DSOX_EMB_FUNC_EN_A, LSM6DSOX_EMB_FUNC_EN_B,
                                        LSM6DSOX_EMB_FUNC_INT1, LSM6DSOX_EMB_FUNC_INT2 };
    ESP_GOTO_ON_ERROR(emb_enter(h), out, TAG, "emb");
    for (size_t i = 0; i < sizeof(emb_regs) && ret == ESP_OK; i++) {
        ret = lsm6dsox_write_reg(h, emb_regs[i], 0x00);
    }
    if (ret == ESP_OK) {
        ret = lsm6dsox_update_reg(h, LSM6DSOX_EMB_PAGE_RW, LSM6DSOX_EMB_FUNC_LIR, 0);
    }
    esp_err_t e2 = emb_exit(h);
    ESP_GOTO_ON_ERROR(ret != ESP_OK ? ret : e2, out, TAG, "emb clear");
    ret = sync_fs_cache(h);
out:
    UNLOCK(h);
    return ret;
}

/* ------------------------------------------------------------------ */
/* Sensor configuration                                                */
/* ------------------------------------------------------------------ */

esp_err_t lsm6dsox_set_accel(lsm6dsox_handle_t h, lsm6dsox_odr_t odr, lsm6dsox_xl_fs_t fs)
{
    ESP_RETURN_ON_FALSE(h && odr <= LSM6DSOX_ODR_1HZ6 && fs <= LSM6DSOX_XL_FS_8G, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    LOCK(h);
    esp_err_t ret = lsm6dsox_update_reg(h, LSM6DSOX_CTRL1_XL, 0xFC, (odr << 4) | (fs << 2));
    if (ret == ESP_OK) {
        h->xl_fs = fs;
    }
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_set_gyro(lsm6dsox_handle_t h, lsm6dsox_odr_t odr, lsm6dsox_gy_fs_t fs)
{
    ESP_RETURN_ON_FALSE(h && odr <= LSM6DSOX_ODR_6667HZ && fs <= LSM6DSOX_GY_FS_125DPS, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    uint8_t v = (odr << 4) | (fs == LSM6DSOX_GY_FS_125DPS ? 0x02 : (fs << 2));
    LOCK(h);
    esp_err_t ret = lsm6dsox_update_reg(h, LSM6DSOX_CTRL2_G, 0xFE, v);
    if (ret == ESP_OK) {
        h->gy_fs = fs;
    }
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_set_high_perf(lsm6dsox_handle_t h, bool xl_hp, bool gy_hp)
{
    LOCK(h);
    esp_err_t ret = lsm6dsox_update_reg(h, LSM6DSOX_CTRL6_C, LSM6DSOX_CTRL6_XL_HM_MODE, xl_hp ? 0 : LSM6DSOX_CTRL6_XL_HM_MODE);
    if (ret == ESP_OK) {
        ret = lsm6dsox_update_reg(h, LSM6DSOX_CTRL7_G, LSM6DSOX_CTRL7_G_HM_MODE, gy_hp ? 0 : LSM6DSOX_CTRL7_G_HM_MODE);
    }
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_set_accel_lpf2(lsm6dsox_handle_t h, bool enable, uint8_t hpcf)
{
    ESP_RETURN_ON_FALSE(h && hpcf <= 7, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    LOCK(h);
    esp_err_t ret = lsm6dsox_update_reg(h, LSM6DSOX_CTRL8_XL, 0xE0, hpcf << 5);
    if (ret == ESP_OK) {
        ret = lsm6dsox_update_reg(h, LSM6DSOX_CTRL1_XL, 0x02, enable ? 0x02 : 0);
    }
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_set_gyro_lpf1(lsm6dsox_handle_t h, bool enable, uint8_t ftype)
{
    ESP_RETURN_ON_FALSE(h && ftype <= 7, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    LOCK(h);
    esp_err_t ret = lsm6dsox_update_reg(h, LSM6DSOX_CTRL6_C, 0x07, ftype);
    if (ret == ESP_OK) {
        ret = lsm6dsox_update_reg(h, LSM6DSOX_CTRL4_C, 0x02, enable ? 0x02 : 0);
    }
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_set_accel_offset(lsm6dsox_handle_t h, int8_t x, int8_t y, int8_t z, bool weight_15mg)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    esp_err_t ret = ESP_OK;
    LOCK(h);
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, LSM6DSOX_CTRL6_C, LSM6DSOX_CTRL6_USR_OFF_W, weight_15mg ? LSM6DSOX_CTRL6_USR_OFF_W : 0), out, TAG, "w");
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_X_OFS_USR, (uint8_t)x), out, TAG, "x");
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_Y_OFS_USR, (uint8_t)y), out, TAG, "y");
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_Z_OFS_USR, (uint8_t)z), out, TAG, "z");
    bool any = x || y || z;
    ret = lsm6dsox_update_reg(h, LSM6DSOX_CTRL7_G, LSM6DSOX_CTRL7_USR_OFF_ON_OUT, any ? LSM6DSOX_CTRL7_USR_OFF_ON_OUT : 0);
out:
    UNLOCK(h);
    return ret;
}

float lsm6dsox_accel_sens_g(lsm6dsox_handle_t h)  { return XL_SENS_G[h->xl_fs]; }
float lsm6dsox_gyro_sens_dps(lsm6dsox_handle_t h) { return GY_SENS_DPS[h->gy_fs]; }

/* ------------------------------------------------------------------ */
/* Data                                                                */
/* ------------------------------------------------------------------ */

static inline int16_t le16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

static void raw_to_vec(const lsm6dsox_raw3_t *r, float s, lsm6dsox_vec3_t *v)
{
    v->x = r->x * s;
    v->y = r->y * s;
    v->z = r->z * s;
}

static void bytes_to_raw(const uint8_t *b, lsm6dsox_raw3_t *r)
{
    r->x = le16(&b[0]);
    r->y = le16(&b[2]);
    r->z = le16(&b[4]);
}

esp_err_t lsm6dsox_get_status(lsm6dsox_handle_t h, uint8_t *status)
{
    return lsm6dsox_read_reg(h, LSM6DSOX_STATUS_REG, status);
}

esp_err_t lsm6dsox_read_accel_raw(lsm6dsox_handle_t h, lsm6dsox_raw3_t *raw)
{
    uint8_t b[6];
    ESP_RETURN_ON_ERROR(lsm6dsox_read_regs(h, LSM6DSOX_OUTX_L_A, b, 6), TAG, "read xl");
    bytes_to_raw(b, raw);
    return ESP_OK;
}

esp_err_t lsm6dsox_read_gyro_raw(lsm6dsox_handle_t h, lsm6dsox_raw3_t *raw)
{
    uint8_t b[6];
    ESP_RETURN_ON_ERROR(lsm6dsox_read_regs(h, LSM6DSOX_OUTX_L_G, b, 6), TAG, "read g");
    bytes_to_raw(b, raw);
    return ESP_OK;
}

esp_err_t lsm6dsox_read_accel(lsm6dsox_handle_t h, lsm6dsox_vec3_t *g)
{
    lsm6dsox_raw3_t r;
    ESP_RETURN_ON_ERROR(lsm6dsox_read_accel_raw(h, &r), TAG, "xl");
    raw_to_vec(&r, XL_SENS_G[h->xl_fs], g);
    return ESP_OK;
}

esp_err_t lsm6dsox_read_gyro(lsm6dsox_handle_t h, lsm6dsox_vec3_t *dps)
{
    lsm6dsox_raw3_t r;
    ESP_RETURN_ON_ERROR(lsm6dsox_read_gyro_raw(h, &r), TAG, "g");
    raw_to_vec(&r, GY_SENS_DPS[h->gy_fs], dps);
    return ESP_OK;
}

esp_err_t lsm6dsox_read_temp(lsm6dsox_handle_t h, float *degc)
{
    uint8_t b[2];
    ESP_RETURN_ON_ERROR(lsm6dsox_read_regs(h, LSM6DSOX_OUT_TEMP_L, b, 2), TAG, "temp");
    *degc = le16(b) / 256.0f + 25.0f;
    return ESP_OK;
}

esp_err_t lsm6dsox_read_all(lsm6dsox_handle_t h, lsm6dsox_sample_t *s)
{
    uint8_t b[14];
    ESP_RETURN_ON_ERROR(lsm6dsox_read_regs(h, LSM6DSOX_OUT_TEMP_L, b, 14), TAG, "burst");
    lsm6dsox_raw3_t r;
    s->temp_c = le16(b) / 256.0f + 25.0f;
    bytes_to_raw(&b[2], &r);
    raw_to_vec(&r, GY_SENS_DPS[h->gy_fs], &s->gyro);
    bytes_to_raw(&b[8], &r);
    raw_to_vec(&r, XL_SENS_G[h->xl_fs], &s->accel);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Timestamp                                                           */
/* ------------------------------------------------------------------ */

esp_err_t lsm6dsox_timestamp_enable(lsm6dsox_handle_t h, bool en)
{
    return lsm6dsox_update_reg(h, LSM6DSOX_CTRL10_C, LSM6DSOX_CTRL10_TIMESTAMP_EN, en ? LSM6DSOX_CTRL10_TIMESTAMP_EN : 0);
}

esp_err_t lsm6dsox_read_timestamp(lsm6dsox_handle_t h, uint32_t *ticks)
{
    uint8_t b[4];
    ESP_RETURN_ON_ERROR(lsm6dsox_read_regs(h, LSM6DSOX_TIMESTAMP0, b, 4), TAG, "ts");
    *ticks = (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Interrupts                                                          */
/* ------------------------------------------------------------------ */

esp_err_t lsm6dsox_int_pin_mode(lsm6dsox_handle_t h, bool active_low, bool open_drain)
{
    uint8_t v = (active_low ? LSM6DSOX_CTRL3_H_LACTIVE : 0) | (open_drain ? LSM6DSOX_CTRL3_PP_OD : 0);
    LOCK(h);
    esp_err_t ret = lsm6dsox_update_reg(h, LSM6DSOX_CTRL3_C, LSM6DSOX_CTRL3_H_LACTIVE | LSM6DSOX_CTRL3_PP_OD, v);
    if (ret == ESP_OK) {
        h->int_active_low = active_low;
    }
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_set_latched(lsm6dsox_handle_t h, bool latched)
{
    LOCK(h);
    esp_err_t ret = lsm6dsox_update_reg(h, LSM6DSOX_TAP_CFG0, 0x01, latched ? 0x01 : 0);
    if (ret == ESP_OK) {
        ret = lsm6dsox_emb_update(h, LSM6DSOX_EMB_PAGE_RW, LSM6DSOX_EMB_FUNC_LIR, latched ? LSM6DSOX_EMB_FUNC_LIR : 0);
    }
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_set_drdy_pulsed(lsm6dsox_handle_t h, bool pulsed)
{
    return lsm6dsox_update_reg(h, LSM6DSOX_COUNTER_BDR_REG1, LSM6DSOX_DATAREADY_PULSED, pulsed ? LSM6DSOX_DATAREADY_PULSED : 0);
}

#define EVT_BASIC_MASK (LSM6DSOX_EVT_SLEEP_CHANGE | LSM6DSOX_EVT_SINGLE_TAP | LSM6DSOX_EVT_DOUBLE_TAP | \
                        LSM6DSOX_EVT_WAKE_UP | LSM6DSOX_EVT_FREE_FALL | LSM6DSOX_EVT_6D)
#define EVT_EMB_MASK   (LSM6DSOX_EVT_STEP_DET | LSM6DSOX_EVT_TILT | LSM6DSOX_EVT_SIG_MOTION)

esp_err_t lsm6dsox_route_int(lsm6dsox_handle_t h, lsm6dsox_int_pin_t pin, uint32_t m)
{
    ESP_RETURN_ON_FALSE(h && (pin == LSM6DSOX_INT1 || pin == LSM6DSOX_INT2), ESP_ERR_INVALID_ARG, TAG, "bad pin");
    if (pin == LSM6DSOX_INT1) {
        ESP_RETURN_ON_FALSE(!(m & (LSM6DSOX_EVT_DRDY_TEMP | LSM6DSOX_EVT_TIMESTAMP)), ESP_ERR_INVALID_ARG, TAG, "event not on INT1");
    } else {
        ESP_RETURN_ON_FALSE(!(m & LSM6DSOX_EVT_BOOT), ESP_ERR_INVALID_ARG, TAG, "BOOT not on INT2");
    }

    uint8_t ctrl = 0, md = 0, emb = 0;
    if (m & LSM6DSOX_EVT_DRDY_XL)   ctrl |= 0x01;
    if (m & LSM6DSOX_EVT_DRDY_G)    ctrl |= 0x02;
    if (m & (LSM6DSOX_EVT_BOOT | LSM6DSOX_EVT_DRDY_TEMP)) ctrl |= 0x04;
    if (m & LSM6DSOX_EVT_FIFO_TH)   ctrl |= 0x08;
    if (m & LSM6DSOX_EVT_FIFO_OVR)  ctrl |= 0x10;
    if (m & LSM6DSOX_EVT_FIFO_FULL) ctrl |= 0x20;
    if (m & LSM6DSOX_EVT_CNT_BDR)   ctrl |= 0x40;

    if (m & LSM6DSOX_EVT_TIMESTAMP)    md |= 0x01;
    if (m & EVT_EMB_MASK)              md |= 0x02;
    if (m & LSM6DSOX_EVT_6D)           md |= 0x04;
    if (m & LSM6DSOX_EVT_DOUBLE_TAP)   md |= 0x08;
    if (m & LSM6DSOX_EVT_FREE_FALL)    md |= 0x10;
    if (m & LSM6DSOX_EVT_WAKE_UP)      md |= 0x20;
    if (m & LSM6DSOX_EVT_SINGLE_TAP)   md |= 0x40;
    if (m & LSM6DSOX_EVT_SLEEP_CHANGE) md |= 0x80;

    if (m & LSM6DSOX_EVT_STEP_DET)   emb |= 0x08;
    if (m & LSM6DSOX_EVT_TILT)       emb |= 0x10;
    if (m & LSM6DSOX_EVT_SIG_MOTION) emb |= 0x20;

    esp_err_t ret = ESP_OK;
    LOCK(h);
    bool p1 = pin == LSM6DSOX_INT1;
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, p1 ? LSM6DSOX_INT1_CTRL : LSM6DSOX_INT2_CTRL, 0x7F, ctrl), out, TAG, "ctrl");
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, p1 ? LSM6DSOX_MD1_CFG : LSM6DSOX_MD2_CFG, 0xFF, md), out, TAG, "md");
    ESP_GOTO_ON_ERROR(lsm6dsox_emb_update(h, p1 ? LSM6DSOX_EMB_FUNC_INT1 : LSM6DSOX_EMB_FUNC_INT2, 0x38, emb), out, TAG, "emb");
    if (m & EVT_BASIC_MASK) {
        ret = lsm6dsox_update_reg(h, LSM6DSOX_TAP_CFG2, LSM6DSOX_TAP_CFG2_INT_EN, LSM6DSOX_TAP_CFG2_INT_EN);
    }
out:
    UNLOCK(h);
    return ret;
}

static void gpio_isr(void *arg)
{
    isr_slot_t *s = (isr_slot_t *)arg;
    if (s->cb) {
        s->cb(s->pin, s->arg);
    }
}

esp_err_t lsm6dsox_attach_isr(lsm6dsox_handle_t h, lsm6dsox_int_pin_t pin, gpio_num_t gpio,
                              lsm6dsox_isr_cb_t cb, void *arg)
{
    ESP_RETURN_ON_FALSE(h && cb && (pin == LSM6DSOX_INT1 || pin == LSM6DSOX_INT2), ESP_ERR_INVALID_ARG, TAG, "bad arg");
    isr_slot_t *s = &h->isr[pin - 1];
    if (s->cb) {
        lsm6dsox_detach_isr(h, pin);
    }

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = h->int_active_low ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = h->int_active_low ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = h->int_active_low ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config");

    static bool s_isr_service;   /* avoid the driver's "already installed" error log */
    if (!s_isr_service) {
        esp_err_t ret = gpio_install_isr_service(0);
        ESP_RETURN_ON_FALSE(ret == ESP_OK || ret == ESP_ERR_INVALID_STATE, ret, TAG, "isr service");
        s_isr_service = true;
    }

    s->dev = h;
    s->pin = pin;
    s->gpio = gpio;
    s->arg = arg;
    s->cb = cb;
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(gpio, gpio_isr, s), TAG, "isr add");
    return ESP_OK;
}

esp_err_t lsm6dsox_detach_isr(lsm6dsox_handle_t h, lsm6dsox_int_pin_t pin)
{
    ESP_RETURN_ON_FALSE(h && (pin == LSM6DSOX_INT1 || pin == LSM6DSOX_INT2), ESP_ERR_INVALID_ARG, TAG, "bad arg");
    isr_slot_t *s = &h->isr[pin - 1];
    if (!s->cb) {
        return ESP_OK;
    }
    gpio_isr_handler_remove(s->gpio);
    gpio_set_intr_type(s->gpio, GPIO_INTR_DISABLE);
    s->cb = NULL;
    return ESP_OK;
}

esp_err_t lsm6dsox_read_events(lsm6dsox_handle_t h, lsm6dsox_events_t *ev)
{
    ESP_RETURN_ON_FALSE(h && ev, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    uint8_t b[4];
    memset(ev, 0, sizeof(*ev));
    LOCK(h);
    esp_err_t ret = lsm6dsox_read_regs(h, LSM6DSOX_ALL_INT_SRC, b, 4);
    if (ret == ESP_OK) {
        ret = lsm6dsox_emb_read(h, LSM6DSOX_EMB_FUNC_STATUS, &ev->emb_func_status, 1);
    }
    UNLOCK(h);
    ESP_RETURN_ON_ERROR(ret, TAG, "read src");

    ev->all_int_src = b[0];
    ev->wake_up_src = b[1];
    ev->tap_src     = b[2];
    ev->d6d_src     = b[3];
    ev->free_fall    = (b[0] & 0x01) || (b[1] & 0x20);
    ev->wake_up      = (b[0] & 0x02) || (b[1] & 0x08);
    ev->single_tap   = (b[0] & 0x04) || (b[2] & 0x20);
    ev->double_tap   = (b[0] & 0x08) || (b[2] & 0x10);
    ev->d6d          = (b[0] & 0x10) || (b[3] & 0x40);
    ev->sleep_change = (b[0] & 0x20) || (b[1] & 0x40);
    ev->sleep_state  = b[1] & 0x10;
    ev->step         = ev->emb_func_status & LSM6DSOX_EMB_IS_STEP_DET;
    ev->tilt         = ev->emb_func_status & LSM6DSOX_EMB_IS_TILT;
    ev->sig_motion   = ev->emb_func_status & LSM6DSOX_EMB_IS_SIGMOT;
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Motion functions                                                    */
/* ------------------------------------------------------------------ */

esp_err_t lsm6dsox_tap_config(lsm6dsox_handle_t h, const lsm6dsox_tap_cfg_t *c)
{
    ESP_RETURN_ON_FALSE(h && c && c->threshold <= 31 && c->shock <= 3 && c->quiet <= 3 && c->duration <= 15,
                        ESP_ERR_INVALID_ARG, TAG, "bad arg");
    uint8_t axes = (c->x_en ? 0x08 : 0) | (c->y_en ? 0x04 : 0) | (c->z_en ? 0x02 : 0);
    esp_err_t ret = ESP_OK;
    LOCK(h);
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, LSM6DSOX_TAP_CFG0, 0x0E, axes), out, TAG, "cfg0");
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, LSM6DSOX_TAP_CFG1, 0x1F, c->threshold), out, TAG, "ths x");
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, LSM6DSOX_TAP_CFG2, 0x1F | LSM6DSOX_TAP_CFG2_INT_EN, c->threshold | LSM6DSOX_TAP_CFG2_INT_EN), out, TAG, "ths y");
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, LSM6DSOX_TAP_THS_6D, 0x1F, c->threshold), out, TAG, "ths z");
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_INT_DUR2, (c->duration << 4) | (c->quiet << 2) | c->shock), out, TAG, "dur2");
    ret = lsm6dsox_update_reg(h, LSM6DSOX_WAKE_UP_THS, 0x80, c->double_tap ? 0x80 : 0);
out:
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_wake_up_config(lsm6dsox_handle_t h, uint8_t ths, uint8_t dur)
{
    ESP_RETURN_ON_FALSE(h && ths <= 63 && dur <= 3, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    esp_err_t ret = ESP_OK;
    LOCK(h);
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, LSM6DSOX_WAKE_UP_THS, 0x3F, ths), out, TAG, "ths");
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, LSM6DSOX_WAKE_UP_DUR, 0x60, dur << 5), out, TAG, "dur");
    ret = lsm6dsox_update_reg(h, LSM6DSOX_TAP_CFG2, LSM6DSOX_TAP_CFG2_INT_EN, LSM6DSOX_TAP_CFG2_INT_EN);
out:
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_free_fall_config(lsm6dsox_handle_t h, uint8_t ths, uint8_t dur)
{
    ESP_RETURN_ON_FALSE(h && ths <= 7 && dur <= 63, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    esp_err_t ret = ESP_OK;
    LOCK(h);
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_FREE_FALL, ((dur & 0x1F) << 3) | ths), out, TAG, "ff");
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, LSM6DSOX_WAKE_UP_DUR, 0x80, (dur & 0x20) << 2), out, TAG, "ff dur5");
    ret = lsm6dsox_update_reg(h, LSM6DSOX_TAP_CFG2, LSM6DSOX_TAP_CFG2_INT_EN, LSM6DSOX_TAP_CFG2_INT_EN);
out:
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_6d_config(lsm6dsox_handle_t h, uint8_t ths, bool d4d)
{
    ESP_RETURN_ON_FALSE(h && ths <= 3, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    esp_err_t ret = ESP_OK;
    LOCK(h);
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, LSM6DSOX_TAP_THS_6D, 0xE0, (d4d ? 0x80 : 0) | (ths << 5)), out, TAG, "6d");
    ret = lsm6dsox_update_reg(h, LSM6DSOX_TAP_CFG2, LSM6DSOX_TAP_CFG2_INT_EN, LSM6DSOX_TAP_CFG2_INT_EN);
out:
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_activity_config(lsm6dsox_handle_t h, uint8_t inact_mode, uint8_t sleep_dur)
{
    ESP_RETURN_ON_FALSE(h && inact_mode <= 3 && sleep_dur <= 15, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    esp_err_t ret = ESP_OK;
    LOCK(h);
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, LSM6DSOX_WAKE_UP_DUR, 0x0F, sleep_dur), out, TAG, "sleep dur");
    ret = lsm6dsox_update_reg(h, LSM6DSOX_TAP_CFG2, 0x60 | LSM6DSOX_TAP_CFG2_INT_EN, (inact_mode << 5) | LSM6DSOX_TAP_CFG2_INT_EN);
out:
    UNLOCK(h);
    return ret;
}

/* ------------------------------------------------------------------ */
/* Embedded functions                                                  */
/* ------------------------------------------------------------------ */

esp_err_t lsm6dsox_pedometer_enable(lsm6dsox_handle_t h, bool en)
{
    return lsm6dsox_emb_update(h, LSM6DSOX_EMB_FUNC_EN_A, LSM6DSOX_EMB_PEDO_EN, en ? LSM6DSOX_EMB_PEDO_EN : 0);
}

esp_err_t lsm6dsox_step_count(lsm6dsox_handle_t h, uint16_t *steps)
{
    uint8_t b[2];
    ESP_RETURN_ON_ERROR(lsm6dsox_emb_read(h, LSM6DSOX_EMB_STEP_COUNTER_L, b, 2), TAG, "steps");
    *steps = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
    return ESP_OK;
}

esp_err_t lsm6dsox_step_reset(lsm6dsox_handle_t h)
{
    return lsm6dsox_emb_update(h, LSM6DSOX_EMB_FUNC_SRC, LSM6DSOX_EMB_PEDO_RST_STEP, LSM6DSOX_EMB_PEDO_RST_STEP);
}

esp_err_t lsm6dsox_tilt_enable(lsm6dsox_handle_t h, bool en)
{
    return lsm6dsox_emb_update(h, LSM6DSOX_EMB_FUNC_EN_A, LSM6DSOX_EMB_TILT_EN, en ? LSM6DSOX_EMB_TILT_EN : 0);
}

esp_err_t lsm6dsox_sig_motion_enable(lsm6dsox_handle_t h, bool en)
{
    return lsm6dsox_emb_update(h, LSM6DSOX_EMB_FUNC_EN_A, LSM6DSOX_EMB_SIGN_MOTION_EN, en ? LSM6DSOX_EMB_SIGN_MOTION_EN : 0);
}

/* ------------------------------------------------------------------ */
/* FIFO                                                                */
/* ------------------------------------------------------------------ */

esp_err_t lsm6dsox_fifo_config(lsm6dsox_handle_t h, const lsm6dsox_fifo_cfg_t *c)
{
    ESP_RETURN_ON_FALSE(h && c && c->watermark <= 511 && c->temp_bdr <= 3 && c->ts_decimation <= 3 &&
                        c->bdr_xl <= LSM6DSOX_ODR_1HZ6 && c->bdr_gy <= LSM6DSOX_ODR_1HZ6,
                        ESP_ERR_INVALID_ARG, TAG, "bad arg");
    esp_err_t ret = ESP_OK;
    LOCK(h);
    /* Pass through bypass to flush old content */
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_FIFO_CTRL4, LSM6DSOX_FIFO_BYPASS), out, TAG, "bypass");
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_FIFO_CTRL1, c->watermark & 0xFF), out, TAG, "wtm");
    ESP_GOTO_ON_ERROR(lsm6dsox_update_reg(h, LSM6DSOX_FIFO_CTRL2, 0x81,
                                          ((c->watermark >> 8) & 0x01) | (c->stop_on_wtm ? 0x80 : 0)), out, TAG, "ctrl2");
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_FIFO_CTRL3, (c->bdr_gy << 4) | c->bdr_xl), out, TAG, "bdr");
    if (c->ts_decimation) {
        ESP_GOTO_ON_ERROR(lsm6dsox_timestamp_enable(h, true), out, TAG, "ts");
    }
    ret = lsm6dsox_write_reg(h, LSM6DSOX_FIFO_CTRL4, (c->ts_decimation << 6) | (c->temp_bdr << 4) | (c->mode & 0x07));
out:
    UNLOCK(h);
    return ret;
}

esp_err_t lsm6dsox_fifo_status(lsm6dsox_handle_t h, lsm6dsox_fifo_status_t *st)
{
    uint8_t b[2];
    ESP_RETURN_ON_ERROR(lsm6dsox_read_regs(h, LSM6DSOX_FIFO_STATUS1, b, 2), TAG, "fifo st");
    st->level = b[0] | ((b[1] & 0x03) << 8);
    st->wtm = b[1] & 0x80;
    st->ovr = b[1] & 0x40;
    st->full = b[1] & 0x20;
    st->cnt_bdr = b[1] & 0x10;
    st->ovr_latched = b[1] & 0x08;
    return ESP_OK;
}

esp_err_t lsm6dsox_fifo_read(lsm6dsox_handle_t h, lsm6dsox_fifo_word_t *w, size_t max, size_t *n)
{
    ESP_RETURN_ON_FALSE(h && w && n, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    *n = 0;
    lsm6dsox_fifo_status_t st;
    esp_err_t ret = ESP_OK;
    LOCK(h);
    ESP_GOTO_ON_ERROR(lsm6dsox_fifo_status(h, &st), out, TAG, "status");
    size_t cnt = st.level < max ? st.level : max;
    for (size_t i = 0; i < cnt; i++) {
        uint8_t b[7];
        ESP_GOTO_ON_ERROR(lsm6dsox_read_regs(h, LSM6DSOX_FIFO_DATA_OUT_TAG, b, 7), out, TAG, "word");
        lsm6dsox_fifo_word_t *o = &w[i];
        o->tag = b[0] >> 3;
        o->cnt = (b[0] >> 1) & 0x03;
        memcpy(o->raw, &b[1], 6);
        lsm6dsox_raw3_t r;
        switch (o->tag) {
        case LSM6DSOX_TAG_ACCEL:
            bytes_to_raw(o->raw, &r);
            raw_to_vec(&r, XL_SENS_G[h->xl_fs], &o->accel);
            break;
        case LSM6DSOX_TAG_GYRO:
            bytes_to_raw(o->raw, &r);
            raw_to_vec(&r, GY_SENS_DPS[h->gy_fs], &o->gyro);
            break;
        case LSM6DSOX_TAG_TEMP:
            o->temp_c = le16(o->raw) / 256.0f + 25.0f;
            break;
        case LSM6DSOX_TAG_TIMESTAMP:
            o->timestamp = (uint32_t)o->raw[0] | ((uint32_t)o->raw[1] << 8) |
                           ((uint32_t)o->raw[2] << 16) | ((uint32_t)o->raw[3] << 24);
            break;
        case LSM6DSOX_TAG_STEP_COUNTER:
            o->steps = (uint16_t)o->raw[0] | ((uint16_t)o->raw[1] << 8);
            break;
        default:
            break;
        }
        (*n)++;
    }
    ret = ESP_OK;
out:
    UNLOCK(h);
    return ret;
}

/* ------------------------------------------------------------------ */
/* Self test (AN5272 procedure)                                        */
/* ------------------------------------------------------------------ */

static esp_err_t wait_flag(lsm6dsox_handle_t h, uint8_t flag)
{
    for (int i = 0; i < 100; i++) {
        uint8_t s;
        ESP_RETURN_ON_ERROR(lsm6dsox_get_status(h, &s), TAG, "status");
        if (s & flag) {
            return ESP_OK;
        }
        DELAY_MS(2);
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t avg5(lsm6dsox_handle_t h, bool gyro, float out[3])
{
    uint8_t flag = gyro ? LSM6DSOX_STATUS_GDA : LSM6DSOX_STATUS_XLDA;
    lsm6dsox_raw3_t r;
    /* discard first sample */
    ESP_RETURN_ON_ERROR(wait_flag(h, flag), TAG, "drdy");
    ESP_RETURN_ON_ERROR(gyro ? lsm6dsox_read_gyro_raw(h, &r) : lsm6dsox_read_accel_raw(h, &r), TAG, "rd");
    out[0] = out[1] = out[2] = 0;
    for (int i = 0; i < 5; i++) {
        ESP_RETURN_ON_ERROR(wait_flag(h, flag), TAG, "drdy");
        ESP_RETURN_ON_ERROR(gyro ? lsm6dsox_read_gyro_raw(h, &r) : lsm6dsox_read_accel_raw(h, &r), TAG, "rd");
        out[0] += r.x / 5.0f;
        out[1] += r.y / 5.0f;
        out[2] += r.z / 5.0f;
    }
    return ESP_OK;
}

esp_err_t lsm6dsox_self_test(lsm6dsox_handle_t h, lsm6dsox_selftest_t *res)
{
    ESP_RETURN_ON_FALSE(h && res, ESP_ERR_INVALID_ARG, TAG, "bad arg");
    memset(res, 0, sizeof(*res));
    uint8_t saved[10];
    float off[3], on[3];
    esp_err_t ret = ESP_OK;

    LOCK(h);
    lsm6dsox_xl_fs_t xl_fs = h->xl_fs;
    lsm6dsox_gy_fs_t gy_fs = h->gy_fs;
    ESP_GOTO_ON_ERROR(lsm6dsox_read_regs(h, LSM6DSOX_CTRL1_XL, saved, 10), out, TAG, "save");

    /* ---- Accelerometer: 52 Hz, 4 g ---- */
    const uint8_t xl_cfg[10] = { 0x38, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    for (int i = 0; i < 10; i++) {
        ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_CTRL1_XL + i, xl_cfg[i]), restore, TAG, "cfg");
    }
    DELAY_MS(100);
    ESP_GOTO_ON_ERROR(avg5(h, false, off), restore, TAG, "xl off");
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_CTRL5_C, 0x01), restore, TAG, "st+");
    DELAY_MS(100);
    ESP_GOTO_ON_ERROR(avg5(h, false, on), restore, TAG, "xl on");
    res->accel_delta_mg.x = fabsf(on[0] - off[0]) * 0.122f;
    res->accel_delta_mg.y = fabsf(on[1] - off[1]) * 0.122f;
    res->accel_delta_mg.z = fabsf(on[2] - off[2]) * 0.122f;
    res->accel_pass = true;
    for (int i = 0; i < 3; i++) {
        float d = (&res->accel_delta_mg.x)[i];
        if (d < 50.0f || d > 1700.0f) {
            res->accel_pass = false;
        }
    }
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_CTRL5_C, 0x00), restore, TAG, "st0");
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_CTRL1_XL, 0x00), restore, TAG, "xl off");

    /* ---- Gyroscope: 208 Hz, 2000 dps ---- */
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_CTRL2_G, 0x5C), restore, TAG, "g cfg");
    DELAY_MS(100);
    ESP_GOTO_ON_ERROR(avg5(h, true, off), restore, TAG, "g off");
    ESP_GOTO_ON_ERROR(lsm6dsox_write_reg(h, LSM6DSOX_CTRL5_C, 0x04), restore, TAG, "gst+");
    DELAY_MS(100);
    ESP_GOTO_ON_ERROR(avg5(h, true, on), restore, TAG, "g on");
    res->gyro_delta_dps.x = fabsf(on[0] - off[0]) * 0.070f;
    res->gyro_delta_dps.y = fabsf(on[1] - off[1]) * 0.070f;
    res->gyro_delta_dps.z = fabsf(on[2] - off[2]) * 0.070f;
    res->gyro_pass = true;
    for (int i = 0; i < 3; i++) {
        float d = (&res->gyro_delta_dps.x)[i];
        if (d < 150.0f || d > 700.0f) {
            res->gyro_pass = false;
        }
    }

restore:
    lsm6dsox_write_reg(h, LSM6DSOX_CTRL5_C, 0x00);
    lsm6dsox_write_reg(h, LSM6DSOX_CTRL2_G, 0x00);
    DELAY_MS(50);
    for (int i = 0; i < 10; i++) {
        esp_err_t e = lsm6dsox_write_reg(h, LSM6DSOX_CTRL1_XL + i, saved[i]);
        if (ret == ESP_OK) {
            ret = e;
        }
    }
    h->xl_fs = xl_fs;
    h->gy_fs = gy_fs;
out:
    UNLOCK(h);
    return ret;
}
