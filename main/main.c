/*
 * Kilya IMU serial test
 *
 * IMU only: no Bluetooth, no Wi-Fi, CPU at 80 MHz. The goal is to see whether
 * the board keeps running (and stays cooler) without the radio, i.e. whether
 * the "stops after 1-2 minutes" problem is caused by the supply / LDO.
 *
 * Serial output (USB):
 *   data lines   every PRINT_PERIOD_MS, CSV, averaged over the period:
 *                t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,pitch_deg,roll_deg,imu_temp_c
 *   "#" lines    boot info and a health line every HEALTH_PERIOD_MS
 *                (uptime, sample rate, I2C errors, chip temperature, heap)
 *
 * pitch/roll here come from the accelerometer only, in sensor axes (no
 * calibration): they are only a sanity check.
 */
#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/i2c_master.h"
#include "driver/temperature_sensor.h"
#include "lsm6dsox.h"

/* ---- board (same wiring as Kilya_Base_Code) ---- */
#define PIN_I2C_SDA         GPIO_NUM_4
#define PIN_I2C_SCL         GPIO_NUM_5
#define I2C_FREQ_HZ         400000

/* ---- test settings ---- */
#define IMU_ODR             LSM6DSOX_ODR_104HZ
#define PRINT_PERIOD_MS     200
#define HEALTH_PERIOD_MS    5000
#define FIFO_POLL_MS        100
#define FIFO_CHUNK          32

static temperature_sensor_handle_t s_tsens;

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:    return "power-on";
    case ESP_RST_EXT:        return "external pin";
    case ESP_RST_SW:         return "software";
    case ESP_RST_PANIC:      return "PANIC / crash";
    case ESP_RST_INT_WDT:    return "interrupt watchdog";
    case ESP_RST_TASK_WDT:   return "task watchdog";
    case ESP_RST_WDT:        return "other watchdog";
    case ESP_RST_BROWNOUT:   return "BROWNOUT (supply voltage dropped)";
    case ESP_RST_USB:        return "USB";
    case ESP_RST_PWR_GLITCH: return "POWER GLITCH";
    case ESP_RST_CPU_LOCKUP: return "CPU lockup";
    default:                 return "unknown";
    }
}

static float chip_temperature(void)
{
    float t = NAN;
    if (s_tsens && temperature_sensor_enable(s_tsens) == ESP_OK) {
        temperature_sensor_get_celsius(s_tsens, &t);
        temperature_sensor_disable(s_tsens);
    }
    return t;
}

static lsm6dsox_handle_t imu_start(i2c_master_bus_handle_t bus)
{
    lsm6dsox_handle_t imu = NULL;
    lsm6dsox_config_t cfg = { .bus = bus, .i2c_addr = 0, .scl_speed_hz = I2C_FREQ_HZ };
    lsm6dsox_fifo_cfg_t fc = {
        .mode = LSM6DSOX_FIFO_CONTINUOUS,
        .bdr_xl = IMU_ODR,
        .bdr_gy = IMU_ODR,
        .temp_bdr = 2,                        /* 12.5 Hz */
    };
    if (lsm6dsox_create(&cfg, &imu) != ESP_OK ||
        lsm6dsox_reset(imu) != ESP_OK ||
        lsm6dsox_set_accel(imu, IMU_ODR, LSM6DSOX_XL_FS_4G) != ESP_OK ||
        lsm6dsox_set_gyro(imu, IMU_ODR, LSM6DSOX_GY_FS_1000DPS) != ESP_OK ||
        lsm6dsox_fifo_config(imu, &fc) != ESP_OK) {
        if (imu) {
            lsm6dsox_delete(imu);
        }
        return NULL;
    }
    return imu;
}

void app_main(void)
{
    esp_reset_reason_t rr = esp_reset_reason();
    printf("\n# Kilya IMU serial test - no BLE / Wi-Fi, CPU %d MHz\n", CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    printf("# reset reason: %s\n", reset_reason_str(rr));

    temperature_sensor_config_t tc = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&tc, &s_tsens) != ESP_OK) {
        s_tsens = NULL;
    }

    i2c_master_bus_handle_t bus;
    i2c_master_bus_config_t bcfg = {
        .i2c_port = -1,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bcfg, &bus));

    lsm6dsox_handle_t imu;
    while ((imu = imu_start(bus)) == NULL) {
        printf("# IMU not responding - retrying in 2 s\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    printf("# IMU ok at 0x%02X, 104 Hz, +-4 g, +-1000 dps\n", lsm6dsox_get_addr(imu));
    printf("t_s,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,pitch_deg,roll_deg,imu_temp_c\n");

    static lsm6dsox_fifo_word_t w[FIFO_CHUNK];
    double sa[3] = { 0 }, sg[3] = { 0 };
    uint32_t na = 0, ng = 0;
    uint32_t total = 0, total_prev = 0, i2c_errors = 0;
    float imu_temp = NAN;
    int64_t t_print = esp_timer_get_time(), t_health = t_print, t_health_prev = t_print;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(FIFO_POLL_MS));

        /* drain the FIFO */
        size_t n;
        do {
            if (lsm6dsox_fifo_read(imu, w, FIFO_CHUNK, &n) != ESP_OK) {
                i2c_errors++;
                break;
            }
            for (size_t i = 0; i < n; i++) {
                switch (w[i].tag) {
                case LSM6DSOX_TAG_ACCEL:
                    sa[0] += w[i].accel.x; sa[1] += w[i].accel.y; sa[2] += w[i].accel.z;
                    na++;
                    total++;
                    break;
                case LSM6DSOX_TAG_GYRO:
                    sg[0] += w[i].gyro.x; sg[1] += w[i].gyro.y; sg[2] += w[i].gyro.z;
                    ng++;
                    break;
                case LSM6DSOX_TAG_TEMP:
                    imu_temp = w[i].temp_c;
                    break;
                default:
                    break;
                }
            }
        } while (n == FIFO_CHUNK);

        int64_t now = esp_timer_get_time();

        if (now - t_print >= PRINT_PERIOD_MS * 1000LL) {
            t_print = now;
            if (na && ng) {
                float ax = sa[0] / na, ay = sa[1] / na, az = sa[2] / na;
                float pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
                float roll = atan2f(ay, az) * 57.29578f;
                printf("%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", now / 1e6,
                       ax, ay, az, sg[0] / ng, sg[1] / ng, sg[2] / ng, pitch, roll, imu_temp);
            } else {
                printf("# no IMU samples in the last %d ms (I2C errors so far: %" PRIu32 ")\n",
                       PRINT_PERIOD_MS, i2c_errors);
            }
            sa[0] = sa[1] = sa[2] = sg[0] = sg[1] = sg[2] = 0;
            na = ng = 0;
        }

        if (now - t_health >= HEALTH_PERIOD_MS * 1000LL) {
            float rate = (total - total_prev) * 1e6f / (float)(now - t_health_prev);
            total_prev = total;
            t_health_prev = t_health = now;
            printf("# health: up %llds | imu %.1f Hz | i2c errors %" PRIu32 " | chip %.1f C | imu %.1f C | heap %" PRIu32 "\n",
                   now / 1000000, rate, i2c_errors, chip_temperature(), imu_temp, esp_get_free_heap_size());
        }
    }
}
