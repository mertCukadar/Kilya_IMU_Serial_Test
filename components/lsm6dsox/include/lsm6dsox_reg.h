/*
 * LSM6DSOX register map (ST DS12814, Rev 5)
 */
#pragma once

#define LSM6DSOX_I2C_ADDR_L             0x6A    /* SDO/SA0 -> GND */
#define LSM6DSOX_I2C_ADDR_H             0x6B    /* SDO/SA0 -> VDD */
#define LSM6DSOX_WHO_AM_I_VAL           0x6C

/* ---------------- Main page ---------------- */
#define LSM6DSOX_FUNC_CFG_ACCESS        0x01
#define   LSM6DSOX_FUNC_CFG_EMB         0x80
#define LSM6DSOX_PIN_CTRL               0x02
#define LSM6DSOX_FIFO_CTRL1             0x07
#define LSM6DSOX_FIFO_CTRL2             0x08
#define LSM6DSOX_FIFO_CTRL3             0x09
#define LSM6DSOX_FIFO_CTRL4             0x0A
#define LSM6DSOX_COUNTER_BDR_REG1       0x0B
#define   LSM6DSOX_DATAREADY_PULSED     0x80
#define LSM6DSOX_COUNTER_BDR_REG2       0x0C
#define LSM6DSOX_INT1_CTRL              0x0D
#define LSM6DSOX_INT2_CTRL              0x0E
#define LSM6DSOX_WHO_AM_I               0x0F
#define LSM6DSOX_CTRL1_XL               0x10
#define LSM6DSOX_CTRL2_G                0x11
#define LSM6DSOX_CTRL3_C                0x12
#define   LSM6DSOX_CTRL3_BOOT           0x80
#define   LSM6DSOX_CTRL3_BDU            0x40
#define   LSM6DSOX_CTRL3_H_LACTIVE      0x20
#define   LSM6DSOX_CTRL3_PP_OD          0x10
#define   LSM6DSOX_CTRL3_IF_INC         0x04
#define   LSM6DSOX_CTRL3_SW_RESET       0x01
#define LSM6DSOX_CTRL4_C                0x13
#define LSM6DSOX_CTRL5_C                0x14
#define LSM6DSOX_CTRL6_C                0x15
#define   LSM6DSOX_CTRL6_XL_HM_MODE     0x10
#define   LSM6DSOX_CTRL6_USR_OFF_W      0x08
#define LSM6DSOX_CTRL7_G                0x16
#define   LSM6DSOX_CTRL7_G_HM_MODE      0x80
#define   LSM6DSOX_CTRL7_USR_OFF_ON_OUT 0x02
#define LSM6DSOX_CTRL8_XL               0x17
#define LSM6DSOX_CTRL9_XL               0x18
#define   LSM6DSOX_CTRL9_I3C_DISABLE    0x02
#define LSM6DSOX_CTRL10_C               0x19
#define   LSM6DSOX_CTRL10_TIMESTAMP_EN  0x20
#define LSM6DSOX_ALL_INT_SRC            0x1A
#define LSM6DSOX_WAKE_UP_SRC            0x1B
#define LSM6DSOX_TAP_SRC                0x1C
#define LSM6DSOX_D6D_SRC                0x1D
#define LSM6DSOX_STATUS_REG             0x1E
#define LSM6DSOX_OUT_TEMP_L             0x20
#define LSM6DSOX_OUTX_L_G               0x22
#define LSM6DSOX_OUTX_L_A               0x28
#define LSM6DSOX_EMB_FUNC_STATUS_MAINPAGE 0x35
#define LSM6DSOX_FIFO_STATUS1           0x3A
#define LSM6DSOX_FIFO_STATUS2           0x3B
#define LSM6DSOX_TIMESTAMP0             0x40
#define LSM6DSOX_TAP_CFG0               0x56
#define LSM6DSOX_TAP_CFG1               0x57
#define LSM6DSOX_TAP_CFG2               0x58
#define   LSM6DSOX_TAP_CFG2_INT_EN      0x80
#define LSM6DSOX_TAP_THS_6D             0x59
#define LSM6DSOX_INT_DUR2               0x5A
#define LSM6DSOX_WAKE_UP_THS            0x5B
#define LSM6DSOX_WAKE_UP_DUR            0x5C
#define LSM6DSOX_FREE_FALL              0x5D
#define LSM6DSOX_MD1_CFG                0x5E
#define LSM6DSOX_MD2_CFG                0x5F
#define LSM6DSOX_INTERNAL_FREQ_FINE     0x63
#define LSM6DSOX_X_OFS_USR              0x73
#define LSM6DSOX_Y_OFS_USR              0x74
#define LSM6DSOX_Z_OFS_USR              0x75
#define LSM6DSOX_FIFO_DATA_OUT_TAG      0x78

/* STATUS_REG bits */
#define LSM6DSOX_STATUS_XLDA            0x01
#define LSM6DSOX_STATUS_GDA             0x02
#define LSM6DSOX_STATUS_TDA             0x04

/* FIFO tags (TAG_SENSOR) */
#define LSM6DSOX_TAG_GYRO               0x01
#define LSM6DSOX_TAG_ACCEL              0x02
#define LSM6DSOX_TAG_TEMP               0x03
#define LSM6DSOX_TAG_TIMESTAMP          0x04
#define LSM6DSOX_TAG_CFG_CHANGE         0x05
#define LSM6DSOX_TAG_STEP_COUNTER       0x12

/* ---------------- Embedded functions page (FUNC_CFG_ACCESS = 0x80) ---------------- */
#define LSM6DSOX_EMB_PAGE_SEL           0x02
#define LSM6DSOX_EMB_FUNC_EN_A          0x04
#define   LSM6DSOX_EMB_SIGN_MOTION_EN   0x20
#define   LSM6DSOX_EMB_TILT_EN          0x10
#define   LSM6DSOX_EMB_PEDO_EN          0x08
#define LSM6DSOX_EMB_FUNC_EN_B          0x05
#define LSM6DSOX_EMB_FUNC_INT1          0x0A
#define LSM6DSOX_EMB_FUNC_INT2          0x0E
#define LSM6DSOX_EMB_FUNC_STATUS        0x12
#define   LSM6DSOX_EMB_IS_SIGMOT        0x20
#define   LSM6DSOX_EMB_IS_TILT          0x10
#define   LSM6DSOX_EMB_IS_STEP_DET      0x08
#define LSM6DSOX_EMB_PAGE_RW            0x17
#define   LSM6DSOX_EMB_FUNC_LIR         0x80
#define LSM6DSOX_EMB_FUNC_FIFO_CFG      0x44
#define LSM6DSOX_EMB_STEP_COUNTER_L     0x62
#define LSM6DSOX_EMB_FUNC_SRC           0x64
#define   LSM6DSOX_EMB_PEDO_RST_STEP    0x80
#define LSM6DSOX_EMB_FUNC_INIT_A        0x66
