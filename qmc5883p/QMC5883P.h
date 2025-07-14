#ifndef QMC5883P_H
#define QMC5883P_H

#include <stdint.h>
#include <stdbool.h>

/* 包含寄存器定义 */
#include "QST_QMC5883P_registers.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 数据溢出标志 */
#define QMC5883P_OVERFLOW_FLAG        INT16_MAX

/* 采样率枚举 */
typedef enum {
    QMC5883P_OSR_512 = QMC5883P_CONFIG1_OSR_512,
    QMC5883P_OSR_256 = QMC5883P_CONFIG1_OSR_256,
    QMC5883P_OSR_128 = QMC5883P_CONFIG1_OSR_128,
    QMC5883P_OSR_64  = QMC5883P_CONFIG1_OSR_64
} qmc5883p_osr_t;

/* 量程枚举 */
typedef enum {
    QMC5883P_RANGE_2G = QMC5883P_CONFIG1_RANGE_2G,
    QMC5883P_RANGE_8G = QMC5883P_CONFIG1_RANGE_8G
} qmc5883p_range_t;

/* 数据速率枚举 */
typedef enum {
    QMC5883P_DR_10Hz  = QMC5883P_CONFIG1_DR_10Hz,
    QMC5883P_DR_50Hz  = QMC5883P_CONFIG1_DR_50Hz,
    QMC5883P_DR_100Hz = QMC5883P_CONFIG1_DR_100Hz,
    QMC5883P_DR_200Hz = QMC5883P_CONFIG1_DR_200Hz
} qmc5883p_dr_t;

/* 设备状态枚举 */
typedef enum {
    QMC5883P_STATE_ERROR = -1,
    QMC5883P_STATE_INIT,
    QMC5883P_STATE_CHECK_ID,
    QMC5883P_STATE_RESET,
    QMC5883P_STATE_WAIT_RESET,
    QMC5883P_STATE_CONFIGURE,
    QMC5883P_STATE_CHECK_CONFIG,
    QMC5883P_STATE_READ,
    QMC5883P_STATE_SUCCESS
} qmc5883p_state_t;

/* I2C操作函数指针类型 */
typedef int (*qmc5883p_i2c_read_t)(void *i2c_handle, uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);
typedef int (*qmc5883p_i2c_write_t)(void *i2c_handle, uint8_t addr, uint8_t reg, const uint8_t *data, uint32_t len);

/* 延时函数指针类型 */
typedef void (*qmc5883p_delay_ms_t)(void *arg, uint32_t ms);

/* 调度函数指针类型 */
typedef void (*qmc5883p_on_interval_t)(void *arg, uint32_t delay_ms);

/* 数据回调函数指针类型 */
typedef void (*qmc5883p_data_callback_t)(int16_t x, int16_t y, int16_t z, int16_t temp, bool overflow, void *user_data);

typedef struct
{
    /* 函数指针 */
    qmc5883p_i2c_read_t i2c_read;     /* I2C读函数 */
    qmc5883p_i2c_write_t i2c_write;   /* I2C写函数 */
    void *i2c_handle;
} qmc5883p_i2c_t;

typedef struct
{
    qmc5883p_delay_ms_t delay_ms;     /* 延时函数 */
    qmc5883p_on_interval_t on_interval;     /* 调度函数 */
    void *schedule_arg;               /* 调度函数参数 */
} qmc5883p_schedule_t;

/* QMC5883P设备结构体 */
typedef struct {
    uint8_t device_address;           /* I2C设备地址 */
    qmc5883p_state_t state;           /* 当前状态 */
    qmc5883p_osr_t osr;               /* 过采样率 */
    qmc5883p_range_t range;           /* 量程 */
    qmc5883p_dr_t data_rate;          /* 数据速率 */
    uint8_t config1;                  /* 配置寄存器1值 */
    uint8_t config2;                  /* 配置寄存器2值 */
    uint8_t reset_count;              /* 复位计数器 */
    uint8_t check_count;              /* 检查计数器 */
    int16_t x, y, z;                  /* 磁场数据 */
    int16_t temperature;              /* 温度数据 */
    bool data_ready;                  /* 数据准备标志 */
    bool overflow;                    /* 溢出标志 */

    qmc5883p_i2c_t *i2c;
    qmc5883p_schedule_t *schedule;

    qmc5883p_data_callback_t callback; /* 数据回调函数 */
    void *callback_user_data;         /* 回调函数用户数据 */
} qmc5883p_dev_t;

/* 初始化QMC5883P设备 */
void qmc5883p_init(qmc5883p_dev_t *dev, 
                   uint8_t device_address,
                   qmc5883p_osr_t osr,
                   qmc5883p_range_t range,
                   qmc5883p_dr_t data_rate,
                   qmc5883p_i2c_t *i2c,
                   qmc5883p_schedule_t *schedule,
                   qmc5883p_data_callback_t callback,
                   void *user_data);

/* 设置QMC5883P配置 */
int qmc5883p_set_config(qmc5883p_dev_t *dev);

/* 启动QMC5883P设备 */
int qmc5883p_start(qmc5883p_dev_t *dev);

/* 运行QMC5883P状态机 */
void qmc5883p_run(qmc5883p_dev_t *dev);

/* 自检函数 */
bool qmc5883p_self_test(qmc5883p_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* QMC5883P_H */    

