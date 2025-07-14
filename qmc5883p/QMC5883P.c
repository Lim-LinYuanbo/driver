#include "QMC5883P.h"
#include <string.h>

/* 初始化QMC5883P设备 */
void qmc5883p_init(qmc5883p_dev_t *dev, 
                   uint8_t device_address,
                   qmc5883p_osr_t osr,
                   qmc5883p_range_t range,
                   qmc5883p_dr_t data_rate,
                   qmc5883p_i2c_t *i2c,
                   qmc5883p_schedule_t *schedule,
                   qmc5883p_data_callback_t callback,
                   void *user_data)
{
    memset(dev, 0, sizeof(qmc5883p_dev_t));
    
    dev->device_address = device_address;
    dev->osr = osr;
    dev->range = range;
    dev->data_rate = data_rate;
    dev->state = QMC5883P_STATE_INIT;
    
    /* 配置寄存器值 */
    dev->config1 = osr | range | data_rate | QMC5883P_CONFIG1_MODE_CONT;
    dev->config2 = QMC5883P_CONFIG2_INT_ENB | QMC5883P_CONFIG2_ROL_PNT;
    
    /* 设置函数指针 */
    dev->i2c = i2c;
    dev->schedule = schedule;
    dev->callback = callback;
    dev->callback_user_data = user_data;
}

static int qmc5883p_i2c_write_reg(void *i2c_handle, uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t data_buf[2] = {reg, value};
    ret = dev->i2c_write(dev->i2c.i2c_handle, dev->device_address, data_buf, 2);
    if (ret != 0) {
        return ret;
    }
    return 0;
}

/* 设置QMC5883P配置 */
int qmc5883p_set_config(qmc5883p_dev_t *dev)
{
    
    int ret = 0;
    
    /* 配置寄存器1 */
    ret = qmc5883p_i2c_write_reg(dev->i2c.i2c_handle, dev->device_address, QMC5883P_ADDR_CONFIG1, dev->config1);
    if (ret != 0) {
        return ret;
    }
    
    /* 配置寄存器2 */
    ret = qmc5883p_i2c_write_reg(dev->i2c.i2c_handle, dev->device_address, QMC5883P_ADDR_CONFIG2, dev->config2);
    if (ret != 0) {
        return ret;
    }
    
    return 0;
}

/* 启动QMC5883P设备 */
int qmc5883p_start(qmc5883p_dev_t *dev)
{
    dev->state = QMC5883P_STATE_CHECK_ID;
    
    /* 调度第一次运行 */
    if (dev->schedule.delay_ms) {
        dev->schedule.delay_ms(dev->schedule.schedule_arg, 1);
    }
    
    return 0;
}

/* 读取状态寄存器 */
static int read_status(qmc5883p_dev_t *dev, uint8_t *status)
{
    return dev->i2c_read(dev->device_address, QMC5883P_ADDR_STATUS, status, 1);
}

/* 读取磁场和温度数据 */
static int read_data(qmc5883p_dev_t *dev)
{
    uint8_t buffer[8];
    int ret;
    
    /* 读取数据寄存器 */
    ret = dev->i2c_read(dev->device_address, QMC5883P_ADDR_DATA_X_LSB, buffer, 6);
    if (ret != 0) {
        return ret;
    }
    
    /* 解析磁场数据 */
    dev->x = (int16_t)((buffer[1] << 8) | buffer[0]);
    dev->y = (int16_t)((buffer[3] << 8) | buffer[2]);
    dev->z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    /* 检查溢出标志 */
    uint8_t status;
    ret = read_status(dev, &status);
    if (ret == 0) {
        dev->overflow = (status & QMC5883P_STATUS_OVL) != 0;
        if (dev->overflow) {
            dev->x = QMC5883P_OVERFLOW_FLAG;
            dev->y = QMC5883P_OVERFLOW_FLAG;
            dev->z = QMC5883P_OVERFLOW_FLAG;
        }
    }
    
    /* 读取温度数据 */
    ret = dev->i2c_read(dev->device_address, QMC5883P_ADDR_TEMP_LSB, buffer, 2);
    if (ret == 0) {
        dev->temperature = (int16_t)((buffer[1] << 8) | buffer[0]);
        /* 温度转换公式: T = (raw_temp / 256) + 20 */
    }
    
    dev->data_ready = true;
    
    return 0;
}

/* 运行QMC5883P状态机 */
void qmc5883p_run(qmc5883p_dev_t *dev)
{
    int ret = 0;
    uint8_t chip_id = 0;
    uint8_t config = 0;
    
    switch (dev->state) {
        case QMC5883P_STATE_INIT:
            /* 初始化已在qmc5883p_init中完成 */
            dev->state = QMC5883P_STATE_CHECK_ID;
            if (dev->schedule.delay_ms) {
                dev->schedule.delay_ms(dev->schedule.schedule_arg, 1);
            }
            break;
            
        case QMC5883P_STATE_CHECK_ID:
            /* 读取芯片ID */
            ret = dev->i2c_read(dev->device_address, QMC5883P_ADDR_CHIP_ID, &chip_id, 1);
            if (ret != 0 || chip_id != QMC5883P_CHIP_ID) {
                dev->state = QMC5883P_STATE_ERROR;
                if (dev->schedule.delay_ms) {
                    dev->schedule.delay_ms(dev->schedule.schedule_arg, 1000); /* 重试间隔1秒 */
                }
                break;
            }
            
            dev->state = QMC5883P_STATE_RESET;
            if (dev->schedule.delay_ms) {
                dev->schedule.delay_ms(dev->schedule.schedule_arg, 1);
            }
            break;
            
        case QMC5883P_STATE_RESET:
            /* 软件复位 */
            ret = qmc5883p_i2c_write_reg(dev->device_address, QMC5883P_ADDR_CONFIG2, QMC5883P_CONFIG2_SOFT_RST);
            if (ret != 0) {
                dev->state = QMC5883P_STATE_ERROR;
                if (dev->schedule.delay_ms) {
                    dev->schedule.delay_ms(dev->schedule.schedule_arg, 1000);
                }
                break;
            }
            
            dev->reset_count = 0;
            dev->state = QMC5883P_STATE_WAIT_RESET;
            if (dev->schedule.delay_ms) {
                dev->schedule.delay_ms(dev->schedule.schedule_arg, 5); /* 等待复位完成 */
            }
            break;
            
        case QMC5883P_STATE_WAIT_RESET:
            /* 等待复位完成 */
            dev->reset_count++;
            if (dev->reset_count > 3) {
                dev->state = QMC5883P_STATE_CONFIGURE;
                if (dev->schedule.delay_ms) {
                    dev->schedule.delay_ms(dev->schedule.schedule_arg, 1);
                }
            } else {
                if (dev->schedule.delay_ms) {
                    dev->schedule.delay_ms(dev->schedule.schedule_arg, 5);
                }
            }
            break;
            
        case QMC5883P_STATE_CONFIGURE:
            /* 配置设备 */
            ret = qmc5883p_set_config(dev);
            if (ret != 0) {
                dev->state = QMC5883P_STATE_ERROR;
                if (dev->schedule.delay_ms) {
                    dev->schedule.delay_ms(dev->schedule.schedule_arg, 1000);
                }
                break;
            }
            
            dev->check_count = 0;
            dev->state = QMC5883P_STATE_CHECK_CONFIG;
            if (dev->schedule.delay_ms) {
                dev->schedule.delay_ms(dev->schedule.schedule_arg, 1);
            }
            break;
            
        case QMC5883P_STATE_CHECK_CONFIG:
            /* 检查配置是否成功 */
            ret = dev->i2c_read(dev->device_address, QMC5883P_ADDR_CONFIG1, &config, 1);
            if (ret != 0 || config != dev->config1) {
                dev->check_count++;
                if (dev->check_count > 3) {
                    dev->state = QMC5883P_STATE_ERROR;
                    if (dev->schedule.delay_ms) {
                        dev->schedule.delay_ms(dev->schedule.schedule_arg, 1000);
                    }
                } else {
                    dev->state = QMC5883P_STATE_CONFIGURE;
                    if (dev->schedule.delay_ms) {
                        dev->schedule.delay_ms(dev->schedule.schedule_arg, 1);
                    }
                }
                break;
            }
            
            dev->state = QMC5883P_STATE_READ;
            if (dev->schedule.delay_ms) {
                /* 根据数据速率设置读取间隔 */
                uint32_t delay = 0;
                switch (dev->data_rate) {
                    case QMC5883P_DR_10Hz:  delay = 100; break;
                    case QMC5883P_DR_50Hz:  delay = 20;  break;
                    case QMC5883P_DR_100Hz: delay = 10;  break;
                    case QMC5883P_DR_200Hz: delay = 5;   break;
                    default: delay = 100; break;
                }
                dev->schedule.delay_ms(dev->schedule.schedule_arg, delay);
            }
            break;
            
        case QMC5883P_STATE_READ:
            /* 读取数据 */
            ret = read_data(dev);
            if (ret != 0) {
                dev->state = QMC5883P_STATE_ERROR;
                if (dev->schedule.delay_ms) {
                    dev->schedule.delay_ms(dev->schedule.schedule_arg, 1000);
                }
                break;
            }
            
            /* 调用数据回调函数 */
            if (dev->callback && dev->data_ready) {
                dev->callback(dev->x, dev->y, dev->z, dev->temperature, dev->overflow, dev->callback_user_data);
                dev->data_ready = false;
            }
            
            dev->state = QMC5883P_STATE_SUCCESS;
            /* 继续读取下一组数据 */
            if (dev->schedule.delay_ms) {
                uint32_t delay = 0;
                switch (dev->data_rate) {
                    case QMC5883P_DR_10Hz:  delay = 100; break;
                    case QMC5883P_DR_50Hz:  delay = 20;  break;
                    case QMC5883P_DR_100Hz: delay = 10;  break;
                    case QMC5883P_DR_200Hz: delay = 5;   break;
                    default: delay = 100; break;
                }
                dev->schedule.delay_ms(dev->schedule.schedule_arg, delay);
            }
            break;
            
        case QMC5883P_STATE_SUCCESS:
            /* 成功状态，继续读取数据 */
            dev->state = QMC5883P_STATE_READ;
            if (dev->schedule.delay_ms) {
                uint32_t delay = 0;
                switch (dev->data_rate) {
                    case QMC5883P_DR_10Hz:  delay = 100; break;
                    case QMC5883P_DR_50Hz:  delay = 20;  break;
                    case QMC5883P_DR_100Hz: delay = 10;  break;
                    case QMC5883P_DR_200Hz: delay = 5;   break;
                    default: delay = 100; break;
                }
                dev->schedule.delay_ms(dev->schedule.schedule_arg, delay);
            }
            break;
            
        case QMC5883P_STATE_ERROR:
            /* 错误状态，尝试恢复 */
            dev->state = QMC5883P_STATE_INIT;
            if (dev->schedule.delay_ms) {
                dev->schedule.delay_ms(dev->schedule.schedule_arg, 1000);
            }
            break;
            
        default:
            dev->state = QMC5883P_STATE_ERROR;
            if (dev->schedule.delay_ms) {
                dev->schedule.delay_ms(dev->schedule.schedule_arg, 1000);
            }
            break;
    }
}

/* 自检函数 */
bool qmc5883p_self_test(qmc5883p_dev_t *dev)
{
    uint8_t chip_id = 0;
    int ret;
    
    /* 读取芯片ID */
    ret = dev->i2c_read(dev->device_address, QMC5883P_ADDR_CHIP_ID, &chip_id, 1);
    if (ret != 0 || chip_id != QMC5883P_CHIP_ID) {
        return false;
    }
    
    return true;
}    
