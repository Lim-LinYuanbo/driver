#include <rtthread.h>
#include <rtdevice.h>
#include "QMC5883P.h"

/* 定义I2C设备名称 */
#define I2C_BUS_NAME        "i2c1"
/* QMC5883P I2C地址 */
#define QMC5883P_I2C_ADDR   0x0D

/* 定义工作队列和工作 */
static rt_work_t qmc5883p_work;
static struct rt_workqueue *qmc5883p_wq = RT_NULL;

/* QMC5883P设备结构体 */
static qmc5883p_dev_t qmc5883p_dev;

/* I2C设备句柄 */
static rt_device_t i2c_dev = RT_NULL;

/* I2C读取函数实现 */
static int qmc5883p_i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len)
{
    rt_uint8_t buf[1];
    struct rt_i2c_msg msgs[2];

    buf[0] = reg;
    msgs[0].addr = addr;
    msgs[0].flags = RT_I2C_WR;
    msgs[0].buf = buf;
    msgs[0].len = 1;

    msgs[1].addr = addr;
    msgs[1].flags = RT_I2C_RD;
    msgs[1].buf = data;
    msgs[1].len = len;

    if (rt_i2c_transfer(i2c_dev, msgs, 2) == 2) {
        return 0;
    } else {
        return -1;
    }
}

/* I2C写入函数实现 */
static int qmc5883p_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, uint32_t len)
{
    rt_uint8_t *buf;
    struct rt_i2c_msg msg;

    buf = (rt_uint8_t *)rt_malloc(len + 1);
    if (buf == RT_NULL) {
        return -1;
    }

    buf[0] = reg;
    rt_memcpy(&buf[1], data, len);

    msg.addr = addr;
    msg.flags = RT_I2C_WR;
    msg.buf = buf;
    msg.len = len + 1;

    int ret = rt_i2c_transfer(i2c_dev, &msg, 1);
    rt_free(buf);

    if (ret == 1) {
        return 0;
    } else {
        return -1;
    }
}

/* I2C写入寄存器函数实现 */
static int qmc5883p_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value)
{
    rt_uint8_t buf[2];
    struct rt_i2c_msg msg;

    buf[0] = reg;
    buf[1] = value;

    msg.addr = addr;
    msg.flags = RT_I2C_WR;
    msg.buf = buf;
    msg.len = 2;

    if (rt_i2c_transfer(i2c_dev, &msg, 1) == 1) {
        return 0;
    } else {
        return -1;
    }
}

/* 延时函数实现 */
static void qmc5883p_delay_ms(uint32_t ms)
{
    rt_thread_mdelay(ms);
}

/* 调度函数实现 */
static void qmc5883p_schedule(void *arg, uint32_t delay_ms)
{
    if (delay_ms > 0) {
        rt_work_schedule_delayed(&qmc5883p_work, RT_TICK_PER_SECOND * delay_ms / 1000);
    } else {
        rt_work_submit(&qmc5883p_work);
    }
}

/* 数据回调函数 */
static void qmc5883p_data_callback(int16_t x, int16_t y, int16_t z, int16_t temp, bool overflow, void *user_data)
{
    static rt_tick_t last_tick = 0;
    rt_tick_t now = rt_tick_get();
    
    /* 计算采样率 */
    if (last_tick > 0) {
        rt_uint32_t freq = 1000 / ((now - last_tick) * 1000 / RT_TICK_PER_SECOND);
        rt_kprintf("QMC5883P Sampling frequency: %d Hz\n", freq);
    }
    last_tick = now;
    
    /* 打印磁场数据 */
    rt_kprintf("Magnetic field: X=%d, Y=%d, Z=%d", x, y, z);
    if (overflow) {
        rt_kprintf(" (overflow!)\n");
    } else {
        rt_kprintf("\n");
    }
    
    /* 打印温度数据 */
    if (temp != 0) {
        float temperature = (float)temp / 256.0f + 20.0f;
        rt_kprintf("Temperature: %.2f C\n", temperature);
    }
}

/* 工作处理函数 */
static void qmc5883p_work_handler(struct rt_work *work)
{
    qmc5883p_run(&qmc5883p_dev);
}

/* QMC5883P初始化函数 */
static int qmc5883p_init(void)
{
    /* 查找I2C设备 */
    i2c_dev = rt_device_find(I2C_BUS_NAME);
    if (i2c_dev == RT_NULL) {
        rt_kprintf("Can't find %s device!\n", I2C_BUS_NAME);
        return -RT_ERROR;
    }
    
    /* 打开I2C设备 */
    if (rt_device_open(i2c_dev, RT_DEVICE_FLAG_RDWR) != 0) {
        rt_kprintf("Failed to open %s device!\n", I2C_BUS_NAME);
        return -RT_ERROR;
    }
    
    /* 创建工作队列 */
    qmc5883p_wq = rt_workqueue_create("qmc5883p_wq", 1024, RT_THREAD_PRIORITY_MAX / 2);
    if (qmc5883p_wq == RT_NULL) {
        rt_kprintf("Failed to create workqueue!\n");
        rt_device_close(i2c_dev);
        return -RT_ERROR;
    }
    
    /* 初始化工作 */
    rt_work_init(&qmc5883p_work, qmc5883p_work_handler);
    rt_work_set_workqueue(&qmc5883p_work, qmc5883p_wq);
    
    /* 初始化QMC5883P设备 */
    qmc5883p_init(&qmc5883p_dev,
                  QMC5883P_I2C_ADDR,
                  QMC5883P_OSR_512,      /* 过采样率512 */
                  QMC5883P_RANGE_2G,      /* 量程2G */
                  QMC5883P_DR_100Hz,      /* 数据速率100Hz */
                  qmc5883p_i2c_read,
                  qmc5883p_i2c_write,
                  qmc5883p_i2c_write_reg,
                  qmc5883p_delay_ms,
                  qmc5883p_schedule,
                  &qmc5883p_work,
                  qmc5883p_data_callback,
                  RT_NULL);
    
    /* 启动QMC5883P设备 */
    if (qmc5883p_start(&qmc5883p_dev) != 0) {
        rt_kprintf("Failed to start QMC5883P device!\n");
        rt_workqueue_destroy(qmc5883p_wq);
        rt_device_close(i2c_dev);
        return -RT_ERROR;
    }
    
    rt_kprintf("QMC5883P initialized successfully!\n");
    
    return 0;
}
INIT_APP_EXPORT(qmc5883p_init);    
