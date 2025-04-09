#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <errno.h>

#include <stdint.h>
#include <stdio.h>

#define AHT10_ADDR          0x38
#define AHT10_CMD_INIT      0xBE
#define AHT10_CMD_TRIGGER   0xAC
#define AHT10_CMD_SOFTRESET 0xBA

#define AHT10_NAME          "[aht10] "
#define AHT10_DATA_LENGTH   6

#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(i2c_0))
#define I2C_DEV_NODE	DT_ALIAS(i2c_0)
#else
#error "Please set the correct I2C device"
#endif

const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));

int init_i2c() {
    printk("Initializing i2c device\n");
    if (!device_is_ready(i2c_dev)) {
        printk("i2c device not ready\n");
        return -ENODEV;
    }
    
    uint32_t config;
    i2c_get_config(i2c_dev, &config);

    printk("i2c device ready at i2c0\n");
    printk("Speed: %d\n", I2C_SPEED_GET(config));
    printk("Clock: %d\n", DT_PROP(I2C_DEV_NODE, clock_frequency));
    return 0;
}

int read_i2c(const struct device *dev, uint8_t devaddr, uint8_t *data, size_t data_length) {
    int ret;

    if (!device_is_ready(dev)) {
        printk("Device not ready\n");
        return -ENODEV;
    }

    ret = i2c_read(dev, data, data_length,  devaddr);
    if (ret) {
        printk("Call i2c_write_read failed: %d\n", ret);
        return ret;
    }

    return 0;
}

int write_i2c(const struct device *dev, uint8_t devaddr, uint8_t command) {
	int ret;

	if (!device_is_ready(dev)) {
		printk("Device not ready\n");
		return -ENODEV;
	}

	uint8_t buf[1] = { command };

	ret = i2c_write(dev, buf, 1, devaddr);
	if (ret) {
		printk("Call `i2c_write` failed: %d\n", ret);
		return ret;
	}

	return 0;
}

void scan_i2c() {
    uint8_t addr;
    for (addr = 0x8; addr <= 0x77; addr++) {
        struct i2c_msg msgs[1] = {
            {
                .buf = NULL,
                .len = 0,
                .flags = I2C_MSG_WRITE | I2C_MSG_STOP
            }
        };
        int ret = i2c_transfer(i2c_dev, &msgs[0], 1, addr);
        if (ret == 0) {
            printk("Device found at 0x%02X\n", addr);
            return;
        }
    }
    printk("No device found\n");
}

//int aht10_soft_reset(const struct device *dev) {
//    int ret;
//    uiint8_t init_cmd[3] = {AHT10_CMD_SOFTRESET, }
//    ret = write_i2c(dev, AHT10_ADDR);
//    if (ret != 0) {
//        printk("Failed to soft reset sensor: %d\n", ret);
//        return ret;
//    }
//    k_msleep(20);
//}

int aht10_init(const struct device *dev) {
    int ret;
    uint8_t init_cmd[3] = {AHT10_CMD_INIT, 0x08, 0x00};
    ret = i2c_write(dev, init_cmd, sizeof(init_cmd), AHT10_ADDR);
    if (ret != 0) {
        printk("Failed to init sensor: %d\n", ret);
        return ret;
    }
    k_msleep(10);
    return ret;
}

int aht10_read_sensor(const struct device *dev, uint8_t *data, size_t data_length) {
    int ret;
    ret = write_i2c(dev, AHT10_ADDR, AHT10_CMD_TRIGGER);
    if (ret != 0) {
        printk("Failed to trigger sensor: %d\n", ret);
        return ret;
    }
    k_msleep(1000);

    ret = read_i2c(dev, AHT10_ADDR, data, data_length);
    if (ret != 0) {
        printk("Failed to read sensor: %d\n", ret);
        return ret;
    }
    return ret;
}

int main(void) {
    int ret = 0;
    
    ret = init_i2c();
    if (ret != 0)
        return 1;

    ret = aht10_init(i2c_dev);
    if (ret != 0)
        return 1;
    printk(AHT10_NAME"Sensor initialization completed\n");

while(1) {
    uint8_t trigger_cmd[3] = {AHT10_CMD_TRIGGER, 0x33, 0x00};
    uint8_t data[6];
    
    i2c_write(i2c_dev, trigger_cmd, sizeof(trigger_cmd), AHT10_ADDR);
    k_msleep(75);

    i2c_read(i2c_dev, data, sizeof(data), AHT10_ADDR);

    uint32_t hum_raw = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4);
    uint32_t temp_raw = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];

    int humidity = (hum_raw * 100) / (1 << 20);
    int temperature = (temp_raw * 200 / (1 << 20)) - 50;
    printk("Read: %x\n", data[0]);
    printk("Read: %x\n", data[1]);
    printk("Read: %x\n", data[2]);
    printk("Read: %x\n", data[3]);
    printk("Read: %x\n", data[4]);
    printk("Read: %x\n", data[5]);
    printk("\n");
    printf("Umidade: %d%%\n", humidity);
    printf("Temperatura: %dºC\n", temperature);
    printk("\n");
k_msleep(2000);
}
    return 0;
}

