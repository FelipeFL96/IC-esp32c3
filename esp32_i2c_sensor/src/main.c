#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <errno.h>

#include <stdint.h>
#include <stdio.h>

#define AHT10_ADDR          0x38
#define AHT10_CMD_INIT      0xBE
#define AHT10_CMD_TRIGGER   0xAC
#define AHT10_CMD_SOFTRESET 0xBA

#if DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(i2c_0))
#define I2C_DEV_NODE	DT_ALIAS(i2c_0)
#else
#error "Please set the correct I2C device"
#endif

static int read(struct device *dev, uint8_t devaddr, uint8_t *data, size_t data_length)
{
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

static int write(struct device *dev, uint8_t devaddr, uint8_t command)
{
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

/*void aht20_get_temp_humidity(float *temperature, float *humidity) {
    uint8_t data[6];
    aht20_write_command(AHT20_CMD_TRIGGER);
    vTaskDelay(pdMS_TO_TICKS(80)); // Wait for the measurement

    if (aht20_read_data(data, 6) == ESP_OK) {
        uint32_t raw_humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4;
        uint32_t raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];

        *humidity = ((float)raw_humidity / 1048576.0) * 100.0;
        *temperature = ((float)raw_temperature / 1048576.0) * 200.0 - 50.0;
    } else {
        *humidity = -1.0;
        *temperature = -1.0;
    }
}*/

int aht10_init(struct device *dev) {
    int ret;
    ret = write(dev, AHT10_ADDR, AHT10_CMD_SOFTRESET);
    if (ret != 0) {
        printk("Failed to soft reset sensor: %d\n", ret);
        return ret;
    }
    k_msleep(50);

    ret = write(dev, AHT10_ADDR, AHT10_CMD_INIT);    
    if (ret != 0) {
        printk("Failed to init sensor: %d\n", ret);
        return ret;
    }
    k_msleep(5);
    return ret;
}

int aht10_read_sensor(struct device *dev, uint8_t *data, size_t data_length) {
    int ret;
    ret = write(dev, AHT10_ADDR, AHT10_CMD_TRIGGER);
    if (ret != 0) {
        printk("Failed to trigger sensor: %d\n", ret);
        return ret;
    }
    k_msleep(100);

    ret = read(dev, AHT10_ADDR, data, data_length);
    if (ret != 0) {
        printk("Failed to read sensor: %d\n", ret);
        return ret;
    }
    return ret;
}


int main(void) {
    printk("Preparing i2c device\n");
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (dev == NULL || !device_is_ready(dev)) {
		printk("Device not ready\n");
		return -ENODEV;
	}
    printk("Device ready\n");

    uint32_t config;
    i2c_get_config(dev, &config);
    printk("Speed: %d\n", I2C_SPEED_GET(config));
    printk("Clock: %d\n", DT_PROP(I2C_DEV_NODE, clock_frequency));
    k_msleep(100);

    int ret;
    
    ret = aht10_init(dev);
    if (!ret) {
		printk("Failed to init aht10 sensor\n");
    }

    //ret = write(dev, AHT10_ADDR, AHT10_CMD_TRIGGER);
    //printk("Trigger: %d\n", ret);
    //k_msleep(100);

    uint8_t data[6];
    aht10_read_sensor(dev, data, 6);
    ret = read(dev, AHT10_ADDR, &data, 6);
    if (!ret) {
		printk("Failed to read from aht10 sensor\n");
    }

    printk("Read: %x\n", data[0]);
    printk("Read: %x\n", data[1]);
    printk("Read: %x\n", data[2]);
    printk("Read: %x\n", data[3]);
    printk("Read: %x\n", data[4]);
    printk("Read: %x\n", data[5]);
    printk("\n", data[5]);

    uint32_t raw_humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4;
    uint32_t raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];

    float humidity = ((float)raw_humidity / 1048576.0) * 100.0;
    float temperature = ((float)raw_temperature / 1048576.0) * 200.0 - 50.0;
    printf("humidity: %f\n", humidity);
    printf("temperature: %f\n", temperature);
    printf("humidity: %f\n", humidity);
    printf("temperature: %f\n", temperature);
    
    k_msleep(100);

    return 0;
}
