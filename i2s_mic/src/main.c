#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma.h>

#define I2S_DEV_NODE DT_NODELABEL(i2s)
#define FAIL -1

int main() {
    printf("Iniciando interface i2s\n");
    const struct device *i2s_dev = DEVICE_DT_GET(I2S_DEV_NODE);
    
    //printf("Iniciando interface i2s mesmo\n");
    //i2s_dev = device_get_binding("I2S_0");
    if (!i2s_dev) {
        printk("I2S device not found\n");
        return FAIL;
    }

    int ret = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
    if (ret < 0) {
        printk("Failed to start I2S: %d\n", ret);
    }
    //if (!device_is_ready(i2s_dev)) {
    //    printk("I2S device not ready\n");
    //    return FAIL;
    //}
    printk("Segue o jogo\n");
    return 0;
}
