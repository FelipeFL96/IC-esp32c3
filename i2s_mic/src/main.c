#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/drivers/dma.h>

#define I2S_DEV_NODE DT_NODELABEL(I2S)
#define FAIL -1

#define MEM_SLAB_NUM_BLOCKS 4
#define MEM_SLAB_BLOCK_SIZE 1024
struct k_mem_slab rx_mem_slab;
K_MEM_SLAB_DEFINE(rx_mem_slab, MEM_SLAB_BLOCK_SIZE, MEM_SLAB_NUM_BLOCKS, 4);

int main() {
    int ret;
    printf("Iniciando interface i2s\n");
    //const struct device *i2s_dev = DEVICE_DT_GET(I2S_DEV_NODE);
    const struct device *i2s_dev = device_get_binding("I2S");
    
    //printf("Iniciando interface i2s mesmo\n");
    //i2s_dev = device_get_binding("I2S_0");
    if (!device_is_ready(i2s_dev)) {
        printk("I2S device not ready\n");
        return FAIL;
    }

    if (!i2s_dev) {
        printk("I2S device not found\n");
        return FAIL;
    }
    printk("DEVICE INITIALIZED SUCCESFULLY\n");

    k_msleep(500);

    printf("i2s: %p\n", i2s_dev);
    printf("name: %s\n", i2s_dev->name);
    printf("init_res: %d\n", i2s_dev->state->init_res);
    printf("initialized: %d\n", i2s_dev->state->initialized);

    //uint32_t *i2s_conf_reg = (uint32_t*) 0x6002D000;
    //printf("i2s_conf_reg = 0x%08X\n", *i2s_conf_reg);
    //
    //uint32_t *clk_reg = (uint32_t *)0x6002D004;
    //printk("I2S_CLKM_CONF_REG: 0x%08X\n", *clk_reg);
    ////printf("i2s_conf_reg = 0x%08X\n", i2s_conf_reg);

    uint32_t *i2s_conf_reg = (uint32_t*) 0x6002D000;
    uint8_t reg[16];
    printk("Registradores i2s:\n");
    for (unsigned i = 0; i < 15; i++) {
        uint32_t *base_addr = (i2s_conf_reg+(4*i));
        memcpy(reg, base_addr, 16);
        printk("(%p)\t", base_addr);
        for (unsigned u = 0; u < 16; u++) {
            if (u != 0 && u%4 == 0) printk(" ");
            printk("%.2X ", reg[u]);
            k_msleep(5);
        }
        printk("\n");
    }

    struct i2s_config config = {
        .word_size = 32, // Amostras de 24 bits
        .channels = 2,   // Mono
        .format = I2S_FMT_DATA_FORMAT_I2S,
        .options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER,
        .frame_clk_freq = 32000, //44100, //32000, // Clock de 32kHz
        .mem_slab = &rx_mem_slab,
        .block_size = MEM_SLAB_BLOCK_SIZE,
	    .timeout = 1000
    };

    printk("BEGIN CONFIGURATION\n");
    ret = i2s_configure(i2s_dev, I2S_DIR_RX, &config);
    printk("END CONFIGURATION\n");
    if (ret < 0) {
        printk("Failed to configure i2s: %d\n", ret);
        return FAIL;
    }
    printk("CONFIGURED SUCCESFULLY\n");


    ret = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
    if (ret < 0) {
        printk("Failed to start I2S: %d\n", ret);
    }
    
    printk("Segue o jogo\n");
    return 0;
}
