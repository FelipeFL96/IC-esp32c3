#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void) {
    while (1) {
        printk("Olá\n");
        k_msleep(1000);
    }
}
