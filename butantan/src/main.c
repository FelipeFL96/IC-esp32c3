#include <stdio.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include "wifi/http.h"
#include "wifi/ping.h"
#include "wifi/wifi.h"

#define SSID "Nayce"
#define PSK "22062012"

#define SERVER_IP "192.168.0.180"
#define SERVER_PORT 8000

#define SLEEP_TIME_MS 500
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void) {
    int ret;
    bool led_state = true;
    int sock = 0;

    if (!gpio_is_ready_dt(&led)) {
        printf("GPIO is not ready\n");
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printf("GPIO pin configure failed\n");
        return 0;
    }

    wifi_init();
    bool connected = wifi_connect(SSID, PSK);
    if (!connected) {
        printk("Connection failed!\n");
        return 1;
    }

    printk("Connected to Wi-Fi!\n");
    wifi_status();

    printk("\nConnecting to HTTP Server:\n");
	struct zsock_addrinfo *res;
	nslookup(SERVER_IP, &res);
	sock = connect_socket(&res, SERVER_PORT);

    while (1) {
        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            printf("GPIO toggle failed\n");
            return 0;
        }

        if (!led_state) {
            k_msleep(500);
        }
        else {
			http_post(sock, SERVER_IP, "/connection");
			k_msleep(500);
        }
        led_state = !led_state;
    }

	zsock_close(sock);

    return 0;
}
