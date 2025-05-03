#include <stdio.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include "wifi/http.h"
#include "wifi/ping.h"
#include "wifi/wifi.h"
#include "i2c_aht/aht.h"

#define SSID "Nayce"
#define PSK "22062012"

#define SERVER_IP "192.168.0.180"
#define SERVER_PORT 8000

#define SLEEP_TIME_MS 500
#define LED0_NODE DT_ALIAS(led0)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

struct environment {
    int humidity;
    int temperature;
};

union env_data {
    struct environment env;
    int32_t raw[2];
};

int blink_led(int on_duration_ms) {
    int ret = gpio_pin_toggle_dt(&led);
    if (ret < 0) {
        printf("GPIO toggle failed\n");
        return ret;
    }
    k_msleep(on_duration_ms);
    ret = gpio_pin_toggle_dt(&led);
    if (ret < 0) {
        printf("GPIO toggle failed\n");
        return ret;
    }
    return ret;
}

int read_temperature_and_humidity(struct environment *env) {
    int ret = 0;
    uint32_t raw_hum = 0, raw_temp = 0;

    int attempts = 0, max_attempts = 3;
    do {
        ret = aht10_read_sensor(&raw_hum, &raw_temp);
        attempts++;
    } while (ret != 0 && attempts <= max_attempts);

    if (ret != 0) {
        return ret;
    }

    env->humidity = aht10_convert_humidity(raw_hum);
    env->temperature = aht10_convert_temperature(raw_temp);

    return ret;
}

void pritv(uint8_t *data, size_t size) {
    printf("(%p) [", data);
    for (int i = 0; i < size; i++) {
        printf(" %.2X", data[i]);
    }
    printf("]\n");
}

int main(void) {
    int ret;
    int sock = 0;
    int temperature, humidity;

    if (!gpio_is_ready_dt(&led)) {
        printf("GPIO is not ready\n");
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printf("GPIO pin configure failed\n");
        return 0;
    }

    ret = aht10_init();
    if (ret < 0) {
        printf("AHT10 intialization failed\n");
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

    http_post(sock, SERVER_IP, "/connection", NULL, 0);
    k_msleep(500);
    printk("\n");
    union env_data data;
    while (1) {
        ret = read_temperature_and_humidity(&(data.env));
        if (ret != 0) {
            printk(AHT10_NAME"Read failed. Resetting sensor\n");
            aht10_soft_reset();
            aht10_init();
            continue;
        }

        blink_led(500);
        k_msleep(100);
        blink_led(500);

        printf("Temperatura: %dºC, Umidade: %d%%\n\n", data.env.temperature, data.env.humidity);

        printf("Dados: %x %x\n", data.raw[0], data.raw[1]);

        //uint8_t payload[8];
        //memcpy(payload, data.raw, 8);

        char *payload[50];
        //payload[14] = 0;
        sprintf(payload, "t=%d&h=%d", data.env.temperature, data.env.humidity);
        int length = strlen(payload);
        printf("PAYLOAD: (%d)%s\n", length, payload);
        pritv(payload, 14);

        http_post(sock, SERVER_IP, "/environment", payload, length);
        k_msleep(900);
    }

	zsock_close(sock);

    return 0;
}
