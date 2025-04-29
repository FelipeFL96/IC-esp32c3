#include <stdbool.h>
#include <inttypes.h>

void wifi_init(void);
bool wifi_connect(const uint8_t *ssid, const uint8_t *psk);
void wifi_disconnect(void);
void wifi_status(void);
