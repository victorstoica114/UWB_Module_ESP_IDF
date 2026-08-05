#ifndef SECRETS_H
#define SECRETS_H

#define APP_WIFI_SSID "your-wifi-ssid"
#define APP_WIFI_PASSWORD "your-wifi-password"
#define APP_OTA_PASSWORD "your-ota-password"

/* Optional NTRIP v2 correction source. Plain HTTP casters use TLS=0. */
#define NTRIP_ENABLED 0
#define NTRIP_MODULE_ID 1
#define NTRIP_USE_TLS 1
#define NTRIP_GGA_INTERVAL_MS 10000
#define NTRIP_SERVER "caster.example.com"
#define NTRIP_PORT 2101
#define NTRIP_MOUNTPOINT "MOUNTPOINT"
#define NTRIP_USERNAME "your-ntrip-username"
#define NTRIP_PASSWORD "your-ntrip-password"

/* Put other non-secret settings in components/config/include/app_config.h. */

#endif /* SECRETS_H */
