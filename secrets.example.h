#ifndef SECRETS_H
#define SECRETS_H

#define APP_WIFI_SSID "your-wifi-ssid"
#define APP_WIFI_PASSWORD "your-wifi-password"
#define APP_OTA_PASSWORD "your-ota-password"

#define APP_WIRELESS_LOG_TARGET "your-pc-ip"
#define APP_WIRELESS_LOG_PORT 6055
#define APP_WIRELESS_LOG_ENABLED 1

#ifndef HOSTNAME
#define HOSTNAME "uwb-module-1"
#endif

#define APP_UWB_BEACON_ENABLED 1
#define APP_UWB_BEACON_MIN_INTERVAL_MS 700
#define APP_UWB_BEACON_MAX_INTERVAL_MS 1900
/* 0 = derive from HOSTNAME suffix, then Wi-Fi MAC fallback. */
#define APP_UWB_SOURCE_ID 0

#ifndef APP_WIFI_USE_WPA3
#define APP_WIFI_USE_WPA3 0
#endif

#endif /* SECRETS_H */
