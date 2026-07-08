#ifndef SECRETS_H
#define SECRETS_H

#define APP_WIFI_SSID "your-wifi-ssid"
#define APP_WIFI_PASSWORD "your-wifi-password"
#define APP_OTA_PASSWORD "your-ota-password"

#define APP_WIRELESS_LOG_TARGET "your-pc-ip"

/* Optional, useful when the same SSID is broadcast by multiple AP/BSSID radios. */
/* #define APP_WIFI_LOCK_BSSID 1 */
/* #define APP_WIFI_BSSID "aa:bb:cc:dd:ee:ff" */
/* #define APP_WIFI_LOCK_CHANNEL 6 */

#ifndef HOSTNAME
#define HOSTNAME "uwb-module-1"
#endif

#endif /* SECRETS_H */
