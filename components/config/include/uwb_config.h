#ifndef UWB_CONFIG_H
#define UWB_CONFIG_H

/*
 * UWB module behavior, IDs, and ranging defaults live here. Board wiring
 * remains in board_config.h; the selected firmware runtime stays in
 * app_config.h.
 */

#define APP_UWB_ROLE_UNSET 0
#define APP_UWB_ROLE_ANCHOR 1
#define APP_UWB_ROLE_TAG 2
#define APP_UWB_ROLE_DISTANCE_TEST_NODE 3

#ifndef APP_UWB_ROLE
#define APP_UWB_ROLE APP_UWB_ROLE_UNSET
#endif

/* 0 = derive from HOSTNAME suffix, then Wi-Fi MAC fallback. */
#ifndef APP_UWB_SOURCE_ID
#define APP_UWB_SOURCE_ID 0
#endif

#ifndef APP_UWB_TAG_ID
#define APP_UWB_TAG_ID 100
#endif

#ifndef APP_UWB_ANCHOR_COUNT
#define APP_UWB_ANCHOR_COUNT 4
#endif

#ifndef APP_UWB_ANCHOR_0_ID
#define APP_UWB_ANCHOR_0_ID 1
#endif

#ifndef APP_UWB_ANCHOR_1_ID
#define APP_UWB_ANCHOR_1_ID 2
#endif

#ifndef APP_UWB_ANCHOR_2_ID
#define APP_UWB_ANCHOR_2_ID 3
#endif

#ifndef APP_UWB_ANCHOR_3_ID
#define APP_UWB_ANCHOR_3_ID 4
#endif

#ifndef APP_UWB_ANTENNA_DELAY_DEFAULT
#define APP_UWB_ANTENNA_DELAY_DEFAULT 0x3FCA
#endif

#ifndef APP_UWB_DW_LEDS_ENABLED
#define APP_UWB_DW_LEDS_ENABLED 1
#endif

#ifndef APP_UWB_DW_LEDS_INIT_BLINK
#define APP_UWB_DW_LEDS_INIT_BLINK 1
#endif

/* DW3000 LED blink time register value. 0x10 is the decadriver default. */
#ifndef APP_UWB_DW_LEDS_BLINK_TIME
#define APP_UWB_DW_LEDS_BLINK_TIME 0x10
#endif

#ifndef APP_UWB_BEACON_ENABLED
#define APP_UWB_BEACON_ENABLED 1
#endif

#ifndef APP_UWB_BEACON_MIN_INTERVAL_MS
#define APP_UWB_BEACON_MIN_INTERVAL_MS 700
#endif

#ifndef APP_UWB_BEACON_MAX_INTERVAL_MS
#define APP_UWB_BEACON_MAX_INTERVAL_MS 1900
#endif

#ifndef APP_UWB_DISTANCE_TEST_PEER_ID
#define APP_UWB_DISTANCE_TEST_PEER_ID 0
#endif

#endif /* UWB_CONFIG_H */
