#ifndef UWB_ANCHOR_RANGE_CACHE_H
#define UWB_ANCHOR_RANGE_CACHE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app_runtime_config.h"
#include "freertos/FreeRTOS.h"

#define UWB_ANCHOR_RANGE_CACHE_CAPACITY \
    (APP_RUNTIME_CONFIG_MAX_ANCHORS * (APP_RUNTIME_CONFIG_MAX_ANCHORS - 1U))

struct uwb_anchor_range_entry {
    bool valid;
    uint8_t initiator_id;
    uint8_t responder_id;
    int32_t distance_mm;
    int32_t raw_distance_mm;
    uint16_t sequence;
    uint32_t slot_id;
    TickType_t updated_tick;
};

/*
 * A cache belongs to exactly one protocol runtime.  It must never be shared
 * between FlexTDOA and Passive DS-TWR: otherwise a hot protocol switch can
 * consume a range produced by the previous protocol.
 */
struct uwb_anchor_range_cache {
    struct uwb_anchor_range_entry entries[UWB_ANCHOR_RANGE_CACHE_CAPACITY];
    size_t piggyback_cursor;
};

void uwb_anchor_range_cache_reset(struct uwb_anchor_range_cache *cache);

int32_t uwb_anchor_range_cache_store(
    struct uwb_anchor_range_cache *cache, uint8_t initiator_id,
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    int32_t distance_mm, int32_t raw_distance_mm);

const struct uwb_anchor_range_entry *uwb_anchor_range_cache_get(
    struct uwb_anchor_range_cache *cache, uint8_t first_id,
    uint8_t second_id);

int32_t uwb_anchor_range_cache_get_mm(
    struct uwb_anchor_range_cache *cache, uint8_t first_id,
    uint8_t second_id);

uint32_t uwb_anchor_range_cache_get_slot_id(
    struct uwb_anchor_range_cache *cache, uint8_t first_id,
    uint8_t second_id);

bool uwb_anchor_range_cache_next_from(
    struct uwb_anchor_range_cache *cache, uint8_t source_id,
    struct uwb_anchor_range_entry *entry);

#endif
