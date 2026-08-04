#include "uwb_anchor_range_cache.h"

#include <limits.h>
#include <string.h>

#include "freertos/task.h"

static struct uwb_anchor_range_entry *find_entry(
    struct uwb_anchor_range_cache *cache, uint8_t initiator_id,
    uint8_t responder_id, bool create)
{
    if (cache == NULL) {
        return NULL;
    }

    struct uwb_anchor_range_entry *free_entry = NULL;
    for (size_t index = 0U; index < UWB_ANCHOR_RANGE_CACHE_CAPACITY;
         ++index) {
        struct uwb_anchor_range_entry *entry = &cache->entries[index];
        if (entry->valid && entry->initiator_id == initiator_id &&
            entry->responder_id == responder_id) {
            return entry;
        }
        if (free_entry == NULL && !entry->valid) {
            free_entry = entry;
        }
    }

    if (!create || free_entry == NULL) {
        return NULL;
    }
    memset(free_entry, 0, sizeof(*free_entry));
    free_entry->valid = true;
    free_entry->initiator_id = initiator_id;
    free_entry->responder_id = responder_id;
    free_entry->updated_tick = xTaskGetTickCount();
    return free_entry;
}

void uwb_anchor_range_cache_reset(struct uwb_anchor_range_cache *cache)
{
    if (cache != NULL) {
        memset(cache, 0, sizeof(*cache));
    }
}

int32_t uwb_anchor_range_cache_store(
    struct uwb_anchor_range_cache *cache, uint8_t initiator_id,
    uint8_t responder_id, uint16_t sequence, uint32_t slot_id,
    int32_t distance_mm, int32_t raw_distance_mm)
{
    if (cache == NULL || initiator_id == 0U || responder_id == 0U ||
        initiator_id == responder_id || distance_mm <= 0) {
        return 0;
    }

    struct uwb_anchor_range_entry *entry = find_entry(
        cache, initiator_id, responder_id, true);
    if (entry == NULL) {
        return 0;
    }
    entry->distance_mm = distance_mm;
    entry->raw_distance_mm = raw_distance_mm;
    entry->sequence = sequence;
    entry->slot_id = slot_id;
    entry->updated_tick = xTaskGetTickCount();
    return entry->distance_mm;
}

const struct uwb_anchor_range_entry *uwb_anchor_range_cache_get(
    struct uwb_anchor_range_cache *cache, uint8_t first_id,
    uint8_t second_id)
{
    const struct uwb_anchor_range_entry *forward =
        find_entry(cache, first_id, second_id, false);
    const struct uwb_anchor_range_entry *reverse =
        find_entry(cache, second_id, first_id, false);
    if (forward == NULL) {
        return reverse;
    }
    if (reverse == NULL) {
        return forward;
    }
    return (int32_t)(forward->updated_tick - reverse->updated_tick) >= 0
               ? forward
               : reverse;
}

int32_t uwb_anchor_range_cache_get_mm(
    struct uwb_anchor_range_cache *cache, uint8_t first_id,
    uint8_t second_id)
{
    const struct uwb_anchor_range_entry *entry =
        uwb_anchor_range_cache_get(cache, first_id, second_id);
    return entry != NULL ? entry->distance_mm : 0;
}

uint32_t uwb_anchor_range_cache_get_slot_id(
    struct uwb_anchor_range_cache *cache, uint8_t first_id,
    uint8_t second_id)
{
    const struct uwb_anchor_range_entry *entry =
        uwb_anchor_range_cache_get(cache, first_id, second_id);
    return entry != NULL ? entry->slot_id : 0U;
}

bool uwb_anchor_range_cache_next_from(
    struct uwb_anchor_range_cache *cache, uint8_t source_id,
    struct uwb_anchor_range_entry *result)
{
    if (cache == NULL || source_id == 0U || result == NULL) {
        return false;
    }
    for (size_t offset = 0U; offset < UWB_ANCHOR_RANGE_CACHE_CAPACITY;
         ++offset) {
        const size_t index =
            (cache->piggyback_cursor + offset) %
            UWB_ANCHOR_RANGE_CACHE_CAPACITY;
        const struct uwb_anchor_range_entry *entry =
            &cache->entries[index];
        if (entry->valid && entry->initiator_id == source_id &&
            entry->responder_id != 0U && entry->distance_mm > 0 &&
            entry->distance_mm <= UINT16_MAX) {
            cache->piggyback_cursor =
                (index + 1U) % UWB_ANCHOR_RANGE_CACHE_CAPACITY;
            *result = *entry;
            return true;
        }
    }
    return false;
}
