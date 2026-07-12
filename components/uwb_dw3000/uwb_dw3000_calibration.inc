/* UWB antenna-delay calibration helpers.
 *
 * This file is included by uwb_dw3000.c after the common DW3000
 * frame/ranging helpers are defined, so the calibration code can stay
 * close to those static internals without exporting a broad private API.
 */

#define UWB_CALIBRATION_CMD_INITIATOR_OFFSET 10U
#define UWB_CALIBRATION_CMD_RESPONDER_OFFSET 11U
#define UWB_CALIBRATION_CMD_SLOT_OFFSET 12U
#define UWB_CALIBRATION_THREE_PAIR_COUNT 6U

struct uwb_calibration_stats {
    uint32_t samples;
    double mean_m;
    double m2_m;
    double min_m;
    double max_m;
    double last_m;
};

static const char *uwb_calibration_method_name(uint8_t method)
{
    switch (method) {
    case APP_UWB_CALIBRATION_METHOD_TWO_MODULE:
        return "two_module";
    case APP_UWB_CALIBRATION_METHOD_THREE_MODULE:
        return "three_module_edm";
    default:
        return "unknown";
    }
}

static double uwb_calibration_known_distance_for_pair_m(uint8_t first_id,
                                                       uint8_t second_id)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint8_t id0 = config->calibration_three_ids[0];
    const uint8_t id1 = config->calibration_three_ids[1];
    const uint8_t id2 = config->calibration_three_ids[2];
    const uint32_t fallback_mm = config->calibration_known_distance_mm;
    uint32_t distance_mm = fallback_mm;

    if ((first_id == id0 && second_id == id1) ||
        (first_id == id1 && second_id == id0)) {
        distance_mm = config->calibration_three_distance_0_1_mm;
    } else if ((first_id == id0 && second_id == id2) ||
               (first_id == id2 && second_id == id0)) {
        distance_mm = config->calibration_three_distance_0_2_mm;
    } else if ((first_id == id1 && second_id == id2) ||
               (first_id == id2 && second_id == id1)) {
        distance_mm = config->calibration_three_distance_1_2_mm;
    }

    return (double)distance_mm / 1000.0;
}

static double uwb_calibration_meters_per_dtu(void)
{
    return UWB_DW3000_TIME_UNIT_SECONDS * UWB_DW3000_SPEED_OF_LIGHT_MPS;
}

static int32_t uwb_calibration_round_to_i32(double value)
{
    return value >= 0.0 ? (int32_t)(value + 0.5)
                        : (int32_t)(value - 0.5);
}

static uint32_t uwb_calibration_random_interval_ms(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint32_t min_ms = config->calibration_min_interval_ms;
    const uint32_t max_ms = config->calibration_max_interval_ms;
    if (max_ms <= min_ms) {
        return min_ms;
    }

    return min_ms + (esp_random() % (max_ms - min_ms + 1U));
}

static void uwb_calibration_stats_update(struct uwb_calibration_stats *stats,
                                         double distance_m)
{
    if (stats->samples == 0) {
        stats->min_m = distance_m;
        stats->max_m = distance_m;
        stats->mean_m = distance_m;
        stats->m2_m = 0.0;
        stats->last_m = distance_m;
        stats->samples = 1;
        return;
    }

    stats->samples++;
    stats->last_m = distance_m;
    if (distance_m < stats->min_m) {
        stats->min_m = distance_m;
    }
    if (distance_m > stats->max_m) {
        stats->max_m = distance_m;
    }

    const double delta = distance_m - stats->mean_m;
    stats->mean_m += delta / (double)stats->samples;
    const double delta2 = distance_m - stats->mean_m;
    stats->m2_m += delta * delta2;
}

static double
uwb_calibration_stats_stddev_m(const struct uwb_calibration_stats *stats)
{
    if (stats->samples < 2) {
        return 0.0;
    }

    return sqrt(stats->m2_m / (double)(stats->samples - 1U));
}

static bool uwb_calibration_should_log_summary(uint32_t samples)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    if (samples == 0) {
        return false;
    }
    if (samples == 1 || samples == config->calibration_sample_count) {
        return true;
    }
    if (config->calibration_summary_every == 0) {
        return false;
    }
    return (samples % config->calibration_summary_every) == 0;
}

static void uwb_calibration_log_stats(
    const char *method, const struct uwb_distance_measurement *measurement,
    const struct uwb_calibration_stats *stats)
{
    const double known_m = uwb_calibration_known_distance_for_pair_m(
        measurement->initiator_id, measurement->responder_id);
    const double error_m = stats->mean_m - known_m;
    const double error_dtu = error_m / uwb_calibration_meters_per_dtu();
    const int32_t suggested_delta =
        uwb_calibration_round_to_i32(error_dtu);
    int32_t suggested_delay =
        (int32_t)s_antenna_delay + suggested_delta;
    if (suggested_delay < 0) {
        suggested_delay = 0;
    } else if (suggested_delay > 0xFFFF) {
        suggested_delay = 0xFFFF;
    }

    ESP_LOGI(TAG,
             "UWB CAL %s pair=%u->%u samples=%lu last=%.3f m mean=%.3f m std=%.3f m min=%.3f m max=%.3f m known=%.3f m error=%+.3f m error_dtu=%+.1f suggested_delta=%ld suggested_delay=0x%04lx",
             method, (unsigned)measurement->initiator_id,
             (unsigned)measurement->responder_id,
             (unsigned long)stats->samples, stats->last_m, stats->mean_m,
             uwb_calibration_stats_stddev_m(stats), stats->min_m,
             stats->max_m, known_m, error_m, error_dtu,
             (long)suggested_delta, (unsigned long)suggested_delay);
}

static void uwb_calibration_record_measurement(
    const char *method, const struct uwb_distance_measurement *measurement,
    struct uwb_calibration_stats *stats)
{
    uwb_calibration_stats_update(stats, measurement->distance_m);
    ESP_LOGI(TAG,
             "UWB CAL sample %s pair=%u->%u sample=%lu seq=%u distance=%.3f m %.1f cm tof=%.2f dtu",
             method, (unsigned)measurement->initiator_id,
             (unsigned)measurement->responder_id,
             (unsigned long)stats->samples, (unsigned)measurement->sequence,
             measurement->distance_m, measurement->distance_m * 100.0,
             measurement->tof_dtu);

    if (uwb_calibration_should_log_summary(stats->samples)) {
        uwb_calibration_log_stats(method, measurement, stats);
    }
}

static void uwb_calibration_two_module_loop(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint8_t reference_id = config->calibration_reference_id;
    const uint8_t dut_id = config->calibration_dut_id;
    struct uwb_calibration_stats stats = {0};
    uint16_t sequence = (uint16_t)(esp_random() & 0xFFFFU);

    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "UWB CAL two-module active: source_id=%u reference_id=%u dut_id=%u known=%u mm delay=0x%04x",
             (unsigned)s_source_id, (unsigned)reference_id,
             (unsigned)dut_id,
             (unsigned)config->calibration_known_distance_mm,
             (unsigned)s_antenna_delay);

    if (reference_id == 0 || dut_id == 0 || reference_id == dut_id) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "Invalid two-module calibration IDs: reference=%u dut=%u",
                 (unsigned)reference_id, (unsigned)dut_id);
        vTaskDelete(NULL);
        return;
    }

    if (s_source_id == reference_id) {
        ESP_LOGI(TAG,
                 "UWB CAL two-module role=reference initiator peer_dut=%u",
                 (unsigned)dut_id);
        while (true) {
            (void)uwb_distance_initiate_once(dut_id, sequence++, false);
            uwb_dw3000_delay_ms(uwb_calibration_random_interval_ms());
        }
    }

    if (s_source_id != dut_id) {
        ESP_LOGW(TAG,
                 "UWB CAL two-module idle: source_id=%u is neither reference nor DUT",
                 (unsigned)s_source_id);
        while (true) {
            uwb_dw3000_delay_ms(1000);
        }
    }

    ESP_LOGI(TAG,
             "UWB CAL two-module role=DUT responder reference_peer=%u",
             (unsigned)reference_id);
    while (true) {
        struct uwb_distance_frame poll = {0};
        esp_err_t err = uwb_distance_receive_matching(
            UWB_DISTANCE_FRAME_POLL, reference_id, false, 0, &poll, 1000);
        if (err == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "UWB CAL two-module POLL wait failed: %s",
                     esp_err_to_name(err));
            uwb_dw3000_delay_ms(20);
            continue;
        }

        struct uwb_distance_measurement measurement = {0};
        err = uwb_distance_respond_to_poll(&poll, &measurement);
        if (err == ESP_OK) {
            uwb_calibration_record_measurement("two_module", &measurement,
                                               &stats);
        }
    }
}

static bool uwb_calibration_id_in_three_set(uint8_t id, const uint8_t ids[3])
{
    return ids != NULL &&
           (id == ids[0] || id == ids[1] || id == ids[2]);
}

static int uwb_calibration_pair_index(uint8_t initiator_id,
                                      uint8_t responder_id,
                                      const uint8_t ids[3])
{
    if (ids == NULL) {
        return -1;
    }

    int index = 0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (i == j) {
                continue;
            }
            if (ids[i] == initiator_id && ids[j] == responder_id) {
                return index;
            }
            index++;
        }
    }

    return -1;
}

static size_t uwb_calibration_three_build_pairs(
    const uint8_t ids[3],
    struct uwb_anchor_survey_pair pairs[UWB_CALIBRATION_THREE_PAIR_COUNT])
{
    size_t pair_count = 0;
    if (ids == NULL || pairs == NULL) {
        return 0;
    }

    for (size_t i = 0; i < APP_RUNTIME_CONFIG_CAL_THREE_COUNT; ++i) {
        for (size_t j = 0; j < APP_RUNTIME_CONFIG_CAL_THREE_COUNT; ++j) {
            if (i == j) {
                continue;
            }
            if (pair_count < UWB_CALIBRATION_THREE_PAIR_COUNT) {
                pairs[pair_count].initiator_id = ids[i];
                pairs[pair_count].responder_id = ids[j];
                pair_count++;
            }
        }
    }
    return pair_count;
}

static void uwb_calibration_build_command(
    const struct uwb_anchor_survey_pair *pair, uint8_t slot_index,
    uint16_t sequence, uint8_t payload[UWB_DW3000_PAYLOAD_LEN])
{
    uwb_distance_build_frame(UWB_DISTANCE_FRAME_CAL_CMD, pair->initiator_id,
                             sequence, payload);
    payload[UWB_CALIBRATION_CMD_INITIATOR_OFFSET] = pair->initiator_id;
    payload[UWB_CALIBRATION_CMD_RESPONDER_OFFSET] = pair->responder_id;
    payload[UWB_CALIBRATION_CMD_SLOT_OFFSET] = slot_index;
}

static bool uwb_calibration_parse_command(
    const struct uwb_distance_frame *frame,
    struct uwb_anchor_survey_pair *pair, uint8_t *slot_index)
{
    if (frame == NULL || pair == NULL || slot_index == NULL ||
        frame->type != UWB_DISTANCE_FRAME_CAL_CMD ||
        frame->payload_len <= UWB_CALIBRATION_CMD_SLOT_OFFSET) {
        return false;
    }

    pair->initiator_id =
        frame->payload[UWB_CALIBRATION_CMD_INITIATOR_OFFSET];
    pair->responder_id =
        frame->payload[UWB_CALIBRATION_CMD_RESPONDER_OFFSET];
    *slot_index = frame->payload[UWB_CALIBRATION_CMD_SLOT_OFFSET];
    return pair->initiator_id != 0 && pair->responder_id != 0 &&
           pair->initiator_id != pair->responder_id;
}

static esp_err_t uwb_calibration_send_command(
    const struct uwb_anchor_survey_pair *pair, uint8_t slot_index,
    uint16_t sequence)
{
    if (pair == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload[UWB_DW3000_PAYLOAD_LEN] = {0};
    uwb_calibration_build_command(pair, slot_index, sequence, payload);
    const esp_err_t err = uwb_dw3000_send_payload(payload, sizeof(payload),
                                                  NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "UWB CAL command TX failed slot=%u seq=%u pair=%u->%u: %s",
                 (unsigned)slot_index, (unsigned)sequence,
                 (unsigned)pair->initiator_id,
                 (unsigned)pair->responder_id,
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UWB CAL command slot=%u seq=%u initiator=%u responder=%u",
             (unsigned)slot_index, (unsigned)sequence,
             (unsigned)pair->initiator_id, (unsigned)pair->responder_id);
    return ESP_OK;
}

static void uwb_calibration_three_handle_poll(
    const struct uwb_distance_frame *frame, const uint8_t ids[3],
    struct uwb_calibration_stats pair_stats[UWB_CALIBRATION_THREE_PAIR_COUNT])
{
    if (frame == NULL || frame->type != UWB_DISTANCE_FRAME_POLL ||
        !uwb_distance_destination_matches(frame->destination_id)) {
        return;
    }

    if (!uwb_calibration_id_in_three_set(frame->source_id, ids)) {
        ESP_LOGD(TAG, "Ignoring calibration poll from non-set source=%u",
                 (unsigned)frame->source_id);
        return;
    }

    struct uwb_distance_measurement measurement = {0};
    const esp_err_t err = uwb_distance_respond_to_poll(frame, &measurement);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "UWB CAL respond failed src=%u seq=%u: %s",
                 (unsigned)frame->source_id, (unsigned)frame->sequence,
                 esp_err_to_name(err));
        return;
    }

    const int pair_index = uwb_calibration_pair_index(
        measurement.initiator_id, measurement.responder_id, ids);
    if (pair_index >= 0 && pair_index < (int)UWB_CALIBRATION_THREE_PAIR_COUNT) {
        uwb_calibration_record_measurement(
            "three_module_edm", &measurement, &pair_stats[pair_index]);
    }
}

static void uwb_calibration_three_handle_command(
    const struct uwb_distance_frame *frame, uint8_t coordinator_id,
    const uint8_t ids[3])
{
    if (frame == NULL || frame->type != UWB_DISTANCE_FRAME_CAL_CMD ||
        frame->source_id != coordinator_id ||
        !uwb_distance_destination_matches(frame->destination_id)) {
        return;
    }

    struct uwb_anchor_survey_pair pair = {0};
    uint8_t slot_index = 0;
    if (!uwb_calibration_parse_command(frame, &pair, &slot_index)) {
        ESP_LOGW(TAG, "UWB CAL invalid command seq=%u",
                 (unsigned)frame->sequence);
        return;
    }

    if (pair.initiator_id != s_source_id ||
        !uwb_calibration_id_in_three_set(pair.responder_id, ids)) {
        return;
    }

    ESP_LOGI(TAG,
             "UWB CAL command accepted slot=%u seq=%u peer=%u delay=%u ms",
             (unsigned)slot_index, (unsigned)frame->sequence,
             (unsigned)pair.responder_id,
             (unsigned)APP_UWB_CALIBRATION_COMMAND_DELAY_MS);
    uwb_dw3000_delay_ms(APP_UWB_CALIBRATION_COMMAND_DELAY_MS);
    const esp_err_t err = uwb_distance_initiate_once(
        pair.responder_id, frame->sequence, false);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "UWB CAL initiated pair=%u->%u seq=%u failed: %s",
                 (unsigned)pair.initiator_id, (unsigned)pair.responder_id,
                 (unsigned)frame->sequence, esp_err_to_name(err));
    }
}

static void uwb_calibration_three_process_frame(
    const struct uwb_distance_frame *frame, uint8_t coordinator_id,
    const uint8_t ids[3],
    struct uwb_calibration_stats pair_stats[UWB_CALIBRATION_THREE_PAIR_COUNT])
{
    if (frame == NULL) {
        return;
    }

    switch (frame->type) {
    case UWB_DISTANCE_FRAME_POLL:
        uwb_calibration_three_handle_poll(frame, ids, pair_stats);
        break;
    case UWB_DISTANCE_FRAME_CAL_CMD:
        uwb_calibration_three_handle_command(frame, coordinator_id, ids);
        break;
    default:
        break;
    }
}

static void uwb_calibration_three_listen_until(
    TickType_t end_tick, uint8_t coordinator_id, const uint8_t ids[3],
    struct uwb_calibration_stats pair_stats[UWB_CALIBRATION_THREE_PAIR_COUNT])
{
    while ((int32_t)(xTaskGetTickCount() - end_tick) < 0) {
        const TickType_t now = xTaskGetTickCount();
        const uint32_t remaining_ms =
            (uint32_t)(end_tick - now) * portTICK_PERIOD_MS;
        uint32_t slice_ms =
            app_runtime_config_get()->calibration_rx_slice_ms;
        if (remaining_ms < slice_ms) {
            slice_ms = remaining_ms;
        }
        if (slice_ms == 0) {
            break;
        }

        struct uwb_distance_frame frame = {0};
        const esp_err_t err = uwb_distance_receive_next(&frame, slice_ms);
        if (err == ESP_OK) {
            uwb_calibration_three_process_frame(&frame, coordinator_id, ids,
                                                pair_stats);
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "UWB CAL deterministic listen failed: %s",
                     esp_err_to_name(err));
            uwb_dw3000_delay_ms(20);
        }
    }
}

static void uwb_calibration_three_module_loop(void)
{
    const app_runtime_config_t *config = app_runtime_config_get();
    const uint8_t ids[3] = {
        config->calibration_three_ids[0],
        config->calibration_three_ids[1],
        config->calibration_three_ids[2],
    };
    struct uwb_calibration_stats
        pair_stats[UWB_CALIBRATION_THREE_PAIR_COUNT] = {0};
    uint16_t sequence = (uint16_t)(esp_random() & 0xFFFFU);
    const uint8_t coordinator_id = ids[0];

    s_status = UWB_DW3000_STATUS_READY;
    ESP_LOGI(TAG,
             "UWB CAL three-module EDM active: source_id=%u ids=[%u,%u,%u] coordinator=%u slot=%u ms round_gap=%u ms rx_slice=%u ms known_edge=%u mm delay=0x%04x",
             (unsigned)s_source_id, (unsigned)ids[0], (unsigned)ids[1],
             (unsigned)ids[2], (unsigned)coordinator_id,
             (unsigned)config->calibration_min_interval_ms,
             (unsigned)config->calibration_max_interval_ms,
             (unsigned)config->calibration_rx_slice_ms,
             (unsigned)config->calibration_known_distance_mm,
             (unsigned)s_antenna_delay);

    if (ids[0] == 0 || ids[1] == 0 || ids[2] == 0 || ids[0] == ids[1] ||
        ids[0] == ids[2] || ids[1] == ids[2]) {
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "Invalid three-module calibration IDs");
        vTaskDelete(NULL);
        return;
    }

    if (!uwb_calibration_id_in_three_set(s_source_id, ids)) {
        ESP_LOGW(TAG,
                 "UWB CAL three-module idle: source_id=%u is not in the configured set",
                 (unsigned)s_source_id);
        while (true) {
            uwb_dw3000_delay_ms(1000);
        }
    }

    if (s_source_id != coordinator_id) {
        ESP_LOGI(TAG,
                 "UWB CAL deterministic follower active: source_id=%u coordinator=%u rx_slice=%u ms",
                 (unsigned)s_source_id, (unsigned)coordinator_id,
                 (unsigned)config->calibration_rx_slice_ms);
        while (true) {
            config = app_runtime_config_get();
            struct uwb_distance_frame frame = {0};
            const esp_err_t err =
                uwb_distance_receive_next(&frame,
                                          config->calibration_rx_slice_ms);
            if (err == ESP_OK) {
                uwb_calibration_three_process_frame(&frame, coordinator_id, ids,
                                                    pair_stats);
            } else if (err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "UWB CAL follower RX failed: %s",
                         esp_err_to_name(err));
                uwb_dw3000_delay_ms(20);
            }
        }
    }

    struct uwb_anchor_survey_pair
        pairs[UWB_CALIBRATION_THREE_PAIR_COUNT] = {0};
    const size_t pair_count = uwb_calibration_three_build_pairs(ids, pairs);
    uint32_t round = 0;
    ESP_LOGI(TAG,
             "UWB CAL deterministic coordinator active: source_id=%u pair_count=%u slot=%u ms round_gap=%u ms",
             (unsigned)s_source_id, (unsigned)pair_count,
             (unsigned)config->calibration_min_interval_ms,
             (unsigned)config->calibration_max_interval_ms);

    while (true) {
        ESP_LOGI(TAG, "UWB CAL round=%lu start", (unsigned long)round);
        for (size_t i = 0; i < pair_count; ++i) {
            config = app_runtime_config_get();
            const struct uwb_anchor_survey_pair *pair = &pairs[i];
            const TickType_t slot_end =
                xTaskGetTickCount() +
                pdMS_TO_TICKS(config->calibration_min_interval_ms);

            if (pair->initiator_id == s_source_id) {
                ESP_LOGI(TAG, "UWB CAL local slot=%u seq=%u pair=%u->%u",
                         (unsigned)i, (unsigned)sequence,
                         (unsigned)pair->initiator_id,
                         (unsigned)pair->responder_id);
                const esp_err_t err = uwb_distance_initiate_once(
                    pair->responder_id, sequence, false);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG,
                             "UWB CAL local pair=%u->%u seq=%u failed: %s",
                             (unsigned)pair->initiator_id,
                             (unsigned)pair->responder_id,
                             (unsigned)sequence, esp_err_to_name(err));
                }
            } else {
                (void)uwb_calibration_send_command(pair, (uint8_t)i,
                                                   sequence);
            }

            sequence++;
            uwb_calibration_three_listen_until(slot_end, coordinator_id, ids,
                                               pair_stats);
        }

        round++;
        ESP_LOGI(TAG, "UWB CAL round=%lu complete",
                 (unsigned long)(round - 1UL));
        uwb_dw3000_delay_ms(config->calibration_max_interval_ms);
    }
}

static void uwb_dw3000_calibration_loop(void)
{
    const uint8_t method = app_runtime_config_get()->calibration_method;
    ESP_LOGI(TAG, "UWB calibration runtime start: method=%s(%u)",
             uwb_calibration_method_name(method), (unsigned)method);

    switch (method) {
    case APP_UWB_CALIBRATION_METHOD_TWO_MODULE:
        uwb_calibration_two_module_loop();
        break;
    case APP_UWB_CALIBRATION_METHOD_THREE_MODULE:
        uwb_calibration_three_module_loop();
        break;
    default:
        s_status = UWB_DW3000_STATUS_FAILED;
        ESP_LOGE(TAG, "Unsupported UWB calibration method: %u",
                 (unsigned)method);
        vTaskDelete(NULL);
        break;
    }
}

