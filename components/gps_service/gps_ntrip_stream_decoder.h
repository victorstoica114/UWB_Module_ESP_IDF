#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef bool (*gps_ntrip_stream_emit_fn_t)(const uint8_t *data,
                                           size_t length,
                                           void *context);

typedef enum {
    GPS_NTRIP_CHUNK_SIZE = 0,
    GPS_NTRIP_CHUNK_SIZE_LF,
    GPS_NTRIP_CHUNK_DATA,
    GPS_NTRIP_CHUNK_DATA_CR,
    GPS_NTRIP_CHUNK_DATA_LF,
    GPS_NTRIP_CHUNK_TRAILER,
    GPS_NTRIP_CHUNK_DONE,
    GPS_NTRIP_CHUNK_ERROR,
} gps_ntrip_chunk_state_t;

typedef struct {
    gps_ntrip_chunk_state_t state;
    uint32_t chunk_size;
    uint32_t remaining;
    uint8_t trailer_match;
    bool size_has_digit;
    bool size_extension;
} gps_ntrip_chunk_decoder_t;

void gps_ntrip_chunk_decoder_init(gps_ntrip_chunk_decoder_t *decoder);

bool gps_ntrip_chunk_decoder_feed(gps_ntrip_chunk_decoder_t *decoder,
                                  const uint8_t *data,
                                  size_t length,
                                  gps_ntrip_stream_emit_fn_t emit,
                                  void *context);

bool gps_ntrip_http_response_is_chunked(const char *header);
