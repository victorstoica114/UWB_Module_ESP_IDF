#include "gps_ntrip_stream_decoder.h"

#include <limits.h>
#include <string.h>

static int hex_value(uint8_t byte)
{
    if (byte >= '0' && byte <= '9') {
        return byte - '0';
    }
    if (byte >= 'a' && byte <= 'f') {
        return byte - 'a' + 10;
    }
    if (byte >= 'A' && byte <= 'F') {
        return byte - 'A' + 10;
    }
    return -1;
}

static bool ascii_equal_ignore_case(const char *left, size_t left_length,
                                    const char *right)
{
    const size_t right_length = strlen(right);
    if (left_length != right_length) {
        return false;
    }
    for (size_t index = 0; index < left_length; ++index) {
        char a = left[index];
        char b = right[index];
        if (a >= 'A' && a <= 'Z') {
            a = (char)(a - 'A' + 'a');
        }
        if (b >= 'A' && b <= 'Z') {
            b = (char)(b - 'A' + 'a');
        }
        if (a != b) {
            return false;
        }
    }
    return true;
}

void gps_ntrip_chunk_decoder_init(gps_ntrip_chunk_decoder_t *decoder)
{
    if (decoder != NULL) {
        memset(decoder, 0, sizeof(*decoder));
        decoder->state = GPS_NTRIP_CHUNK_SIZE;
    }
}

static bool fail(gps_ntrip_chunk_decoder_t *decoder)
{
    decoder->state = GPS_NTRIP_CHUNK_ERROR;
    return false;
}

bool gps_ntrip_chunk_decoder_feed(gps_ntrip_chunk_decoder_t *decoder,
                                  const uint8_t *data,
                                  size_t length,
                                  gps_ntrip_stream_emit_fn_t emit,
                                  void *context)
{
    if (decoder == NULL || (data == NULL && length != 0) || emit == NULL ||
        decoder->state == GPS_NTRIP_CHUNK_ERROR) {
        return false;
    }

    size_t offset = 0;
    while (offset < length) {
        switch (decoder->state) {
        case GPS_NTRIP_CHUNK_SIZE: {
            const uint8_t byte = data[offset++];
            const int digit = hex_value(byte);
            if (!decoder->size_extension && digit >= 0) {
                if (decoder->chunk_size >
                    (UINT32_MAX - (uint32_t)digit) / 16U) {
                    return fail(decoder);
                }
                decoder->chunk_size =
                    decoder->chunk_size * 16U + (uint32_t)digit;
                decoder->size_has_digit = true;
            } else if (byte == ';' && decoder->size_has_digit &&
                       !decoder->size_extension) {
                decoder->size_extension = true;
            } else if (byte == '\r' && decoder->size_has_digit) {
                decoder->state = GPS_NTRIP_CHUNK_SIZE_LF;
            } else if (decoder->size_extension && byte != '\n') {
                /* Chunk extensions are metadata and are intentionally ignored. */
            } else {
                return fail(decoder);
            }
            break;
        }
        case GPS_NTRIP_CHUNK_SIZE_LF:
            if (data[offset++] != '\n') {
                return fail(decoder);
            }
            if (decoder->chunk_size == 0) {
                /* The zero-size line already supplied the first CRLF. */
                decoder->trailer_match = 2;
                decoder->state = GPS_NTRIP_CHUNK_TRAILER;
            } else {
                decoder->remaining = decoder->chunk_size;
                decoder->state = GPS_NTRIP_CHUNK_DATA;
            }
            break;
        case GPS_NTRIP_CHUNK_DATA: {
            const size_t available = length - offset;
            const size_t emitted = available < decoder->remaining
                                       ? available
                                       : decoder->remaining;
            if (emitted > 0 && !emit(data + offset, emitted, context)) {
                return fail(decoder);
            }
            offset += emitted;
            decoder->remaining -= (uint32_t)emitted;
            if (decoder->remaining == 0) {
                decoder->state = GPS_NTRIP_CHUNK_DATA_CR;
            }
            break;
        }
        case GPS_NTRIP_CHUNK_DATA_CR:
            if (data[offset++] != '\r') {
                return fail(decoder);
            }
            decoder->state = GPS_NTRIP_CHUNK_DATA_LF;
            break;
        case GPS_NTRIP_CHUNK_DATA_LF:
            if (data[offset++] != '\n') {
                return fail(decoder);
            }
            decoder->chunk_size = 0;
            decoder->size_has_digit = false;
            decoder->size_extension = false;
            decoder->state = GPS_NTRIP_CHUNK_SIZE;
            break;
        case GPS_NTRIP_CHUNK_TRAILER: {
            static const uint8_t terminator[] = {'\r', '\n', '\r', '\n'};
            const uint8_t byte = data[offset++];
            if (byte == terminator[decoder->trailer_match]) {
                decoder->trailer_match++;
                if (decoder->trailer_match == sizeof(terminator)) {
                    decoder->state = GPS_NTRIP_CHUNK_DONE;
                }
            } else {
                decoder->trailer_match = byte == '\r' ? 1U : 0U;
            }
            break;
        }
        case GPS_NTRIP_CHUNK_DONE:
            offset = length;
            break;
        case GPS_NTRIP_CHUNK_ERROR:
        default:
            return false;
        }
    }
    return true;
}

bool gps_ntrip_http_response_is_chunked(const char *header)
{
    if (header == NULL) {
        return false;
    }

    const char *line = header;
    while (*line != '\0') {
        const char *line_end = strstr(line, "\r\n");
        if (line_end == NULL) {
            line_end = line + strlen(line);
        }
        const char *colon = memchr(line, ':', (size_t)(line_end - line));
        if (colon != NULL &&
            ascii_equal_ignore_case(line, (size_t)(colon - line),
                                    "Transfer-Encoding")) {
            const char *token = colon + 1;
            while (token < line_end) {
                while (token < line_end &&
                       (*token == ' ' || *token == '\t' || *token == ',')) {
                    token++;
                }
                const char *token_end = token;
                while (token_end < line_end && *token_end != ',' &&
                       *token_end != ';' && *token_end != ' ' &&
                       *token_end != '\t') {
                    token_end++;
                }
                if (ascii_equal_ignore_case(
                        token, (size_t)(token_end - token), "chunked")) {
                    return true;
                }
                while (token_end < line_end && *token_end != ',') {
                    token_end++;
                }
                token = token_end;
            }
        }
        if (*line_end == '\0') {
            break;
        }
        line = line_end + 2;
    }
    return false;
}
