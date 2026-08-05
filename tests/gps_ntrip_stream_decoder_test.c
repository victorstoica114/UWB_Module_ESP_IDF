#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "../components/gps_service/gps_ntrip_stream_decoder.h"

typedef struct {
    uint8_t bytes[256];
    size_t length;
} output_t;

static bool append_output(const uint8_t *data, size_t length, void *context)
{
    output_t *output = context;
    if (output->length + length > sizeof(output->bytes)) {
        return false;
    }
    memcpy(output->bytes + output->length, data, length);
    output->length += length;
    return true;
}

static void test_headers(void)
{
    assert(gps_ntrip_http_response_is_chunked(
        "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n"));
    assert(gps_ntrip_http_response_is_chunked(
        "HTTP/1.1 200 OK\r\ntransfer-encoding: gzip, Chunked\r\n\r\n"));
    assert(!gps_ntrip_http_response_is_chunked(
        "HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\n"));
}

static void test_all_split_boundaries(void)
{
    static const uint8_t encoded[] =
        "4\r\n\xD3\x00\x01\x02\r\n"
        "3;source=test\r\n\x03\x04\x05\r\n"
        "2\r\n\x06\x07\r\n"
        "0\r\nAudit: complete\r\n\r\n";
    static const uint8_t expected[] = {
        0xD3, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    };

    for (size_t split = 1; split <= sizeof(encoded) - 1U; ++split) {
        gps_ntrip_chunk_decoder_t decoder;
        output_t output = {0};
        gps_ntrip_chunk_decoder_init(&decoder);
        size_t offset = 0;
        while (offset < sizeof(encoded) - 1U) {
            size_t length = sizeof(encoded) - 1U - offset;
            if (length > split) {
                length = split;
            }
            assert(gps_ntrip_chunk_decoder_feed(
                &decoder, encoded + offset, length, append_output, &output));
            offset += length;
        }
        assert(decoder.state == GPS_NTRIP_CHUNK_DONE);
        assert(output.length == sizeof(expected));
        assert(memcmp(output.bytes, expected, sizeof(expected)) == 0);
    }
}

static void test_invalid_stream(void)
{
    static const uint8_t invalid[] = "Z\r\nabc\r\n";
    gps_ntrip_chunk_decoder_t decoder;
    output_t output = {0};
    gps_ntrip_chunk_decoder_init(&decoder);
    assert(!gps_ntrip_chunk_decoder_feed(
        &decoder, invalid, sizeof(invalid) - 1U, append_output, &output));
    assert(decoder.state == GPS_NTRIP_CHUNK_ERROR);
    assert(output.length == 0);
}

static void test_empty_trailer(void)
{
    static const uint8_t encoded[] = "1\r\nX\r\n0\r\n\r\n";
    gps_ntrip_chunk_decoder_t decoder;
    output_t output = {0};
    gps_ntrip_chunk_decoder_init(&decoder);
    assert(gps_ntrip_chunk_decoder_feed(
        &decoder, encoded, sizeof(encoded) - 1U, append_output, &output));
    assert(decoder.state == GPS_NTRIP_CHUNK_DONE);
    assert(output.length == 1 && output.bytes[0] == 'X');
}

int main(void)
{
    test_headers();
    test_all_split_boundaries();
    test_empty_trailer();
    test_invalid_stream();
    puts("gps_ntrip_stream_decoder_test: PASS");
    return 0;
}
