// Part of dump1090, a Mode S message decoder for RTLrtsa devices.
//
// rtsa_stub.c: Aaronia RTSAA support
//
// Copyright (c) 2021 FlightAware LLC
// Copyright (c) 2022 g3gg0
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "dump1090.h"
#include "sdr_rtsa.h"

#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>

typedef struct {
    int16_t i;
    int16_t q;
} sample_t;

static struct {
    int rtsa_http;
    uint32_t center_frequency;
    double rtsa_filter_width;
    double sample_rate;
    double resample_remain;
    sample_t prev_sample;

    iq_convert_fn converter;
    struct converter_state *converter_state;
} rtsa_dev;

#define SOCKET int
#define INVALID_SOCKET -1


/* a simple float parser for the invariant format in the JSON string */
static double rtsa_atod_invariant(const char* str)
{
    double result = 0;
    int pos = 0;

    while (str[pos] >= '0' && str[pos] <= '9') {
        result *= 10;
        result += str[pos] - '0';
        pos++;
    }
    if (str[pos] == '.') {
        pos++;
        double multiplier = 0.1;
        while (str[pos] >= '0' && str[pos] <= '9') {
            result += (str[pos] - '0') * multiplier;
            multiplier /= 10;
            pos++;
        }
    }
    return result;
}

/* search and parse a specific field in the JSON string as double */
static double rtsa_parse_field(const char *json, const char *field)
{
    double ret = 0;
    char *match_buffer = malloc(strlen(field)+4);

    sprintf(match_buffer, "\"%s\":", field);
    uint32_t match_length = strlen(match_buffer);

    if(strlen(json) < match_length)
    {
        free(match_buffer);
        printf("JSON too short\n");
        return 0;
    }

    uint32_t max_pos = strlen(json) - match_length;

    for(uint32_t pos = 0; pos < max_pos; pos++)
    {
        if(!strncmp(&json[pos], match_buffer, match_length))
        {
            const char *value_str = &json[pos + strlen(match_buffer)];
            ret = rtsa_atod_invariant(value_str);
            break;
        }
    }

    free(match_buffer);

    return ret;
}

/* for a given filter width, determine decimation and underlying sampling rate */
static uint32_t rtsa_match_width(uint32_t width, uint32_t max_width, uint32_t rate)
{
    for(int decim = 0; decim < 11; decim++) {
        if(width == (max_width >> decim)) {
            //printf("rtsa: new rate: %d, decim: %d\n", rate, (1<<decim));
            return rate >> decim;
        }
    }

    return 0;
}

static void rtsa_parse(const char *json)
{
    double start_freq = rtsa_parse_field(json, "startFrequency");
    double end_freq = rtsa_parse_field(json, "endFrequency");
    double width = end_freq - start_freq;
    double center = start_freq + width / 2;

    /* old and new center frequency doesn't match */
    if(rtsa_dev.center_frequency != (uint32_t)center) {
        rtsa_dev.center_frequency = (uint32_t)center;
    }

    /* filter width has changed, thus we have a new sampling rate */
    if(rtsa_dev.rtsa_filter_width != width) {
        rtsa_dev.rtsa_filter_width = width;

        /* determine decimation and sampling rate from filter width (until it is present in the JSON info in an upcoming version) */
        uint32_t rate = rtsa_match_width(width, 153000000, 184319943);
        if(!rate) {
            rate = rtsa_match_width(width, 204000000, 245759924);
        }

        if(rate) {
            rtsa_dev.sample_rate = rate;
        }
    }
}

static void rtsa_callback(unsigned char *buf, uint32_t len, void *ctx)
{
    static unsigned dropped = 0;
    static uint64_t sampleCounter = 0;

    MODES_NOTUSED(ctx);

    sdrMonitor();

    if (Modes.exit) {
        return;
    }
    
    uint32_t pairs = len / 4;
    uint32_t outPos = 0;
    sample_t *iqBuf = (sample_t*)buf;
    double ratio = rtsa_dev.sample_rate / Modes.sample_rate;
    double inPos = rtsa_dev.resample_remain;

    /* resampling within the same buffer only works if we are downsampling */
    if (pairs < 1 || ratio < 1.0f) {
        return;
    }

    while(inPos < pairs)
    {
        /* keep the integer position we are interpolating at and also the relative position inbetween */
        uint32_t inPosBase = inPos;
        double w = inPos - inPosBase;

        /* capture for both I and Q the samples to interpolate inbetween */
        sample_t before = (inPosBase > 0) ? iqBuf[inPosBase - 1] : rtsa_dev.prev_sample;
        sample_t after = iqBuf[inPosBase];
        
        iqBuf[outPos].i = before.i * (1.0f - w) + after.i * w;
        iqBuf[outPos].q = before.q * (1.0f - w) + after.q * w;

        /* advance input by the ratio and output by one */
        inPos += ratio;
        outPos++;
    }

    /* capture the last value of this block, it will be the next block's first value */
    rtsa_dev.prev_sample = iqBuf[pairs - 1];
    /* keep the ratio for the next block */
    rtsa_dev.resample_remain = inPos - pairs;
    /* from now on use the new, resampled length */
    len = outPos * 4;

    unsigned samples_read = len / 2;
    if (!samples_read)
        return; // that wasn't useful

    struct mag_buf *outbuf = fifo_acquire(0 /* don't wait */);
    if (!outbuf) {
        // FIFO is full. Drop this block.
        dropped += samples_read;
        sampleCounter += samples_read;
        return;
    }

    outbuf->flags = 0;

    if (dropped) {
        // We previously dropped some samples due to no buffers being available
        outbuf->flags |= MAGBUF_DISCONTINUOUS;
        outbuf->dropped = dropped;
    }

    dropped = 0;

    // Compute the sample timestamp and system timestamp for the start of the block
    outbuf->sampleTimestamp = sampleCounter * 12e6 / Modes.sample_rate;
    sampleCounter += samples_read;

    // Get the approx system time for the start of this block
    uint64_t block_duration = 1e3 * samples_read / Modes.sample_rate;
    outbuf->sysTimestamp = mstime() - block_duration;

    // Convert the new data
    unsigned to_convert = samples_read;
    if (to_convert + outbuf->overlap > outbuf->totalLength) {
        // how did that happen?
        to_convert = outbuf->totalLength - outbuf->overlap;
        dropped = samples_read - to_convert;
    }

    rtsa_dev.converter(buf, &outbuf->data[outbuf->overlap], to_convert, rtsa_dev.converter_state, &outbuf->mean_level, &outbuf->mean_power);
    outbuf->validLength = outbuf->overlap + to_convert;

    // Push to the demodulation thread
    fifo_enqueue(outbuf);
}


void rtsaInitConfig()
{
    rtsa_dev.rtsa_http = -1;
    rtsa_dev.resample_remain = 0;
    rtsa_dev.prev_sample.i = 0;
    rtsa_dev.prev_sample.q = 0;
    rtsa_dev.center_frequency = 0;
    rtsa_dev.rtsa_filter_width = 0;
    rtsa_dev.converter = NULL;
    rtsa_dev.converter_state = NULL;
}

void rtsaShowHelp()
{
    /* nothing */
}

bool rtsaHandleOption(int argc, char **argv, int *jptr)
{
    MODES_NOTUSED(argc);
    MODES_NOTUSED(argv);
    MODES_NOTUSED(jptr);
    return false;
}

bool rtsaOpen()
{
    char *host = "localhost";
    char *port = "54664";

    fprintf(stderr, "rtsa_http input from %s port %s\n", host, port);

#ifdef _WIN32
    WSADATA wsa;

    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        perror("WSAStartup()");
        return -1;
    }
#endif

    struct addrinfo hints, *res, *res0;
    int ret;
    SOCKET sock;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family   = PF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = 0;
    hints.ai_flags    = AI_ADDRCONFIG;

    ret = getaddrinfo(host, port, &hints, &res0);
    if (ret) {
        fprintf(stderr, "%s\n", gai_strerror(ret));
        return -1;
    }
    sock = INVALID_SOCKET;
    for (res = res0; res; res = res->ai_next) {
        sock = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
        if (sock >= 0) {
            ret = connect(sock, res->ai_addr, res->ai_addrlen);
            if (ret == -1) {
                perror("connect");
                sock = INVALID_SOCKET;
            }
            else
                break; // success
        }
    }
    freeaddrinfo(res0);
    if (sock == INVALID_SOCKET) {
        perror("socket");
        return -1;
    }

    const char *req = "GET /stream?format=int16 HTTP/1.1\n\n";
    send(sock, req, strlen(req), 0);

    unsigned int newlines = 0;

    do {
        unsigned char ch;

        int r = recv(sock, &ch, 1, MSG_WAITALL);
        if (r <= 0)
        {
            perror("recv");
            return -1;
        }

        if(ch == '\n')
        {
            newlines++;
        }
        else if(ch == '\r')
        {
        }
        else
        {
            newlines = 0;
        }
    } while (newlines < 2);


    fprintf(stderr, "rtsa_http connected to %s:%s\n", host, port);

    rtsa_dev.rtsa_http = sock;

    rtsa_dev.converter = init_converter(INPUT_SC16,
                                      Modes.sample_rate,
                                      Modes.dc_filter,
                                      &rtsa_dev.converter_state);
    if (!rtsa_dev.converter) {
        fprintf(stderr, "rtsa: can't initialize sample converter\n");
        rtsaClose();
        return false;
    }

    return true;
}

void rtsaRun()
{
    /* reserve a buffer for receiving the HTTP chunks */
    uint8_t *http_buf = malloc(1);
    uint32_t http_buf_length = 1;

    do {
        char length_chars[16];
        uint32_t length_char_num = 0;
        uint8_t ch;

        /* initialize length string */
        length_chars[0] = 0;

        do
        {
            /* receive the HTTP chunk length in hex */
            int ret = recv(rtsa_dev.rtsa_http, &ch, 1, MSG_WAITALL);
            
            if (ret == 0 || length_char_num >= sizeof(length_chars)) {
                perror("rtsa_http");
                break;
            }

            /* skip the first CR/LFs and stop at the second newline */
            if(ch == '\n') {
                if(length_char_num > 2) {
                    break;
                }
            }
            else if(ch == '\r') {
            }
            else {
                /* all characters inbetween are captured */
                length_chars[length_char_num++] = ch;
                length_chars[length_char_num] = 0;
            }
        } while(Modes.exit == 0);

        if(Modes.exit != 0)
        {
            break;
        }

        /* parse the chunk size */
        uint32_t http_block_length = strtoul((const char*)length_chars, NULL, 16);

        /* make sure the HTTP chunk buffer is big enough */
        if(http_buf_length < http_block_length) {
            http_buf = realloc(http_buf, http_block_length);
            http_buf_length = http_block_length;
        }

        /* receive the whole chunk, mixed JSON and binary data */
        int r = recv(rtsa_dev.rtsa_http, http_buf, http_block_length, MSG_WAITALL);
        if (r <= 0) {
            fprintf(stderr, "failed to read\n");
            break;
        }

        /* determine JSON length by separator char 0x1E */
        uint32_t json_len = 0;

        while(http_buf[json_len] != 0x1E && json_len < http_block_length) {
            json_len++;
        }

        if(http_buf[json_len] != 0x1E) {
            fprintf(stderr, "failed to detect JSON\n");
            break;
        }

        http_buf[json_len++] = 0;

        /* parse JSON */
        rtsa_parse((const char*)http_buf);

        /* process binary buffer */
        uint32_t http_payload = http_block_length - json_len;

        /* make sure we have it an an aligned buffer */
        rtsa_callback(&http_buf[json_len], http_payload, NULL);

    } while (Modes.exit == 0);

    free(http_buf);
}

void rtsaClose()
{
    int ret = shutdown(rtsa_dev.rtsa_http, SHUT_RDWR);
    if (ret == -1) {
        perror("shutdown");
    }

    ret = close(rtsa_dev.rtsa_http);
    if (ret == -1) {
        perror("close");
    }
}

