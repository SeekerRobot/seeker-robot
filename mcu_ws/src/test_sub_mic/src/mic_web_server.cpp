/**
 * @file test_sub_mic/src/mic_web_server.cpp
 * @author Tal Avital
 * @date 4/5/2026
 * @brief ESP32 Sense Mic web server
 */

#include "mic_web_server.h"
#include <Arduino.h>
#include <I2S.h>
#include <esp_http_server.h>

// Buffer size for reading from I2S and sending over HTTP
#define AUDIO_CHUNK_SIZE 1024 

httpd_handle_t stream_httpd = NULL;

// Standard WAV header struct. 
// __attribute__((packed)) ensures the compiler doesn't add padding.
struct __attribute__((packed)) WavHeader {
    char riff[4] = {'R', 'I', 'F', 'F'};
    uint32_t file_size = 0xFFFFFFFF;     // Unknown size (Streaming)
    char wave[4] = {'W', 'A', 'V', 'E'};
    char fmt[4] = {'f', 'm', 't', ' '};
    uint32_t fmt_size = 16;
    uint16_t audio_format = 1;           // PCM
    uint16_t num_channels = 1;           // Mono
    uint32_t sample_rate = 16000;        // 16 kHz (Matches I2S.begin)
    uint32_t byte_rate = 16000 * 2;      // sample_rate * num_channels * bytes_per_sample
    uint16_t block_align = 2;            // num_channels * bytes_per_sample
    uint16_t bits_per_sample = 16;       // 16-bit
    char data[4] = {'d', 'a', 't', 'a'};
    uint32_t data_size = 0xFFFFFFFF;     // Unknown size (Streaming)
};

static esp_err_t stream_handler(httpd_req_t* req) {
    esp_err_t res = ESP_OK;
    uint8_t audio_buf[AUDIO_CHUNK_SIZE];

    // 1. Set the content type to WAV audio
    res = httpd_resp_set_type(req, "audio/wav");
    if (res != ESP_OK) return res;

    // 2. Send the WAV header as the very first chunk
    WavHeader header;
    res = httpd_resp_send_chunk(req, (const char*)&header, sizeof(WavHeader));
    if (res != ESP_OK) return res;

    Serial.println("Audio stream connected!");

    // 3. Continuously read from I2S and stream to the client
    while (true) {
        // Read block of data from the I2S microphone
        // Note: Using Stream's readBytes is much faster than reading sample-by-sample
        size_t bytes_read = I2S.readBytes((char*)audio_buf, AUDIO_CHUNK_SIZE);

        if (bytes_read > 0) {
            // Push the chunk to the HTTP client
            res = httpd_resp_send_chunk(req, (const char*)audio_buf, bytes_read);
        } else {
            // A small delay to yield if no data was immediately available
            delay(10); 
        }

        // If the client disconnects, httpd_resp_send_chunk returns ESP_FAIL
        if (res != ESP_OK) {
            Serial.println("Audio stream disconnected.");
            break;
        }
    }

    // Send empty chunk to cleanly close the HTTP chunked transfer
    httpd_resp_send_chunk(req, NULL, 0);
    return res;
}

void startMicWebServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    
    // Optional: Increase the timeout for stable audio streaming over poor WiFi
    // config.send_wait_timeout = 10; 

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = stream_handler,
        .user_ctx = NULL
    };

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        Serial.println("Audio Stream Server started on port 80");
        Serial.println("Connect via: http://<IP_ADDRESS>/stream");
    }
}