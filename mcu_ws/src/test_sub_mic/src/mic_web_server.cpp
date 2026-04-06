/**
 * @file test_sub_mic/src/mic_web_server.cpp
 * @author Tal Avital
 * @date 4/5/2026
 * @brief ESP32 Sense Mic web server
 */

#include "mic_web_server.h"
#include <Arduino.h>
#include <driver/i2s.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/sockets.h>

#define AUDIO_CHUNK_SIZE 2048
#define SAMPLE_RATE      16000
#define MIC_GAIN         4     // linear multiplier (1 = unity, 4 = ~12dB)

// HTML player that fetches /audio as raw PCM and plays via Web Audio API.
// This avoids the browser's native WAV buffering (which adds ~5s of delay).
static const char INDEX_HTML[] = R"html(<!DOCTYPE html>
<html>
<head><title>Mic Stream</title></head>
<body>
<button id="btn">Start Listening</button>
<p id="status"></p>
<script>
const SAMPLE_RATE = 16000;
document.getElementById('btn').onclick = async () => {
  document.getElementById('status').textContent = 'Connecting...';
  const ctx = new AudioContext({ sampleRate: SAMPLE_RATE });
  const res = await fetch('/audio');
  const reader = res.body.getReader();
  let nextTime = ctx.currentTime + 0.1;
  document.getElementById('status').textContent = 'Streaming';

  while (true) {
    const { done, value } = await reader.read();
    if (done) break;

    // value is raw 16-bit signed PCM at SAMPLE_RATE Hz, mono
    const pcm = new Int16Array(value.buffer, value.byteOffset, value.byteLength / 2);
    const f32 = new Float32Array(pcm.length);
    for (let i = 0; i < pcm.length; i++) f32[i] = pcm[i] / 32768.0;

    const buf = ctx.createBuffer(1, f32.length, SAMPLE_RATE);
    buf.copyToChannel(f32, 0);
    const src = ctx.createBufferSource();
    src.buffer = buf;
    src.connect(ctx.destination);
    // Schedule each chunk to start exactly when the previous one ends
    const t = Math.max(ctx.currentTime + 0.05, nextTime);
    src.start(t);
    nextTime = t + buf.duration;
  }

  document.getElementById('status').textContent = 'Disconnected';
};
</script>
</body>
</html>)html";

static httpd_handle_t stream_httpd = NULL;
static httpd_req_t*   g_active_req = NULL;
static TaskHandle_t   g_stream_task = NULL;
static SemaphoreHandle_t g_req_ready = NULL;

static void audio_stream_task(void* arg) {
    uint8_t* buf = (uint8_t*)malloc(AUDIO_CHUNK_SIZE);
    if (!buf) {
        Serial.println("audio_stream_task: malloc failed");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        httpd_req_t* req = g_active_req;
        Serial.println("Audio stream connected!");

        while (true) {
            size_t bytes_read = 0;
            i2s_read(I2S_NUM_0, buf, AUDIO_CHUNK_SIZE, &bytes_read, portMAX_DELAY);
            if (bytes_read > 0) {
                int16_t* samples = (int16_t*)buf;
                size_t n = bytes_read / 2;
                for (size_t i = 0; i < n; i++) {
                    int32_t s = (int32_t)samples[i] * MIC_GAIN;
                    if (s >  32767) s =  32767;
                    if (s < -32768) s = -32768;
                    samples[i] = (int16_t)s;
                }
                if (httpd_resp_send_chunk(req, (const char*)buf, bytes_read) != ESP_OK)
                    break;
            }
        }

        Serial.println("Audio stream disconnected.");
        httpd_resp_send_chunk(req, NULL, 0);
        g_active_req = NULL;
        xSemaphoreGive(g_req_ready);
    }

    free(buf);
    vTaskDelete(NULL);
}

static esp_err_t index_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t audio_handler(httpd_req_t* req) {
    if (g_active_req != NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Stream busy");
        return ESP_FAIL;
    }
    // Disable Nagle algorithm so TCP doesn't stall waiting for delayed ACKs.
    // Without this, Nagle + 200ms delayed-ACK on the client causes 5Hz gaps.
    int fd = httpd_req_to_sockfd(req);
    int nodelay = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    httpd_resp_set_type(req, "application/octet-stream");
    g_active_req = req;
    xTaskNotifyGive(g_stream_task);
    xSemaphoreTake(g_req_ready, portMAX_DELAY);
    return ESP_OK;
}

void startMicWebServer() {
    g_req_ready = xSemaphoreCreateBinary();

    xTaskCreatePinnedToCore(
        audio_stream_task, "audio_stream", 4096, NULL, 5, &g_stream_task, 1);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.send_wait_timeout = 30;

    httpd_uri_t index_uri  = { .uri = "/stream", .method = HTTP_GET,
                                .handler = index_handler, .user_ctx = NULL };
    httpd_uri_t audio_uri  = { .uri = "/audio",  .method = HTTP_GET,
                                .handler = audio_handler,  .user_ctx = NULL };

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &index_uri);
        httpd_register_uri_handler(stream_httpd, &audio_uri);
        Serial.println("Audio Stream Server started on port 80");
    }
}
