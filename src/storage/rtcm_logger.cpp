#include "rtcm_logger.h"
#include "../settings_manager/settings_manager.h"
#include "../rtc_bsp/rtc_bsp.h"
#include <Arduino.h>
#include <FS.h>
#include "esp_heap_caps.h"
#include <stdlib.h>

#define RTCM_RING_BUF_SIZE 16384

extern SemaphoreHandle_t fs_mux;

static volatile bool s_rtcm_active = false;
static uint32_t s_bytes_logged = 0;
static char s_rtcm_filename[64] = {0};
static File s_rtcm_file;
static SemaphoreHandle_t s_rtcm_buf_mux = NULL;
static SemaphoreHandle_t s_rtcm_file_mux = NULL;

static uint8_t *s_ring_buf = NULL;
static volatile size_t s_ring_head = 0;
static volatile size_t s_ring_tail = 0;
static uint32_t s_last_sd_flush_ms = 0;

static bool ensure_rtcm_ring_buf(void)
{
    if (s_ring_buf) return true;
    s_ring_buf = (uint8_t *)heap_caps_malloc(RTCM_RING_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ring_buf) {
        s_ring_buf = (uint8_t *)malloc(RTCM_RING_BUF_SIZE);
    }
    return (s_ring_buf != NULL);
}

extern "C" bool rtcm_logger_start(void)
{
    if (!s_rtcm_buf_mux) {
        s_rtcm_buf_mux = xSemaphoreCreateMutex();
    }
    if (!s_rtcm_file_mux) {
        s_rtcm_file_mux = xSemaphoreCreateMutex();
    }
    if (s_rtcm_file_mux) xSemaphoreTake(s_rtcm_file_mux, portMAX_DELAY);

    if (!ensure_rtcm_ring_buf()) {
        if (s_rtcm_file_mux) xSemaphoreGive(s_rtcm_file_mux);
        return false;
    }

    if (s_rtcm_active) {
        if (s_rtcm_file_mux) xSemaphoreGive(s_rtcm_file_mux);
        return true;
    }

    if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);

    if (s_rtcm_file) {
        s_rtcm_file.close();
    }

    if (!get_active_fs().exists("/rtcm")) {
        get_active_fs().mkdir("/rtcm");
    }

    char ts[64];
    get_formatted_timestamp(ts, sizeof(ts), true);
    snprintf(s_rtcm_filename, sizeof(s_rtcm_filename), "/rtcm/rtcm-base_%s.txt", ts);

    s_rtcm_file = get_active_fs().open(s_rtcm_filename, FILE_WRITE);
    if (!s_rtcm_file) {
        // Fallback to flat root filename if /rtcm directory is not supported
        snprintf(s_rtcm_filename, sizeof(s_rtcm_filename), "/rtcm-base_%s.txt", ts);
        s_rtcm_file = get_active_fs().open(s_rtcm_filename, FILE_WRITE);
    }
    if (!s_rtcm_file) {
        s_rtcm_active = false;
        if (fs_mux) xSemaphoreGive(fs_mux);
        if (s_rtcm_file_mux) xSemaphoreGive(s_rtcm_file_mux);
        Serial.printf("[RTCM LOG] Failed to open %s for write\n", s_rtcm_filename);
        return false;
    }

    if (s_rtcm_buf_mux) xSemaphoreTake(s_rtcm_buf_mux, portMAX_DELAY);
    s_ring_head = 0;
    s_ring_tail = 0;
    s_bytes_logged = 0;
    s_last_sd_flush_ms = millis();
    if (s_rtcm_buf_mux) xSemaphoreGive(s_rtcm_buf_mux);

    s_rtcm_active = true;

    if (fs_mux) xSemaphoreGive(fs_mux);
    if (s_rtcm_file_mux) xSemaphoreGive(s_rtcm_file_mux);
    Serial.printf("[RTCM LOG] Started recording raw RTCM base stream to %s\n", s_rtcm_filename);
    return true;
}

// Drain ring buffer to SD/FFat WITHOUT holding s_rtcm_buf_mux across slow VFS write/flush calls.
// Caller must already hold s_rtcm_file_mux and (if initialized) fs_mux.
static void rtcm_logger_drain_to_file(bool force_sync)
{
    if (!s_rtcm_file || !s_ring_buf) return;

    uint8_t write_buf[1024];
    bool wrote_any = false;

    for (;;) {
        size_t bytes_to_write = 0;
        if (s_rtcm_buf_mux && xSemaphoreTake(s_rtcm_buf_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
            while (s_ring_tail != s_ring_head && bytes_to_write < sizeof(write_buf)) {
                write_buf[bytes_to_write++] = s_ring_buf[s_ring_tail];
                s_ring_tail = (s_ring_tail + 1) % RTCM_RING_BUF_SIZE;
            }
            xSemaphoreGive(s_rtcm_buf_mux);
        } else {
            break;
        }

        if (bytes_to_write == 0) {
            break;
        }

        s_rtcm_file.write(write_buf, bytes_to_write);
        s_bytes_logged += bytes_to_write;
        wrote_any = true;
    }

    uint32_t now_ms = millis();
    if (force_sync || (wrote_any && (now_ms - s_last_sd_flush_ms >= 500))) {
        s_last_sd_flush_ms = now_ms;
        s_rtcm_file.flush();
    }
}

extern "C" void rtcm_logger_stop(void)
{
    if (!s_rtcm_file_mux) return;
    xSemaphoreTake(s_rtcm_file_mux, portMAX_DELAY);
    bool stopped = false;
    uint32_t total_logged = 0;
    if (s_rtcm_active) {
        if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);
        rtcm_logger_drain_to_file(true);
        if (s_rtcm_file) {
            s_rtcm_file.flush();
            s_rtcm_file.close();
        }
        s_rtcm_active = false;
        total_logged = s_bytes_logged;
        stopped = true;
        if (fs_mux) xSemaphoreGive(fs_mux);
    }
    xSemaphoreGive(s_rtcm_file_mux);
    if (stopped) {
        Serial.printf("[RTCM LOG] Stopped. Total logged: %lu bytes to %s\n", (unsigned long)total_logged, s_rtcm_filename);
    }
}

extern "C" bool rtcm_logger_is_active(void)
{
    return s_rtcm_active;
}

extern "C" const char* rtcm_logger_get_current_filename(void)
{
    return s_rtcm_filename;
}

extern "C" uint32_t rtcm_logger_get_bytes_logged(void)
{
    return s_bytes_logged;
}

extern "C" void rtcm_logger_feed(const uint8_t *data, size_t len)
{
    if (!s_rtcm_active || !s_ring_buf || !data || len == 0) return;

    if (!s_rtcm_buf_mux) {
        s_rtcm_buf_mux = xSemaphoreCreateMutex();
    }

    if (s_rtcm_buf_mux && xSemaphoreTake(s_rtcm_buf_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        for (size_t i = 0; i < len; i++) {
            size_t next_head = (s_ring_head + 1) % RTCM_RING_BUF_SIZE;
            if (next_head != s_ring_tail) {
                s_ring_buf[s_ring_head] = data[i];
                s_ring_head = next_head;
            } else {
                // Buffer overflow: drop byte to prevent stalling
                break;
            }
        }
        xSemaphoreGive(s_rtcm_buf_mux);
    }
}

extern "C" void rtcm_logger_flush(void)
{
    if (!s_rtcm_active || !s_ring_buf) return;

    bool has_data = (s_ring_head != s_ring_tail);
    bool needs_periodic_sync = (millis() - s_last_sd_flush_ms >= 500);
    if (!has_data && !needs_periodic_sync) return;

    if (!s_rtcm_file_mux || xSemaphoreTake(s_rtcm_file_mux, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    if (fs_mux && xSemaphoreTake(fs_mux, pdMS_TO_TICKS(15)) != pdTRUE) {
        xSemaphoreGive(s_rtcm_file_mux);
        return;
    }

    if (s_rtcm_active) {
        rtcm_logger_drain_to_file(false);
    }

    if (fs_mux) xSemaphoreGive(fs_mux);
    xSemaphoreGive(s_rtcm_file_mux);
}
