#pragma once

// Minimal ESP error compatibility for the platform-neutral public API. These
// numeric values match ESP-IDF so callers can keep their existing checks.
typedef int esp_err_t;

#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103

