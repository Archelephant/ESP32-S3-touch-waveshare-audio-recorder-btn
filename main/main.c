#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "es8311.h"
// Include for micbias()
#include "driver/i2c.h"

// ============================
// CONFIGURATION & DEFINITIONS
// ============================
static const char *TAG = "AUDIO_APP";

// Button GPIOs
#define BUTTON_REC_GPIO     GPIO_NUM_0
#define BUTTON_PLAY_GPIO    GPIO_NUM_2

#define I2S_NUM             I2S_NUM_0
#define I2S_MCK_IO          GPIO_NUM_16
#define I2S_BCK_IO          GPIO_NUM_9
#define I2S_WS_IO           GPIO_NUM_45
#define I2S_DO_IO           GPIO_NUM_8
#define I2S_DI_IO           GPIO_NUM_10

// I2C Pins
#define I2C_PORT_NUM        I2C_NUM_0
#define I2C_SCL_IO          GPIO_NUM_14
#define I2C_SDA_IO          GPIO_NUM_15
#define ES8311_I2C_ADDR     0x18

// Power Amplifier Enable
#define PA_ENABLE_PIN       GPIO_NUM_46

// Audio Format Configuration
#define SAMPLE_RATE         48000
#define SAMPLE_BITS         I2S_DATA_BIT_WIDTH_16BIT
#define MCLK_MULTIPLE       256
#define REC_BUFFER_SIZE     1024
#define SILENCE_THRESHOLD   500
#define SILENCE_DURATION_MS 1500

// Recording buffer in PSRAM
#define REC_SECONDS_MAX     10
#define SAMPLES_TO_REC      (SAMPLE_RATE * 2 * REC_SECONDS_MAX) //For whatever reason the lengh of recording capped at 5 seconds, so doubling the length of the buffer

// Beep Generation
#define BEEP_FREQ           1000
#define BEEP_DURATION_MS    1000

// ============================
// GLOBAL VARIABLES & HANDLES
// ============================
static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;
static es8311_handle_t es_handle = NULL;

// ============================
// BUTTON CONFIGURATION (Single button)
// ============================
#define BUTTON_GPIO         GPIO_NUM_0   // Single button for both actions
#define LONG_PRESS_MS       500          // 500ms to distinguish long press
#define DEBOUNCE_MS         50           // Debounce time

// Button states
typedef enum {
    BUTTON_IDLE,
    BUTTON_PRESS_DETECTED,
    BUTTON_RELEASE_DETECTED
} button_state_t;

// Global variables
static QueueHandle_t g_button_event_queue = NULL;
static volatile uint32_t g_press_start_time = 0;
static volatile bool g_button_pressed = false;

// Button events for queue
typedef enum {
    BUTTON_SHORT_PRESS,
    BUTTON_LONG_PRESS_START,   // start recording
    BUTTON_RELEASE             // stop recording
} button_event_t;

// Audio recording variables
static int16_t *g_audio_buffer = NULL;
static volatile size_t g_recorded_samples = 0;
static volatile bool g_is_recording = false;
static volatile bool g_audio_ready = false;
static volatile bool g_stop_recording = false;  // Signal to stop recording

// ============================
// FUNCTION DECLARATIONS
// ============================
static void gpio_isr_handler(void* arg);
static void beep_task(void *pvParameters);
static void record_audio_task(void *pvParameters);
static void playback_audio_task(void *pvParameters);
static void generate_beep(int16_t *buffer, size_t num_samples, uint32_t freq, uint32_t sample_rate);
static float calculate_rms(int16_t *samples, size_t count);

// Test the interrupt via button polling
// Removing - we tested the buttons and need it no more
/*
static void test_button_polling(void) {
    ESP_LOGI(TAG, "Testing button polling for 10 seconds...");
    int last_state = 1;
    for (int i = 0; i < 100; i++) {
        int state = gpio_get_level(BUTTON_GPIO);
        if (state != last_state) {
            ESP_LOGI(TAG, "Button state changed: %d -> %d", last_state, state);
            last_state = state;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "Polling test finished.");
}
    */

// ============================
// 0. Audio power amplifier
// ============================
static void enable_power_amplifier(void) {
    ESP_LOGI(TAG, "Enabling power amplifier on GPIO %d", PA_ENABLE_PIN);
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PA_ENABLE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PA_ENABLE_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Helper: microphone bias

/*
static void enable_micbias(void) {
    uint8_t reg14 = 0x1A;
    // Read current value (optional)
    // Write with bit0 set
    reg14 |= 0x01; // MICBIAS enable
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ES8311_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x14, true);
    i2c_master_write_byte(cmd, reg14, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MICBIAS enabled");
    } else {
        ESP_LOGE(TAG, "Failed to enable MICBIAS");
    }
}*/

// Simpler version with old driver

static void enable_micbias(void) {
    uint8_t write_buf[2] = {0x14, 0x1B};  // register 0x14, value 0x1B (0x1A | 0x01)
    esp_err_t ret = i2c_master_write_to_device(I2C_PORT_NUM, ES8311_I2C_ADDR, write_buf, 2, 1000 / portTICK_PERIOD_MS);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MICBIAS enabled (reg14=0x1B)");
    } else {
        ESP_LOGE(TAG, "Failed to enable MICBIAS: %s", esp_err_to_name(ret));
    }
}

// ============================
// 1. INITIALIZE PSRAM FOR AUDIO BUFFER
// ============================
static esp_err_t init_psram_buffer(void) {
    ESP_LOGI(TAG, "Allocating audio buffer...");
    
    size_t buffer_size_needed = SAMPLES_TO_REC * sizeof(int16_t);
    ESP_LOGI(TAG, "Buffer size needed: %d bytes", buffer_size_needed);
    
    g_audio_buffer = (int16_t *)heap_caps_malloc(buffer_size_needed, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (g_audio_buffer == NULL) {
        ESP_LOGE(TAG, "PSRAM allocation failed");
        return ESP_FAIL;
    }
    
    memset(g_audio_buffer, 0, buffer_size_needed);
    ESP_LOGI(TAG, "Audio buffer allocated. Capacity: %d seconds.", REC_SECONDS_MAX);
    return ESP_OK;
}

// Initialize button

//ISR version

/*
static esp_err_t init_button(void) {
    ESP_LOGI(TAG, "Initializing button on GPIO%d with interrupts...", BUTTON_GPIO);

    gpio_reset_pin(BUTTON_GPIO);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    g_button_event_queue = xQueueCreate(10, sizeof(button_event_t));
    if (g_button_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create button event queue");
        return ESP_FAIL;
    }

    static bool isr_installed = false;
    if (!isr_installed) {
        ESP_ERROR_CHECK(gpio_install_isr_service(0));
        isr_installed = true;
    }

    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, NULL));
    gpio_intr_enable(BUTTON_GPIO);

    ESP_LOGI(TAG, "Button interrupt initialized.");
    return ESP_OK;
}
    */

// Polling version

// ISR alternative: polling every 10 ms

static void button_polling_task(void *pvParameters) {
    int last_state = 1;
    uint32_t press_start = 0;
    bool press_detected = false;
    bool long_press_sent = false;

    while (1) {
        int state = gpio_get_level(BUTTON_GPIO);
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

        if (state == 0 && !press_detected) {          // pressed
            press_detected = true;
            press_start = now;
            long_press_sent = false;
        }
        else if (state == 0 && press_detected && !long_press_sent &&
                 (now - press_start) >= LONG_PRESS_MS) {
            long_press_sent = true;
            button_event_t evt = BUTTON_LONG_PRESS_START;
            xQueueSend(g_button_event_queue, &evt, 0);
        }
        else if (state == 1 && press_detected) {      // released
            press_detected = false;
            if (!long_press_sent) {
                button_event_t evt = BUTTON_SHORT_PRESS;
                xQueueSend(g_button_event_queue, &evt, 0);
            } else {
                button_event_t evt = BUTTON_RELEASE;
                xQueueSend(g_button_event_queue, &evt, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t init_button(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    g_button_event_queue = xQueueCreate(10, sizeof(button_event_t));
    if (!g_button_event_queue) return ESP_FAIL;

    xTaskCreate(button_polling_task, "button_poll", 2048, NULL, 3, NULL);
    return ESP_OK;
}

// ISR handler

//static volatile uint32_t press_start_us = 0;
// Try ticks instead of microseconds?
static volatile enum { IDLE, PRESSED, LONG_PRESS_ACTIVE } button_state = IDLE;
static volatile uint32_t press_start_ticks = 0;
static volatile bool long_press_active = false;
//static volatile uint32_t last_interrupt_us = 0;
// Ticks version
static volatile uint32_t last_interrupt_ticks = 0;

// Back to milliseconds
static volatile uint32_t press_start_ms = 0;
static volatile bool press_detected = false;
static volatile bool long_press_sent = false;
static volatile uint32_t last_interrupt_ms = 0;
#define DEBOUNCE_MS 50
#define LONG_PRESS_MS 500

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t now_ms = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    if (now_ms - last_interrupt_ms < DEBOUNCE_MS) return;
    last_interrupt_ms = now_ms;

    int level = gpio_get_level(BUTTON_GPIO);
    if (level == 0) { // pressed
        if (!press_detected) {
            press_detected = true;
            press_start_ms = now_ms;
            long_press_sent = false;
        }
        // Check for long press while still pressed
        if (!long_press_sent && (now_ms - press_start_ms) >= LONG_PRESS_MS) {
            long_press_sent = true;
            button_event_t evt = BUTTON_LONG_PRESS_START;
            xQueueSendFromISR(g_button_event_queue, &evt, NULL);
        }
    } else { // released
        if (press_detected) {
            press_detected = false;
            if (!long_press_sent) {
                button_event_t evt = BUTTON_SHORT_PRESS;
                xQueueSendFromISR(g_button_event_queue, &evt, NULL);
            } else {
                button_event_t evt = BUTTON_RELEASE;
                xQueueSendFromISR(g_button_event_queue, &evt, NULL);
            }
        }
    }
}

/*
//#define DEBOUNCE_US 50000      // 50 ms debounce
//#define LONG_PRESS_MS 500
//In ticks
#define DEBOUNCE_TICKS   (50 / portTICK_PERIOD_MS)   // 50 ms debounce
#define LONG_PRESS_TICKS (500 / portTICK_PERIOD_MS)  // 500 ms

// New version: ticks
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t now_ticks = xTaskGetTickCountFromISR();
    if (now_ticks - last_interrupt_ticks < DEBOUNCE_TICKS) return;
    last_interrupt_ticks = now_ticks;

    int level = gpio_get_level(BUTTON_GPIO);
    if (level == 0) { // pressed
        if (button_state == IDLE) {
            button_state = PRESSED;
            press_start_ticks = now_ticks;
        }
        // Check for long press while still pressed
        if (button_state == PRESSED && (now_ticks - press_start_ticks) >= LONG_PRESS_TICKS) {
            button_state = LONG_PRESS_ACTIVE;
            button_event_t evt = BUTTON_LONG_PRESS_START;
            xQueueSendFromISR(g_button_event_queue, &evt, NULL);
        }
    } else { // released
        if (button_state == PRESSED) {
            button_state = IDLE;
            button_event_t evt = BUTTON_SHORT_PRESS;
            xQueueSendFromISR(g_button_event_queue, &evt, NULL);
        } else if (button_state == LONG_PRESS_ACTIVE) {
            button_state = IDLE;
            button_event_t evt = BUTTON_RELEASE;
            xQueueSendFromISR(g_button_event_queue, &evt, NULL);
        }
    }
}*/

// Monitoring task to print ISR activity
/*
static void monitor_isr_task(void *pvParameters) {
    uint32_t last_count = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (isr_fire_count != last_count) {
            ESP_LOGI(TAG, "ISR fired %u times (delta %u)", isr_fire_count, isr_fire_count - last_count);
            last_count = isr_fire_count;
        } else {
            ESP_LOGI(TAG, "No ISR in last second");
        }
    }
}*/

// ============================
// 3. INITIALIZE I2C (OLD DRIVER)
// ============================
static esp_err_t init_i2c(void) {
    ESP_LOGI(TAG, "Initializing I2C...");
    
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,  // 100 kHz
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT_NUM, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT_NUM, I2C_MODE_MASTER, 0, 0, 0));
    
    return ESP_OK;
}

// ============================
// 4. INITIALIZE I2S PERIPHERAL
// ============================
static esp_err_t init_i2s_driver(void) {
    ESP_LOGI(TAG, "Initializing I2S driver...");

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    chan_cfg.dma_desc_num = 8;   // More descriptors for continuous streaming
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_chan, &i2s_rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(SAMPLE_BITS, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCK_IO,
            .bclk = I2S_BCK_IO,
            .ws = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din = I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = MCLK_MULTIPLE;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx_chan));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx_chan));

    ESP_LOGI(TAG, "I2S driver initialized at %d Hz", SAMPLE_RATE);
    return ESP_OK;
}

// ============================
// 5. INITIALIZE ES8311 CODEC
// ============================

static esp_err_t init_es8311_codec(void) {
    ESP_LOGI(TAG, "Initializing ES8311 codec...");
    
    // Create ES8311 handle
    es_handle = es8311_create(I2C_PORT_NUM, ES8311_I2C_ADDR);
    if (es_handle == NULL) {
        ESP_LOGE(TAG, "es8311 create failed");
        return ESP_FAIL;
    }
    
    // Configure clock (same as working example)
    const es8311_clock_config_t es_clk = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = SAMPLE_RATE * MCLK_MULTIPLE,
        .sample_frequency = SAMPLE_RATE
    };
    
    // Initialize ES8311
    ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
    
    // Configure sample frequency
    ESP_ERROR_CHECK(es8311_sample_frequency_config(es_handle, SAMPLE_RATE * MCLK_MULTIPLE, SAMPLE_RATE));
    
    // Set volume
    ESP_ERROR_CHECK(es8311_voice_volume_set(es_handle, 90, NULL));
    
    // Configure microphone
    ESP_ERROR_CHECK(es8311_microphone_config(es_handle, false));

    // Small delay to make usre microphone is fully powered
    vTaskDelay(pdMS_TO_TICKS(50));

    // Set microphone gain
    ESP_ERROR_CHECK(es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_36DB));
    // =============TROUBLESHOOTING===========
    // Dump the registers for checking them
    es8311_register_dump(es_handle);
    
    ESP_LOGI(TAG, "ES8311 codec initialized successfully.");
    return ESP_OK;
}

// Helper: beep buffer init
static int16_t *beep_buffer = NULL;
static size_t beep_buffer_samples = 0;

static void init_beep_buffer(void) {
    // Stereo: 1 second at sample rate
    beep_buffer_samples = SAMPLE_RATE * 2;   // 2 samples per frame (L+R)
    size_t bytes = beep_buffer_samples * sizeof(int16_t);
    beep_buffer = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (beep_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate beep buffer");
        return;
    }
    // Precompute sine wave (same for left and right)
    for (size_t i = 0; i < beep_buffer_samples; i += 2) {
        float t = (float)(i/2) / SAMPLE_RATE;
        int16_t sample = (int16_t)(15000.0 * sin(2 * M_PI * BEEP_FREQ * t));
        beep_buffer[i] = sample;     // left
        beep_buffer[i+1] = sample;   // right
    }
    ESP_LOGI(TAG, "Beep buffer precomputed (%d bytes)", bytes);
}

// ============================
// HELPER: Generate a sine wave beep
// ============================
static void generate_beep(int16_t *buffer, size_t num_samples, uint32_t freq, uint32_t sample_rate) {
    // Generate stereo beep
    for (size_t i = 0; i < num_samples; i += 2) {
        int16_t sample = (int16_t)(15000.0 * sin(2 * M_PI * freq * (i/2) / sample_rate));
        buffer[i] = sample;     // Left channel
        buffer[i+1] = sample;   // Right channel
    }
}

// ============================
// HELPER: Calculate RMS of audio samples
// ============================
static float calculate_rms(int16_t *samples, size_t count) {
    int64_t sum = 0;
    for (size_t i = 0; i < count; i++) {
        sum += (int64_t)samples[i] * samples[i];
    }
    return sqrtf((float)sum / count);
}

// ============================
// TASK: Play a beep
// ============================

// Simplified beep task with pre-generated sine wave
static void beep_task(void *pvParameters) {
    if (beep_buffer == NULL) {
        ESP_LOGE(TAG, "Beep buffer not initialized");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Playing beep...");
    size_t bytes_written = 0;
    i2s_channel_write(i2s_tx_chan, beep_buffer,
                      beep_buffer_samples * sizeof(int16_t),
                      &bytes_written, portMAX_DELAY);
    vTaskDelete(NULL);
}

// ============================
// TASK: Record audio until silence
// ============================
/*
static void record_audio_task(void *pvParameters) {
    int16_t read_buffer[REC_BUFFER_SIZE];
    g_recorded_samples = 0;
    g_is_recording = true;
    uint32_t silence_start_time = 0;
    bool silence_detected = false;

    ESP_LOGI(TAG, "Recording started... Speak into the microphone.");

    while (g_is_recording && g_recorded_samples < SAMPLES_TO_REC) {
        size_t bytes_read = 0;
        i2s_channel_read(i2s_rx_chan, read_buffer, sizeof(read_buffer), &bytes_read, pdMS_TO_TICKS(100));
        size_t samples_read = bytes_read / sizeof(int16_t);

        if (samples_read > 0) {
            float rms = calculate_rms(read_buffer, samples_read);

            if (rms < SILENCE_THRESHOLD) {
                if (!silence_detected) {
                    silence_detected = true;
                    silence_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    ESP_LOGI(TAG, "Silence detected, starting timer...");
                } else if ((xTaskGetTickCount() * portTICK_PERIOD_MS - silence_start_time) >= SILENCE_DURATION_MS) {
                    ESP_LOGI(TAG, "Silence duration exceeded. Stopping recording.");
                    g_is_recording = false;
                    break;
                }
            } else {
                silence_detected = false;
            }

            memcpy(&g_audio_buffer[g_recorded_samples], read_buffer, samples_read * sizeof(int16_t));
            g_recorded_samples += samples_read;
            
            if (g_recorded_samples % (SAMPLE_RATE * 2) == 0) { // Log every 2 seconds
                ESP_LOGI(TAG, "Recorded: %.1f seconds", (float)g_recorded_samples / (SAMPLE_RATE * 2));
            }
        }
    }
    
    g_is_recording = false;
    g_audio_ready = (g_recorded_samples > SAMPLE_RATE);
    ESP_LOGI(TAG, "Recording stopped. Total: %.1f seconds", (float)g_recorded_samples / (SAMPLE_RATE * 2));
    vTaskDelete(NULL);
}*/

// ============================
// TASK: Record audio while button is held
// ============================

// V4 - debug mode with echo

static void record_audio_task(void *pvParameters) {
    ESP_LOGI(TAG, "Recording started (long press)...");
    g_recorded_samples = 0;
    g_is_recording = true;
    g_stop_recording = false;

    //xTaskCreate(beep_task, "start_beep", 2048, NULL, 5, NULL);
    //vTaskDelay(pdMS_TO_TICKS(200));
    //resetting channel to clear any stale I2S comm
    i2s_channel_disable(i2s_rx_chan);
    i2s_channel_enable(i2s_rx_chan);

    int16_t read_buffer[REC_BUFFER_SIZE];
    uint32_t last_log_time = 0;
    int32_t sum = 0;
    uint32_t samples_since_log = 0;

    while (g_is_recording && !g_stop_recording && g_recorded_samples < SAMPLES_TO_REC) {
        // Fallback button polling
        if (gpio_get_level(BUTTON_GPIO) == 1) {
            ESP_LOGI(TAG, "Button released (polled), stopping.");
            break;
        }

        size_t bytes_read = 0;
        esp_err_t ret = i2s_channel_read(i2s_rx_chan, read_buffer, sizeof(read_buffer), &bytes_read, pdMS_TO_TICKS(1000));
        // Adding recovery mechanism for timeouts
        if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "I2S read timeout, resetting RX channel");
        i2s_channel_disable(i2s_rx_chan);
        i2s_channel_enable(i2s_rx_chan);
        continue;
        } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S read error: %s", esp_err_to_name(ret));
        break;
        }
        if (ret == ESP_OK && bytes_read > 0) {
            size_t samples_read = bytes_read / sizeof(int16_t);
            ESP_LOGD(TAG, "Read %d bytes -> %d samples", bytes_read, samples_read);

            // Accumulate for average
            for (size_t i = 0; i < samples_read; i++) {
                sum += abs(read_buffer[i]);
            }
            samples_since_log += samples_read;

            // Copy to main buffer (clip if necessary)
            size_t samples_to_copy = samples_read;
            if (g_recorded_samples + samples_to_copy > SAMPLES_TO_REC) {
                samples_to_copy = SAMPLES_TO_REC - g_recorded_samples;
            }
            if (samples_to_copy > 0) {
                memcpy(&g_audio_buffer[g_recorded_samples], read_buffer, samples_to_copy * sizeof(int16_t));
                g_recorded_samples += samples_to_copy;
                ESP_LOGD(TAG, "Copied %d samples, total now %d", samples_to_copy, g_recorded_samples);
            }

            // Log every second
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_log_time > 1000) {
                int avg = (samples_since_log > 0) ? (sum / samples_since_log) : 0;
                ESP_LOGI(TAG, "Recording: %.1f sec, avg sample = %d", 
                         (float)g_recorded_samples / (SAMPLE_RATE * 2), avg);
                last_log_time = now;
                sum = 0;
                samples_since_log = 0;
            }
        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2S read error: %s", esp_err_to_name(ret));
        }
    }

    g_is_recording = false;
    g_audio_ready = (g_recorded_samples > 0);
    ESP_LOGI(TAG, "Recording stopped. Total: %.1f seconds", (float)g_recorded_samples / (SAMPLE_RATE * 2));
    ESP_ERROR_CHECK(es8311_voice_volume_set(es_handle, 10, NULL));
    xTaskCreate(beep_task, "end_beep", 2048, NULL, 5, NULL);
    vTaskDelete(NULL);
}

// ============================
// TASK: Playback recorded audio or beeps
// ============================

static void playback_audio_task(void *pvParameters) {
    if (!g_audio_ready || g_recorded_samples == 0) {
        ESP_LOGW(TAG, "No audio recorded. Playing beep.");
        xTaskCreate(beep_task, "beep", 2048, NULL, 5, NULL);
    } else {
        ESP_ERROR_CHECK(es8311_voice_volume_set(es_handle, 90, NULL));
        ESP_LOGI(TAG, "Playing back recorded audio (%.1f seconds)...", 
                 (float)g_recorded_samples / (SAMPLE_RATE * 2));
        size_t bytes_written = 0;
        esp_err_t ret = i2s_channel_write(i2s_tx_chan, g_audio_buffer, 
                                         g_recorded_samples * sizeof(int16_t), 
                                         &bytes_written, portMAX_DELAY);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Playback finished.");
        } else {
            ESP_LOGE(TAG, "Playback failed: %s", esp_err_to_name(ret));
        }
    }
    vTaskDelete(NULL);
}

// ============================
// TASK: Test button monitoring
// ============================
/*
static void button_test_task(void *pvParameters) {
    ESP_LOGI(TAG, "Button test task started. Press buttons to test...");
    
    while (1) {
        // Read button states directly (polling)
        int rec_state = gpio_get_level(BUTTON_REC_GPIO);
        int play_state = gpio_get_level(BUTTON_PLAY_GPIO);
        
        static int last_rec_state = 1;
        static int last_play_state = 1;
        
        if (rec_state != last_rec_state) {
            ESP_LOGI(TAG, "REC button state changed: %d -> %d", last_rec_state, rec_state);
            last_rec_state = rec_state;
        }
        
        if (play_state != last_play_state) {
            ESP_LOGI(TAG, "PLAY button state changed: %d -> %d", last_play_state, play_state);
            last_play_state = play_state;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100ms
    }
}
    */

/*
    static void button_test_task(void *pvParameters) {
    ESP_LOGI(TAG, "Button test task started. Press buttons to test...");
    
    int last_rec_state = -1;
    int last_play_state = -1;
    uint32_t last_print_time = 0;
    
    while (1) {
        // Read button states directly
        int rec_state = gpio_get_level(BUTTON_REC_GPIO);
        int play_state = gpio_get_level(BUTTON_PLAY_GPIO);
        
        // Check for state changes
        if (rec_state != last_rec_state || play_state != last_play_state) {
            ESP_LOGI(TAG, "Button states - REC: %d, PLAY: %d", rec_state, play_state);
            last_rec_state = rec_state;
            last_play_state = play_state;
        }
        
        // Print status every 5 seconds even if no change
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_print_time > 5000) {
            ESP_LOGI(TAG, "Button monitoring active...");
            last_print_time = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));  // Check every 50ms
    }
}*/

// ============================
// DEBUG: Test GPIO2 functionality
// ============================
/*
static void test_gpio2(void) {
    ESP_LOGI(TAG, "Testing GPIO2 functionality...");
    
    // Reset GPIO2
    gpio_reset_pin(GPIO_NUM_2);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Configure as input with pull-up
    gpio_config_t test_config = {
        .pin_bit_mask = (1ULL << GPIO_NUM_2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&test_config);
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // Test reading
    ESP_LOGI(TAG, "GPIO2 state: %d", gpio_get_level(GPIO_NUM_2));
    
    // Test toggling by reconfiguring as output
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    
    ESP_LOGI(TAG, "Toggling GPIO2 as output test...");
    for (int i = 0; i < 5; i++) {
        gpio_set_level(GPIO_NUM_2, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(GPIO_NUM_2, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Restore as input
    gpio_reset_pin(GPIO_NUM_2);
    gpio_config(&test_config);
    
    ESP_LOGI(TAG, "GPIO2 test complete.");
}*/

// ============================
// MAIN APPLICATION
// ============================

void app_main(void) {
    ESP_LOGI(TAG, "=== ESP32-S3 Audio Recorder/Player ===\n");
    
    // 1. Enable Power Amplifier
    enable_power_amplifier();
    
    // 2. Initialize PSRAM buffer
    if (init_psram_buffer() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to allocate PSRAM buffer");
        return;
    }
    
    // 3. Initialize I2C (OLD driver only)
    if (init_i2c() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }
    
    // 4. Initialize ES8311
    if (init_es8311_codec() != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 init failed");
        return;
    }

    enable_micbias();
    
    // 5. Initialize I2S
    if (init_i2s_driver() != ESP_OK) {
        ESP_LOGE(TAG, "I2S init failed");
        return;
    }

   // 6. Initialize single button
    if (init_button() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize button");
        return;
    }
    
    //6.1. Initialize beep buffer
    init_beep_buffer();
    
    // 7. Start button test task (for debugging)
    //xTaskCreate(button_test_task, "button_test", 2048, NULL, 2, NULL);
    //test_button_polling();
    //xTaskCreate(monitor_isr_task, "monitor_isr", 2048, NULL, 2, NULL);

    // 8. Play test beep
    ESP_LOGI(TAG, "Playing test beep...");
    // beep_task(NULL);
    xTaskCreate(beep_task, "startup_beep", 2048, NULL, 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    ESP_LOGI(TAG, "\n=== System Ready ===");
    //    ESP_LOGI(TAG, "Press buttons to control:");
    // ESP_LOGI(TAG, "  - GPIO%d: Record (starts with beep)", BUTTON_REC_GPIO);
    // ESP_LOGI(TAG, "  - GPIO%d: Playback", BUTTON_PLAY_GPIO);
    // New
    // ESP_LOGI(TAG, "\nListening for button presses...");

    ESP_LOGI(TAG, "Single button on GPIO%d:", BUTTON_GPIO);
    ESP_LOGI(TAG, "  - Short press (< %dms): Playback (or beep if none)", LONG_PRESS_MS);
    ESP_LOGI(TAG, "  - Long press (>= %dms): Record while held", LONG_PRESS_MS);
    ESP_LOGI(TAG, "\nListening for button actions...");
    
    // Main event loop - updated
    button_event_t evt;
    uint32_t event_count = 0;
    
    while (1) {
        if (xQueueReceive(g_button_event_queue, &evt, portMAX_DELAY) == pdTRUE) {
            // Printing the button events received to check that queue comm works
            ESP_LOGI(TAG, "Received button event: %d", evt);
            event_count++;
            ESP_LOGI(TAG, "Free stack: %d", uxTaskGetStackHighWaterMark(NULL));
                        
            switch (evt) {
                case BUTTON_SHORT_PRESS:
                    ESP_LOGI(TAG, "Short press -> playback");
                    xTaskCreate(playback_audio_task, "playback", 16384, NULL, 5, NULL);
                    break;
                case BUTTON_LONG_PRESS_START:
                    ESP_LOGI(TAG, "Long press start -> recording");
                    if (!g_is_recording) {
                        xTaskCreate(record_audio_task, "record", 16384, NULL, 6, NULL);
                    }
                    break;
                case BUTTON_RELEASE:
                    ESP_LOGI(TAG, "Button released -> stop recording");
                    g_stop_recording = true;
                    break;
            }

        }
    }
}