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

// ============================
// CONFIGURATION & DEFINITIONS
// ============================
static const char *TAG = "AUDIO_APP";

// Button GPIOs
#define BUTTON_REC_GPIO     GPIO_NUM_0
#define BUTTON_PLAY_GPIO    GPIO_NUM_2

// I2S Pins (using your working example's pins)
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
#define SAMPLES_TO_REC      (SAMPLE_RATE * REC_SECONDS_MAX)

// Beep Generation
#define BEEP_FREQ           1000
#define BEEP_DURATION_MS    1000

// ============================
// GLOBAL VARIABLES & HANDLES
// ============================
static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;
static es8311_handle_t es_handle = NULL;

static int16_t *g_audio_buffer = NULL;
static volatile size_t g_recorded_samples = 0;
static volatile bool g_is_recording = false;
static volatile bool g_audio_ready = false;
static QueueHandle_t g_button_event_queue = NULL;

typedef enum {
    BUTTON_REC_EVENT,
    BUTTON_PLAY_EVENT
} button_event_t;

// ============================
// FUNCTION DECLARATIONS
// ============================
static void gpio_isr_handler(void* arg);
static void beep_task(void *pvParameters);
static void record_audio_task(void *pvParameters);
static void playback_audio_task(void *pvParameters);
static void generate_beep(int16_t *buffer, size_t num_samples, uint32_t freq, uint32_t sample_rate);
static float calculate_rms(int16_t *samples, size_t count);

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

// ============================
// 2. INITIALIZE BUTTONS & INTERRUPTS
// ============================

static esp_err_t init_buttons(void) {
    ESP_LOGI(TAG, "Initializing buttons...");

    // FIRST: Check the current state of the buttons
    gpio_set_direction(BUTTON_REC_GPIO, GPIO_MODE_INPUT);
    gpio_set_direction(BUTTON_PLAY_GPIO, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(10));

    int rec_state = gpio_get_level(BUTTON_REC_GPIO);
    int play_state = gpio_get_level(BUTTON_PLAY_GPIO);
    ESP_LOGI(TAG, "Button initial states - REC GPIO%d: %d, PLAY GPIO%d: %d", 
             BUTTON_REC_GPIO, rec_state, BUTTON_PLAY_GPIO, play_state);

    // Configure GPIO0 (BOOT button) - This button is typically active-low (0 when pressed)
    // It usually has an external pull-up resistor
    gpio_config_t io_conf_rec = {
        .pin_bit_mask = (1ULL << BUTTON_REC_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,  // GPIO0 usually has external pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,     // Trigger when goes LOW (button press)
    };
    
    // Configure GPIO2 - We'll enable internal pull-up
    gpio_config_t io_conf_play = {
        .pin_bit_mask = (1ULL << BUTTON_PLAY_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // Enable internal pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,     // Trigger when goes LOW (button press)
    };

    ESP_ERROR_CHECK(gpio_config(&io_conf_rec));
    ESP_ERROR_CHECK(gpio_config(&io_conf_play));


    // Create button event queue
    g_button_event_queue = xQueueCreate(10, sizeof(button_event_t));
    if (g_button_event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create button event queue");
        return ESP_FAIL;
    }

        // Install GPIO ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    
    // Add ISR handlers
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_REC_GPIO, gpio_isr_handler, (void *)BUTTON_REC_EVENT));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_PLAY_GPIO, gpio_isr_handler, (void *)BUTTON_PLAY_EVENT));

    // Check final states
    rec_state = gpio_get_level(BUTTON_REC_GPIO);
    play_state = gpio_get_level(BUTTON_PLAY_GPIO);
    ESP_LOGI(TAG, "Button configured states - REC: %d, PLAY: %d", rec_state, play_state);
    
    // If either is LOW (0), it might indicate a wiring issue or button is pressed
    if (rec_state == 0) ESP_LOGW(TAG, "REC button is LOW (might be pressed or shorted)");
    if (play_state == 0) ESP_LOGW(TAG, "PLAY button is LOW (might be pressed or shorted)");

    ESP_LOGI(TAG, "Buttons initialized successfully.");
    return ESP_OK;

}

/*
// ============================
// GPIO INTERRUPT SERVICE ROUTINE
// ============================
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    button_event_t evt = (button_event_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(g_button_event_queue, &evt, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}
    */

// ============================
// GPIO INTERRUPT SERVICE ROUTINE WITH DEBOUNCING
// ============================
static volatile uint32_t last_isr_time = 0;
#define DEBOUNCE_TIME_MS 50  // 50ms debounce time

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t current_time = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    
    // Simple debounce - ignore interrupts that come too quickly
    if (current_time - last_isr_time < DEBOUNCE_TIME_MS) {
        return;
    }
    last_isr_time = current_time;
    
    button_event_t evt = (button_event_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Send event to queue
    if (xQueueSendFromISR(g_button_event_queue, &evt, &xHigherPriorityTaskWoken) != pdTRUE) {
        // Queue full - this shouldn't happen often
    }
    
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

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
    ESP_ERROR_CHECK(es8311_voice_volume_set(es_handle, 60, NULL));
    
    // Configure microphone
    ESP_ERROR_CHECK(es8311_microphone_config(es_handle, false));
    
    // Set microphone gain
    ESP_ERROR_CHECK(es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_30DB));
    
    ESP_LOGI(TAG, "ES8311 codec initialized successfully.");
    return ESP_OK;
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
static void beep_task(void *pvParameters) {
    size_t beep_samples = (SAMPLE_RATE * BEEP_DURATION_MS / 1000) * 2; // Stereo
    int16_t *beep_buffer = malloc(beep_samples * sizeof(int16_t));
    if (beep_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate beep buffer");
        vTaskDelete(NULL);
        return;
    }

    generate_beep(beep_buffer, beep_samples, BEEP_FREQ, SAMPLE_RATE);
    ESP_LOGI(TAG, "Playing beep...");

    size_t bytes_written = 0;
    esp_err_t ret = i2s_channel_write(i2s_tx_chan, beep_buffer, 
                                      beep_samples * sizeof(int16_t), 
                                      &bytes_written, portMAX_DELAY);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Beep playback failed: %s", esp_err_to_name(ret));
    }
    
    free(beep_buffer);
    vTaskDelete(NULL);
}

// ============================
// TASK: Record audio until silence
// ============================
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
}

// ============================
// TASK: Playback recorded audio or beeps
// ============================
static void playback_audio_task(void *pvParameters) {
    if (!g_audio_ready || g_recorded_samples == 0) {
        ESP_LOGW(TAG, "No audio recorded. Playing beep.");
        beep_task(NULL);
    } else {
        ESP_LOGI(TAG, "Playing back recorded audio...");
        size_t bytes_written = 0;
        i2s_channel_write(i2s_tx_chan, g_audio_buffer, 
                         g_recorded_samples * sizeof(int16_t), 
                         &bytes_written, portMAX_DELAY);
        ESP_LOGI(TAG, "Playback finished.");
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
}

// ============================
// DEBUG: Test GPIO2 functionality
// ============================
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
}

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
    
    // 5. Initialize I2S
    if (init_i2s_driver() != ESP_OK) {
        ESP_LOGE(TAG, "I2S init failed");
        return;
    }
    
    // 6. Initialize buttons
    // Test GPIO2 functionality
    test_gpio2();
    if (init_buttons() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize buttons");
        return;
    }
    
    // 7. Start button test task (for debugging)
    xTaskCreate(button_test_task, "button_test", 2048, NULL, 2, NULL);

    // 8. Play test beep
    ESP_LOGI(TAG, "Playing test beep...");
    beep_task(NULL);
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    ESP_LOGI(TAG, "\n=== System Ready ===");
    ESP_LOGI(TAG, "Press buttons to control:");
    ESP_LOGI(TAG, "  - GPIO%d: Record (starts with beep)", BUTTON_REC_GPIO);
    ESP_LOGI(TAG, "  - GPIO%d: Playback", BUTTON_PLAY_GPIO);
    // New
    ESP_LOGI(TAG, "\nListening for button presses...");
    
    /*
    // Main event loop
    button_event_t evt;
    while (1) {
        if (xQueueReceive(g_button_event_queue, &evt, portMAX_DELAY)) {
            switch (evt) {
                case BUTTON_REC_EVENT:
                    ESP_LOGI(TAG, "Record button pressed");
                    if (!g_is_recording) {
                        beep_task(NULL);  // Start beep
                        vTaskDelay(pdMS_TO_TICKS(1100));
                        record_audio_task(NULL);  // Record
                    }
                    break;
                    
                case BUTTON_PLAY_EVENT:
                    ESP_LOGI(TAG, "Playback button pressed");
                    playback_audio_task(NULL);
                    break;
            }
        }
    }
        */

    // Main event loop with timeout - previous
    /*
    button_event_t evt;
    while (1) {
        // Wait for button event with 100ms timeout
        if (xQueueReceive(g_button_event_queue, &evt, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (evt) {
                case BUTTON_REC_EVENT:
                    ESP_LOGI(TAG, ">>> RECORD button pressed!");
                    if (!g_is_recording) {
                        ESP_LOGI(TAG, "Starting recording...");
                        // Play start beep
                        xTaskCreate(beep_task, "start_beep", 2048, NULL, 5, NULL);
                        vTaskDelay(pdMS_TO_TICKS(1100));
                        // Start recording
                        xTaskCreate(record_audio_task, "record", 4096, NULL, 6, NULL);
                    } else {
                        ESP_LOGI(TAG, "Already recording...");
                    }
                    break;
                    
                case BUTTON_PLAY_EVENT:
                    ESP_LOGI(TAG, ">>> PLAYBACK button pressed!");
                    if (g_audio_ready) {
                        ESP_LOGI(TAG, "Playing recorded audio...");
                        xTaskCreate(playback_audio_task, "playback", 4096, NULL, 5, NULL);
                    } else {
                        ESP_LOGI(TAG, "No audio recorded yet. Playing beep.");
                        xTaskCreate(beep_task, "beep", 2048, NULL, 5, NULL);
                    }
                    break;
            }
        }
        
        // Optional: Add a heartbeat every 10 seconds
        static uint32_t last_heartbeat = 0;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_heartbeat > 10000) {
            ESP_LOGI(TAG, "System active...");
            last_heartbeat = now;
        }
    }
        */
    
    // Main event loop - updated
    button_event_t evt;
    uint32_t event_count = 0;
    
    while (1) {
        // Wait for button events
        if (xQueueReceive(g_button_event_queue, &evt, portMAX_DELAY) == pdTRUE) {
            event_count++;
            
            switch (evt) {
                case BUTTON_REC_EVENT:
                    ESP_LOGI(TAG, "[EVENT #%d] >>> RECORD button pressed!", event_count);
                    if (!g_is_recording) {
                        ESP_LOGI(TAG, "Starting recording...");
                        // Play start beep
                        xTaskCreate(beep_task, "start_beep", 2048, NULL, 5, NULL);
                        vTaskDelay(pdMS_TO_TICKS(1100));
                        // Start recording task
                        xTaskCreate(record_audio_task, "record_task", 4096, NULL, 6, NULL);
                    } else {
                        ESP_LOGI(TAG, "Already recording...");
                    }
                    break;
                    
                case BUTTON_PLAY_EVENT:
                    ESP_LOGI(TAG, "[EVENT #%d] >>> PLAYBACK button pressed!", event_count);
                    if (g_audio_ready && g_recorded_samples > 0) {
                        ESP_LOGI(TAG, "Playing recorded audio (%d samples)...", g_recorded_samples);
                        xTaskCreate(playback_audio_task, "playback_task", 4096, NULL, 5, NULL);
                    } else {
                        ESP_LOGI(TAG, "No audio recorded yet. Playing beep.");
                        xTaskCreate(beep_task, "beep_fallback", 2048, NULL, 5, NULL);
                    }
                    break;
                    
                default:
                    ESP_LOGW(TAG, "Unknown button event: %d", evt);
                    break;
            }
        }
    }
}