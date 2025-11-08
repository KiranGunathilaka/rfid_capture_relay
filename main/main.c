/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "errno.h"
#include "driver/gpio.h"
#include <inttypes.h>
#include "config.h"
#include "nvs_flash.h"

#include "hid_host.h"
#include "hid_usage_keyboard.h"
#include "hid_usage_mouse.h"

#include "esp_wifi.h"
#include "esp_now.h"

#include "led_strip.h"     // IDF 5.x LED strip abstraction
#include "driver/rmt_tx.h" // RMT transmitter

#include "driver/ledc.h"
#include "esp_timer.h"

static const char *TAG_NOW = "ESP-NOW";
static const char *TAG_LED = "WS2812";
static const char *TAG_BUZ = "BUZZER";

// --------------- Buzzer -----------------------------

static bool buzzer_is_init = false;
static esp_timer_handle_t buzzer_stop_timer = NULL;

// Safe-stop helper: only stop a timer if it's active
static inline void timer_stop_if_active(esp_timer_handle_t t)
{
    if (!t)
        return;
    if (esp_timer_is_active(t))
    {
        ESP_ERROR_CHECK(esp_timer_stop(t));
    }
}

static void buzzer_gpio_init_active(void)
{
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BUZZER_GPIO, 0);
}

static void buzzer_ledc_init_passive(void)
{
    // Timer
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT, // 10-bit duty (0..1023)
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 2000, // default 2 kHz; will change per tone
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    // Channel
    ledc_channel_config_t ccfg = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0, // start silent
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg));
}

static void buzzer_stop_cb(void *arg)
{
#if BUZZER_IS_PASSIVE
    // stop PWM
    ESP_ERROR_CHECK(ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
#else
    gpio_set_level(BUZZER_GPIO, 0);
#endif
}

static void buzzer_init(void)
{
    if (buzzer_is_init)
        return;

#if BUZZER_IS_PASSIVE
    buzzer_ledc_init_passive();
#else
    buzzer_gpio_init_active();
#endif

    // Create a reusable one-shot timer to stop tones
    const esp_timer_create_args_t oneshot_args = {
        .callback = &buzzer_stop_cb,
        .name = "buz_stop"};
    ESP_ERROR_CHECK(esp_timer_create(&oneshot_args, &buzzer_stop_timer));

    buzzer_is_init = true;
    ESP_LOGI(TAG_BUZ, "Buzzer init done (GPIO %d, %s)", BUZZER_GPIO,
             BUZZER_IS_PASSIVE ? "passive (LEDC)" : "active (GPIO)");
}

// Non-blocking: start tone for `duration_ms`, auto-stop via timer
static void buzzer_tone(uint32_t freq_hz, uint32_t duration_ms, uint16_t volume /*0..1023*/)
{
    if (!buzzer_is_init)
        buzzer_init();

#if BUZZER_IS_PASSIVE
    if (freq_hz < 50)
        freq_hz = 50;
    if (volume > 1023)
        volume = 1023;

    // update frequency
    ESP_ERROR_CHECK(ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq_hz));
    // set duty (volume)
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, volume));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
#else
    // active buzzer ignores freq; just drive it
    (void)freq_hz;
    (void)volume;
    gpio_set_level(BUZZER_GPIO, 1);
#endif

    if (duration_ms == 0)
        return; // continuous tone until buzzer_stop() called

    // arm one-shot stop
    timer_stop_if_active(buzzer_stop_timer);
    ESP_ERROR_CHECK(esp_timer_start_once(buzzer_stop_timer, (uint64_t)duration_ms * 1000));
}

static void buzzer_stop(void)
{
    if (!buzzer_is_init)
        return;
    buzzer_stop_cb(NULL);
}

// Convenience one-beep (200 ms), nice for clicks
static void buzzer_beep(void)
{
#if BUZZER_IS_PASSIVE
    buzzer_tone(2000, 180, 400); // 2 kHz, 180 ms, mid volume
#else
    buzzer_tone(0, 180, 0);
#endif
}

// Simple melodies (success/error) - non-blocking sequencer using esp_timer chain
typedef struct
{
    const uint16_t *freqs;
    const uint16_t *durms;
    const uint8_t *vols; // 0..1023, may be NULL to use a default
    size_t count;
    size_t index;
    esp_timer_handle_t timer;
} buzzer_seq_t;

static void buzzer_seq_step(void *arg);

static void buzzer_seq_start(const uint16_t *f, const uint16_t *d, const uint8_t *v, size_t n)
{
    static buzzer_seq_t seq = {0};
    if (!buzzer_is_init)
        buzzer_init();

    // stop any current tone & sequence
    buzzer_stop();

    seq.freqs = f;
    seq.durms = d;
    seq.vols = v;
    seq.count = n;
    seq.index = 0;

    if (seq.timer == NULL)
    {
        const esp_timer_create_args_t args = {.callback = buzzer_seq_step, .arg = &seq, .name = "buz_seq"};
        ESP_ERROR_CHECK(esp_timer_create(&args, &seq.timer));
    }
    else
    {
        timer_stop_if_active(seq.timer);
    }

    // kick first step immediately
    buzzer_seq_step(&seq);
}

static void buzzer_seq_step(void *arg)
{
    buzzer_seq_t *s = (buzzer_seq_t *)arg;
    if (s->index >= s->count)
    {
        buzzer_stop();
        return;
    }
    uint32_t freq = s->freqs[s->index];
    uint32_t dur = s->durms[s->index];
    uint16_t vol = s->vols ? s->vols[s->index] : 400;

#if BUZZER_IS_PASSIVE
    buzzer_tone(freq, dur, vol);
#else
    (void)freq;
    (void)vol;
    buzzer_tone(0, dur, 0);
#endif
    s->index++;
    // schedule next note right after this one ends
    ESP_ERROR_CHECK(esp_timer_start_once(s->timer, (uint64_t)dur * 1000));
}

// Example tunes
static void buzzer_success(void)
{
    static const uint16_t f[] = {1200, 1600, 2000};
    static const uint16_t d[] = {100, 100, 160};
    buzzer_seq_start(f, d, NULL, 3);
}
static void buzzer_error(void)
{
    static const uint16_t f[] = {400, 300, 250, 200};
    static const uint16_t d[] = {120, 120, 120, 240};
    buzzer_seq_start(f, d, NULL, 4);
}

//----------------------------WS2812 ------------------------

static led_strip_handle_t strip;

static void set_pixel(uint16_t i, uint8_t r, uint8_t g, uint8_t b)
{
    // The driver takes raw RGB; color order is handled by config.
    // Optionally add a global brightness scale here if necessary
    ESP_ERROR_CHECK(led_strip_set_pixel(strip, i, r, g, b));
}

static void show(void)
{
    ESP_ERROR_CHECK(led_strip_refresh(strip));
}

static void clear_all(void)
{
    ESP_ERROR_CHECK(led_strip_clear(strip));
}

// Simple wheel for rainbow
static void color_wheel(uint8_t pos, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (pos < 85)
    {
        *r = pos * 3;
        *g = 255 - pos * 3;
        *b = 0;
    }
    else if (pos < 170)
    {
        pos -= 85;
        *r = 255 - pos * 3;
        *g = 0;
        *b = pos * 3;
    }
    else
    {
        pos -= 170;
        *r = 0;
        *g = pos * 3;
        *b = 255 - pos * 3;
    }
}

static void effect_fill(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < LED_COUNT; i++)
        set_pixel(i, r, g, b);
    show();
}

static void effect_wipe(uint8_t r, uint8_t g, uint8_t b, int delay_ms)
{
    clear_all();
    for (int i = 0; i < LED_COUNT; i++)
    {
        set_pixel(i, r, g, b);
        show();
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

static void effect_rainbow(int delay_ms)
{
    static uint8_t offset = 0;
    for (int i = 0; i < LED_COUNT; i++)
    {
        uint8_t r, g, b;
        color_wheel((i + offset) & 0xFF, &r, &g, &b);
        set_pixel(i, r, g, b);
    }
    show();
    offset++;
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
}

// ---------- LED one-shot flash helpers (non-blocking) ----------
static esp_timer_handle_t led_clear_timer = NULL;

static void led_clear_cb(void *arg)
{
    clear_all();
}

// set whole strip to a color, then auto-clear after duration_ms (non-blocking)
static void led_flash_color(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms)
{
    effect_fill(r, g, b); // fast (single refresh)
    if (!led_clear_timer)
    {
        const esp_timer_create_args_t targs = {.callback = led_clear_cb, .name = "led_clear"};
        ESP_ERROR_CHECK(esp_timer_create(&targs, &led_clear_timer));
    }
    else
    {
        timer_stop_if_active(led_clear_timer);
    }
    ESP_ERROR_CHECK(esp_timer_start_once(led_clear_timer, (uint64_t)duration_ms * 1000));
}

// quick double flash in the same color (non-blocking chain)
static void led_dbl_step(void *arg)
{
    led_dbl_t *s = (led_dbl_t *)arg;
    if (!s->count)
    {
        clear_all();
        return;
    }
    s->count--;
    effect_fill(s->r, s->g, s->b);
    // off after 120ms, then on again in 240ms total
    static esp_timer_handle_t off_t = NULL;
    if (!off_t)
    {
        const esp_timer_create_args_t off_args = {.callback = led_clear_cb, .name = "led_off"};
        ESP_ERROR_CHECK(esp_timer_create(&off_args, &off_t));
    }
    else
    {
        timer_stop_if_active(off_t);
    }
    ESP_ERROR_CHECK(esp_timer_start_once(off_t, 120000)); // 120ms

    // schedule next on in ~240ms
    if (!s->t)
    {
        const esp_timer_create_args_t on_args = {.callback = led_dbl_step, .arg = s, .name = "led_on"};
        ESP_ERROR_CHECK(esp_timer_create(&on_args, &s->t));
    }
    else
    {
        timer_stop_if_active(s->t);
    }
    ESP_ERROR_CHECK(esp_timer_start_once(s->t, 240000)); // 240ms
}
static void led_double_flash(uint8_t r, uint8_t g, uint8_t b)
{
    static led_dbl_t seq = {0};
    seq.r = r;
    seq.g = g;
    seq.b = b;
    seq.count = 2;
    led_dbl_step(&seq);
}


// -------------------------USB-HID -----------------------------
/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto)
{
    static hid_protocol_t prev_proto_output = HID_PROTOCOL_NONE;

    if (prev_proto_output != proto)
    {
        prev_proto_output = proto;
        printf("\r\n");
        if (proto == HID_PROTOCOL_MOUSE)
        {
            printf("Mouse\r\n");
        }
        if (proto == HID_PROTOCOL_KEYBOARD)
        {
            printf("Keyboard\r\n");
        }
        fflush(stdout);
    }
}

/**
 * @brief HID Keyboard modifier verification for capitalization application (right or left shift)
 *
 * @param[in] modifier
 * @return true  Modifier was pressed (left or right shift)
 * @return false Modifier was not pressed (left or right shift)
 *
 */
static inline bool hid_keyboard_is_modifier_shift(uint8_t modifier)
{
    if ((modifier && HID_LEFT_SHIFT) ||
        (modifier && HID_RIGHT_SHIFT))
    {
        return true;
    }
    return false;
}

/**
 * @brief HID Keyboard get char symbol from key code
 *
 * @param[in] modifier  Keyboard modifier data
 * @param[in] key_code  Keyboard key code
 * @param[in] key_char  Pointer to key char data
 *
 * @return true  Key scancode converted successfully
 * @return false Key scancode unknown
 */
static inline bool hid_keyboard_get_char(uint8_t modifier,
                                         uint8_t key_code,
                                         unsigned char *key_char)
{
    uint8_t mod = (hid_keyboard_is_modifier_shift(modifier)) ? 1 : 0;

    if ((key_code >= HID_KEY_A) && (key_code <= HID_KEY_SLASH))
    {
        *key_char = keycode2ascii[key_code][mod];
    }
    else
    {
        // All other key pressed
        return false;
    }

    return true;
}

/**
 * @brief HID Keyboard print char symbol
 *
 * @param[in] key_char  Keyboard char to stdout
 */
static inline void hid_keyboard_print_char(unsigned int key_char)
{
    if (!!key_char)
    {
        putchar(key_char);
#if (KEYBOARD_ENTER_LF_EXTEND)
        if (KEYBOARD_ENTER_MAIN_CHAR == key_char)
        {
            putchar('\n');
        }
#endif // KEYBOARD_ENTER_LF_EXTEND
        fflush(stdout);
    }
}

/**
 * @brief Key Event. Key event with the key code, state and modifier.
 *
 * @param[in] key_event Pointer to Key Event structure
 *
 */
static void key_event_callback(key_event_t *key_event)
{

    if (KEY_STATE_PRESSED == key_event->state)
    {
        // Enter ends the tag
        if (key_event->key_code == HID_KEY_ENTER)
        {
            if (rfid_len > 0)
            {
                rfid_buf[rfid_len] = '\0';
                printf("\n[RFID] %s\n", rfid_buf);
                send_uid(rfid_buf);
                rfid_len = 0;
            }
            return;
        }

        unsigned char key_char;
        if (hid_keyboard_get_char(key_event->modifier, key_event->key_code, &key_char))
        {
            // Keep only 0-9 A-F a-f (common tag formats); extend if your reader sends other chars
            if (((key_char >= '0' && key_char <= '9') ||
                 (key_char >= 'A' && key_char <= 'F') ||
                 (key_char >= 'a' && key_char <= 'f')) &&
                rfid_len < sizeof(rfid_buf) - 1)
            {
                rfid_buf[rfid_len++] = (char)key_char;
            }
        }
    }
}

/**
 * @brief Key buffer scan code search.
 *
 * @param[in] src       Pointer to source buffer where to search
 * @param[in] key       Key scancode to search
 * @param[in] length    Size of the source buffer
 */
static inline bool key_found(const uint8_t *const src,
                             uint8_t key,
                             unsigned int length)
{
    for (unsigned int i = 0; i < length; i++)
    {
        if (src[i] == key)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(const uint8_t *const data, const int length)
{
    hid_keyboard_input_report_boot_t *kb_report = (hid_keyboard_input_report_boot_t *)data;

    if (length < sizeof(hid_keyboard_input_report_boot_t))
    {
        return;
    }

    static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};
    key_event_t key_event;

    for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++)
    {

        // key has been released verification
        if (prev_keys[i] > HID_KEY_ERROR_UNDEFINED &&
            !key_found(kb_report->key, prev_keys[i], HID_KEYBOARD_KEY_MAX))
        {
            key_event.key_code = prev_keys[i];
            key_event.modifier = 0;
            key_event.state = KEY_STATE_RELEASED;
            key_event_callback(&key_event);
        }

        // key has been pressed verification
        if (kb_report->key[i] > HID_KEY_ERROR_UNDEFINED &&
            !key_found(prev_keys, kb_report->key[i], HID_KEYBOARD_KEY_MAX))
        {
            key_event.key_code = kb_report->key[i];
            key_event.modifier = kb_report->modifier.val;
            key_event.state = KEY_STATE_PRESSED;
            key_event_callback(&key_event);
        }
    }

    memcpy(prev_keys, &kb_report->key, HID_KEYBOARD_KEY_MAX);
}

/**
 * @brief USB HID Host Mouse Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_mouse_report_callback(const uint8_t *const data, const int length)
{
    hid_mouse_input_report_boot_t *mouse_report = (hid_mouse_input_report_boot_t *)data;

    if (length < sizeof(hid_mouse_input_report_boot_t))
    {
        return;
    }

    static int x_pos = 0;
    static int y_pos = 0;

    // Calculate absolute position from displacement
    x_pos += mouse_report->x_displacement;
    y_pos += mouse_report->y_displacement;

    hid_print_new_device_report_header(HID_PROTOCOL_MOUSE);

    printf("X: %06d\tY: %06d\t|%c|%c|\r",
           x_pos, y_pos,
           (mouse_report->buttons.button1 ? 'o' : ' '),
           (mouse_report->buttons.button2 ? 'o' : ' '));
    fflush(stdout);
}

/**
 * @brief USB HID Host event callback. Handle such event as device connection and removing
 *
 * @param[in] event  HID device event
 * @param[in] arg    Pointer to arguments, does not used
 */
static void hid_host_event_callback(const hid_host_event_t *event, void *arg)
{
    if (event->event == HID_DEVICE_CONNECTED)
    {
        // Obtained USB device address is placed after application events
        xEventGroupSetBits(usb_flags, DEVICE_CONNECTED | (event->device.address << 4));
    }
    else if (event->event == HID_DEVICE_DISCONNECTED)
    {
        xEventGroupSetBits(usb_flags, DEVICE_DISCONNECTED);
    }
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] event  HID interface event
 * @param[in] arg    Pointer to arguments, does not used
 */
static void hid_host_interface_event_callback(const hid_host_interface_event_t *event, void *arg)
{
    switch (event->event)
    {
    case HID_DEVICE_INTERFACE_INIT:
        ESP_LOGI(TAG, "Interface number %d, protocol %s",
                 event->interface.num,
                 (event->interface.proto == HID_PROTOCOL_KEYBOARD)
                     ? "Keyboard"
                     : "Mouse");

        if (event->interface.proto == HID_PROTOCOL_KEYBOARD)
        {
            const hid_host_interface_config_t hid_keyboard_config = {
                .proto = HID_PROTOCOL_KEYBOARD,
                .callback = hid_host_keyboard_report_callback,
            };

            hid_host_claim_interface(&hid_keyboard_config, &keyboard_handle);
        }

        if (event->interface.proto == HID_PROTOCOL_MOUSE)
        {
            const hid_host_interface_config_t hid_mouse_config = {
                .proto = HID_PROTOCOL_MOUSE,
                .callback = hid_host_mouse_report_callback,
            };

            hid_host_claim_interface(&hid_mouse_config, &mouse_handle);
        }

        break;
    case HID_DEVICE_INTERFACE_TRANSFER_ERROR:
        ESP_LOGD(TAG, "Interface number %d, transfer error",
                 event->interface.num);
        break;

    case HID_DEVICE_INTERFACE_CLAIM:
    case HID_DEVICE_INTERFACE_RELEASE:
        // ... do nothing here for now
        break;

    default:
        ESP_LOGI(TAG, "%s Unhandled event %X, Interface number %d",
                 __FUNCTION__,
                 event->event,
                 event->interface.num);
        break;
    }
}

/**
 * @brief Handle common USB host library events
 *
 * @param[in] args  Pointer to arguments, does not used
 */
static void handle_usb_events(void *args)
{
    while (1)
    {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        // Release devices once all clients has deregistered
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
        {
            usb_host_device_free_all();
            xEventGroupSetBits(usb_flags, HOST_NO_CLIENT);
        }
        // Give ready_to_uninstall_usb semaphore to indicate that USB Host library
        // can be deinitialized, and terminate this task.
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE)
        {
            xEventGroupSetBits(usb_flags, HOST_ALL_FREE);
        }
    }

    vTaskDelete(NULL);
}

static bool wait_for_event(EventBits_t event, TickType_t timeout)
{
    return xEventGroupWaitBits(usb_flags, event, pdTRUE, pdTRUE, timeout) & event;
}


// -------------------- ESP-NOW ------------------------------------
rfid_msg_t msg;
status_msg_t resp;

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    ESP_LOGI(TAG_NOW, "Send status: %s", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void espnow_init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    esp_now_register_send_cb(espnow_send_cb);
}

void reader_setup_peer(void)
{
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, peer_mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void send_uid(const char *uid_str)
{
    memset(&msg, 0, sizeof(msg));

    strncpy(msg.uid, uid_str, sizeof(msg.uid) - 1);
    msg.dev_id = device_id;

    // simple obfuscation: XOR every char with SECRET_KEY LSB
    for (size_t i = 0; i < strlen(msg.uid); i++)
    {
        msg.uid[i] ^= (SECRET_KEY & 0xFF);
    }

    esp_now_send(peer_mac, (uint8_t *)&msg, sizeof(msg));
}

static void reader_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len)
{
    if (len < sizeof(status_msg_t))
        return;

    memcpy(&resp, data, sizeof(resp));

    if (msg.uid[0] == '\0')
    {
        ESP_LOGW(TAG_NOW, "UID not initialized yet");
        return;
    }
    if (resp.auth_key != ((uint8_t)msg.uid[0] ^ (SECRET_KEY & 0xFF)))
    {
        ESP_LOGW(TAG_NOW, "Bad auth");
        // Red single flash + error chirp
        led_flash_color(60, 0, 0, 300);
        buzzer_error();
        return;
    }

    ESP_LOGI(TAG_NOW,
             "status=%d event=%d ticket=%s name=%s time=%" PRIu32,
             resp.status, resp.event_type,
             resp.ticket_id, resp.name, resp.timestamp);


    // ---- Decide LED + buzzer pattern ----
    if (resp.status == 1)
    {
        // granted
        switch (resp.event_type)
        {
        case EVT_ENTRY:
            // green double-flash + success tri-tone
            led_double_flash(0, 60, 0);
            buzzer_success();
            break;
        case EVT_EXIT:
            // cyan single flash + short beep
            led_flash_color(0, 40, 40, 250);
            buzzer_beep();
            break;
        default:
            // granted but unknown event → soft white flash + single beep
            led_flash_color(40, 40, 40, 250);
            buzzer_beep();
            break;
        }
    }
    else
    {
        // denied / warning
        switch (resp.event_type)
        {
        case EVT_DENIED:
            // strong red flash (longer) + error melody
            led_flash_color(80, 0, 0, 600);
            buzzer_error();
            break;
        case EVT_WRONG_GATE:
            // amber (yellowish) double flash + two medium beeps
            led_double_flash(70, 35, 0);
            buzzer_tone(1200, 120, 500);
            buzzer_tone(900, 120, 500); // queued via timer chain in melody? Keep quick second call after ~150ms if you want
            break;
        default:
            // unknown not-granted → magenta flash + short beep
            led_flash_color(60, 0, 60, 300);
            buzzer_beep();
            break;
        }
    }
}


void app_main(void)
{
    // 0) Configure status pins as outputs once
    const gpio_config_t out_pins = {
        .pin_bit_mask = (1ULL << GPIO_NUM_2) | (1ULL << GPIO_NUM_4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&out_pins));
    gpio_set_level(GPIO_NUM_2, 0);
    gpio_set_level(GPIO_NUM_4, 0);

    // 1) Buzzer init
    buzzer_init();

    // 2) LED Init
    ESP_LOGI(TAG_LED, "Initializing RMT + LED strip driver...");
    gpio_reset_pin(LED_GPIO); // (helps if the pin was used earlier)
    // optional: gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_COUNT,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_COLOR_ORDER,
        .flags = {.invert_out = 0},
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RES_HZ,
        .mem_block_symbols = 64,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &strip));
    clear_all();

    // 3) Startup animation (quick and non-blocking)
    //   • blue flash, then green, then white; success chirp
    led_flash_color(0, 0, 100, 500);
    vTaskDelay(pdMS_TO_TICKS(240));
    led_flash_color(0, 100, 0, 500);
    vTaskDelay(pdMS_TO_TICKS(240));
    led_flash_color(100, 0, 0, 500);
    buzzer_success();

    // RTOS Init and HID init
    TaskHandle_t usb_events_task_handle;
    hid_host_device_handle_t hid_device;
    BaseType_t task_created;

    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin));

    espnow_init();
    esp_now_register_recv_cb(reader_recv_cb);
    reader_setup_peer();

    ESP_LOGI(TAG, "RFID Capture & Relay Started");

    usb_flags = xEventGroupCreate();
    assert(usb_flags);

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1};
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    task_created = xTaskCreate(handle_usb_events, "usb_events", 4096, NULL, 2, &usb_events_task_handle);
    assert(task_created);

    const hid_host_driver_config_t hid_host_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_event_callback,
        .callback_arg = NULL};
    ESP_ERROR_CHECK(hid_host_install(&hid_host_config));

    do
    {
        EventBits_t event = xEventGroupWaitBits(
            usb_flags, USB_EVENTS_TO_WAIT, pdTRUE, pdFALSE,
            pdMS_TO_TICKS(APP_QUIT_PIN_POLL_MS));

        if (event & DEVICE_CONNECTED)
        {
            xEventGroupClearBits(usb_flags, DEVICE_CONNECTED);
            hid_device_connected = true;
        }
        if (event & DEVICE_ADDRESS_MASK)
        {
            xEventGroupClearBits(usb_flags, DEVICE_ADDRESS_MASK);
            const hid_host_device_config_t hid_host_device_config = {
                .dev_addr = (event & DEVICE_ADDRESS_MASK) >> 4,
                .iface_event_cb = hid_host_interface_event_callback,
                .iface_event_arg = NULL,
            };
            ESP_ERROR_CHECK(hid_host_install_device(&hid_host_device_config, &hid_device));
        }
        if (event & DEVICE_DISCONNECTED)
        {
            xEventGroupClearBits(usb_flags, DEVICE_DISCONNECTED);
            hid_host_release_interface(keyboard_handle);
            hid_host_release_interface(mouse_handle);
            ESP_ERROR_CHECK(hid_host_uninstall_device(hid_device));
            hid_device_connected = false;
        }
    } while (gpio_get_level(APP_QUIT_PIN) != 0);

    if (hid_device_connected)
    {
        ESP_LOGI(TAG, "Uninitializing HID Device");
        hid_host_release_interface(keyboard_handle);
        hid_host_release_interface(mouse_handle);
        ESP_ERROR_CHECK(hid_host_uninstall_device(hid_device));
        hid_device_connected = false;
    }

    ESP_LOGI(TAG, "Uninitializing USB");
    ESP_ERROR_CHECK(hid_host_uninstall());
    wait_for_event(READY_TO_UNINSTALL, portMAX_DELAY);
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(usb_events_task_handle);
    vEventGroupDelete(usb_flags);
    ESP_LOGI(TAG, "Done");
}
