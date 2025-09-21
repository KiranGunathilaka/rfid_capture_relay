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
#include <string.h>
#include <inttypes.h>
#include "config.h"
#include "nvs_flash.h"

#include "hid_host.h"
#include "hid_usage_keyboard.h"
#include "hid_usage_mouse.h"

#include "esp_wifi.h"
#include "esp_now.h"

static const char *TAG_NOW = "ESP-NOW";

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
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
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
    if (len < sizeof(status_msg_t)) return;

    memcpy(&resp, data, sizeof(resp));

    if (msg.uid[0] == '\0') { // not initialized yet
        ESP_LOGW(TAG_NOW, "UID not initialized yet");
        return;
    }

    if (resp.auth_key != ((uint8_t)msg.uid[0] ^ (SECRET_KEY & 0xFF))) {
        ESP_LOGW(TAG_NOW, "Bad auth");
        return;
    }

    ESP_LOGI(TAG_NOW,
             "status=%d event=%d ticket=%s name=%s time=%" PRIu32,
             resp.status, resp.event_type,
             resp.ticket_id, resp.name, resp.timestamp);

    if (resp.status == 1) {
        gpio_set_level(GPIO_NUM_2, 1);
    } else {
        gpio_set_level(GPIO_NUM_4, 1);
    }
}

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
    // unsigned char key_char;

    // hid_print_new_device_report_header(HID_PROTOCOL_KEYBOARD);

    // if (KEY_STATE_PRESSED == key_event->state) {
    //     if (hid_keyboard_get_char(key_event->modifier,
    //                               key_event->key_code, &key_char)) {

    //         hid_keyboard_print_char(key_char);

    //     }
    // }

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

void app_main(void)
{
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

    // hid host driver config
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
        EventBits_t event = xEventGroupWaitBits(usb_flags, USB_EVENTS_TO_WAIT, pdTRUE, pdFALSE, pdMS_TO_TICKS(APP_QUIT_PIN_POLL_MS));

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
