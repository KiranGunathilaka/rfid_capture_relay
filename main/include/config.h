#ifndef CONFIG_H
#define CONFIG_H

#include "hid_host.h"
#include "esp_timer.h"

#define APP_QUIT_PIN GPIO_NUM_0
#define APP_QUIT_PIN_POLL_MS 500

#define READY_TO_UNINSTALL (HOST_NO_CLIENT | HOST_ALL_FREE)

/* Main char symbol for ENTER key */
#define KEYBOARD_ENTER_MAIN_CHAR '\r'
/* When set to 1 pressing ENTER will be extending with LineFeed during serial debug output */
#define KEYBOARD_ENTER_LF_EXTEND 1

static char rfid_buf[64];
static size_t rfid_len = 0;

static uint8_t const device_id = 101;

/**
 * @brief Application Event from USB Host driver
 *
 */
typedef enum
{
    HOST_NO_CLIENT = 0x1,
    HOST_ALL_FREE = 0x2,
    DEVICE_CONNECTED = 0x4,
    DEVICE_DISCONNECTED = 0x8,
    DEVICE_ADDRESS_MASK = 0xFF0,
} app_event_t;

/**
 * @brief Key event
 *
 */
typedef struct
{
    enum key_state
    {
        KEY_STATE_PRESSED = 0x00,
        KEY_STATE_RELEASED = 0x01
    } state;
    uint8_t modifier;
    uint8_t key_code;
} key_event_t;

#define USB_EVENTS_TO_WAIT (DEVICE_CONNECTED | DEVICE_ADDRESS_MASK | DEVICE_DISCONNECTED)

static const char *TAG = "RFID";
static EventGroupHandle_t usb_flags;
static bool hid_device_connected = false;

hid_host_interface_handle_t keyboard_handle = NULL;
hid_host_interface_handle_t mouse_handle = NULL;

/**
 * @brief Scancode to ascii table
 */
const uint8_t keycode2ascii[57][2] = {
    {0, 0},                                               /* HID_KEY_NO_PRESS        */
    {0, 0},                                               /* HID_KEY_ROLLOVER        */
    {0, 0},                                               /* HID_KEY_POST_FAIL       */
    {0, 0},                                               /* HID_KEY_ERROR_UNDEFINED */
    {'a', 'A'},                                           /* HID_KEY_A               */
    {'b', 'B'},                                           /* HID_KEY_B               */
    {'c', 'C'},                                           /* HID_KEY_C               */
    {'d', 'D'},                                           /* HID_KEY_D               */
    {'e', 'E'},                                           /* HID_KEY_E               */
    {'f', 'F'},                                           /* HID_KEY_F               */
    {'g', 'G'},                                           /* HID_KEY_G               */
    {'h', 'H'},                                           /* HID_KEY_H               */
    {'i', 'I'},                                           /* HID_KEY_I               */
    {'j', 'J'},                                           /* HID_KEY_J               */
    {'k', 'K'},                                           /* HID_KEY_K               */
    {'l', 'L'},                                           /* HID_KEY_L               */
    {'m', 'M'},                                           /* HID_KEY_M               */
    {'n', 'N'},                                           /* HID_KEY_N               */
    {'o', 'O'},                                           /* HID_KEY_O               */
    {'p', 'P'},                                           /* HID_KEY_P               */
    {'q', 'Q'},                                           /* HID_KEY_Q               */
    {'r', 'R'},                                           /* HID_KEY_R               */
    {'s', 'S'},                                           /* HID_KEY_S               */
    {'t', 'T'},                                           /* HID_KEY_T               */
    {'u', 'U'},                                           /* HID_KEY_U               */
    {'v', 'V'},                                           /* HID_KEY_V               */
    {'w', 'W'},                                           /* HID_KEY_W               */
    {'x', 'X'},                                           /* HID_KEY_X               */
    {'y', 'Y'},                                           /* HID_KEY_Y               */
    {'z', 'Z'},                                           /* HID_KEY_Z               */
    {'1', '!'},                                           /* HID_KEY_1               */
    {'2', '@'},                                           /* HID_KEY_2               */
    {'3', '#'},                                           /* HID_KEY_3               */
    {'4', '$'},                                           /* HID_KEY_4               */
    {'5', '%'},                                           /* HID_KEY_5               */
    {'6', '^'},                                           /* HID_KEY_6               */
    {'7', '&'},                                           /* HID_KEY_7               */
    {'8', '*'},                                           /* HID_KEY_8               */
    {'9', '('},                                           /* HID_KEY_9               */
    {'0', ')'},                                           /* HID_KEY_0               */
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR}, /* HID_KEY_ENTER           */
    {0, 0},                                               /* HID_KEY_ESC             */
    {'\b', 0},                                            /* HID_KEY_DEL             */
    {0, 0},                                               /* HID_KEY_TAB             */
    {' ', ' '},                                           /* HID_KEY_SPACE           */
    {'-', '_'},                                           /* HID_KEY_MINUS           */
    {'=', '+'},                                           /* HID_KEY_EQUAL           */
    {'[', '{'},                                           /* HID_KEY_OPEN_BRACKET    */
    {']', '}'},                                           /* HID_KEY_CLOSE_BRACKET   */
    {'\\', '|'},                                          /* HID_KEY_BACK_SLASH      */
    {'\\', '|'},
    /* HID_KEY_SHARP           */ // HOTFIX: for NonUS Keyboards repeat HID_KEY_BACK_SLASH
    {';', ':'},                   /* HID_KEY_COLON           */
    {'\'', '"'},                  /* HID_KEY_QUOTE           */
    {'`', '~'},                   /* HID_KEY_TILDE           */
    {',', '<'},                   /* HID_KEY_LESS            */
    {'.', '>'},                   /* HID_KEY_GREATER         */
    {'/', '?'}                    /* HID_KEY_SLASH           */
};

typedef struct {
    uint8_t dev_id; // device ID
    char uid[64];   // RFID string
} rfid_msg_t;

typedef struct {
    uint8_t status;        // 0 = Do Not Proceed, 1 = Proceed
    uint32_t timestamp;    // UNIX epoch, or millis since boot
    uint8_t event_type;    // e.g. ENTRY / EXIT / DENIED / WRONG_GATE
    char ticket_id[16];    // or user ID
    char name[32];         // user name string
    uint32_t auth_key;     // simple shared secret for authenticity
} status_msg_t;

// ---- Event type codes from the server side ----
#define EVT_ENTRY       1   // granted entry
#define EVT_EXIT        2   // granted exit
#define EVT_DENIED      3   // denied (bad/unknown)
#define EVT_WRONG_GATE  4   // valid but wrong gate


#define SECRET_KEY 0xA5A5F00D

static uint8_t peer_mac[] = {0xF8,0xB3,0xB7,0x26,0x47,0x0C}; //F8:B3:B7:26:47:0C

// --------- LED Settings ----------
#define LED_GPIO            18         // Your data pin to the strip
#define LED_COUNT           60         // Number of LEDs
#define LED_COLOR_ORDER     LED_STRIP_COLOR_COMPONENT_FMT_GRB   // WS2812 = GRB
#define RMT_RES_HZ          10*1000*1000 // 10 MHz RMT tick for accurate WS2812 timings

typedef struct
{
    uint8_t r, g, b;
    uint8_t count;
    esp_timer_handle_t t;
} led_dbl_t;

// --- Buzzer config ------------
#define BUZZER_GPIO        7
#define BUZZER_IS_PASSIVE  1   // 1 = passive (needs tone via PWM/LEDC), 0 = active (on/off)


#endif // CONFIG_H