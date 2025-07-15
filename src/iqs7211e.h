#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

// Regs
#define IQS7211E_MM_PROD_NUM 0x00
#define IQS7211E_INFO_FLAGS_REG 0x0F
#define IQS7211E_SYSTEM_CONTROL_REG 0x33
#define IQS7211E_MM_GESTURES 0x0E
#define IQS7211E_MM_FINGER_2_X 0x14
#define IQS7211E_MM_CONFIG_SETTINGS 0x34
// Prod num
#define IQS7211E_PRODUCT_NUM 0x0458

// Config Settings Bits
#define IQS7211E_EVENT_MODE_BIT 0 // 8
#define IQS7211E_SHOW_RESET_BIT 7
#define IQS7211E_SW_RESET_BIT 1 // (bit 9)
#define IQS7211E_ACK_RESET_BIT 7
#define IQS7211E_TP_RE_ATI_BIT 5
#define IQS7211E_RE_ATI_OCCURRED_BIT 4
#define IQS7211E_SUSPEND_BIT 3 // 11
#define IQS7211E_COMM_REQ_BIT 4

// Power mode Bits
#define IQS7211E_CHARGING_MODE_BIT_0 0
#define IQS7211E_CHARGING_MODE_BIT_1 1
#define IQS7211E_CHARGING_MODE_BIT_2 2
#define IQS7211E_ACTIVE_BITS 0b000
#define IQS7211E_IDLE_TOUCH_BITS 0b001
#define IQS7211E_IDLE_BITS 0b010
#define IQS7211E_LP1_BITS 0b011
#define IQS7211E_LP2_BITS 0b100
// Fingure Bits
#define IQS7211E_NUM_FINGERS_BIT_0 0 // 8
#define IQS7211E_NUM_FINGERS_BIT_1 1 //
// Timeout for suspend
#define IQS7211E_SUSPEND_TIMEOUT_MS 300000

enum iqs7211e_init_state
{
    IQS7211E_INIT_NONE,
    IQS7211E_INIT_VERIFY_PRODUCT,
    IQS7211E_INIT_READ_RESET,
    IQS7211E_INIT_CHIP_RESET,
    IQS7211E_INIT_UPDATE_SETTINGS,
    IQS7211E_INIT_ACK_RESET,
    IQS7211E_INIT_ATI,
    IQS7211E_INIT_WAIT_FOR_ATI,
    IQS7211E_INIT_READ_DATA,
    IQS7211E_INIT_ACTIVATE_EVENT_MODE,
    IQS7211E_INIT_ACTIVATE_STREAM_MODE,
    IQS7211E_INIT_ACTIVATE_COMM_REQ_EN,
    IQS7211E_INIT_DONE,
};

enum iqs7211e_power_mode
{
    IQS7211E_ACTIVE,
    IQS7211E_IDLE_TOUCH,
    IQS7211E_IDLE,
    IQS7211E_LP1,
    IQS7211E_LP2,
    IQS7211E_POWER_UNKNOWN
};

struct iqs7211e_config
{
    struct i2c_dt_spec i2c;
    const struct gpio_dt_spec irq_gpio;
};

struct iqs7211e_data
{
    const struct device *dev;
    struct gpio_callback gpio_cb;
    struct k_work work;
    enum iqs7211e_init_state init_state;
    bool reset_called;
    uint8_t gestures[2];
    uint8_t info_flags[2];
    uint16_t finger_1_x;
    uint16_t finger_1_y;
    uint16_t finger_2_x;
    uint16_t finger_2_y;
    uint16_t finger_1_prev_x;
    uint16_t finger_1_prev_y;
    int16_t finger_1_prev_dx;
    int16_t finger_1_prev_dy;
};
