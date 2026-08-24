#include "display_app.h"
#include "OLED.h"
#include "cmsis_os.h"
#include "main.h"
#include <stdint.h>

typedef enum { DISPLAY_SELECT, DISPLAY_CONFIRM, DISPLAY_ENABLED } DisplayState_e;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState state;
    uint8_t count;
} DisplayButton_s;

static const Motor_Type_e motor_types[] = {GM6020, M3508};
static const char *const motor_names[] = {"GM6020", "M3508"};
static DisplayButton_s buttons[] = {
    {KEY1_GPIO_Port, KEY1_Pin, GPIO_PIN_SET, 0U},
    {KEY2_GPIO_Port, KEY2_Pin, GPIO_PIN_SET, 0U},
    {KEY3_GPIO_Port, KEY3_Pin, GPIO_PIN_SET, 0U},
    {KEY4_GPIO_Port, KEY4_Pin, GPIO_PIN_SET, 0U},
};
static uint8_t selected_index = 1U;
static DisplayState_e state = DISPLAY_SELECT;
static volatile uint8_t type_confirmed = 0U;
static volatile Motor_Type_e selected_type = M3508;

#define MOTOR_COUNT (sizeof(motor_types) / sizeof(motor_types[0]))

static uint8_t ButtonPressed(DisplayButton_s *button)
{
    GPIO_PinState raw = HAL_GPIO_ReadPin(button->port, button->pin);
    if (raw == button->state) {
        button->count = 0U;
        return 0U;
    }
    if (++button->count < 3U) return 0U;
    button->count = 0U;
    button->state = raw;
    return raw == GPIO_PIN_RESET;
}

static void DrawSelect(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, "MOTOR TYPE", OLED_6X8);
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++) {
        OLED_ShowChar(0, (int16_t)(16U + i * 16U), i == selected_index ? '>' : ' ', OLED_6X8);
        OLED_ShowString(12, (int16_t)(16U + i * 16U), (char *)motor_names[i], OLED_6X8);
    }
    OLED_ShowString(0, 56, "K3:OK", OLED_6X8);
    OLED_Update();
}

static void DrawConfirm(void)
{
    OLED_Clear();
    OLED_ShowString(0, 0, "ENABLE MOTOR?", OLED_6X8);
    OLED_ShowString(0, 24, (char *)motor_names[selected_index], OLED_8X16);
    OLED_ShowString(0, 56, "K3:OK K4:BACK", OLED_6X8);
    OLED_Update();
}

static void DrawEnabled(void)
{
    OLED_Clear();
    OLED_ShowString(0, 8, "MOTOR ENABLED", OLED_6X8);
    OLED_ShowString(0, 32, (char *)motor_names[selected_index], OLED_8X16);
    OLED_Update();
}

void DisplayAppInit(void)
{
    OLED_Init();
    for (uint8_t i = 0U; i < 4U; i++) {
        buttons[i].state = HAL_GPIO_ReadPin(buttons[i].port, buttons[i].pin);
    }
    DrawSelect();
}

uint8_t DisplayAppMotorTypeConfirmed(void) { return type_confirmed; }
Motor_Type_e DisplayAppGetSelectedMotorType(void) { return selected_type; }

void DisplayAppTask(void *argument)
{
    (void)argument;
    DisplayAppInit();
    for (;;) {
        uint8_t up = ButtonPressed(&buttons[0]);
        uint8_t down = ButtonPressed(&buttons[1]);
        uint8_t ok = ButtonPressed(&buttons[2]);
        uint8_t back = ButtonPressed(&buttons[3]);

        if (state == DISPLAY_SELECT) {
            if (up) {
                selected_index = selected_index == 0U ? (uint8_t)(MOTOR_COUNT - 1U) : selected_index - 1U;
                DrawSelect();
            } else if (down) {
                selected_index = (uint8_t)((selected_index + 1U) % MOTOR_COUNT);
                DrawSelect();
            } else if (ok) {
                state = DISPLAY_CONFIRM;
                DrawConfirm();
            }
        } else if (state == DISPLAY_CONFIRM) {
            if (back) {
                state = DISPLAY_SELECT;
                DrawSelect();
            } else if (ok) {
                selected_type = motor_types[selected_index];
                type_confirmed = 1U;
                state = DISPLAY_ENABLED;
                DrawEnabled();
            }
        }
        osDelay(10);
    }
}
