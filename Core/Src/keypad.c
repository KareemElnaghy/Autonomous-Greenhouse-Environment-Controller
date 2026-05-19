#include "main.h"
#include "keypad.h"

static I2C_HandleTypeDef *kp_hi2c = NULL;
static uint16_t kp_addr = 0;

static const char keymap[16] = {
    '1', '2', '3', 'A',
    '4', '5', '6', 'B',
    '7', '8', '9', 'C',
    '0', 'F', 'E', 'D'
};

uint8_t keypad_init(I2C_HandleTypeDef *hi2c) {
    kp_hi2c = hi2c;
    uint8_t addrs[] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26,
                       0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F};
    for (uint8_t i = 0; i < sizeof(addrs); i++) {
        uint16_t test = (uint16_t)addrs[i] << 1;
        if (HAL_I2C_IsDeviceReady(kp_hi2c, test, 3, 10) == HAL_OK) {
            kp_addr = test;
            return 0;
        }
    }
    return 1;
}

static char last_key = 0;

char keypad_get_key(void) {
    if (!kp_hi2c || !kp_addr) return 0;

    char pressed = 0;
    for (uint8_t row = 0; row < 4; row++) {
        uint8_t out = 0xF0 | (~(1u << row) & 0x0Fu);
        if (HAL_I2C_Master_Transmit(kp_hi2c, kp_addr, &out, 1, 10) != HAL_OK)
            return 0;
        uint8_t in;
        if (HAL_I2C_Master_Receive(kp_hi2c, kp_addr, &in, 1, 10) != HAL_OK)
            return 0;
        uint8_t cols = (~in >> 4) & 0x0Fu;
        if (cols) {
            for (uint8_t col = 0; col < 4; col++) {
                if (cols & (1u << col)) {
                    uint8_t idx = row * 4 + col;
                    if (idx < 16) {
                        pressed = keymap[idx];
                        break;
                    }
                }
            }
            if (pressed) break;
        }
    }

    if (pressed && pressed == last_key)
        return 0;
    last_key = pressed;
    return pressed;
}
