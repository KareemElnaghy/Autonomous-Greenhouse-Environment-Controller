#include "keypad.h"
#include "cmsis_os.h"

// PmodKYPD layout:
// [ 1 ][ 2 ][ 3 ][ A ]
// [ 4 ][ 5 ][ 6 ][ B ]
// [ 7 ][ 8 ][ 9 ][ C ]
// [ 0 ][ F ][ E ][ D ]

static const char keyMap[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'0', 'F', 'E', 'D'}
};

// Row pins: R1=PA0, R2=PA5, R3=PA9, R4=PA10
static GPIO_TypeDef* rowPorts[4] = {GPIOA, GPIOA, GPIOA, GPIOA};
static uint16_t      rowPins[4]  = {GPIO_PIN_0, GPIO_PIN_5,
                                     GPIO_PIN_9, GPIO_PIN_10};

// Col pins: C1=PA11, C2=PA12, C3=PB0, C4=PB1
static GPIO_TypeDef* colPorts[4] = {GPIOA, GPIOA, GPIOB, GPIOB};
static uint16_t      colPins[4]  = {GPIO_PIN_11, GPIO_PIN_12,
                                     GPIO_PIN_0,  GPIO_PIN_1};

char Keypad_Scan(void)
{
    for (int row = 0; row < 4; row++)
    {
        // Pull current row LOW, all others HIGH
        for (int r = 0; r < 4; r++)
            HAL_GPIO_WritePin(rowPorts[r], rowPins[r],
                              (r == row) ? GPIO_PIN_RESET : GPIO_PIN_SET);

        osDelay(5); // settle time

        // Read columns
        for (int col = 0; col < 4; col++)
        {
            if (HAL_GPIO_ReadPin(colPorts[col], colPins[col]) == GPIO_PIN_RESET)
            {
                // Restore all rows before returning
                for (int r = 0; r < 4; r++)
                    HAL_GPIO_WritePin(rowPorts[r], rowPins[r], GPIO_PIN_SET);
                return keyMap[row][col];
            }
        }
    }

    // No key pressed — restore rows to idle HIGH
    for (int r = 0; r < 4; r++)
        HAL_GPIO_WritePin(rowPorts[r], rowPins[r], GPIO_PIN_SET);

    return 0;
}