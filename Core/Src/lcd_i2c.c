#include "main.h"
#include "lcd_i2c.h"
#include "cmsis_os.h"
/* -- Private state --------------------------------------------------------- */
static I2C_HandleTypeDef *_hi2c;
static uint8_t _backlight = LCD_BL;   /* backlight on by default */

/* -- Private helpers ------------------------------------------------------- */

static void PCF8574_Write(uint8_t data)
{
    HAL_I2C_Master_Transmit(_hi2c, LCD_I2C_ADDR, &data, 1, HAL_MAX_DELAY);
}

static void LCD_PulseEnable(uint8_t data)
{
    PCF8574_Write(data | LCD_EN);    /* EN high */
    osDelay(1);
    PCF8574_Write(data & ~LCD_EN);   /* EN low  */
    osDelay(1);
}

static void LCD_SendNibble(uint8_t nibble, uint8_t rs)
{
    uint8_t byte = (nibble & 0xF0) | _backlight | rs;
    LCD_PulseEnable(byte);
}

static void LCD_Send(uint8_t value, uint8_t rs)
{
    LCD_SendNibble(value & 0xF0,        rs);   /* high nibble first */
    LCD_SendNibble((value << 4) & 0xF0, rs);   /* low  nibble second */
}

/* -- Public API ------------------------------------------------------------ */

void LCD_Init(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;
    osDelay(50);

    /* HD44780 reset sequence in 8-bit mode (datasheet requirement) */
    LCD_SendNibble(0x30, 0);  osDelay(5);
    LCD_SendNibble(0x30, 0);  osDelay(1);
    LCD_SendNibble(0x30, 0);  osDelay(1);

    /* Switch to 4-bit mode */
    LCD_SendNibble(0x20, 0);  osDelay(1);

    /* Configure display */
    LCD_Send(LCD_FUNCTION_4B, 0);   /* 4-bit, 2 lines, 5x8 font */
    LCD_Send(LCD_DISPLAY_ON,  0);   /* display on, cursor off    */
    LCD_Send(LCD_CLEAR,       0);   /* clear display             */
    osDelay(2);
    LCD_Send(LCD_ENTRY_MODE,  0);   /* cursor moves right        */
}

void LCD_Clear(void)
{
    LCD_Send(LCD_CLEAR, 0);
    osDelay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t row_offsets[] = {0x00, 0x40};
    LCD_Send(0x80 | (row_offsets[row] + col), 0);
}

void LCD_PrintChar(char c)
{
    LCD_Send((uint8_t)c, LCD_RS);
}

void LCD_Print(const char *str)
{
    while (*str)
        LCD_PrintChar(*str++);
}

void LCD_Backlight(uint8_t on)
{
    _backlight = on ? LCD_BL : 0x00;
    PCF8574_Write(_backlight);
}