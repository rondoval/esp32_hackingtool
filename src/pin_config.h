#pragma once

/* LCD CONFIG */
#define LVGL_TICK 2                          // ms
#define LCD_PIXEL_CLOCK_HZ (2 * 1000 * 1000) // 10 non-PSRAM
// The pixel number in horizontal and vertical
#define LCD_H_RES 320
#define LCD_V_RES 170
#define LVGL_LCD_BUF_SIZE (LCD_H_RES * LCD_V_RES)
#define PSRAM_DATA_ALIGNMENT 64

/*ESP32S3*/
#define PIN_LCD_BL 38

#define PIN_LCD_D0 39
#define PIN_LCD_D1 40
#define PIN_LCD_D2 41
#define PIN_LCD_D3 42
#define PIN_LCD_D4 45
#define PIN_LCD_D5 46
#define PIN_LCD_D6 47
#define PIN_LCD_D7 48

#define PIN_POWER_ON 15

#define PIN_LCD_RES 5
#define PIN_LCD_CS 6
#define PIN_LCD_DC 7
#define PIN_LCD_WR 8
#define PIN_LCD_RD 9

#define PIN_BUTTON_1 0
#define PIN_BUTTON_2 14
#define PIN_BAT_VOLT 4
