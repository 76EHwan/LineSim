/* ST7735 LCD API stand-in (the firmware uses the ST7789 one; this keeps user_init.h compiling). */
#ifndef BSP_ST7735_ST7735_LCD_H
#define BSP_ST7735_ST7735_LCD_H

#include "main.h"
#include <stdio.h>
#include "SDcard.h"

#ifdef __cplusplus
extern "C" {
#endif

extern uint16_t LCD7735_POINT_COLOR;
extern uint16_t LCD7735_BACK_COLOR;
extern uint16_t LCD7735_BACK_BRIGHT;

void LCD7735_Test(void);
void LCD7735_Clear(void);
void LCD7735_Printf(uint8_t x, uint8_t y, const char *text, ...);
void LCD7735_Set_Color(uint16_t point, uint16_t back);

#ifdef __cplusplus
}
#endif
#endif /* BSP_ST7735_ST7735_LCD_H */
