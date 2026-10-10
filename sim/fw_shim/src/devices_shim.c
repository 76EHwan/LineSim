/*
 * Stand-ins for board devices the plant model does not need to emulate at
 * register level: LCD (kept as a character grid), DRV8316C gate drivers,
 * MT6701 magnetic encoder, QSPI flash, DFU bootloader, and main.c helpers.
 */
#include "main.h"
#include "sim_mcu.h"
#include "st7789_lcd.h"
#include "st7735_lcd.h"
#include "drv8316crq1.h"
#include "mt6701.h"
#include "w25qxx.h"
#include "bootloader.h"

/* ---------------- LCD (character grid) ---------------- */
static char g_lcd[SIM_LCD_ROWS][SIM_LCD_COLS + 1];
static int g_lcd_inv;
static uint32_t g_lcd_ver;
uint16_t LCD7789_POINT_COLOR = WHITE;
uint16_t LCD7789_BACK_COLOR = BLACK;
uint16_t LCD7789_BACK_BRIGHT = 100;
uint16_t LCD7735_POINT_COLOR = WHITE;
uint16_t LCD7735_BACK_COLOR = BLACK;
uint16_t LCD7735_BACK_BRIGHT = 100;

static int g_lcd_inited;

static void lcd_clear(void) {
	g_lcd_inited = 1;
	for (int r = 0; r < SIM_LCD_ROWS; r++) {
		memset(g_lcd[r], ' ', SIM_LCD_COLS);
		g_lcd[r][SIM_LCD_COLS] = '\0';
	}
	g_lcd_ver++;
}

static void lcd_vprintf(int x, int y, const char *fmt, va_list ap) {
	if (!g_lcd_inited)
		lcd_clear();
	if (y < 0 || y >= SIM_LCD_ROWS)
		return;
	char buf[256];
	vsnprintf(buf, sizeof buf, fmt, ap);
	for (int i = 0; buf[i] && x + i < SIM_LCD_COLS; i++)
		if (x + i >= 0)
			g_lcd[y][x + i] = (buf[i] == '\n') ? ' ' : buf[i];
	g_lcd_ver++;
}

const char* sim_lcd_row(int row) {
	if (!g_lcd_inited)
		lcd_clear();
	return (row >= 0 && row < SIM_LCD_ROWS) ? g_lcd[row] : "";
}
int sim_lcd_inverted(void) { return g_lcd_inv; }
uint32_t sim_lcd_version(void) { return g_lcd_ver; }

void LCD7789_Test(void) { lcd_clear(); }
void LCD7789_SetBrightness(uint32_t Brightness) { (void) Brightness; }
uint32_t LCD7789_GetBrightness(void) { return 100; }
void LCD7789_SoftPWMEnable(uint8_t enable) { (void) enable; }
uint8_t LCD7789_SoftPWMIsEnable(void) { return 0; }
void LCD7789_SoftPWMCtrlInit(void) {}
void LCD7789_SoftPWMCtrlDeInit(void) {}
void LCD7789_SoftPWMCtrlRun(void) {}
void LCD7789_Light(uint32_t Brightness_Dis, uint32_t time) { (void) Brightness_Dis; (void) time; }
void LCD7789_ShowChar(uint16_t x, uint16_t y, uint8_t num, uint8_t size, uint8_t mode) { (void) x; (void) y; (void) num; (void) size; (void) mode; }
void LCD7789_ShowString(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t size, uint8_t *p) {
	(void) x; (void) y; (void) width; (void) height; (void) size; (void) p;
}
void LCD7789_Clear(void) { lcd_clear(); }
void LCD7789_Printf(uint16_t x, uint16_t y, const char *text, ...) {
	va_list ap;
	va_start(ap, text);
	lcd_vprintf(x, y, text, ap);
	va_end(ap);
}
void LCD7789_Display_Random_BMP_From_SD(const TCHAR *address) { (void) address; }
void LCD7789_Set_Color(uint16_t point, uint16_t back) { LCD7789_POINT_COLOR = point; LCD7789_BACK_COLOR = back; }
void LCD7789_Sleep(uint8_t enable) { (void) enable; }
void LCD7789_Invert(uint8_t enable) {
	if (g_lcd_inv != enable)
		g_lcd_ver++;
	g_lcd_inv = enable;
}

void LCD7735_Test(void) { lcd_clear(); }
void LCD7735_Clear(void) { lcd_clear(); }
void LCD7735_Printf(uint8_t x, uint8_t y, const char *text, ...) {
	va_list ap;
	va_start(ap, text);
	lcd_vprintf(x, y, text, ap);
	va_end(ap);
}
void LCD7735_Set_Color(uint16_t point, uint16_t back) { LCD7735_POINT_COLOR = point; LCD7735_BACK_COLOR = back; }

/* ---------------- DRV8316C gate drivers ---------------- */
DRV8316C_Handle_t DRV8316C_L;
DRV8316C_Handle_t DRV8316C_R;
static uint8_t g_drv_regs[2][16];

static int drv_idx(DRV8316C_Handle_t *h) { return h == &DRV8316C_R ? 1 : 0; }

void DRV8316C_Init(DRV8316C_Handle_t *hdrv, SPI_HandleTypeDef *hspi, GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin,
		GPIO_TypeDef *nSLEEP_Port, uint16_t nSLEEP_Pin, GPIO_TypeDef *nFAULT_Port, uint16_t nFAULT_Pin,
		GPIO_TypeDef *DRVOFF_Port, uint16_t DRVOFF_Pin) {
	hdrv->hspi = hspi;
	hdrv->nCS_Port = nCS_Port;
	hdrv->nCS_Pin = nCS_Pin;
	hdrv->nSLEEP_Port = nSLEEP_Port;
	hdrv->nSLEEP_Pin = nSLEEP_Pin;
	hdrv->nFAULT_Port = nFAULT_Port;
	hdrv->nFAULT_Pin = nFAULT_Pin;
	hdrv->DRVOFF_Port = DRVOFF_Port;
	hdrv->DRVOFF_Pin = DRVOFF_Pin;
}
HAL_StatusTypeDef DRV8316C_WriteRegister(DRV8316C_Handle_t *hdrv, uint8_t regAddr, uint8_t data) {
	g_drv_regs[drv_idx(hdrv)][regAddr & 0x0F] = data;
	return HAL_OK;
}
HAL_StatusTypeDef DRV8316C_ReadRegister(DRV8316C_Handle_t *hdrv, uint8_t regAddr, uint8_t *pData) {
	*pData = g_drv_regs[drv_idx(hdrv)][regAddr & 0x0F];
	return HAL_OK;
}
HAL_StatusTypeDef DRV8316C_UnlockRegister(DRV8316C_Handle_t *hdrv) { (void) hdrv; return HAL_OK; }
HAL_StatusTypeDef DRV8316C_LockRegister(DRV8316C_Handle_t *hdrv) { (void) hdrv; return HAL_OK; }
HAL_StatusTypeDef DRV8316C_ApplyDefaultConfig(DRV8316C_Handle_t *hdrv) { (void) hdrv; return HAL_OK; }
HAL_StatusTypeDef DRV8316C_ClearFaults(DRV8316C_Handle_t *hdrv) { (void) hdrv; return HAL_OK; }
void Test_DRV8316C_Read_Status(DRV8316C_Handle_t *hdrv) { (void) hdrv; }

void MX_DRV8316C_Init(void) {
	DRV8316C_Init(&DRV8316C_L, &hspi2, MTR_CS_L_GPIO_Port, MTR_CS_L_Pin, MTR_nSLEEP_L_GPIO_Port, MTR_nSLEEP_L_Pin,
			MTR_nFAULT_L_GPIO_Port, MTR_nFAULT_L_Pin, MTR_DRVOFF_L_GPIO_Port, MTR_DRVOFF_L_Pin);
	DRV8316C_Init(&DRV8316C_R, &hspi2, MTR_CS_R_GPIO_Port, MTR_CS_R_Pin, MTR_nSLEEP_R_GPIO_Port, MTR_nSLEEP_R_Pin,
			MTR_nFAULT_R_GPIO_Port, MTR_nFAULT_R_Pin, MTR_DRVOFF_R_GPIO_Port, MTR_DRVOFF_R_Pin);
	DRV8316C_WAKEUP(&DRV8316C_L);
	DRV8316C_WAKEUP(&DRV8316C_R);
	DRV8316C_DRVOFF_LOW(&DRV8316C_L);
	DRV8316C_DRVOFF_LOW(&DRV8316C_R);
	/* nFAULT is active low: report "no fault" */
	sim_gpio_set_input((int) (MTR_nFAULT_L_GPIO_Port - sim_gpio), MTR_nFAULT_L_Pin, 1);
	sim_gpio_set_input((int) (MTR_nFAULT_R_GPIO_Port - sim_gpio), MTR_nFAULT_R_Pin, 1);
}

/* ---------------- MT6701 magnetic encoder (test menu only) ---------------- */
MT6701_Data_t encDataL;
MT6701_Data_t encDataR;
HAL_StatusTypeDef MT6701_Init(MT6701_Data_t *encData, uint8_t *rxBuffer) { (void) encData; (void) rxBuffer; return HAL_OK; }
void MT6701_ReadSSI(MT6701_Data_t *encData) { (void) encData; }
void Swap_MT6701_Spi_Mode(void) {}

/* ---------------- QSPI NOR flash ---------------- */
uint8_t W25Qx_Init(void) { return 0; }
void W25Qx_Reset(void) {}
uint8_t W25Qx_GetStatus(void) { return 0; }
uint8_t W25Qx_WriteEnable(void) { return 0; }
void W25Qx_Read_ID(uint16_t *ID) { *ID = 0; }
uint8_t W25Qx_Read(uint8_t *pData, uint32_t ReadAddr, uint32_t Size) { (void) ReadAddr; memset(pData, 0xFF, Size); return 0; }
uint8_t W25Qx_Write(uint8_t *pData, uint32_t WriteAddr, uint32_t Size) { (void) pData; (void) WriteAddr; (void) Size; return 0; }
uint8_t W25Qx_Erase_Block(uint32_t Address) { (void) Address; return 0; }
uint8_t W25Qx_Erase_Chip(void) { return 0; }
void W25QXX_Test(void) {}

/* ---------------- DFU bootloader ---------------- */
void JumpToBootloader(void) { sim_fw_halt("JumpToBootloader"); }
void Check_Bootloader_Request(void) {}
void Boot_Loading(void) {
	LCD7789_Clear();
	LCD7789_Printf(0, 0, "DFU (sim: ignored)");
	HAL_Delay(500);
	LCD7789_Clear();
}

/* ---------------- main.c helpers ---------------- */
void Error_Handler(void) { sim_fw_halt("Error_Handler"); }
void LED_Blink(uint32_t delay) {
	HAL_GPIO_TogglePin(E3_GPIO_Port, E3_Pin);
	HAL_Delay(delay);
}
void LED_Test(void) {}
