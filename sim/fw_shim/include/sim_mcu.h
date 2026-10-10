/*
 * State shared between the HAL shim (C, firmware side) and the simulator (C++).
 * Plain C types only, so C++ code can include it without the HAL headers.
 */
#ifndef SIM_MCU_H
#define SIM_MCU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	volatile uint32_t tick;         /* HAL tick [ms] */

	uint8_t  tim_it[18];            /* update interrupt enabled (HAL_TIM_Base_Start_IT) */
	uint8_t  tim_run[18];           /* counter running */
	uint8_t  tim_pwm[18];           /* bit n: PWM channel n+1 output enabled */

	uint8_t  lptim_enc[6];          /* encoder mode running */
	uint8_t  lptim_it[6];           /* counter interrupt running */

	uint8_t  adc_dma[4];            /* regular conversions + DMA running */
	uint16_t *adc_dma_buf[4];
	uint32_t adc_dma_len[4];
	uint8_t  adc_inj[4];            /* injected conversions + IRQ running */

	uint8_t  dac_dma;               /* buzzer waveform running */

	/* one outstanding I2C DMA read (the IMU) */
	uint8_t  i2c_dma_busy;
	void    *i2c_dma_handle;
	uint16_t i2c_dma_dev, i2c_dma_reg, i2c_dma_len;
	uint8_t *i2c_dma_buf;

	uint8_t  flash_unlocked;
	uint32_t bkp_reg[32];
} SimMcuState;

extern SimMcuState sim_mcu;

/* Updates time-derived registers (DWT->CYCCNT at SystemCoreClock). */
void sim_mcu_set_time(uint64_t t_ns);
/* Timer CR1 access for timers the firmware drives through registers (TIM2 one-pulse). */
uint32_t sim_tim_cr1(int tim);
void sim_tim_clear_cen(int tim);

/* Applies pending BSRR writes to ODR on every GPIO port. */
void sim_gpio_commit(void);
/* Drives an input pin (IDR) on GPIO port `port` (0 = A ... 10 = K). */
void sim_gpio_set_input(int port, uint16_t pin_mask, int level);

/* LCD character grid kept by the LCD stand-in. */
#define SIM_LCD_COLS 40
#define SIM_LCD_ROWS 24
const char *sim_lcd_row(int row);
int sim_lcd_inverted(void);
uint32_t sim_lcd_version(void);   /* increments on every change */

/* Erased-flash region the firmware addresses directly (menu.c "Last Used"). */
int sim_flash_map(void);

/* ---- Hooks implemented by the simulator ---- */
uint64_t sim_now_ns(void);
int      sim_in_fw_main(void);
void     sim_fw_yield(void);
void     sim_fw_sleep_until_ns(uint64_t t_ns);
void     sim_fw_halt(const char *why);
int      sim_i2c_mem_read(uint16_t dev, uint16_t reg, uint8_t *buf, uint16_t len);
int      sim_i2c_mem_write(uint16_t dev, uint16_t reg, const uint8_t *buf, uint16_t len);
const char *sim_sd_root(void);
void     sim_buzzer(int on);

#ifdef __cplusplus
}
#endif
#endif /* SIM_MCU_H */
