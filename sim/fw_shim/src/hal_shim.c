/*
 * HAL stand-in: peripheral register blocks, CubeMX handles, and the HAL calls the
 * firmware makes. Starting/stopping a peripheral is recorded in sim_mcu; the
 * simulator turns that into interrupts on the virtual timeline.
 */
#include "stm32h7xx_hal.h"
#include "main.h"
#include "sim_mcu.h"

SimMcuState sim_mcu;

/* ---------------- register blocks ---------------- */
GPIO_TypeDef   sim_gpio[11];
GPIO_TypeDef   sim_sensor_port[SIM_PORT_COUNT];
TIM_TypeDef    sim_tim[18];
LPTIM_TypeDef  sim_lptim[6];
ADC_TypeDef    sim_adc[4];
DAC_TypeDef    sim_dac1;
I2C_TypeDef    sim_i2c[5];
SPI_TypeDef    sim_spi[7];
DWT_Type       sim_DWT;
CoreDebug_Type sim_CoreDebug;
uint32_t SystemCoreClock = 480000000UL;

/* ---------------- CubeMX handles (Core/Src/*.c are not compiled) ---------------- */
TIM_HandleTypeDef htim1  = { TIM1 };
TIM_HandleTypeDef htim2  = { TIM2 };
TIM_HandleTypeDef htim3  = { TIM3 };
TIM_HandleTypeDef htim4  = { TIM4 };
TIM_HandleTypeDef htim5  = { TIM5 };
TIM_HandleTypeDef htim6  = { TIM6 };
TIM_HandleTypeDef htim7  = { TIM7 };
TIM_HandleTypeDef htim13 = { TIM13 };
TIM_HandleTypeDef htim14 = { TIM14 };
TIM_HandleTypeDef htim15 = { TIM15 };
LPTIM_HandleTypeDef hlptim1 = { LPTIM1 };
LPTIM_HandleTypeDef hlptim2 = { LPTIM2 };
LPTIM_HandleTypeDef hlptim3 = { LPTIM3 };
LPTIM_HandleTypeDef hlptim4 = { LPTIM4 };
ADC_HandleTypeDef hadc1 = { ADC1 };
ADC_HandleTypeDef hadc2 = { ADC2 };
ADC_HandleTypeDef hadc3 = { ADC3 };
DAC_HandleTypeDef hdac1 = { DAC1 };
I2C_HandleTypeDef hi2c1 = { I2C1 };
SPI_HandleTypeDef hspi1 = { SPI1 };
SPI_HandleTypeDef hspi2 = { SPI2 };
SPI_HandleTypeDef hspi4 = { SPI4 };
QSPI_HandleTypeDef hqspi;
RNG_HandleTypeDef  hrng;
RTC_HandleTypeDef  hrtc;
SD_HandleTypeDef   hsd1;
UART_HandleTypeDef huart2;

static int tim_index(const TIM_HandleTypeDef *h) { return (int) (h->Instance - sim_tim); }
static int lptim_index(const LPTIM_HandleTypeDef *h) { return (int) (h->Instance - sim_lptim); }
static int adc_index(const ADC_HandleTypeDef *h) { return (int) (h->Instance - sim_adc); }

void sim_mcu_set_time(uint64_t t_ns) {
	sim_DWT.CYCCNT = (uint32_t) ((t_ns * (uint64_t) (SystemCoreClock / 1000000u)) / 1000u);
}

uint32_t sim_tim_cr1(int tim) { return (tim > 0 && tim < 18) ? sim_tim[tim].CR1 : 0; }
void sim_tim_clear_cen(int tim) {
	if (tim > 0 && tim < 18)
		sim_tim[tim].CR1 &= ~TIM_CR1_CEN;
}

/* ---------------- core / tick ---------------- */
uint32_t HAL_GetTick(void) { return sim_mcu.tick; }
void HAL_IncTick(void) { sim_mcu.tick++; }

void HAL_Delay(uint32_t Delay) {
	/* Same contract as the ST implementation: wait for Delay+1 tick increments
	 * (minimum wait guarantee), but sleep on the virtual timeline instead of spinning. */
	uint32_t wait = Delay;
	if (wait < HAL_MAX_DELAY)
		wait += 1;
	if (!sim_in_fw_main())
		return;
	sim_fw_sleep_until_ns(((uint64_t) sim_mcu.tick + wait) * 1000000ull);
}

void SCB_CleanInvalidateDCache_by_Addr(volatile void *addr, int32_t dsize) { (void) addr; (void) dsize; }
void SCB_InvalidateDCache_by_Addr(volatile void *addr, int32_t dsize) { (void) addr; (void) dsize; }
void SCB_CleanDCache_by_Addr(volatile void *addr, int32_t dsize) { (void) addr; (void) dsize; }
void NVIC_SystemReset(void) { sim_fw_halt("NVIC_SystemReset"); }
void HAL_NVIC_SystemReset(void) { sim_fw_halt("HAL_NVIC_SystemReset"); }

/* ---------------- GPIO ---------------- */
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState) {
	if (PinState != GPIO_PIN_RESET)
		GPIOx->ODR |= GPIO_Pin;
	else
		GPIOx->ODR &= ~(uint32_t) GPIO_Pin;
}

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	return (GPIOx->IDR & GPIO_Pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIOx->ODR ^= GPIO_Pin;
}

static void commit_port(GPIO_TypeDef *p) {
	uint32_t v = p->BSRR;
	if (v) {
		p->ODR |= (v & 0xFFFFu);
		p->ODR &= ~(v >> 16);
		p->BSRR = 0;
	}
}

void sim_gpio_commit(void) {
	for (int i = 0; i < 11; i++)
		commit_port(&sim_gpio[i]);
	for (int i = 0; i < SIM_PORT_COUNT; i++)
		commit_port(&sim_sensor_port[i]);
}

void sim_gpio_set_input(int port, uint16_t pin_mask, int level) {
	if (port < 0 || port >= 11)
		return;
	if (level)
		sim_gpio[port].IDR |= pin_mask;
	else
		sim_gpio[port].IDR &= ~(uint32_t) pin_mask;
}

/* ---------------- TIM ---------------- */
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim) {
	sim_mcu.tim_run[tim_index(htim)] = 1;
	htim->Instance->CR1 |= TIM_CR1_CEN;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim) {
	sim_mcu.tim_run[tim_index(htim)] = 0;
	htim->Instance->CR1 &= ~TIM_CR1_CEN;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim) {
	int i = tim_index(htim);
	sim_mcu.tim_run[i] = 1;
	sim_mcu.tim_it[i] = 1;
	htim->Instance->CR1 |= TIM_CR1_CEN;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim) {
	int i = tim_index(htim);
	sim_mcu.tim_it[i] = 0;
	sim_mcu.tim_run[i] = 0;
	htim->Instance->CR1 &= ~TIM_CR1_CEN;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel) {
	int i = tim_index(htim);
	sim_mcu.tim_pwm[i] |= (uint8_t) (1u << (Channel >> 2));
	sim_mcu.tim_run[i] = 1;
	htim->Instance->CR1 |= TIM_CR1_CEN;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel) {
	int i = tim_index(htim);
	sim_mcu.tim_pwm[i] &= (uint8_t) ~(1u << (Channel >> 2));
	if (!sim_mcu.tim_pwm[i] && !sim_mcu.tim_it[i]) {
		sim_mcu.tim_run[i] = 0;
		htim->Instance->CR1 &= ~TIM_CR1_CEN;
	}
	return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel) { return HAL_TIM_PWM_Start(htim, Channel); }
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel) { return HAL_TIM_PWM_Stop(htim, Channel); }
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel) { (void) htim; (void) Channel; return HAL_OK; }
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel) { (void) htim; (void) Channel; return HAL_OK; }

/* ---------------- LPTIM ---------------- */
HAL_StatusTypeDef HAL_LPTIM_Encoder_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period) {
	hlptim->Instance->ARR = Period;
	sim_mcu.lptim_enc[lptim_index(hlptim)] = 1;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_LPTIM_Encoder_Stop(LPTIM_HandleTypeDef *hlptim) {
	sim_mcu.lptim_enc[lptim_index(hlptim)] = 0;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_LPTIM_Counter_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period) {
	hlptim->Instance->ARR = Period;
	sim_mcu.lptim_it[lptim_index(hlptim)] = 1;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_LPTIM_Counter_Stop_IT(LPTIM_HandleTypeDef *hlptim) {
	sim_mcu.lptim_it[lptim_index(hlptim)] = 0;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_LPTIM_Counter_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period) { hlptim->Instance->ARR = Period; return HAL_OK; }
HAL_StatusTypeDef HAL_LPTIM_Counter_Stop(LPTIM_HandleTypeDef *hlptim) { (void) hlptim; return HAL_OK; }

/* ---------------- ADC ---------------- */
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t CalibrationMode, uint32_t SingleDiff) {
	(void) hadc; (void) CalibrationMode; (void) SingleDiff;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc) { (void) hadc; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc) { (void) hadc; return HAL_OK; }
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout) { (void) hadc; (void) Timeout; return HAL_OK; }
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc) { return hadc->Instance->DR; }

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length) {
	int i = adc_index(hadc);
	sim_mcu.adc_dma[i] = 1;
	sim_mcu.adc_dma_buf[i] = (uint16_t*) pData;
	sim_mcu.adc_dma_len[i] = Length;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc) {
	sim_mcu.adc_dma[adc_index(hadc)] = 0;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *hadc) {
	sim_mcu.adc_inj[adc_index(hadc)] = 1;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef *hadc) {
	sim_mcu.adc_inj[adc_index(hadc)] = 0;
	return HAL_OK;
}

/* ---------------- DAC (buzzer) ---------------- */
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *hdac, uint32_t Channel) { (void) hdac; (void) Channel; return HAL_OK; }
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef *hdac, uint32_t Channel) { (void) hdac; (void) Channel; return HAL_OK; }
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data) {
	(void) Channel; (void) Alignment;
	hdac->Instance->DHR12R1 = Data;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel, const uint32_t *pData, uint32_t Length, uint32_t Alignment) {
	(void) hdac; (void) Channel; (void) pData; (void) Length; (void) Alignment;
	if (!sim_mcu.dac_dma)
		sim_buzzer(1);
	sim_mcu.dac_dma = 1;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel) {
	(void) hdac; (void) Channel;
	if (sim_mcu.dac_dma)
		sim_buzzer(0);
	sim_mcu.dac_dma = 0;
	return HAL_OK;
}

/* ---------------- I2C (IMU) ---------------- */
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
		uint8_t *pData, uint16_t Size, uint32_t Timeout) {
	(void) hi2c; (void) MemAddSize; (void) Timeout;
	return sim_i2c_mem_read(DevAddress, MemAddress, pData, Size) ? HAL_OK : HAL_ERROR;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
		uint8_t *pData, uint16_t Size, uint32_t Timeout) {
	(void) hi2c; (void) MemAddSize; (void) Timeout;
	return sim_i2c_mem_write(DevAddress, MemAddress, pData, Size) ? HAL_OK : HAL_ERROR;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize,
		uint8_t *pData, uint16_t Size) {
	(void) MemAddSize;
	if (sim_mcu.i2c_dma_busy)
		return HAL_BUSY;
	sim_mcu.i2c_dma_busy = 1;
	sim_mcu.i2c_dma_handle = hi2c;
	sim_mcu.i2c_dma_dev = DevAddress;
	sim_mcu.i2c_dma_reg = MemAddress;
	sim_mcu.i2c_dma_buf = pData;
	sim_mcu.i2c_dma_len = Size;
	return HAL_OK;
}

/* ---------------- SPI (gate drivers, magnetic encoder: not modelled) ---------------- */
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, const uint8_t *pData, uint16_t Size, uint32_t Timeout) {
	(void) hspi; (void) pData; (void) Size; (void) Timeout;
	return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
	(void) hspi; (void) Timeout;
	memset(pData, 0, Size);
	return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
	(void) hspi; (void) pTxData; (void) Timeout;
	memset(pRxData, 0, Size);
	return HAL_OK;
}

/* ---------------- internal flash (Bank2 sector 7, mapped by the simulator) ---------------- */
#define SIM_FLASH_LO 0x081E0000UL
#define SIM_FLASH_HI 0x08200000UL

HAL_StatusTypeDef HAL_FLASH_Unlock(void) { sim_mcu.flash_unlocked = 1; return HAL_OK; }
HAL_StatusTypeDef HAL_FLASH_Lock(void) { sim_mcu.flash_unlocked = 0; return HAL_OK; }

HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t FlashAddress, uint32_t DataAddress) {
	(void) TypeProgram;
	if (!sim_mcu.flash_unlocked || FlashAddress < SIM_FLASH_LO || FlashAddress + 32 > SIM_FLASH_HI)
		return HAL_ERROR;
	memcpy((void*) (uintptr_t) FlashAddress, (const void*) (uintptr_t) DataAddress, 32); /* one 256-bit flash word */
	return HAL_OK;
}

HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError) {
	*SectorError = 0xFFFFFFFFu;
	if (!sim_mcu.flash_unlocked)
		return HAL_ERROR;
	uint32_t base = (pEraseInit->Banks == FLASH_BANK_2 ? FLASH_BANK2_BASE : FLASH_BASE);
	for (uint32_t s = 0; s < pEraseInit->NbSectors || (pEraseInit->NbSectors == 0 && s == 0); s++) {
		uint32_t addr = base + (pEraseInit->Sector + s) * FLASH_SECTOR_SIZE;
		if (addr >= SIM_FLASH_LO && addr < SIM_FLASH_HI)
			memset((void*) (uintptr_t) addr, 0xFF, FLASH_SECTOR_SIZE);
	}
	return HAL_OK;
}

/* ---------------- RTC backup registers ---------------- */
void HAL_PWR_EnableBkUpAccess(void) {}
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data) {
	(void) hrtc;
	if (BackupRegister < 32)
		sim_mcu.bkp_reg[BackupRegister] = Data;
}
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister) {
	(void) hrtc;
	return BackupRegister < 32 ? sim_mcu.bkp_reg[BackupRegister] : 0;
}
