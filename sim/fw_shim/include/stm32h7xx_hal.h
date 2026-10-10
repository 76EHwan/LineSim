/*
 * Minimal STM32H7 HAL / CMSIS surface for running the firmware on a PC.
 *
 * Peripherals are plain memory structs: the firmware reads/writes registers
 * (TIMx->CCR1, LPTIMx->CNT, ADCx->JDR1, GPIOx->BSRR, DWT->CYCCNT ...) and the
 * simulator fills in / reads back the values around every interrupt and every
 * time it resumes the firmware main loop. HAL calls that start or stop a
 * peripheral only record that fact in `sim_mcu` (see sim_mcu.h).
 */
#ifndef SIM_STM32H7XX_HAL_H
#define SIM_STM32H7XX_HAL_H

#include "sim_compat.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- CMSIS core ---------------- */
#define __IO volatile
#define __I  volatile const
#define __O  volatile
#define __STATIC_INLINE static __inline
#define __STATIC_FORCEINLINE static __inline
#define __weak
#define __packed
#define __ALIGN_BEGIN
#define __ALIGN_END
#define __NOP()        ((void)0)
#define __DSB()        ((void)0)
#define __ISB()        ((void)0)
#define __DMB()        ((void)0)
#define __WFI()        ((void)0)
#define __disable_irq() ((void)0)
#define __enable_irq()  ((void)0)
#define UNUSED(x) ((void)(x))

typedef enum { RESET = 0, SET = !RESET } FlagStatus, ITStatus;
typedef enum { DISABLE = 0, ENABLE = !DISABLE } FunctionalState;

typedef struct {
	__IO uint32_t CTRL, CYCCNT, CPICNT, EXCCNT, SLEEPCNT, LSUCNT, FOLDCNT, PCSR;
	uint32_t RESERVED[1000];
	__O uint32_t LAR;
} DWT_Type;
typedef struct { __IO uint32_t DHCSR, DCRSR, DCRDR, DEMCR; } CoreDebug_Type;
extern DWT_Type sim_DWT;
extern CoreDebug_Type sim_CoreDebug;
#define DWT       (&sim_DWT)
#define CoreDebug (&sim_CoreDebug)
#define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#define DWT_CTRL_CYCCNTENA_Msk     (1UL)

extern uint32_t SystemCoreClock;

void SCB_CleanInvalidateDCache_by_Addr(volatile void *addr, int32_t dsize);
void SCB_InvalidateDCache_by_Addr(volatile void *addr, int32_t dsize);
void SCB_CleanDCache_by_Addr(volatile void *addr, int32_t dsize);
void NVIC_SystemReset(void);

/* ---------------- HAL common ---------------- */
typedef enum { HAL_OK = 0x00, HAL_ERROR = 0x01, HAL_BUSY = 0x02, HAL_TIMEOUT = 0x03 } HAL_StatusTypeDef;
typedef enum { HAL_UNLOCKED = 0x00, HAL_LOCKED = 0x01 } HAL_LockTypeDef;
#define HAL_MAX_DELAY 0xFFFFFFFFU

void     HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
void     HAL_IncTick(void);
void     HAL_NVIC_SystemReset(void);

/* ---------------- GPIO ---------------- */
typedef struct {
	__IO uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
} GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;

extern GPIO_TypeDef sim_gpio[11];
#define GPIOA (&sim_gpio[0])
#define GPIOB (&sim_gpio[1])
#define GPIOC (&sim_gpio[2])
#define GPIOD (&sim_gpio[3])
#define GPIOE (&sim_gpio[4])
#define GPIOF (&sim_gpio[5])
#define GPIOG (&sim_gpio[6])
#define GPIOH (&sim_gpio[7])
#define GPIOI (&sim_gpio[8])
#define GPIOJ (&sim_gpio[9])
#define GPIOK (&sim_gpio[10])

#define GPIO_PIN_0   ((uint16_t)0x0001)
#define GPIO_PIN_1   ((uint16_t)0x0002)
#define GPIO_PIN_2   ((uint16_t)0x0004)
#define GPIO_PIN_3   ((uint16_t)0x0008)
#define GPIO_PIN_4   ((uint16_t)0x0010)
#define GPIO_PIN_5   ((uint16_t)0x0020)
#define GPIO_PIN_6   ((uint16_t)0x0040)
#define GPIO_PIN_7   ((uint16_t)0x0080)
#define GPIO_PIN_8   ((uint16_t)0x0100)
#define GPIO_PIN_9   ((uint16_t)0x0200)
#define GPIO_PIN_10  ((uint16_t)0x0400)
#define GPIO_PIN_11  ((uint16_t)0x0800)
#define GPIO_PIN_12  ((uint16_t)0x1000)
#define GPIO_PIN_13  ((uint16_t)0x2000)
#define GPIO_PIN_14  ((uint16_t)0x4000)
#define GPIO_PIN_15  ((uint16_t)0x8000)
#define GPIO_PIN_All ((uint16_t)0xFFFF)

void          HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void          HAL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/* ---------------- DMA ---------------- */
typedef struct { void *Instance; } DMA_HandleTypeDef;

/* ---------------- TIM ---------------- */
typedef struct {
	__IO uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR,
	              CCR1, CCR2, CCR3, CCR4, BDTR, DCR, DMAR, RESERVED1, CCMR3, CCR5, CCR6, AF1, AF2, TISEL;
} TIM_TypeDef;
typedef struct {
	uint32_t Prescaler, CounterMode, Period, ClockDivision, RepetitionCounter, AutoReloadPreload;
} TIM_Base_InitTypeDef;
typedef struct {
	TIM_TypeDef *Instance;
	TIM_Base_InitTypeDef Init;
	uint32_t Channel;
	DMA_HandleTypeDef *hdma[7];
	HAL_LockTypeDef Lock;
	__IO uint32_t State;
} TIM_HandleTypeDef;

extern TIM_TypeDef sim_tim[18];   /* index = timer number */
#define TIM1  (&sim_tim[1])
#define TIM2  (&sim_tim[2])
#define TIM3  (&sim_tim[3])
#define TIM4  (&sim_tim[4])
#define TIM5  (&sim_tim[5])
#define TIM6  (&sim_tim[6])
#define TIM7  (&sim_tim[7])
#define TIM8  (&sim_tim[8])
#define TIM12 (&sim_tim[12])
#define TIM13 (&sim_tim[13])
#define TIM14 (&sim_tim[14])
#define TIM15 (&sim_tim[15])
#define TIM16 (&sim_tim[16])
#define TIM17 (&sim_tim[17])

#define TIM_CHANNEL_1   0x00000000U
#define TIM_CHANNEL_2   0x00000004U
#define TIM_CHANNEL_3   0x00000008U
#define TIM_CHANNEL_4   0x0000000CU
#define TIM_CHANNEL_ALL 0x0000003CU
#define TIM_CR1_CEN     (1UL << 0)
#define TIM_CR1_OPM     (1UL << 3)
#define TIM_SR_UIF      (1UL << 0)

/* CCR1..CCR4 are consecutive, exactly like the real register map. */
#define __HAL_TIM_SET_COMPARE(h, ch, v) (*(&((h)->Instance->CCR1) + ((ch) >> 2U)) = (v))
#define __HAL_TIM_GET_COMPARE(h, ch)    (*(&((h)->Instance->CCR1) + ((ch) >> 2U)))
#define __HAL_TIM_SET_COUNTER(h, v)     ((h)->Instance->CNT = (v))
#define __HAL_TIM_GET_COUNTER(h)        ((h)->Instance->CNT)
#define __HAL_TIM_SET_AUTORELOAD(h, v)  ((h)->Instance->ARR = (v))
#define __HAL_TIM_GET_AUTORELOAD(h)     ((h)->Instance->ARR)
#define __HAL_TIM_ENABLE(h)             ((h)->Instance->CR1 |= TIM_CR1_CEN)
#define __HAL_TIM_DISABLE(h)            ((h)->Instance->CR1 &= ~TIM_CR1_CEN)

HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

/* ---------------- LPTIM ---------------- */
typedef struct {
	__IO uint32_t ISR, ICR, IER, CFGR, CR, CMP, ARR, CNT, RESERVED1, CFGR2;
} LPTIM_TypeDef;
typedef struct {
	LPTIM_TypeDef *Instance;
	struct { struct { uint32_t Source, Prescaler; } Clock; struct { uint32_t Source; } Trigger; } Init;
	HAL_LockTypeDef Lock;
	__IO uint32_t State;
} LPTIM_HandleTypeDef;

extern LPTIM_TypeDef sim_lptim[6]; /* index = LPTIM number */
#define LPTIM1 (&sim_lptim[1])
#define LPTIM2 (&sim_lptim[2])
#define LPTIM3 (&sim_lptim[3])
#define LPTIM4 (&sim_lptim[4])
#define LPTIM5 (&sim_lptim[5])

HAL_StatusTypeDef HAL_LPTIM_Encoder_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period);
HAL_StatusTypeDef HAL_LPTIM_Encoder_Stop(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_Counter_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period);
HAL_StatusTypeDef HAL_LPTIM_Counter_Stop_IT(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_Counter_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period);
HAL_StatusTypeDef HAL_LPTIM_Counter_Stop(LPTIM_HandleTypeDef *hlptim);

/* ---------------- ADC ---------------- */
typedef struct {
	__IO uint32_t ISR, IER, CR, CFGR, CFGR2, SMPR1, SMPR2, PCSEL, LTR1, HTR1,
	              SQR1, SQR2, SQR3, SQR4, DR, JSQR, OFR1, OFR2, OFR3, OFR4,
	              JDR1, JDR2, JDR3, JDR4;
} ADC_TypeDef;
typedef struct {
	ADC_TypeDef *Instance;
	struct { uint32_t Resolution, ContinuousConvMode, ExternalTrigConv; } Init;
	DMA_HandleTypeDef *DMA_Handle;
	HAL_LockTypeDef Lock;
	__IO uint32_t State;
	__IO uint32_t ErrorCode;
} ADC_HandleTypeDef;

extern ADC_TypeDef sim_adc[4]; /* index = ADC number */
#define ADC1 (&sim_adc[1])
#define ADC2 (&sim_adc[2])
#define ADC3 (&sim_adc[3])

#define ADC_CALIB_OFFSET           0x00000000U
#define ADC_CALIB_OFFSET_LINEARITY 0x00000001U
#define ADC_SINGLE_ENDED           0x00000000U
#define ADC_DIFFERENTIAL_ENDED     0x00000001U

HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef *hadc, uint32_t CalibrationMode, uint32_t SingleDiff);
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout);
uint32_t          HAL_ADC_GetValue(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef *hadc);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);

/* ---------------- DAC ---------------- */
typedef struct { __IO uint32_t CR, SWTRIGR, DHR12R1; } DAC_TypeDef;
typedef struct { DAC_TypeDef *Instance; __IO uint32_t State; HAL_LockTypeDef Lock; } DAC_HandleTypeDef;
extern DAC_TypeDef sim_dac1;
#define DAC1 (&sim_dac1)
#define DAC_CHANNEL_1    0x00000000U
#define DAC_CHANNEL_2    0x00000010U
#define DAC_ALIGN_12B_R  0x00000000U
#define DAC_ALIGN_12B_L  0x00000004U
#define DAC_ALIGN_8B_R   0x00000008U
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef *hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel, const uint32_t *pData, uint32_t Length, uint32_t Alignment);
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel);

/* ---------------- I2C ---------------- */
typedef struct { __IO uint32_t CR1, CR2, OAR1, OAR2, TIMINGR, TIMEOUTR, ISR, ICR, PECR, RXDR, TXDR; } I2C_TypeDef;
typedef struct { I2C_TypeDef *Instance; __IO uint32_t State; __IO uint32_t ErrorCode; HAL_LockTypeDef Lock; } I2C_HandleTypeDef;
extern I2C_TypeDef sim_i2c[5];
#define I2C1 (&sim_i2c[1])
#define I2C2 (&sim_i2c[2])
#define I2C3 (&sim_i2c[3])
#define I2C4 (&sim_i2c[4])
#define I2C_MEMADD_SIZE_8BIT  0x00000001U
#define I2C_MEMADD_SIZE_16BIT 0x00000002U
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

/* ---------------- SPI / QSPI / misc handles (present, unused by the sim) ---------------- */
typedef struct { __IO uint32_t CR1; } SPI_TypeDef;
typedef struct { SPI_TypeDef *Instance; __IO uint32_t State; } SPI_HandleTypeDef;
extern SPI_TypeDef sim_spi[7];
#define SPI1 (&sim_spi[1])
#define SPI2 (&sim_spi[2])
#define SPI4 (&sim_spi[4])
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, const uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

typedef struct { void *Instance; } QSPI_HandleTypeDef;
typedef struct { void *Instance; } RNG_HandleTypeDef;
typedef struct { void *Instance; } RTC_HandleTypeDef;
typedef struct { void *Instance; } SD_HandleTypeDef;
typedef struct { void *Instance; } UART_HandleTypeDef;
typedef struct { void *Instance; } PCD_HandleTypeDef;

/* ---------------- Internal flash ---------------- */
typedef struct { uint32_t TypeErase, Banks, Sector, NbSectors, VoltageRange; } FLASH_EraseInitTypeDef;
#define FLASH_TYPEERASE_SECTORS     0x00U
#define FLASH_TYPEERASE_MASSERASE   0x01U
#define FLASH_BANK_1                0x01U
#define FLASH_BANK_2                0x02U
#define FLASH_SECTOR_0              0U
#define FLASH_SECTOR_7              7U
#define FLASH_VOLTAGE_RANGE_3       0x20U
#define FLASH_TYPEPROGRAM_FLASHWORD 0x01U
#define FLASH_SECTOR_SIZE           0x00020000UL
#define FLASH_BASE                  0x08000000UL
#define FLASH_BANK2_BASE            0x08100000UL
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t FlashAddress, uint32_t DataAddress);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);

/* ---------------- RTC backup (bootloader request flag) ---------------- */
void     HAL_PWR_EnableBkUpAccess(void);
void     HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data);
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister);
#define RTC_BKP_DR0 0x00U

#ifdef __cplusplus
}
#endif
#endif /* SIM_STM32H7XX_HAL_H */
