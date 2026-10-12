/*
 * The one file that sees both the firmware (its headers and globals) and the
 * simulator interface (fw_bridge.h / sim_mcu.h).
 */
#include "main.h"
#include "ff.h"
#include "foc.h"
#include "motor.h"
#include "drive.h"
#include "sensor.h"
#include "lsm6ds3tr-c.h"
#include "sd_ui.h"
#include "menu.h"
#include "user_init.h"
#include "drv8316crq1.h"

#include "sim_mcu.h"
#include "fw_bridge.h"

/* Interrupt handlers defined in the firmware (names after the foc.h / drive.h macros). */
void TIM13_IRQ_Handler(void);   /* Speed_TIM_IRQ_Handler (foc.c) */
void TIM14_IRQ_Handler(void);   /* Ramp_TIM_IRQ_Handler (drive.c) */
void LPTIM3_IRQ_Handler(void);  /* buzzer.c */

/* ---------------- interrupt context ---------------- */

void fwb_systick(void) {
	/* stm32h7xx_it.c SysTick_Handler, USER CODE SysTick_IRQn 0: IMU read every IMU_READ_DIV ms
	   (lsm6ds3tr-c.h; firmware versions without the macro read every 10 ms) */
#ifndef IMU_READ_DIV
#define IMU_READ_DIV 10
#endif
	static uint8_t imu_trigger_tick = 0;
	imu_trigger_tick++;
	if (imu_trigger_tick >= IMU_READ_DIV) {
		imu_trigger_tick = 0;
		if (!imu_dma_busy) {
			LSM6DS3_ReadGyroZ_DMA_Start();
		}
	}
	HAL_IncTick();
}

void fwb_tim_update_irq(int tim) {
	/* stm32h7xx_it.c HAL_TIM_PeriodElapsedCallback dispatch, for the timers the firmware uses */
	switch (tim) {
	case 7:  TIM7_IRQ_Handler();  break;
	case 13: TIM13_IRQ_Handler(); break;
	case 14: TIM14_IRQ_Handler(); break;
	default: break;
	}
}

void fwb_lptim_irq(int lptim) {
	if (lptim == 3)
		LPTIM3_IRQ_Handler();
}

void fwb_adc3_conversion(uint16_t raw) {
	if (!sim_mcu.adc_dma[3] || !sim_mcu.adc_dma_buf[3])
		return;
	sim_mcu.adc_dma_buf[3][0] = raw;
	ADC3->DR = raw;
	ADC3_IRQ_Cplt_Handler();   /* via HAL_ADC_ConvCpltCallback */
}

void fwb_i2c_dma_complete(void) {
	if (!sim_mcu.i2c_dma_busy)
		return;
	sim_i2c_mem_read(sim_mcu.i2c_dma_dev, sim_mcu.i2c_dma_reg, sim_mcu.i2c_dma_buf, sim_mcu.i2c_dma_len);
	sim_mcu.i2c_dma_busy = 0;
	HAL_I2C_MemRxCpltCallback((I2C_HandleTypeDef*) sim_mcu.i2c_dma_handle);
}

void fwb_adc_injected_irq(int adc) {
	HAL_ADCEx_InjectedConvCpltCallback(adc == 1 ? &hadc1 : &hadc2);
}

/* ---------------- firmware main fiber ---------------- */

void fwb_boot(void) { User_Init(); }
void fwb_menu_step(void) { Menu_ProcessLoop(); }

void fwb_drive(int n) {
	switch (n) {
	case 1: Drive_First();  break;
	case 2: Drive_Second(); break;
	case 3: Drive_Third();  break;
	case 4: Drive_Fourth(); break;
	default: break;
	}
}

int fwb_save_sensor_calibration(void) { return (int) Sensor_Save_Calibration(); }

/* ---------------- plant <-> firmware ---------------- */

void fwb_set_button(int btn, int pressed) {
	/* button.c Button_init(): all five inputs are active high */
	GPIO_TypeDef *port;
	uint16_t pin;
	switch (btn) {
	case FWB_BTN_U: port = SWITCH_UP_GPIO_Port;    pin = SWITCH_UP_Pin;    break;
	case FWB_BTN_D: port = SWITCH_DOWN_GPIO_Port;  pin = SWITCH_DOWN_Pin;  break;
	case FWB_BTN_L: port = SWITCH_LEFT_GPIO_Port;  pin = SWITCH_LEFT_Pin;  break;
	case FWB_BTN_R: port = SWITCH_RIGHT_GPIO_Port; pin = SWITCH_RIGHT_Pin; break;
	case FWB_BTN_K: port = KEY_GPIO_Port;          pin = KEY_Pin;          break;
	default: return;
	}
	sim_gpio_set_input((int) (port - sim_gpio), pin, pressed);
}

int fwb_sensor_channel(void) {
	uint8_t slot = IR_Sensor.data->idx % SCAN_CYCLE_LEN_HALF;
	if (slot == SCAN_SLOT_MARK_L)
		return LEFT_MARK_SENSOR_INDEX;
	if (slot == SCAN_SLOT_MARK_R)
		return RIGHT_MARK_SENSOR_INDEX;
	int ch = 0;
	if (SENSOR_MUX0_GPIO_Port->ODR & SENSOR_MUX0_Pin) ch |= 1;
	if (SENSOR_MUX1_GPIO_Port->ODR & SENSOR_MUX1_Pin) ch |= 2;
	if (SENSOR_MUX2_GPIO_Port->ODR & SENSOR_MUX2_Pin) ch |= 4;
	if (SENSOR_MUX3_GPIO_Port->ODR & SENSOR_MUX3_Pin) ch |= 8;
	return ch;
}

int fwb_sensor_scan_running(void) { return sim_mcu.tim_it[7] && sim_mcu.adc_dma[3]; }

static FOC_Handle_t* foc_side(int side) { return side ? &foc_R : &foc_L; }

void fwb_set_encoder(int side, uint16_t raw_cnt) {
	FOC_Handle_t *h = foc_side(side);
	if (h->LPTIMx)
		h->LPTIMx->Instance->CNT = raw_cnt;
}

int fwb_encoder_running(int side) {
	FOC_Handle_t *h = foc_side(side);
	return h->LPTIMx && sim_mcu.lptim_enc[h->LPTIMx->Instance - sim_lptim];
}

int fwb_motor_powered(int side) {
	FOC_Handle_t *h = foc_side(side);
	if (!h->is_running || !h->TIMx)
		return 0;
	int t = (int) (h->TIMx->Instance - sim_tim);
	int pwm = (sim_mcu.tim_pwm[t] & 0x7) == 0x7;
	int gate = (MTR_PWM_INLX_GPIO_Port->ODR & MTR_PWM_INLX_Pin) != 0;
	return pwm && gate;
}

float fwb_motor_iq_cmd(int side) { return foc_side(side)->target_Iq; }

float fwb_fan_duty(void) {
	if (!(sim_mcu.tim_pwm[15] & (1u << (TIM_CHANNEL_2 >> 2))))
		return 0.0f;
	float d = (float) TIM15->CCR2 / 1000.0f;   /* TIM15 ARR = 1000-1 (tim.c) */
	return d > 1.0f ? 1.0f : d;
}

void fwb_set_vbus(float volts) {
	float raw = (volts - VBUS_ADC_OFFSET) / VBUS_ADC_SCALE;
	if (raw < 0.0f) raw = 0.0f;
	if (raw > 65535.0f) raw = 65535.0f;
	adc1_dma_buf[0] = (uint16_t) raw;
	adc2_dma_buf[0] = (uint16_t) raw;
}

void fwb_set_phase_adc(int side, uint16_t ja, uint16_t jc) {
	/* foc_L samples ADC2, foc_R samples ADC1 (MTR_Update_Setup / User_Init) */
	ADC_TypeDef *a = side ? ADC1 : ADC2;
	a->JDR1 = ja;
	a->JDR2 = jc;
}

void fwb_install_sensor_calibration(const uint16_t white[18], const uint16_t black[18]) {
	for (int i = 0; i < NUM_SENSORS; i++) {
		sensorData.whitemax[i] = white[i];
		sensorData.blackmax[i] = black[i];
		uint16_t range = (white[i] > black[i]) ? (uint16_t) (white[i] - black[i]) : 0;
		sensorData.normalized_coef_bias[i] = range ? (uint16_t) ((255U << 8) / range) : 0;
	}
	IR_Sensor.is_calibration = 1;
}

void fwb_get_drive_params(FwDriveParams *p) {
	p->base_mps = driveData.base_mps;
	p->max_mps = driveData.max_mps;
	p->accel = driveData.accel;
	p->decel = driveData.decel;
	p->steer_kp = driveData.steer_gain_p;
	p->steer_kd = driveData.steer_gain_d;
	p->pos_atten_gain = driveData.pos_atten_gain;
	p->pit_in_m = driveData.pit_in_distance_m;
	p->target_shift = driveData.target_shift_val;
	p->fan_en = driveData.fan_en;
#ifdef DRIVE_HAS_CASCADE_STEER
	p->yaw_kp = driveData.yaw_gain_p;
	p->yaw_ki = driveData.yaw_gain_i;
	p->lat_acc_max = driveData.lat_acc_max;
#else
	p->yaw_kp = p->yaw_ki = p->lat_acc_max = 0;
#endif
#ifdef DRIVE_HAS_TARGET_CURVE_GAIN
	p->target_curve_gain = driveData.target_curve_gain;
#else
	p->target_curve_gain = 1.0f;
#endif
#ifdef DRIVE_HAS_SLALOM_RATIO
	p->slalom_ratio = driveData.slalom_ratio;
#else
	p->slalom_ratio = 0.0f;
#endif
}

void fwb_set_drive_params(const FwDriveParams *p) {
	driveData.base_mps = p->base_mps;
	driveData.max_mps = p->max_mps;
	driveData.accel = p->accel;
	driveData.decel = p->decel;
	driveData.steer_gain_p = p->steer_kp;
	driveData.steer_gain_d = p->steer_kd;
	driveData.pos_atten_gain = p->pos_atten_gain;
	driveData.pit_in_distance_m = p->pit_in_m;
	driveData.target_shift_val = p->target_shift;
	driveData.fan_en = (uint8_t) p->fan_en;
#ifdef DRIVE_HAS_CASCADE_STEER
	driveData.yaw_gain_p = p->yaw_kp;
	driveData.yaw_gain_i = p->yaw_ki;
	driveData.lat_acc_max = p->lat_acc_max;
#endif
#ifdef DRIVE_HAS_TARGET_CURVE_GAIN
	driveData.target_curve_gain = p->target_curve_gain;
#endif
#ifdef DRIVE_HAS_SLALOM_RATIO
	driveData.slalom_ratio = p->slalom_ratio;
#endif
}

void fwb_telemetry(FwTelemetry *t) {
	t->target_base_mps = g_target_base_mps;
	t->current_base_mps = g_current_base_mps;
	t->target_pos = IR_Sensor.data->target_pos;
	t->odom_m = g_odom_distance_m;
	t->yaw_deg = imu_data.Yaw_Angle;
	t->cmd_mps = 0.5f * (foc_L.target_omega - foc_R.target_omega) / MPS_TO_OMEGA;   /* right is negated */
	t->omega_target[0] = foc_L.target_omega;
	t->omega_target[1] = foc_R.target_omega;
	t->omega_meas[0] = foc_L.omega_e_meas;
	t->omega_meas[1] = foc_R.omega_e_meas;
	t->iq_cmd[0] = foc_L.target_Iq;
	t->iq_cmd[1] = foc_R.target_Iq;
	t->sensor_state = IR_Sensor.data->state;
	t->lost = IR_Sensor.is_lost_position;
	t->mark_left = IR_Sensor.data->mark_left;
	t->mark_right = IR_Sensor.data->mark_right;
	t->marker_count = g_cross_log_count;
	for (int i = 0; i < NUM_SENSORS; i++) {
		t->normalized[i] = (uint8_t) IR_Sensor.data->normalized[i];
		t->raw[i] = IR_Sensor.data->raw[i];
	}
}

int fwb_marker_log(FwMarker *out, int max) {
	int n = g_cross_log_count < CROSS_LOG_MAX ? g_cross_log_count : CROSS_LOG_MAX;
	if (n > max)
		n = max;
	for (int i = 0; i < n; i++) {
		out[i].type = (int) g_cross_log[i].type;
		out[i].dist_m = g_cross_log[i].dist_from_prev_m;
		out[i].yaw_deg = g_cross_log[i].yaw_angle;
	}
	return n;
}
