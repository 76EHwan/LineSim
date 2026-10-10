/*
 * Entry points into the firmware for the simulator: interrupt handlers, the
 * functions run on the firmware "main" fiber, and read/write access to the
 * firmware variables the plant model needs. Implemented in fw_bridge.c,
 * which is the only file that sees both worlds.
 */
#ifndef FW_BRIDGE_H
#define FW_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- interrupt context (simulator fiber) ---- */
void fwb_systick(void);                     /* 1 kHz: HAL tick + SysTick user code */
void fwb_tim_update_irq(int tim);           /* TIMx update -> HAL_TIM_PeriodElapsedCallback */
void fwb_lptim_irq(int lptim);              /* LPTIMx autoreload match */
void fwb_adc3_conversion(uint16_t raw);     /* ADC3 regular conversion + DMA complete */
void fwb_i2c_dma_complete(void);            /* finishes sim_mcu's outstanding I2C DMA read */
void fwb_adc_injected_irq(int adc);         /* ADC1/2 injected conversion (FOC current loop) */

/* ---- firmware main fiber ---- */
void fwb_boot(void);                        /* User_Init() */
void fwb_menu_step(void);                   /* one Menu_ProcessLoop() pass */
void fwb_drive(int n);                      /* 1..4: Drive_First .. Drive_Fourth */
int  fwb_save_sensor_calibration(void);     /* Sensor_Save_Calibration() */

/* ---- plant <-> firmware ---- */
enum { FWB_BTN_U, FWB_BTN_D, FWB_BTN_L, FWB_BTN_R, FWB_BTN_K, FWB_BTN_COUNT };
void  fwb_set_button(int btn, int pressed);  /* drives the 5-way switch inputs (active high) */
int   fwb_sensor_channel(void);             /* physical IR channel 0..17 selected by MUX/slot */
int   fwb_sensor_scan_running(void);
void  fwb_set_encoder(int side, uint16_t raw_cnt);   /* side 0 = left (foc_L), 1 = right */
int   fwb_encoder_running(int side);
int   fwb_motor_powered(int side);          /* gate driver + PWM on and FOC running */
float fwb_motor_iq_cmd(int side);           /* foc_X.target_Iq [A] */
float fwb_fan_duty(void);                   /* 0..1 */
void  fwb_set_vbus(float volts);
void  fwb_set_phase_adc(int side, uint16_t ja, uint16_t jc);
void  fwb_install_sensor_calibration(const uint16_t white[18], const uint16_t black[18]);

typedef struct {
	float base_mps, max_mps, accel, decel, steer_kp, steer_kd, pos_atten_gain, pit_in_m, target_shift;
	int   fan_en;
} FwDriveParams;
void fwb_get_drive_params(FwDriveParams *p);
void fwb_set_drive_params(const FwDriveParams *p);

typedef struct {
	float target_base_mps, current_base_mps;
	float target_pos;
	float odom_m;
	float yaw_deg;
	float omega_target[2], omega_meas[2];  /* electrical rad/s, firmware sign */
	float iq_cmd[2];
	uint32_t sensor_state;
	uint8_t lost, mark_left, mark_right;
	uint16_t marker_count;
	uint8_t normalized[18];
	uint16_t raw[18];
} FwTelemetry;
void fwb_telemetry(FwTelemetry *t);

typedef struct { int type; float dist_m; float yaw_deg; } FwMarker;  /* type: 1 L, 2 R, 3 cross, 4 stop */
int fwb_marker_log(FwMarker *out, int max);

#ifdef __cplusplus
}
#endif
#endif /* FW_BRIDGE_H */
