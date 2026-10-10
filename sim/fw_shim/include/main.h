/*
 * Wraps the firmware's real Core/Inc/main.h (pin map, prototypes) and moves the
 * IR-sensor MUX / enable pins onto ports of their own.
 *
 * sensor.c drives those pins with consecutive BSRR writes on the same port
 * (GPIOE). A plain-memory BSRR keeps only the last write, so each of these pins
 * gets a dedicated port whose BSRR the simulator can decode after TIM7 fires.
 */
#ifndef SIM_MAIN_H_WRAPPER
#define SIM_MAIN_H_WRAPPER

#include SIM_FW_REAL_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

enum { SIM_PORT_MUX0, SIM_PORT_MUX1, SIM_PORT_MUX2, SIM_PORT_MUX3, SIM_PORT_IR_EN, SIM_PORT_PT_EN, SIM_PORT_COUNT };
extern GPIO_TypeDef sim_sensor_port[SIM_PORT_COUNT];

#undef  SENSOR_MUX0_GPIO_Port
#define SENSOR_MUX0_GPIO_Port (&sim_sensor_port[SIM_PORT_MUX0])
#undef  SENSOR_MUX1_GPIO_Port
#define SENSOR_MUX1_GPIO_Port (&sim_sensor_port[SIM_PORT_MUX1])
#undef  SENSOR_MUX2_GPIO_Port
#define SENSOR_MUX2_GPIO_Port (&sim_sensor_port[SIM_PORT_MUX2])
#undef  SENSOR_MUX3_GPIO_Port
#define SENSOR_MUX3_GPIO_Port (&sim_sensor_port[SIM_PORT_MUX3])
#undef  SENSOR_IR_EN_GPIO_Port
#define SENSOR_IR_EN_GPIO_Port (&sim_sensor_port[SIM_PORT_IR_EN])
#undef  SENSOR_PT_EN_GPIO_Port
#define SENSOR_PT_EN_GPIO_Port (&sim_sensor_port[SIM_PORT_PT_EN])

#ifdef __cplusplus
}
#endif
#endif /* SIM_MAIN_H_WRAPPER */
