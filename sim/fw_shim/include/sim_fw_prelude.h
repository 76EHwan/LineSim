/*
 * Force-included (/FI) into every firmware translation unit.
 *
 * The firmware's main context busy-waits on state that interrupts change
 * (e.g. `while (g_current_base_mps > 0.001f) {}`). On the MCU an interrupt
 * preempts the loop; here interrupts run on the simulator's fiber, so every
 * loop condition check first hands control back to the simulator, which
 * advances virtual time and fires due interrupts. sim_fw_yield() is a no-op
 * when called from interrupt context, so loops inside ISRs are unaffected.
 */
#ifndef SIM_FW_PRELUDE_H
#define SIM_FW_PRELUDE_H

#include "sim_compat.h"

void sim_fw_yield(void);

#define while(cond) while ((sim_fw_yield(), (cond)))

#endif /* SIM_FW_PRELUDE_H */
