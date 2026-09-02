#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

void servo_init(void);
void servo_process_sample(int32_t adc_sample);

bool servo_is_converged(void);
bool servo_is_saturated(void);
bool servo_is_clipping(void);
int64_t servo_get_requested_current_q16(void);
int64_t servo_get_actual_current_q16(void);
uint16_t servo_get_coarse_pwm_code(void);
uint16_t servo_get_fine_pwm_code(void);

#endif
