#ifndef FIR_H
#define FIR_H

#include <stdint.h>
#include <stdbool.h>

void fir_init(void);

bool fir_process_sample(int32_t adc_sample, int32_t *out);

uint32_t fir_group_delay_samples(void);

#endif
