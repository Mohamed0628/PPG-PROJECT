#include "ppg_acq.h"

/*
 * Linker-validation harness only.
 *
 * Referencing ppg_acq_init forces ppg_acq.c into the final ELF so the
 * DRDY ISR and its call path can be inspected in the linker map.
 *
 * We intentionally do not call it because no hardware is configured yet.
 */
static void *volatile keep_ppg_acq_linked = (void *)&ppg_acq_init;

void app_main(void)
{
    (void)keep_ppg_acq_linked;
}
