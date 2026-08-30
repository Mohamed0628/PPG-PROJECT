
#include <stdint.h>
#include <stdbool.h>
#define SERVO_VREF_INIT 0   // TODO: replace after fixed-point format is chosen
#define BASELINE_FRAC_BITS 16 //used for making baseline fractional bits for updating servo
#define KP_FIXED 51070 // derive from ADC scale + TIA/cancellation plant
#define KI_FAST_FIXED 12751
#define KI_SLOW_FIXED 424
#define GAIN_FRAC_BITS 24
#define I_FINE_MAX_Q16 32768000000LL // 500 nA
#define I_FINE_MIN_Q16 (-32768000000LL) // - 500nA
#define I_FINE_STEP_Q16 3997696LL // 61 pA which is one step in the system
#define I_COARSE_STEP_Q16 88000000LL // 1342 pA which is one course step 
#define PWM_MAX_CODE       16383LL
#define PWM_SUPPLY_UV      3300000LL
#define V_SUM_UV           1650000LL
#define PA_Q16_TO_UV_DENOM (65536LL * 1000000LL)
#define R_COARSE_OHMS 150000LL
#define R_FINE_OHMS   3300000LL

typedef struct {

    int32_t tia_value;          // Latest instantaneous TIA measurement
                                // Unit: TBD voltage/ADC fixed-point representation

    int64_t baseline_q16;                // Current slow baseline estimate B[n]
                                // Unit: same representation as tia_value

    int64_t B_error;          // Baseline error: V_REF - B[n]
                                // Unit: same representation as tia_value

    int64_t I_integral;         // PI integral memory
                                // Physical meaning: remembered cancellation current
                                // Unit: TBD fixed-point current representation

    int64_t I_cancel;           // Total cancellation current requested by PI
                                // Unit: TBD fixed-point current representation

    int64_t I_proportional;   // Proportional cancellation-current request
                          // Unit: TBD fixed-point current representation
                           // Proportional current contribution: I_P = Kp * error
    int64_t I_coarse; // current assigned to 150k path
    int64_t I_fine;  // current assigned to 
    int64_t I_actual;
    bool actuator_saturated;

} servo_state_t;

static servo_state_t servo;

void servo_init(void)
{
    
    servo.tia_value = SERVO_VREF_INIT; 

    servo.baseline_q16  = SERVO_VREF_INIT;

    servo.B_error = -servo.baseline_q16;
    servo.I_integral = 0;
    servo.I_cancel = 0;
    servo.I_proportional = 0;
    servo.I_coarse = 0;
    servo.I_fine = 0;
    servo.I_actual = 0;
    servo.actuator_saturated = false;

    
}

static const uint32_t ALPHA_FAST_Q24 = 174773u;  // fc = 2 Hz
static const uint32_t ALPHA_SLOW_Q24 = 4392u;   // fc = 0.05 Hz

void servo_update_baseline(int32_t new_tia_sample){
    uint32_t alpha;
    if(supervisor_servo_fast_mode()){
        alpha = ALPHA_FAST_Q24;
    }
    else{
        alpha = ALPHA_SLOW_Q24;
    }

    servo.tia_value = new_tia_sample;
    int64_t sample_q16 = ((int64_t)servo.tia_value) << BASELINE_FRAC_BITS; // 64 bit version of tia value

    int64_t difference = sample_q16 - servo.baseline_q16;
    //B[n]

    int64_t product = (int64_t)difference * (int64_t)alpha;

    int64_t step = (product >> 24);

    servo.baseline_q16 += step;

    servo.B_error = -servo.baseline_q16;

}
void servo_update_proportional(void)
{
    // I_P = Kp * error

    // TODO:
    // multiply servo.B_error by KP_FIXED
    servo.I_proportional = (KP_FIXED * servo.B_error)>> GAIN_FRAC_BITS;

    // use a wide intermediate

    // rescale back to the chosen current format
    // store result in servo.I_proportional
}

void servo_update_integral(void)
{
    int64_t Ki_term;
    int64_t integral_step;
    bool pushing_further = false;

    if (supervisor_servo_fast_mode()){
        Ki_term = KI_FAST_FIXED;
    }
    else{
        Ki_term = KI_SLOW_FIXED;
    }

    integral_step =
        (servo.B_error * Ki_term) >> GAIN_FRAC_BITS;

    if (servo.actuator_saturated){

        /*
         * Positive saturation:
         * requested current is above what hardware can deliver.
         * A positive integral step would push farther into saturation.
         */
        if ((servo.I_cancel > servo.I_actual) &&
            (integral_step > 0)){
            pushing_further = true;
        }

        /*
         * Negative saturation:
         * requested current is below what hardware can deliver.
         * A negative integral step would push farther into saturation.
         */
        else if ((servo.I_cancel < servo.I_actual) &&
                 (integral_step < 0)){
            pushing_further = true;
        }
    }

    /*
     * Integrate normally unless doing so would push
     * farther into actuator saturation.
     */
    if (!pushing_further){
        servo.I_integral += integral_step;
    }
}

void servo_update_cancel(void)
{
    servo.I_cancel = servo.I_integral + servo.I_proportional;
}

int64_t quantize_current(int64_t requested_current, int64_t step_size)
{
    int64_t number_of_steps;
    int64_t quantized_current;

    number_of_steps = requested_current / step_size;
    quantized_current = number_of_steps * step_size;

    return quantized_current;
}

int64_t quantize_current(int64_t requested_current, int64_t step_size);

void servo_allocate_current(void)
{
    int64_t I_remainder;
    servo.actuator_saturated = false;

    I_remainder = servo.I_cancel;

    if(servo.I_cancel <= I_FINE_MAX_Q16 && servo.I_cancel >= I_FINE_MIN_Q16){
        servo.I_coarse = 0;
    }
    else{
        servo.I_coarse = quantize_current(
            servo.I_cancel,
            I_COARSE_STEP_Q16
        );
    }

    I_remainder = servo.I_cancel - servo.I_coarse;

    if(I_remainder > I_FINE_MAX_Q16){
        I_remainder = I_FINE_MAX_Q16;
        servo.actuator_saturated = true;
    }
    else if(I_remainder < I_FINE_MIN_Q16){
        I_remainder = I_FINE_MIN_Q16;
        servo.actuator_saturated = true;
    }

    servo.I_fine = quantize_current(
        I_remainder,
        I_FINE_STEP_Q16
    );

    servo.I_actual = servo.I_coarse + servo.I_fine;
}

uint16_t current_to_pwm_code( int64_t current_q16, int64_t resistance_ohms)
{
    int64_t numerator;
    int64_t delta_v_uv;
    int64_t v_ctrl_uv;
    int64_t pwm_code;

    /*
     * V_CTRL = V_SUM + I_CTRL * R_CTRL
     *
     * current_q16 is current in pA, Q16.
     * Convert I * R into microvolts.
     */
    numerator = current_q16 * resistance_ohms;

    if(numerator >= 0){
        delta_v_uv = (numerator + (PA_Q16_TO_UV_DENOM / 2)) / PA_Q16_TO_UV_DENOM;
    }
    else{
        delta_v_uv = (numerator - (PA_Q16_TO_UV_DENOM / 2)) / PA_Q16_TO_UV_DENOM;
    }

    v_ctrl_uv = V_SUM_UV + delta_v_uv;

    /*
     * Keep requested control voltage inside the
     * nominal 0 V to 3.3 V PWM range.
     */
    if(v_ctrl_uv < 0){
        v_ctrl_uv = 0;
    }
    else if(v_ctrl_uv > PWM_SUPPLY_UV){
        v_ctrl_uv = PWM_SUPPLY_UV;
    }

    /*
     * Convert V_CTRL to a 14-bit PWM code.
     *
     * code = V_CTRL / 3.3V * 16384
     */
    pwm_code = ((v_ctrl_uv * PWM_MAX_CODE) + (PWM_SUPPLY_UV / 2)) / PWM_SUPPLY_UV;

    return (uint16_t)pwm_code;
}
uint16_t coarse_pwm;
uint16_t fine_pwm;

void servo_update_pwm_codes(void)
{
    coarse_pwm = current_to_pwm_code(
        servo.I_coarse,
        R_COARSE_OHMS
    );

    fine_pwm = current_to_pwm_code(
        servo.I_fine,
        R_FINE_OHMS
    );
}