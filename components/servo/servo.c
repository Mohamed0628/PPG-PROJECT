#include "servo.h"
#include "state_machine.h"
#define SERVO_VREF_INIT 0   // TODO: replace after fixed-point format is chosen
#define BASELINE_FRAC_BITS 16 //used for making baseline fractional bits for updating servo
#define KP_FIXED 240000 // beta * (V_LSB / Rf), Rf = 100k
#define KI_FAST_FIXED 60000
#define KI_SLOW_FIXED 2000
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

// TODO VALIDATION: provisional convergence threshold, in ordinary ADC codes.
// With the ADS131M02 at gain 1, one code is about 143.05 nV. The old 15000-code
// threshold therefore still represents about 2.146 mV electrically, but after
// changing Rf from 470k to 100k the expected 10 nA PPG amplitude is only about
// 1.0 mV. Leaving the threshold at 15000 codes would make it about 215% of the
// expected PPG amplitude and much too loose. Scaling the threshold by 100/470
// preserves approximately the old current-domain criterion: 3200 codes is about
// 458 uV, or 45.8% of the expected 1.0 mV PPG amplitude.
// The convergence test runs on a slow-filtered copy of B_error (below), so the
// threshold is intentionally larger than the residual heartbeat component after
// the 0.2 Hz convergence filter. Replace with measured residual error after a
// real acquisition on skin.
#define SERVO_CONVERGENCE_THRESHOLD_CODES 3200LL

// Same threshold expressed in the Q16 format that servo.B_error uses.
#define SERVO_CONVERGENCE_THRESHOLD_Q16 \
    (SERVO_CONVERGENCE_THRESHOLD_CODES << BASELINE_FRAC_BITS)

// TODO VALIDATION: provisional persistence, in consecutive servo updates.
// 300 updates at 1200 SPS is 250 ms, about 3.1 time constants of the 2 Hz
// fast estimator, so a transient dip through the threshold cannot latch
// convergence. Validated fast settling is 0.97 s, so total time to declare
// convergence is about 1.22 s against a 30 s supervisor timeout.
#define SERVO_CONVERGENCE_COUNT 300u

// Convergence-only low-pass on B_error, fc = 0.2 Hz at 1200 SPS.
// This is not a control filter and does not feed the PI loop. It exists so
// the convergence test measures the DC baseline rather than the heartbeat.
// At 100k, a 10 nA PPG current produces about 1.0 mV at the TIA. The 0.2 Hz
// convergence filter attenuates a 1.2 Hz pulse to about 0.164 of that value,
// leaving roughly 164 uV (about 1150 ADC codes), comfortably inside the
// 3200-code threshold. Time constant 0.80 s, so detection lags acquisition.
#define CONVERGENCE_ALPHA_Q24 17560u

// With Rf = 100k, the full +/-11 uA coarse-path authority moves the TIA by
// only +/-1.1 V, inside the approximately +/-1.65 V output headroom. The TIA
// is therefore no longer the binding limit. Clamp at the actuator's actual
// +/-11 uA coarse-path authority so anti-windup sees the same limit the PWM
// hardware can deliver. Bench validation should confirm usable control-voltage
// swing and op-amp output headroom before this value is frozen.
#define I_CANCEL_MAX_Q16 720896000000LL   // 11 uA
#define I_CANCEL_MIN_Q16 (-720896000000LL)

// Raw ADC magnitude above which the converter is effectively railed and the
// servo is blind. 95% of a 24-bit signed full scale.
#define SERVO_ADC_CLIP_CODES 7969176L

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

    int64_t conv_error_q16;       // slow-filtered B_error, convergence test only
    uint32_t convergence_counter; // consecutive updates with acceptable B_error
    bool converged;               // servo_converged reported to the supervisor
    bool adc_clipping;            // converter at or near full scale

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

    servo.conv_error_q16 = 0;
    servo.convergence_counter = 0;
    servo.converged = false;
    servo.adc_clipping = false;

    
}

static const uint32_t ALPHA_FAST_Q24 = 174773u;  // fc = 2 Hz
static const uint32_t ALPHA_SLOW_Q24 = 4392u;   // fc = 0.05 Hz

static void servo_update_baseline(int32_t new_tia_sample){
    uint32_t alpha;
    if(supervisor_servo_fast_mode()){
        alpha = ALPHA_FAST_Q24;
    }
    else{
        alpha = ALPHA_SLOW_Q24;
    }

    servo.tia_value = new_tia_sample;

    /*
     * Clipping detection. Above this magnitude the converter is railed and
     * the loop cannot see how large the real error is, so the servo response
     * is meaningless until the disturbance passes. The supervisor needs this
     * to mark data invalid; the servo cannot fix it.
     */
    servo.adc_clipping =
        (new_tia_sample >=  SERVO_ADC_CLIP_CODES) ||
        (new_tia_sample <= -SERVO_ADC_CLIP_CODES);

    int64_t sample_q16 = ((int64_t)servo.tia_value) << BASELINE_FRAC_BITS; // 64 bit version of tia value

    int64_t difference = sample_q16 - servo.baseline_q16;
    //B[n]

    int64_t product = (int64_t)difference * (int64_t)alpha;

    int64_t step = (product >> 24);

    servo.baseline_q16 += step;

    servo.B_error = -servo.baseline_q16;

    /*
     * Convergence test on the FILTERED baseline error, never on the
     * instantaneous sample. Range comparison rather than an abs() so that
     * INT64_MIN cannot overflow.
     *
     * The error must stay inside the acceptable band for
     * SERVO_CONVERGENCE_COUNT CONSECUTIVE updates. Leaving the band at any
     * point resets both the counter and the result, so isolated good
     * samples can never accumulate into a convergence declaration.
     */
    int64_t conv_diff = servo.B_error - servo.conv_error_q16;
    servo.conv_error_q16 +=
        (conv_diff * (int64_t)CONVERGENCE_ALPHA_Q24) >> GAIN_FRAC_BITS;

    if((servo.conv_error_q16 >= -SERVO_CONVERGENCE_THRESHOLD_Q16) &&
       (servo.conv_error_q16 <=  SERVO_CONVERGENCE_THRESHOLD_Q16) &&
       (!servo.adc_clipping)){

        if(servo.convergence_counter < SERVO_CONVERGENCE_COUNT){
            servo.convergence_counter++;
        }

        if(servo.convergence_counter >= SERVO_CONVERGENCE_COUNT){
            servo.converged = true;
        }
    }
    else{
        servo.convergence_counter = 0;
        servo.converged = false;
    }

}

bool servo_is_converged(void)
{
    return servo.converged;
}

static void servo_update_proportional(void)
{
    // I_P = Kp * error

    // TODO:
    // multiply servo.B_error by KP_FIXED
    servo.I_proportional = (KP_FIXED * servo.B_error)>> GAIN_FRAC_BITS;

    // use a wide intermediate

    // rescale back to the chosen current format
    // store result in servo.I_proportional
}

static void servo_update_integral(void)
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

static void servo_update_cancel(void)
{
    servo.I_cancel = servo.I_integral + servo.I_proportional;
}

static int64_t quantize_current(int64_t requested_current, int64_t step_size)
{
    int64_t number_of_steps;
    int64_t quantized_current;

    number_of_steps = requested_current / step_size;
    quantized_current = number_of_steps * step_size;

    return quantized_current;
}

static void servo_allocate_current(void)
{
    int64_t I_remainder;
    int64_t I_request;
    servo.actuator_saturated = false;

    /*
     * Clamp the request to the actuator's usable current authority before
     * splitting it. A larger request would make current_to_pwm_code clip the
     * control voltage while I_actual still reported the unclipped current,
     * preventing actuator_saturated and anti-windup from reflecting the real
     * hardware limit. Detect saturation at the actual +/-11 uA authority here.
     */
    I_request = servo.I_cancel;

    if(I_request > I_CANCEL_MAX_Q16){
        I_request = I_CANCEL_MAX_Q16;
        servo.actuator_saturated = true;
    }
    else if(I_request < I_CANCEL_MIN_Q16){
        I_request = I_CANCEL_MIN_Q16;
        servo.actuator_saturated = true;
    }

    I_remainder = I_request;

    if(I_request <= I_FINE_MAX_Q16 && I_request >= I_FINE_MIN_Q16){
        servo.I_coarse = 0;
    }
    else{
        servo.I_coarse = quantize_current(
            I_request,
            I_COARSE_STEP_Q16
        );
    }

    I_remainder = I_request - servo.I_coarse;

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

static uint16_t current_to_pwm_code(int64_t current_q16, int64_t resistance_ohms)
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
static uint16_t coarse_pwm;
static uint16_t fine_pwm;

static void servo_update_pwm_codes(void)
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

void servo_process_sample(int32_t adc_sample)
{
    /*
     * Ordering is part of the controller design:
     *
     * 1. Estimate the baseline and form the new error.
     * 2. Form the proportional term from that error.
     * 3. Update the integrator. Anti-windup intentionally uses the
     *    previous tick's I_cancel, I_actual, and saturation result.
     * 4. Form and allocate the new cancellation-current request.
     * 5. Convert the allocated currents into hardware PWM codes.
     */
    servo_update_baseline(adc_sample);
    servo_update_proportional();
    servo_update_integral();
    servo_update_cancel();
    servo_allocate_current();
    servo_update_pwm_codes();
}

bool servo_is_saturated(void)
{
    return servo.actuator_saturated;
}

bool servo_is_clipping(void)
{
    return servo.adc_clipping;
}

int64_t servo_get_requested_current_q16(void)
{
    return servo.I_cancel;
}

int64_t servo_get_actual_current_q16(void)
{
    return servo.I_actual;
}

uint16_t servo_get_coarse_pwm_code(void)
{
    return coarse_pwm;
}

uint16_t servo_get_fine_pwm_code(void)
{
    return fine_pwm;
}