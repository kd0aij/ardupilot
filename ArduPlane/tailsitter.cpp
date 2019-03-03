/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  control code for tailsitters. Enabled by setting Q_FRAME_CLASS=10
  or by setting Q_TAILSIT_MOTMX nonzero and Q_FRAME_CLASS and Q_FRAME_TYPE
  to a configuration supported by AP_MotorsMatrix
 */

#include <math.h>

#include "Plane.h"

#define DEBUG_SPEED_SCALING 1

/*
  return true when flying a tailsitter
 */
bool QuadPlane::is_tailsitter(void) const
{
    return available() && ((frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER) ||
                           (tailsitter.motor_mask != 0));
}

/*
  check if we are flying as a tailsitter
 */
bool QuadPlane::tailsitter_active(void)
{
    if (!is_tailsitter()) {
        return false;
    }
    if (in_vtol_mode()) {
        return true;
    }
    // check if we are in ANGLE_WAIT fixed wing transition
    if (transition_state == TRANSITION_ANGLE_WAIT_FW) {
        return true;
    }
    return false;
}

/*
  run output for tailsitters
 */
void QuadPlane::tailsitter_output(void)
{
    if (!is_tailsitter()) {
        return;
    }

    float tilt_left = 0.0f;
    float tilt_right = 0.0f;
    uint16_t mask = tailsitter.motor_mask;

    // handle forward flight modes and transition to VTOL modes
    if (!tailsitter_active() || in_tailsitter_vtol_transition()) {
        // in forward flight: set motor tilt servos and throttles using FW controller
        if (tailsitter.vectored_forward_gain > 0) {
            // thrust vectoring in fixed wing flight
            float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
            float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
            tilt_left  = (elevator + aileron) * tailsitter.vectored_forward_gain;
            tilt_right = (elevator - aileron) * tailsitter.vectored_forward_gain;
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);

        // get FW controller throttle demand and mask of motors enabled during forward flight
        float throttle = SRV_Channels::get_output_scaled(SRV_Channel::k_throttle);
        if (hal.util->get_soft_armed()) {
            if (in_tailsitter_vtol_transition() && !throttle_wait && is_flying()) {
            /*
              during transitions to vtol mode set the throttle to
              hover thrust, center the rudder and set the altitude controller
              integrator to the same throttle level
             */
            throttle = motors->get_throttle_hover() * 100;
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
            pos_control->get_accel_z_pid().set_integrator(throttle*10);

            if (mask == 0) {
                // override AP_MotorsTailsitter throttles during back transition
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle);
                SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle);
            }
        }
        if (mask != 0) {
            // set AP_MotorsMatrix throttles enabled for forward flight
            motors->output_motor_mask(throttle * 0.01f, mask, plane.rudder_dt);
            }
        }
        return;
    }

    // handle VTOL modes
    // the MultiCopter rate controller has already been run in an earlier call
    // to motors_output() from quadplane.update()
    motors_output(false);
    plane.pitchController.reset_I();
    plane.rollController.reset_I();

    if (hal.util->get_soft_armed()) {
        // scale surfaces for throttle
        tailsitter_speed_scaling();
    }

    if (tailsitter.vectored_hover_gain > 0) {
        // thrust vectoring VTOL modes
        tilt_left = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
        tilt_right = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);
        /*
          apply extra elevator when at high pitch errors, using a
          power law. This allows the motors to point straight up for
          takeoff without integrator windup
         */
        int32_t pitch_error_cd = (plane.nav_pitch_cd - ahrs_view->pitch_sensor) * 0.5;
        float extra_pitch = constrain_float(pitch_error_cd, -4500, 4500) / 4500.0;
        float extra_sign = extra_pitch > 0?1:-1;
        float extra_elevator = extra_sign * powf(fabsf(extra_pitch), tailsitter.vectored_hover_power) * 4500;
        tilt_left  = extra_elevator + tilt_left * tailsitter.vectored_hover_gain;
        tilt_right = extra_elevator + tilt_right * tailsitter.vectored_hover_gain;
        if (fabsf(tilt_left) >= 4500 || fabsf(tilt_right) >= 4500) {
            // prevent integrator windup
            motors->limit.roll_pitch = 1;
            motors->limit.yaw = 1;
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
    }


    if (tailsitter.input_mask_chan > 0 &&
        tailsitter.input_mask > 0 &&
        RC_Channels::get_radio_in(tailsitter.input_mask_chan-1) > 1700) {
        // the user is learning to prop-hang
        if (tailsitter.input_mask & TAILSITTER_MASK_AILERON) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_ELEVATOR) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_THROTTLE) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_RUDDER) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, plane.channel_rudder->get_control_in_zero_dz());
        }
    }
}


/*
  return true when we have completed enough of a transition to switch to fixed wing control
 */
bool QuadPlane::tailsitter_transition_fw_complete(void)
{
    if (plane.fly_inverted()) {
        // transition immediately
        return true;
    }
    int32_t roll_cd = labs(ahrs_view->roll_sensor);
    if (roll_cd > 9000) {
        roll_cd = 18000 - roll_cd;
    }
    if (labs(ahrs_view->pitch_sensor) > tailsitter.transition_angle*100 ||
        roll_cd > tailsitter.transition_angle*100 ||
        AP_HAL::millis() - transition_start_ms > uint32_t(transition_time_ms)) {
        return true;
    }
    // still waiting
    return false;
}


/*
  return true when we have completed enough of a transition to switch to VTOL control
 */
bool QuadPlane::tailsitter_transition_vtol_complete(void) const
{
    if (plane.fly_inverted()) {
        // transition immediately
        return true;
    }
    if (labs(plane.ahrs.pitch_sensor) > tailsitter.transition_angle*100 ||
        labs(plane.ahrs.roll_sensor) > tailsitter.transition_angle*100 ||
        AP_HAL::millis() - transition_start_ms > 2000) {
        return true;
    }
    // still waiting
    attitude_control->reset_rate_controller_I_terms();
    return false;
}

// handle different tailsitter input types
void QuadPlane::tailsitter_check_input(void)
{
    if (tailsitter_active() &&
        (tailsitter.input_type == TAILSITTER_INPUT_BF_ROLL ||
         tailsitter.input_type == TAILSITTER_INPUT_PLANE)) {
        // the user has asked for body frame controls when tailsitter
        // is active. We switch around the control_in value for the
        // channels to do this, as that ensures the value is
        // consistent throughout the code
        int16_t roll_in = plane.channel_roll->get_control_in();
        int16_t yaw_in = plane.channel_rudder->get_control_in();
        plane.channel_roll->set_control_in(yaw_in);
        // TODO: with earth-frame yaw, right aileron results in left yaw
        // when the nose is not vertical. Hence it is preferable to use
        // TAILSITTER_INPUT_MULTICOPTER when off vertical. This problem
        // should go away with body-frame yaw, as in QACRO mode.
        plane.channel_rudder->set_control_in(-roll_in);
    }
}

/*
  return true if we are a tailsitter transitioning to VTOL flight
 */
bool QuadPlane::in_tailsitter_vtol_transition(void) const
{
    return is_tailsitter() && in_vtol_mode() && transition_state == TRANSITION_ANGLE_WAIT_VTOL;
}

#ifdef DEBUG_SPEED_SCALING
// first order IIR high pass filter
// coefficients b0 = b, b1 = -b
// equation: y_n = b0 x_n + b1 x_{n-1} - a y_{n-1}
// given b1 = -b0 = b: y_n = b (x_n - x_{n-1} - a y_{n-1}
// x1 is the new input value, y is the filtered result
// x0 is used to save the previous input
inline void high_pass(float x1, float &x0, float &y) {
    // coefficients for nyquist/30 cutoff:
    const float b = 0.974482;
    const float a = -0.948964;
    y = b * (x1 - x0) - a * y;
    x0 = x1;
}

// log speed scaler and highpass filtered gyro rates for validation
void log_CTHP(float spd_scaler, bool airspeed_enabled, Vector3f rates) {

    // detect oscillation by monitoring gyros
    static float in0[3] = {0};
    static float hpOut[3] = {0};

    // highpass filter each gyro output
    high_pass(rates.x, in0[0], hpOut[0]);
    high_pass(rates.y, in0[1], hpOut[1]);
    high_pass(rates.z, in0[2], hpOut[2]);

    // square then lowpass filter the highpass outputs
    constexpr float lpcoef = 0.02f;
    static float msq[3] = {0};

    for (int i=0; i<3; i++) {
        msq[i] = (1 - lpcoef) * msq[i] +lpcoef * sq(hpOut[i]);
    }

    static uint32_t last_log_time = 0;
    uint32_t now = AP_HAL::millis();
    if ((now - last_log_time) >= (1000 / 100)) {
        last_log_time = now;

        AP::logger().Write("CTHP", "TimeUS,MSQr,MSQp,MSQy,HPr,HPp,HPy,Scl,ASpd", "Qfffffffb",
                            AP_HAL::micros64(),
                            msq[0], msq[1], msq[2],
                            hpOut[0], hpOut[1], hpOut[2],
                            spd_scaler, airspeed_enabled);
    }
}
#endif

/*
  account for speed scaling of control surfaces in VTOL modes
*/
void QuadPlane::tailsitter_speed_scaling(void)
{
    const float hover_throttle = motors->get_throttle_hover();
    const float throttle = motors->get_throttle();
    float tthr = 1.25f * hover_throttle;

    // critical parameter: violent oscillations if too high
    // sudden loss of attitude control if too low
    constexpr float max_atten = 0.2f;
    float aspeed;
    float spd_scaler = 1;
    bool airspeed_enabled = ahrs.airspeed_sensor_enabled();

    if (tailsitter.motor_mask == 0) {
        if (is_zero(throttle)) {
            spd_scaler = tailsitter.throttle_scale_max;
        } else {
            spd_scaler = constrain_float(hover_throttle / throttle, 0, tailsitter.throttle_scale_max);
        }
    } else {
        // This is a "copter" tailsitter which does not need to boost gain
        // in hover, but does need reduced gains when flying at high speed in Q modes:

        // If there is an airspeed sensor use the measured airspeed
        // The airspeed estimate based only on GPS and (estimated) wind is
        // not sufficiently accurate for tailsitters.
        // (based on tests in RealFlight 8 with 10kph wind)
        // TODO: SITL always returns false from airspeed_sensor_enabled()
        //  (sensor "unhealthy")
        //  but airspeed_estimate() returns a value
        if (airspeed_enabled && ahrs.airspeed_estimate(&aspeed)) {
            // plane.get_speed_scaler() doesn't work well for copter tailsitters
            // ramp down from 1 to max_atten as speed increases to airspeed_max
            spd_scaler = constrain_float(1 - (aspeed / plane.aparm.airspeed_max), max_atten, 1.0f);
        } else {
            // if no airspeed sensor reduce control surface throws at large tilt
            // angles (assuming high airspeed)

            // ramp down from 1 to max_atten at tilt angles over trans_angle
            // (angles here are represented by their cosines)
            constexpr float c_trans_angle = cosf(.125f * M_PI);
            constexpr float alpha = (1 - max_atten) / (c_trans_angle - cosf(radians(90)));
            constexpr float beta = 1 - alpha * c_trans_angle;
            const float c_tilt = ahrs_view->get_rotation_body_to_ned().c.z;
            if (c_tilt < c_trans_angle) {
                spd_scaler = constrain_float(beta + alpha * c_tilt, max_atten, 1.0f);
                // reduce throttle attenuation threshold too
                tthr = 0.5f * hover_throttle;
            }
        }
        // if throttle is above hover thrust, apply additional attenuation
        if (throttle > tthr) {
            const float throttle_atten = 1 - (throttle - tthr) / (1 - tthr);
            spd_scaler *= throttle_atten;
            spd_scaler = constrain_float(spd_scaler, max_atten, 1.0f);
        }
    }
    // limit positive and negative slew rates of applied speed scaling
    constexpr float posTC = 5.0f;   // seconds
    constexpr float negTC = 2.0f;   // seconds
    const float posdelta = plane.G_Dt / posTC;
    const float negdelta = plane.G_Dt / negTC;
    static float last_scale = 0;
    static float scale = 0;
    if ((spd_scaler - last_scale) > 0) {
        if ((spd_scaler - last_scale) > posdelta) {
            scale += posdelta;
        } else {
            scale = spd_scaler;
        }
    } else {
        if ((spd_scaler - last_scale) < -negdelta) {
            scale -= negdelta;
        } else {
            scale = spd_scaler;
        }
    }
    last_scale = scale;

    if (DEBUG_SPEED_SCALING) {
        log_CTHP(scale, airspeed_enabled, ahrs_view->get_gyro_latest());
    }

    const SRV_Channel::Aux_servo_function_t functions[4] = {
        SRV_Channel::Aux_servo_function_t::k_aileron,
        SRV_Channel::Aux_servo_function_t::k_elevator,
        SRV_Channel::Aux_servo_function_t::k_tiltMotorLeft,
        SRV_Channel::Aux_servo_function_t::k_tiltMotorRight};
    for (uint8_t i=0; i<ARRAY_SIZE(functions); i++) {
        int32_t v = SRV_Channels::get_output_scaled(functions[i]);
        v *= scale;
        v = constrain_int32(v, -SERVO_MAX, SERVO_MAX);
        SRV_Channels::set_output_scaled(functions[i], v);
    }
}

