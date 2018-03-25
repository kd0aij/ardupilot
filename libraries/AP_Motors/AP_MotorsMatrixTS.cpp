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
 *       AP_MotorsMatrixTS.cpp - tailsitters with multicopter motor configuration
 */

#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrixTS.h"

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500
#define THROTTLE_RANGE       100

void AP_MotorsMatrixTS::output_to_motors()
{
    // calls calc_thrust_to_pwm(_thrust_rpyt_out[i]) for each enabled motor
    AP_MotorsMatrix::output_to_motors();

    // also actuate control surfaces
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron,  -_yaw_in * SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, _pitch_in * SERVO_OUTPUT_RANGE);
}

void AP_MotorsMatrixTS::output_armed_stabilizing()
{
    // calculates thrust values _thrust_rpyt_out
    AP_MotorsMatrix::output_armed_stabilizing();
}

void AP_MotorsMatrixTS::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }

    bool success = false;

    switch (frame_class) {

        case MOTOR_FRAME_TS_QUAD:
            switch (frame_type) {
                case MOTOR_FRAME_TYPE_PLUS:
                    add_motor(AP_MOTORS_MOT_1,  90, 0, 2);
                    add_motor(AP_MOTORS_MOT_2, -90, 0, 4);
                    add_motor(AP_MOTORS_MOT_3,   0, 0,  1);
                    add_motor(AP_MOTORS_MOT_4, 180, 0,  3);
                    success = true;
                    break;
                default:
                    // matrixTS doesn't support the configured frame_type
                    break;
            }

        default:
            // matrixTS doesn't support the configured frame_class
            break;
        } // switch frame_class

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();

    _flags.initialised_ok = success;
}
