#include "mode.h"
#include "Plane.h"

bool ModeQSFTHR::_enter()
{
    return plane.mode_qstabilize._enter();
}

// this update method differs from ModeQStabilize::update()
// only in the roll_limit setting; it is higher since this is a more aerobatic
// use case than qstabilize
// TODO: add an argument to ModeQStabilize::update() instead of duplicating code
void ModeQSFTHR::update()
{
    // set nav_roll and nav_pitch using sticks

    // use larger of Plane and quadplane roll limits (or just the plane limit)
    int16_t roll_limit = MAX(plane.roll_limit_cd, plane.quadplane.aparm.angle_max);

    float pitch_input = plane.channel_pitch->norm_input();
    // Scale from normalized input [-1,1] to centidegrees
    if (plane.quadplane.tailsitter_active()) {
        // separate limit for tailsitter roll, if set
        if (plane.quadplane.tailsitter.max_roll_angle > 0) {
            roll_limit = plane.quadplane.tailsitter.max_roll_angle * 100.0f;
        }

        // angle max for tailsitter pitch
        plane.nav_pitch_cd = pitch_input * plane.quadplane.aparm.angle_max;
    } else {
        // NOT a tailsitter
        if (pitch_input > 0) {
            // (back stick, up elevator)
            // pitch is further constrained by LIM_PITCH_MIN/MAX which may impose
            // tighter (possibly asymmetrical) limits than Q_ANGLE_MAX
            plane.nav_pitch_cd = pitch_input * MIN(plane.aparm.pitch_limit_max_cd, plane.quadplane.aparm.angle_max);
        } else {
            // (forward stick, down elevator) pitch up enough to compensate for motor tilt
            float mixScale = 0.5;
            if (plane.quadplane.rc_fwd_thr_ch != nullptr) {
                mixScale = 0.5f * (plane.quadplane.rc_fwd_thr_ch->norm_input() + 1);
            }
            plane.nav_pitch_cd = -pitch_input * mixScale * 1000;
        }
        plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min_cd, plane.aparm.pitch_limit_max_cd.get());
    }

    plane.nav_roll_cd = (plane.channel_roll->get_control_in() / 4500.0) * roll_limit;
    plane.nav_roll_cd = constrain_int32(plane.nav_roll_cd, -roll_limit, roll_limit);
}
