#include "mode.h"
#include "Plane.h"

bool ModeQBoat::_enter()
{
    return plane.mode_qstabilize._enter();
}

void ModeQBoat::update()
{
    plane.mode_qstabilize.update();
    
    // override nav_pitch
    if (plane.nav_pitch_cd < 0.5 * plane.pitch_limit_min_cd) {
        plane.nav_pitch_cd -= 0.5 * plane.pitch_limit_min_cd;
    } else if (plane.nav_pitch_cd < 0) {
    plane.nav_pitch_cd = 0;
    }
}
