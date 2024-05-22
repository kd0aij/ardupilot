#include "mode.h"
#include "Plane.h"

bool ModeFBWB::_enter()
{
#if HAL_SOARING_ENABLED
    // for ArduSoar soaring_controller
    plane.g2.soaring_controller.init_cruising();
#endif

    plane.set_target_altitude_current();

    return true;
}

void ModeFBWB::update()
{
    // Thanks to Yury MonZon for the altitude limit code!
    calc_nav_roll_cd_from_stick_input();
    plane.update_load_factor();
    plane.update_fbwb_speed_height();

}

