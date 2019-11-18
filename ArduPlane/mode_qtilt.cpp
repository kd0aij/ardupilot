#include "mode.h"
#include "Plane.h"

bool ModeQTilt::_enter()
{
    return plane.mode_qstabilize._enter();
//    return plane.mode_qacro._enter();
}

void ModeQTilt::update()
{
    return plane.mode_qstabilize.update();
//    return plane.mode_qacro.update();
}
