//
// Unit tests for the AP_Common code
//

#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_SerialManager/AP_SerialManager.h>

void setup();
void loop();
void test_print(void);

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_SerialManager _serialmanager;
GCS_Dummy _gcs;

extern mavlink_system_t mavlink_system;

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};


/*
 *  euler angle tests
 */
void setup(void)
{
    ::printf("DBGprint tests\n\n");
    gcs().init();
    gcs().setup_console();

    hal.scheduler->delay(1000);
}

void loop(void)
{
    if (AP_HAL::millis() < 10000) {
        DBGprint::printf(DBGTYPE::PRINTF, 250, 10, "printf %d\n", (int)AP_HAL::millis());
        DBGprint::printf(DBGTYPE::CONSOLE, 500, 10, "console %d\n", (int)AP_HAL::millis());
        DBGprint::printf(DBGTYPE::GCS, 500, 1000, "gcs %d\n", (int)AP_HAL::millis());

        static bool value = false;
        if ((AP_HAL::millis() % 1000) == 0) {
            value = !value;
        }
        DBGprint::watch(DBGTYPE::PRINTF, value, 0, "watch, value: %d, time: %d\n", value, (int)AP_HAL::millis());
    }

    // Delay 1 mS for 1 KHz loop rate
    hal.scheduler->delay(1);
}

AP_HAL_MAIN();
