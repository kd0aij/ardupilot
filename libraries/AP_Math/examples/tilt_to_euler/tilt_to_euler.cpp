//
// tests for lateral_tilt_to_euler_roll conversion
//

#include <AP_HAL/AP_HAL.h>

float lateral_tilt_to_euler_roll(const float roll_out, const float pitch_out);
void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <iostream>
#include <fstream>
#include <vector>
using namespace std;

void print_line(std::vector<double> &vec, ofstream &f);

void print_line(vector<double> &vec, ofstream &f)
{
    for (double v : vec) {
        f << v << ",";
    }
    f << "\n";
}

float lateral_tilt_to_euler_roll(const float roll_out, const float pitch_out) {
    // do lateral tilt to euler roll conversion
    return (18000/M_PI) * atanf(cosf(pitch_out*(M_PI/18000))*tanf(roll_out*(M_PI/18000)));
}

void test_conversion()
{
    ofstream csv_out;

    const int ns = 101;
    const int ns_2 = 50;
    vector<double> xy_values(ns, 0);
    // ns x ns result
    vector<vector<double>> curve(ns, vector<double>(ns, 0));
    
    const float max_angle = 8000;
    // range -max_angle to +max_angle centidegrees
    for (int j=0; j<ns; j++) {
        double pitch_cd =  (max_angle / (ns_2)) * (j-ns_2);
        xy_values.at(j) = pitch_cd;
        for (int i=0; i<ns; i++) {
            double roll_cd =  (max_angle / (ns_2)) * (i-ns_2);
            curve[j].at(i) = lateral_tilt_to_euler_roll(roll_cd, pitch_cd);
        }
    }

    // write plot data to csv file
    csv_out.open("tilt_conversion_data.csv");
    // x axis
    print_line(xy_values, csv_out);
    // result
    for (int j=0; j<ns; j++) {
        print_line(curve[j], csv_out);
    }
    csv_out.close();

    int status = system("python ./libraries/AP_Math/examples/tilt_to_euler/tilt_to_euler.py");
    status = system("rm tilt_conversion_data.csv");
    status = system("mv tilt_conversion.png ./libraries/AP_Math/examples/tilt_to_euler");
    
    exit(status);
}
#else
void test_conversion()
{
    hal.console->begin(115200);
    hal.console->printf("tilt_to_euler.cpp must be built for SITL: \n");
    hal.console->printf("./waf configure --board sitl\n");
    hal.console->printf("./waf --target examples/tilt_to_euler");
    exit(0);
}
#endif

/*
 *  throttle_curve tests
 */
void setup(void)
{
    hal.console->printf("test_conversion unit tests\n\n");

    test_conversion();
}

void loop(void) {}

AP_HAL_MAIN();
