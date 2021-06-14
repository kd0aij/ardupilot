/*
 *       Unit Tests for RC_Channel library.
 *       Based on original example by Jason Short. 2010
 */

#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

//#define DBGPRINT
#ifdef DBGPRINT
#define  DEBUG_PRINTF(fmt, ...)  printf(fmt, __VA_ARGS__);
#else
#define DEBUG_PRINTF(fmt, ...)    /* Do nothing */
#endif

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class RC_Channel_Test : public RC_Channel
{
public:
    void set_reverse(bool v) { reversed.set(v); }
    void set_dead_zone(uint16_t v) { dead_zone.set(v); }
};

class RC_Channels_Test : public RC_Channels
{
public:

    RC_Channel_Test obj_channels[NUM_RC_CHANNELS];

    RC_Channel_Test *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    int8_t flight_mode_channel_number() const override { return 5; }

private:

};

#define RC_CHANNELS_SUBCLASS RC_Channels_Test
#define RC_CHANNEL_SUBCLASS RC_Channel_Test

#include <RC_Channel/RC_Channels_VarInfo.h>

static RC_Channels_Test rc_channels;

// Use source code from stable release of ArduPlane to define correct behavior
class GoldStandard
{
public:
    // copied from ArduPlane-stable (4.0.9)
    static float norm_input(int16_t radio_min, int16_t radio_max,
                                     int16_t radio_trim, int16_t radio_in, bool reversed)
    {
        float ret;
        int16_t reverse_mul = (reversed?-1:1);
        if (radio_in < radio_trim) {
            if (radio_min >= radio_trim) {
                return 0.0f;
            }
            ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_trim - radio_min);
        } else {
            if (radio_max <= radio_trim) {
                return 0.0f;
            }
            ret = reverse_mul * (float)(radio_in - radio_trim) / (float)(radio_max  - radio_trim);
        }
        return constrain_float(ret, -1.0f, 1.0f);
    }

    static float norm_input_dz(int16_t dead_zone, int16_t radio_min, int16_t radio_max,
                                        int16_t radio_trim, int16_t radio_in, bool reversed)
    {
        int16_t dz_min = radio_trim - dead_zone;
        int16_t dz_max = radio_trim + dead_zone;
        float ret;
        int16_t reverse_mul = (reversed?-1:1);
        if (radio_in < dz_min && dz_min > radio_min) {
            ret = reverse_mul * (float)(radio_in - dz_min) / (float)(dz_min - radio_min);
        } else if (radio_in > dz_max && radio_max > dz_max) {
            ret = reverse_mul * (float)(radio_in - dz_max) / (float)(radio_max  - dz_max);
        } else {
            ret = 0;
        }
        return constrain_float(ret, -1.0f, 1.0f);
    }

    static int16_t pwm_to_angle_dz_trim(uint16_t _dead_zone, uint16_t _trim, int16_t radio_in,
                                        int16_t radio_min, int16_t radio_max, int16_t high_in, bool reversed)
    {
        int16_t radio_trim_high = _trim + _dead_zone;
        int16_t radio_trim_low  = _trim - _dead_zone;

        int16_t reverse_mul = (reversed?-1:1);

        // don't allow out of range values
        int16_t r_in = constrain_int16(radio_in, radio_min, radio_max);

        if (r_in > radio_trim_high && radio_max != radio_trim_high) {
            return reverse_mul * ((int32_t)high_in * (int32_t)(r_in - radio_trim_high)) / (int32_t)(radio_max  - radio_trim_high);
        } else if (r_in < radio_trim_low && radio_trim_low != radio_min) {
            return reverse_mul * ((int32_t)high_in * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_trim_low - radio_min);
        } else {
            return 0;
        }
    }

    static int16_t pwm_to_range_dz(uint16_t _dead_zone, int16_t radio_in, int16_t radio_min,
                                   int16_t radio_max, int16_t high_in, bool reversed)
    {
        int16_t r_in = constrain_int16(radio_in, radio_min, radio_max);

        if (reversed) {
            r_in = radio_max - (r_in - radio_min);
        }

        int16_t radio_trim_low  = radio_min + _dead_zone;

        if (r_in > radio_trim_low) {
            return (((int32_t)(high_in) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
        }
        return 0;
    }

};

const int16_t pwm_min = 900;
const int16_t pwm_trim = 1500;
const int16_t pwm_max = 2100;
const uint16_t dead_zone = 30;
const float accuracy = 1.0e-7f;

// Test norm_input and norm_input_dz with set_radio_in(pwm)
TEST(RCInputTest, norm_input_set_radio_in)
{
    RC_Channel_Test chan = *rc_channels.channel(CH_1);

    chan.set_radio_min(pwm_min);
    chan.set_radio_trim(pwm_trim);
    chan.set_radio_max(pwm_max);
    chan.set_dead_zone(dead_zone);

    float orig_norm_input;
    float new_norm_input;
    float orig_norm_input_dz;
    float new_norm_input_dz;

    bool reversed = true;
    do {
        reversed = !reversed;
        chan.set_reverse(reversed);
        DEBUG_PRINTF("reversed: %d, dead_zone: %d\n", reversed, dead_zone);
        // compare with original norm_input over the pwm range [pwm_min, pwm_max]
        for (int16_t pwm=pwm_min; pwm<=pwm_max; pwm+=1) {
            orig_norm_input = GoldStandard::norm_input(pwm_min, pwm_max, pwm_trim, pwm, reversed);
            orig_norm_input_dz = GoldStandard::norm_input_dz(dead_zone, pwm_min, pwm_max, pwm_trim, pwm, reversed);
            chan.set_radio_in(pwm);
            new_norm_input = chan.norm_input();
            new_norm_input_dz = chan.norm_input_dz();
            DEBUG_PRINTF("pwm: %4i, orig: %10.7f, new: %10.7f\n", pwm, orig_norm_input_dz, new_norm_input_dz);
            EXPECT_NEAR(new_norm_input, orig_norm_input, accuracy);
            EXPECT_NEAR(new_norm_input_dz, orig_norm_input_dz, accuracy);
        }
        // test out of range pwm inputs
        float rmin = reversed ? 1 : -1;
        float rmax = -rmin;
        chan.set_radio_in(pwm_min - 100);
        EXPECT_NEAR(chan.norm_input(), rmin, accuracy);
        chan.set_radio_in(pwm_max + 100);
        EXPECT_NEAR(chan.norm_input(), rmax, accuracy);

        // test out of range trim values
        chan.set_radio_in(pwm_min);
        chan.set_radio_trim(pwm_min - 100);
        EXPECT_NEAR(chan.norm_input(), rmin, accuracy);
        chan.set_radio_trim(pwm_max + 100);
        EXPECT_NEAR(chan.norm_input(), rmin, accuracy);

        chan.set_radio_trim(pwm_min - 100);
        chan.set_radio_in(pwm_max);
        EXPECT_NEAR(chan.norm_input(), rmax, accuracy);
        chan.set_radio_trim(pwm_max);
        chan.set_radio_in(pwm_max + 100);
        EXPECT_NEAR(chan.norm_input(), 0, accuracy);
        chan.set_radio_trim(pwm_trim);
    } while (!reversed);

}

// Test norm_input and norm_input_dz with set_override(pwm) followed by update()
TEST(RCInputTest, norm_input_update)
{
    RC_Channel_Test chan = *rc_channels.channel(CH_1);

    chan.set_radio_min(pwm_min);
    chan.set_radio_trim(pwm_trim);
    chan.set_radio_max(pwm_max);
    chan.set_dead_zone(dead_zone);

    float orig_norm_input;
    float new_norm_input;
    float orig_norm_input_dz;
    float new_norm_input_dz;

    bool reversed = true;
    do {
        reversed = !reversed;
        chan.set_reverse(reversed);
        DEBUG_PRINTF("reversed: %d\n", reversed);
        // compare with original norm_input over the pwm range [pwm_min, pwm_max]
        for (int16_t pwm=pwm_min; pwm<=pwm_max; pwm+=1) {
            orig_norm_input = GoldStandard::norm_input(pwm_min, pwm_max, pwm_trim, pwm, reversed);
            orig_norm_input_dz = GoldStandard::norm_input_dz(dead_zone, pwm_min, pwm_max, pwm_trim, pwm, reversed);

            chan.set_override(pwm, 0);
            chan.update();
            new_norm_input = chan.norm_input();
            new_norm_input_dz = chan.norm_input_dz();
            DEBUG_PRINTF("pwm: %4i, orig: %10.7f, new: %10.7f\n", pwm, orig_norm_input, new_norm_input);
            EXPECT_NEAR(new_norm_input, orig_norm_input, accuracy);
            EXPECT_NEAR(new_norm_input_dz, orig_norm_input_dz, accuracy);
        }
        // test out of range inputs
        float rmin = reversed ? 1 : -1;
        float rmax = -rmin;
        chan.set_radio_in(pwm_min - 100);
        EXPECT_NEAR(chan.norm_input_dz(), rmin, accuracy);
        chan.set_radio_in(pwm_max + 100);
        EXPECT_NEAR(chan.norm_input_dz(), rmax, accuracy);
    } while (!reversed);
}

// Test control_input and control_input_no_dz with set_override(pwm) followed by update()
// for RC_CHANNEL_TYPE_RANGE
TEST(RCInputTest, control_input_range)
{
    RC_Channel_Test chan = *rc_channels.channel(CH_1);

    chan.set_radio_min(pwm_min);
    chan.set_radio_trim(pwm_trim);
    chan.set_radio_max(pwm_max);
    chan.set_dead_zone(dead_zone);
    int16_t high_in = 1000;
    // sets high_in
    chan.set_range(high_in);

    // test RC_CHANNEL_TYPE_RANGE
    // control_in maps [pwm_min, pwm_max] to [0, high_in]
    bool reversed = true;
    do {
        reversed = !reversed;
        chan.set_reverse(reversed);
        DEBUG_PRINTF("reversed: %d\n", reversed);
        // test over the pwm range [pwm_min, pwm_max]
        for (int16_t pwm=pwm_min; pwm<=pwm_max; pwm+=1) {
            int16_t tc_in = GoldStandard::pwm_to_range_dz(dead_zone, pwm, pwm_min, pwm_max, high_in, reversed);
            int16_t tc_in_no_dz = GoldStandard::pwm_to_range_dz(0, pwm, pwm_min, pwm_max, high_in, reversed);
            chan.set_override(pwm, 0);
            chan.update();
            int16_t c_in = chan.get_control_in();
            int16_t c_in_no_dz = chan.get_control_in_zero_dz();
            DEBUG_PRINTF("pwm: %4i, tc_in: %d, c_in: %d\n", pwm, tc_in, c_in);
            EXPECT_EQ(c_in, tc_in);
            EXPECT_EQ(c_in_no_dz, tc_in_no_dz);
        }
        // test out of range inputs
        int16_t rmin = reversed ? high_in : 0;
        int16_t rmax = high_in - rmin;
        chan.set_radio_in(pwm_min - 100);
        EXPECT_EQ(chan.get_control_in(), rmin);
        chan.set_radio_in(pwm_max + 100);
        EXPECT_EQ(chan.get_control_in(), rmax);
    } while (!reversed);

}

// Test control_input and control_input_no_dz with set_override(pwm) followed by update()
// for RC_CHANNEL_TYPE_ANGLE
TEST(RCInputTest, control_input_angle)
{
    RC_Channel_Test chan = *rc_channels.channel(CH_1);

    // for angle type channels, trim is expected to be near (pwm_max-pwm_min)/2
    int16_t trim = 1533;

    int16_t high_in = 4500;
    // sets high_in
    chan.set_angle(high_in);

    chan.set_radio_min(pwm_min);
    chan.set_radio_trim(trim);
    chan.set_radio_max(pwm_max);
    chan.set_dead_zone(dead_zone);

    // test RC_CHANNEL_TYPE_ANGLE
    // control_in maps [pwm_min, pwm_max] to [-high_in, high_in]
    bool reversed = true;
    do {
        reversed = !reversed;
        chan.set_reverse(reversed);
        DEBUG_PRINTF("reversed: %d\n", reversed);
        // test over the pwm range [pwm_min, pwm_max]
        for (int16_t pwm=pwm_min; pwm<=pwm_max; pwm+=1) {
            int16_t tc_in = GoldStandard::pwm_to_angle_dz_trim(dead_zone, trim, pwm, pwm_min, pwm_max, high_in, reversed);
            int16_t tc_in_no_dz = GoldStandard::pwm_to_angle_dz_trim(0, trim, pwm, pwm_min, pwm_max, high_in, reversed);
            chan.set_override(pwm, 0);
            chan.update();
            int16_t c_in = chan.get_control_in();
            int16_t c_in_no_dz = chan.get_control_in_zero_dz();
            DEBUG_PRINTF("pwm: %4i, tc_in: %d, c_in: %d\n", pwm, tc_in, c_in);
            EXPECT_EQ(c_in, tc_in);
            EXPECT_EQ(c_in_no_dz, tc_in_no_dz);
        }
        // test out of range inputs
        int16_t rmin = reversed ? high_in : -high_in;
        int16_t rmax = -rmin;
        chan.set_radio_in(pwm_min - 100);
        EXPECT_EQ(chan.get_control_in(), rmin);
        chan.set_radio_in(pwm_max + 100);
        EXPECT_EQ(chan.get_control_in(), rmax);
    } while (!reversed);

}

// Verify that set_control_in() results in correct values for norm_in
// for channels of type RC_CHANNEL_TYPE_ANGLE
TEST(RCInputTest, set_control_in_angle)
{
    RC_Channel_Test chan = *rc_channels.channel(CH_1);

    // for angle type channels, trim is expected to be near (pwm_max-pwm_min)/2
    int16_t trim = 1533;

    int16_t high_in = (pwm_max - pwm_min) / 2;
    // sets high_in
    chan.set_angle(high_in);

    chan.set_radio_min(pwm_min);
    chan.set_radio_trim(trim);
    chan.set_radio_max(pwm_max);
    chan.set_dead_zone(dead_zone);

    /* test RC_CHANNEL_TYPE_ANGLE
     * control_in maps (reversed = false):
     * [pwm_min, trim-dz] to [-high_in, 0]
     * [trim-dz, trim+dz] to zero
     * [trim+dz, pwm_max] to [0, high_in]
     *
     * control_in maps (reversed = true):
     * [pwm_min, trim-dz] to [high_in, 0]
     * [trim-dz, trim+dz] to zero
     * [trim+dz, pwm_max] to [0, -high_in]
     */
    bool reversed = true;
    do {
        reversed = !reversed;
        chan.set_reverse(reversed);
        DEBUG_PRINTF("reversed: %d\n", reversed);
        // test over the control_in range [-high_in, high_in]
        for (int16_t c_in=-high_in; c_in<=high_in; c_in+=10) {
            chan.set_control_in(c_in);
            // norm_in should range [-1, 1]
            float n_in = chan.norm_input();

            // to compare normalized value to control_in value
            // map n_in [-1, 1] to control_in [-high_in, high_in]
            int16_t tc_in = round(n_in * high_in);
            DEBUG_PRINTF("control_in: %4i, tc_in: %4i, norm_input: %10.7f\n", c_in, tc_in, n_in);

            EXPECT_NEAR(c_in, tc_in, 1);
            EXPECT_NEAR(n_in, chan.norm_input_dz(), 1e-6);
        }
    } while (!reversed);

}

// Verify that set_control_in() results in correct values for norm_in
// for channels of type RC_CHANNEL_TYPE_RANGE
TEST(RCInputTest, set_control_in_range)
{
    RC_Channel_Test chan = *rc_channels.channel(CH_1);

    // for range type channels, trim is expected to be near pwm_min
    int16_t trim = pwm_min;

    int16_t high_in = (pwm_max - pwm_min);
    // sets high_in
    chan.set_range(high_in);

    chan.set_radio_min(pwm_min);
    chan.set_radio_trim(trim);
    chan.set_radio_max(pwm_max);
    chan.set_dead_zone(dead_zone);

    // test RC_CHANNEL_TYPE_RANGE
    // control_in maps [pwm_min, pwm_max] to [0, high_in]
    bool reversed = true;
    do {
        reversed = !reversed;
        chan.set_reverse(reversed);
        DEBUG_PRINTF("reversed: %d\n", reversed);
        // test over the control_in range [0, high_in]
        for (int16_t c_in=0; c_in<=high_in; c_in+=10) {
            chan.set_control_in(c_in);
            // norm_in should range [-1, 1]
            float n_in = chan.norm_input();

            // map n_in [-1, 1] to control_in [0, high_in]
            // to compare normalized value to control_in value
            int16_t tc_in = round(n_in * high_in);
            DEBUG_PRINTF("control_in: %4i, tc_in: %4i, norm_input: %10.7f\n", c_in, tc_in, n_in);

            EXPECT_NEAR(c_in, tc_in, 1);
            EXPECT_NEAR(n_in, chan.norm_input_dz(), 1e-6);
        }
    } while (!reversed);

}

AP_GTEST_MAIN()
