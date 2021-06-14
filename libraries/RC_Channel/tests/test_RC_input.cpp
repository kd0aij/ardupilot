/*
 *       Unit Tests for RC_Channel library.
 *       Based on original example by Jason Short. 2010
 */

#include <AP_gtest.h>

#include <AP_HAL/AP_HAL.h>
#include <RC_Channel/RC_Channel.h>

// copied from ArduPlane-stable (4.0.9)
float original_norm_input(uint16_t min, uint16_t max, uint16_t trim, uint16_t in, bool reversed)
{
    float ret;
    int16_t reverse_mul = (reversed?-1:1);
    if (in < trim) {
        if (min >= trim) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(in - trim) / (float)(trim - min);
    } else {
        if (max <= trim) {
            return 0.0f;
        }
        ret = reverse_mul * (float)(in - trim) / (float)(max  - trim);
    }
    return constrain_float(ret, -1.0f, 1.0f);
}

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class RC_Channel_Example : public RC_Channel
{
};

class RC_Channels_Example : public RC_Channels
{
public:

    RC_Channel_Example obj_channels[NUM_RC_CHANNELS];

    RC_Channel_Example *channel(const uint8_t chan) override {
        if (chan >= NUM_RC_CHANNELS) {
            return nullptr;
        }
        return &obj_channels[chan];
    }

protected:

    int8_t flight_mode_channel_number() const override { return 5; }

private:

};

#define RC_CHANNELS_SUBCLASS RC_Channels_Example
#define RC_CHANNEL_SUBCLASS RC_Channel_Example

#include <RC_Channel/RC_Channels_VarInfo.h>

static RC_Channels_Example rc_channels;

TEST(RCInputTest, norm_input_set_radio_in)
{
    rc().init();

    const uint16_t pwm_min = 900;
    const uint16_t pwm_trim = 1500;
    const uint16_t pwm_max = 2100;

    RC_Channel chan = *rc().channel(CH_1);

    chan.set_radio_min(pwm_min);
    chan.set_radio_trim(pwm_trim);
    chan.set_radio_max(pwm_max);

    // this compiles and runs but get_reverse() always returns zero
//    float rev_f = reversed ? 1:0;
//
//    const AP_Param::GroupInfo* const *rc1_GroupInfo = &rc_channels.var_info[0].group_info;
//    if (!AP_Param::set_object_value(&rc_channels, *rc1_GroupInfo, "REVERSED", rev_f)) {
//        printf("WARNING: AP_Param::set object value \"%s::%s\" Failed.\n",
//                rc_channels.var_info->name, "REVERSED");
//    }
//    rev_f = chan.get_reverse();
//    printf("RC1_REVERSED: %d\n", chan.get_reverse());

    const float accuracy = 1.0e-7f;
    float orig_result;
    float new_result;
    
    // test norm_input over the pwm range [pwm_min, pwm_max]
    bool reversed = true;
    do {        
        reversed = !reversed;
        chan.set_reverse(reversed);
        printf("reversed: %d\n", reversed);
        for (uint16_t pwm=pwm_min; pwm<=pwm_max; pwm+=100) {
            orig_result = original_norm_input(pwm_min, pwm_max, pwm_trim, pwm, reversed);
            chan.set_radio_in(pwm);
            new_result = chan.norm_input();
    //        printf("pwm: %4i, orig: %10.7f, new: %10.7f\n", pwm, orig_result, new_result);
            EXPECT_NEAR(new_result, orig_result, accuracy);
        }
    } while (!reversed);
}

TEST(RCInputTest, norm_input_update)
{
    const uint16_t pwm_min = 900;
    const uint16_t pwm_trim = 1500;
    const uint16_t pwm_max = 2100;

    RC_Channel chan = *rc().channel(CH_1);

    chan.set_radio_min(pwm_min);
    chan.set_radio_trim(pwm_trim);
    chan.set_radio_max(pwm_max);

    // no set_reversed in RC_Channel
    bool reversed = chan.get_reverse();

    const float accuracy = 1.0e-7f;
    float orig_result;
    float new_result;
    
    reversed = true;
    do {        
        reversed = !reversed;
        chan.set_reverse(reversed);
        printf("reversed: %d\n", reversed);
        // test norm_input over the pwm range [pwm_min, pwm_max]
        for (uint16_t pwm=pwm_min; pwm<=pwm_max; pwm+=100) {
            orig_result = original_norm_input(pwm_min, pwm_max, pwm_trim, pwm, reversed);

            chan.set_override(pwm, 0);
            chan.update();
            new_result = chan.norm_input();
    //        printf("pwm: %4i, orig: %10.7f, new: %10.7f\n", pwm, orig_result, new_result);
            EXPECT_NEAR(new_result, orig_result, accuracy);
        }
    } while (!reversed);
}

AP_GTEST_MAIN()
