/* 
 * File:   MotorFrame_Sub.h
 * Author: markw
 * These are the motor frame types supported by AP_Motors6DOF
 *
 * Created on November 9, 2020, 11:32 AM
 */
#pragma once

#include <stdint.h>
#include "MotorFrame_sub_values.h"

class MotorFrame_Sub {
public:
    // Create the frame class enum (CLASS) from MotorFrame_sub_values.h.
    // Also use frame_class_decl.h to create an array of string values
    // and a virtual method to retrieve the string equivalent for a FRAME value.
    #undef ENUM_VAL
    #define ENUM_VAL( element ) element
    enum class CLASS : uint8_t {
        CLASS_VALUES_SUB,
        NFRAMES
    };

    static const char* class_string_sub [(int)CLASS::NFRAMES];
    static const char* get_class_string(CLASS index);
    static void list_all_classes();
};
