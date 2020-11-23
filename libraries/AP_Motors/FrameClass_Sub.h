/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   FrameClass_Sub.h
 * Author: markw
 *
 * Created on November 22, 2020, 12:33 PM
 */

#pragma once

#include <stdint.h>
#include "frame_class_sub_values.h"

class MotorFrame_Sub {
public:
    // Create the frame class enum (FRAME) from frame_class_values.h.
    // Also use frame_class_decl.h to create an array of string values
    // and a virtual method to retrieve the string equivalent for a FRAME value.
    #undef DECL_ENUM_ELEMENT
    #define DECL_ENUM_ELEMENT( element ) element
    enum class CLASS : uint8_t {
        FRAME_CLASS_SUB_VALUES,
        NFRAMES
    };

    static const char* class_string_sub [(int)CLASS::NFRAMES];
    static const char* get_class_string(CLASS index);
};
