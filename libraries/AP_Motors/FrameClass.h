/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   enum_class_frame.h
 * Author: markw
 *
 * Created on November 22, 2020, 12:33 PM
 */

#pragma once

#include "frame_class_values.h"

class FrameClass {
public:
    // Create the frame class enum (FRAME) from frame_class_values.h.
    // Also use frame_class_decl.h to create an array of string values
    // and a virtual method to retrieve the string equivalent for a FRAME value.
    #undef DECL_ENUM_ELEMENT
    #define DECL_ENUM_ELEMENT( element ) element
    enum class FRAME : uint8_t {
        FRAME_CLASS_VALUES,
        NFRAMES
    };

    static const char* frame_class_string_base [(int)FRAME::NFRAMES];
    virtual const char* get_frame_string(FRAME index);
};
