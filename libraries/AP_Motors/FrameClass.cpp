/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "FrameClass.h"

#undef DECL_ENUM_ELEMENT
#define DECL_ENUM_ELEMENT( element ) #element
const char* MotorFrame::class_string_base [(int)CLASS::NFRAMES] =
{
    FRAME_CLASS_VALUES
};
const char* MotorFrame::get_class_string(CLASS index)
{
    if ((uint8_t)index < (uint8_t)CLASS::NFRAMES) {
        return class_string_base[(int)index];
    } else {
        return "INVALID";
    }
}
