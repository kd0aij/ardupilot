/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include "FrameClass_Sub.h"

#undef DECL_ENUM_ELEMENT
#define DECL_ENUM_ELEMENT( element ) #element
const char* MotorFrame_Sub::class_string_sub [(int)MotorFrame_Sub::CLASS::NFRAMES] =
{
    FRAME_CLASS_SUB_VALUES
};
const char* MotorFrame_Sub::get_class_string(MotorFrame_Sub::CLASS index)
{
    if ((uint8_t)index < (uint8_t)MotorFrame_Sub::CLASS::NFRAMES) {
        return class_string_sub[(int)index];
    } else {
        return "INVALID";
    }
}
