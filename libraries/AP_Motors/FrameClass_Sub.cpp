#include <string.h>
#include <stdio.h>
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

void MotorFrame_Sub::list_all_classes() {
    for (int i=0; i<(int)CLASS::NFRAMES; i++) {
        const char* frame_string = get_class_string((CLASS)i);
        if (!strcmp(frame_string, "INVALID")) break;
        ::printf("class %d: %s\n", i, frame_string);
    }
}

