/*
 * Implement methods to supply string values for enum class enumerator values.
 * Uses the C preprocessor to create each string value from its enumerator.
 * concept from: https://www.codeproject.com/Articles/10500/Converting-C-enums-to-strings
 */

#include <string.h>
#include <stdio.h>
#include "MotorFrame.h"

#undef ENUM_VAL
#define ENUM_VAL( element ) #element

const char* MotorFrame::class_string_base [(int)CLASS::NFRAMES] =
{
    CLASS_VALUES
};

const char* MotorFrame::get_class_string(CLASS index)
{
    if ((uint8_t)index < (uint8_t)CLASS::NFRAMES) {
        return class_string_base[(int)index];
    } else {
        return "INVALID";
    }
}

void MotorFrame::list_all_classes() {
    for (int i=0; i<(int)CLASS::NFRAMES; i++) {
        const char* frame_string = get_class_string((CLASS)i);
        if (!strcmp(frame_string, "INVALID")) break;
        ::printf("class %d: %s\n", i, frame_string);
    }
}
