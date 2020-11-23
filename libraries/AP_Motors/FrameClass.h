#pragma once

#include <stdint.h>
#include "frame_class_values.h"

class MotorFrame {
public:
    // Create the frame class enum (FRAME) from frame_class_values.h.
    // Also use frame_class_decl.h to create an array of string values
    // and a virtual method to retrieve the string equivalent for a FRAME value.
    #undef DECL_ENUM_ELEMENT
    #define DECL_ENUM_ELEMENT( element ) element
    enum class CLASS : uint8_t {
        FRAME_CLASS_VALUES,
        NFRAMES
    };

    static const char* class_string_base [(int)CLASS::NFRAMES];
    static const char* get_class_string(CLASS index);
    static void list_all_classes();
};
