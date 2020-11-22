/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#undef DECL_ENUM_ELEMENT
#define DECL_ENUM_ELEMENT( element ) #element
    const char* FrameClass::frame_class_string_base [(int)FRAME::NFRAMES] =
    {
        FRAME_CLASS_VALUES
    };
    virtual const char* FrameClass::get_frame_string(FRAME index)
    {
        if ((uint8_t)index < (uint8_t)FRAME::NFRAMES) {
            return frame_class_string_base[(int)index];
        } else {
            return "INVALID";
        }
    }
