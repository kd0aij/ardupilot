/* 
 * File:   frame_class_getstring.h
 * Author: markw
 *
 * Created on November 8, 2020, 5:07 PM
 */
#if ( !defined(MOTOR_FRAME_CLASS_STRING_H))
#define MOTOR_FRAME_CLASS_STRING_H

#include "EnumDef.h"
#include "EnumGetString.h"

///////////////////////////////
// The enum declaration
///////////////////////////////
BEGIN_ENUM(FRAME)
{
#include "frame_class_decl.h"
}
END_ENUM(FRAME)

#endif // (!defined(MOTOR_FRAME_CLASS_STRING_H)
