/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
///
/// @file       DBGprint.h
/// @brief		convenience functions for debug prints.
///

#pragma once

#include <stdio.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>

class DBGprint {
public:
    // static methods which can be placed in fast loops, but with actual output 
    // rate constrained by a minimum interval
    static void printf(const uint32_t interval_ms, const char *fmt, ...) FMT_PRINTF(2, 3) {
        void *uid = __builtin_return_address(0);
        va_list ap;
        va_start(ap, fmt);
        output_txt(uid, PRINTF, interval_ms, fmt, ap);
        va_end(ap);
    }
    
    static void console(const uint32_t interval_ms, const char *fmt, ...) FMT_PRINTF(2, 3) {
        void *uid = __builtin_return_address(0);
        va_list ap;
        va_start(ap, fmt);
        output_txt(uid, CONSOLE, interval_ms, fmt, ap);
        va_end(ap);
    }

    static void gcstxt(const uint32_t interval_ms, const char *fmt, ...) FMT_PRINTF(2, 3) {
        void *uid = __builtin_return_address(0);
        va_list ap;
        va_start(ap, fmt);
        output_txt(uid, GCS, interval_ms, fmt, ap);
        va_end(ap);
    }
    
    static void dflog(const uint32_t interval_ms, const char *recnam, const char *fields, const char *fmt, ...) {
        void *call_loc = __builtin_return_address(0);
        DBGprint *instance = get_instance(call_loc);
        uint32_t now = AP_HAL::millis();
        if ((instance->last_millis == 0) ||
            (now - instance->last_millis) >= interval_ms) {
            instance->last_millis = now;
            va_list ap;
            va_start(ap, fmt);
            AP::logger().WriteV(recnam, fields, nullptr, nullptr, fmt, ap);
            va_end(ap);
        }
    }
    
private:
    enum Otype {
        PRINTF,
        CONSOLE,
        GCS
    };

    // find instance pointer for this call_loc
    static DBGprint *get_instance(void* call_loc) {
        uint32_t index = 0;
        while ((call_loc != _call_loc[index]) && (index < _num_instances)) {
            index++;
        }
        if (index >= MAX_INSTANCES) {
            // too many instances, fail
            ::printf("too many DBGprint instances %d: %lx\n", index, (long)call_loc);
            return nullptr;
        }
        if (index == _num_instances) {
            // call_loc not found and instance slot available
            // create new instance for this call location
            _num_instances++;
            _call_loc[index] = call_loc;
            _instance[index] = new DBGprint();
            ::printf("construct new DBGprint instance %d: %lx\n", index, (long)call_loc);
        }
        // successful
        return _instance[index];
    }

    // generate output at intervals no less than interval_ms
    static void output_txt(void *call_loc, const Otype type, const uint32_t interval_ms, const char *fmt, va_list ap) {
        DBGprint *instance = get_instance(call_loc);
        // failure: too many DBGprint instances
        if (!instance) return;
        uint32_t now = AP_HAL::millis();
        // if this is the first call, or if interval_ms has elapsed since the last call
        if ((instance->last_millis == 0) ||
            (now - instance->last_millis) >= interval_ms) {
            // generate the requested output
            instance->last_millis = now;
            switch (type) {
                case PRINTF:
                    vprintf(fmt,ap);
                    break;
            case CONSOLE:
                AP_HAL::get_HAL().console->vprintf(fmt, ap);
                break;
            case GCS:
                gcs().send_textv(MAV_SEVERITY_INFO, fmt, ap);
                break;
            }
        }
    }

    static const uint32_t MAX_INSTANCES = 10;
    static uint32_t _num_instances;
    static DBGprint *_instance[MAX_INSTANCES];
    static void *_call_loc[MAX_INSTANCES];

    uint32_t last_millis;
};
