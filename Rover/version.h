#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

// NOTE: The 'N' in 'Vx.y.z-N-gfr' needs to match the 'N' in '(FIRMWARE_VERSION_TYPE_DEV+N)' to set FW_TYPE
#define THISFIRMWARE "ArduRover V4.4.0-2-gfr"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,4,0,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 4
#define FW_MINOR 4
#define FW_PATCH 0
#define FW_TYPE (FIRMWARE_VERSION_TYPE_DEV+2)

#include <AP_Common/AP_FWVersionDefine.h>
