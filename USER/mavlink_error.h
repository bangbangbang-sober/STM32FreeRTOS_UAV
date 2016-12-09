#ifndef MAVLINK_AVOID_ERRORS_H
#define MAVLINK_AVOID_ERRORS_H
#include "mavlink.h"
#include "mavlink_types.h"
#include "mavlink_helpers.h"
/*??..\MAVLINK\common\../mavlink_types.h(53): error: #20: identifier "pack" is undefined*/
#define MAVPACKED( __Declaration__ ) __Declaration__
/*??..\MAVLINK\common\../mavlink_types.h(53): error: #3092: anonymous unions are only supported in --gnu mode, or when enabled with #pragma anon_unions*/
#pragma anon_unions
#define inline __INLINE

#include"mavlink_types.h"
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEPARATE_HELPERS
//mavlink_system_t mavlink_system = {0,0};
mavlink_system_t mavlink_system ={
1,
1
};// System ID, 1-255, Component/Subsystem ID, 1-255


#endif//AVLINK_AVOID_ERRORS_H
