#pragma once
#if defined(GPIO_PIN_PWM_OUTPUTS) || defined(PLATFORM_STM32)
// #if defined(GPIO_PIN_PWM_OUTPUTS) 

#include "ServoMgr.h"

#include "device.h"
#include "common.h"

extern device_t ServoOut_device;
#define HAS_SERVO_OUTPUT

#if defined(GPIO_PIN_PWM_OUTPUTS)
#define OPT_HAS_SERVO_OUTPUT (GPIO_PIN_PWM_OUTPUTS_COUNT > 0)
#endif

#if defined(PLATFORM_STM32)
#define OPT_HAS_SERVO_OUTPUT 1
#endif

// Notify this unit that new channel data has arrived
void servoNewChannelsAvaliable();
// Convert eServoOutputMode to microseconds, or 0 for non-servo modes
uint16_t servoOutputModeToUs(eServoOutputMode mode);
#else
inline void servoNewChannelsAvaliable(){};
#endif
