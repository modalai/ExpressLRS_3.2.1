#pragma once
#if defined(GPIO_PIN_PWM_OUTPUTS) || defined(PLATFORM_STM32)

#include "ServoMgr.h"

#include "device.h"
#include "common.h"

extern device_t ServoOut_device;
#define HAS_SERVO_OUTPUT

#ifdef FRSKY_R9MM
#define NO_INPUT 0x0F
#define GPIO_PIN_PWM_OUTPUTS_COUNT 4
extern bool updatePWM;
extern uint8_t pwmPin;
extern uint8_t pwmCmd;
extern uint8_t pwmOutputChannel;
extern uint8_t pwmInputChannel;
extern uint8_t pwmType;
extern uint16_t pwmValue; 
#endif // End FRSKY_R9MM

#if defined(GPIO_PIN_PWM_OUTPUTS)
#define OPT_HAS_SERVO_OUTPUT (GPIO_PIN_PWM_OUTPUTS_COUNT > 0)
#endif

#if defined(PLATFORM_STM32) || defined(FRSKY_R9MM)
#define OPT_HAS_SERVO_OUTPUT 1
#endif

// Notify this unit that new channel data has arrived
void servoNewChannelsAvaliable();
// Convert eServoOutputMode to microseconds, or 0 for non-servo modes
uint16_t servoOutputModeToUs(eServoOutputMode mode);
#else
inline void servoNewChannelsAvaliable(){};
#endif
