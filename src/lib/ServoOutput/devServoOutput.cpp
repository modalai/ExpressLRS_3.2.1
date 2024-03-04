// #define PLATFORM_STM32
// #define FRSKY_R9MM
#if defined(GPIO_PIN_PWM_OUTPUTS) || defined(PLATFORM_STM32)

#include "devServoOutput.h"
#include "telemetry.h"
#include "CRSF.h"
#include "config.h"
#include "helpers.h"
#include "rxtx_intf.h"
#include "logging.h"

#ifdef FRSKY_R9MM
extern bool currentPwmConfig;
bool servoInitialized;
static uint8_t SERVO_PINS[GPIO_PIN_PWM_OUTPUTS_COUNT];
static uint8_t OUTPUT_CHANNELS[GPIO_PIN_PWM_OUTPUTS_COUNT] = {NO_INPUT, NO_INPUT, NO_INPUT, NO_INPUT};
static uint8_t GPIO_PIN_PWM_OUTPUTS[GPIO_PIN_PWM_OUTPUTS_COUNT] = {R9m_Ch1, R9m_Ch2, R9m_Ch3, R9m_Ch4};
#else
static uint8_t SERVO_PINS[PWM_MAX_CHANNELS];
#endif 

static ServoMgr *servoMgr;
// true when the RX has a new channels packet
static bool newChannelsAvailable;
// Absolute max failsafe time if no update is received, regardless of LQ
static constexpr uint32_t FAILSAFE_ABS_TIMEOUT_MS = 1000U;
extern bool InForceUnbindMode;


void ICACHE_RAM_ATTR servoNewChannelsAvaliable()
{
    newChannelsAvailable = true;
}

uint16_t servoOutputModeToUs(eServoOutputMode mode)
{
    switch (mode)
    {
    case som50Hz:
        return (1000000U / 50U);
    case som60Hz:
        return (1000000U / 60U);
    case som100Hz:
        return (1000000U / 100U);
    case som160Hz:
        return (1000000U / 160U);
    case som333Hz:
        return (1000000U / 333U);
    case som400Hz:
        return (1000000U / 400U);
    case som10KHzDuty:
        return (1000000U / 10000U);
    default:
        return 0;
    }
}

static void servoWrite(uint8_t ch, uint16_t us)
{
    const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
    if ((eServoOutputMode)chConfig->val.mode == somOnOff)
    {
        servoMgr->writeDigital(ch, us > 1500U);
    }
    else
    {
        if ((eServoOutputMode)chConfig->val.mode == som10KHzDuty)
        {
            servoMgr->writeDuty(ch, constrain(us, 1000, 2000) - 1000);
        }
        else
        {
            servoMgr->writeMicroseconds(ch, us / (chConfig->val.narrow + 1));
        }
    }
}

static void servosFailsafe()
{
    /* Using failsafe value of 900us(886us) to ensure that LED Bar Overt LEDs COMPLETELY OFF*/
    constexpr unsigned SERVO_FAILSAFE_MIN = 886U;
    for (unsigned ch = 0; ch < servoMgr->getOutputCnt(); ++ch)
    {
        const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
        // Note: Failsafe values do not respect the inverted flag, failsafes are absolute
        uint16_t us = chConfig->val.failsafe + SERVO_FAILSAFE_MIN;
        // Always write the failsafe position even if the servo never has been started,
        // so all the servos go to their expected position
        servoWrite(ch, us);
    }
}

static int servosUpdate(unsigned long now)
{
    constexpr unsigned SERVO_FAILSAFE_MIN = 886U;
    static uint32_t lastUpdate;
    if (newChannelsAvailable)
    {
        newChannelsAvailable = false;
        lastUpdate = now;
        for (unsigned ch = 0; ch < servoMgr->getOutputCnt(); ++ch)
        {
            const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);

// #ifdef FRSKY_R9MM
//             // Don't bother updating pins not being used 
//             if (chConfig->val.inputChannel == NO_INPUT){
//                 continue;
//             }
// #endif 

            const unsigned crsfVal = CRSF::ChannelData[chConfig->val.inputChannel];
            // crsfVal might 0 if this is a switch channel and it has not been
            // received yet. Delay initializing the servo until the channel is valid
            if (crsfVal == 0)
            {
                continue;
            }

            uint16_t us = CRSF_to_US(crsfVal);
            // Flip the output around the mid value if inverted
            // (1500 - usOutput) + 1500
            if (chConfig->val.inverted)
            {
                us = 3000U - us;
            }
            if (us < 1050U){
                servoWrite(ch, SERVO_FAILSAFE_MIN);
            }
            else if (us > 1500 && us < 1600){
                servoWrite(ch, 1450);
            }
            else{
                servoWrite(ch, us);
            }
        } /* for each servo */
    }     /* if newChannelsAvailable */

    // LQ goes to 0 (100 packets missed in a row)
    // OR last update older than FAILSAFE_ABS_TIMEOUT_MS
    // go to failsafe
    else if (lastUpdate &&
             ((getLq() == 0) || (now - lastUpdate > FAILSAFE_ABS_TIMEOUT_MS)))
    {
        servosFailsafe();
        lastUpdate = 0;
    }

    return DURATION_IMMEDIATELY;
}

static void initialize()
{
    if (!OPT_HAS_SERVO_OUTPUT)
    {
        DBGLN("RX HAS NO SERVO OUTPUT");
        return;
    }

#ifdef FRSKY_R9MM
        DBGLN("RX PWM INITIALIZE DEV_SERVO_OUTPUT");
        servoInitialized = false;
        if (!currentPwmConfig){
            for (uint8_t channel = 0; channel < GPIO_PIN_PWM_OUTPUTS_COUNT; ++channel){
                config.SetPwmChannel(channel, 0, OUTPUT_CHANNELS[channel], false, som50Hz, false);
            }
            config.Commit();
        }
        for (uint8_t channel = 0; channel < GPIO_PIN_PWM_OUTPUTS_COUNT; ++channel){
            const rx_config_pwm_t* ch_temp = config.GetPwmChannel(channel);
            DBGLN("[Ch %u] Input: %u\tFailsafe: %u", channel, ch_temp->val.inputChannel, ch_temp->val.failsafe);
        }
#endif

    // Assign each output channel to a output pin
    for (unsigned ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ++ch)
    {
        uint8_t pin = GPIO_PIN_PWM_OUTPUTS[ch];
#if (defined(DEBUG_LOG) || defined(DEBUG_RCVR_LINKSTATS)) && (defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32))
        // Disconnect the debug UART pins if DEBUG_LOG
        if (pin == 1 || pin == 3)
        {
            pin = servoMgr->PIN_DISCONNECTED;
        }
#endif
        SERVO_PINS[ch] = pin;
        DBGLN("SERVO_PINS[Ch:%u] -> Pin: %u", ch, pin);
    }

    // Initialize all servos to low ASAP
    servoMgr = new ServoMgr(SERVO_PINS, GPIO_PIN_PWM_OUTPUTS_COUNT, 20000U);
    servoMgr->initialize();
    servoInitialized = true;
}

#ifdef FRSKY_R9MM
static void updatePwmChannels(uint8_t inputChannel, uint8_t outputChannel, uint8_t outputPin)
{
    if (!OPT_HAS_SERVO_OUTPUT)
    {
        return;
    }

    // If new input channel is already being used, reset it so two output channels aren't controlled by one input channel
    for (int channel = 0; channel < GPIO_PIN_PWM_OUTPUTS_COUNT; channel++){
        if (OUTPUT_CHANNELS[channel] == inputChannel){
            OUTPUT_CHANNELS[channel] = NO_INPUT;
        }
    }
    OUTPUT_CHANNELS[outputChannel] = inputChannel;  // Bind output pin/channel to input channel

    // Update output channel to output pin configuration
    for (int channel = 0; channel < GPIO_PIN_PWM_OUTPUTS_COUNT; channel++){
        // If the PWM pin we are updating is already assigned, disconnect it and label as available 
        if (SERVO_PINS[channel] == GPIO_PIN_PWM_OUTPUTS[outputPin]){
            SERVO_PINS[channel] = servoMgr->PIN_AVAILABLE;
        }
    }
    // if (inputChannel == NO_INPUT){
    //     SERVO_PINS[outputChannel] = servoMgr->PIN_AVAILABLE;  
    // } else{
    //     SERVO_PINS[outputChannel] = GPIO_PIN_PWM_OUTPUTS[outputPin];    
    // }

    delete servoMgr;

    // Set new configuration
    for (uint8_t channel = 0; channel < GPIO_PIN_PWM_OUTPUTS_COUNT; channel++){
        config.SetPwmChannel(channel,(uint16_t)0, OUTPUT_CHANNELS[channel], false, som50Hz, false);    
    }
    config.Commit();

    // Initialize all servos to low ASAP
    servoMgr = new ServoMgr(SERVO_PINS, GPIO_PIN_PWM_OUTPUTS_COUNT, 20000U);
    servoMgr->initialize();
}
#endif // End FRSKY_R9MM

static uint8_t getOutputChannel(uint8_t inputChannel){
    for (int channel = 0; channel < GPIO_PIN_PWM_OUTPUTS_COUNT; channel++){
        if (OUTPUT_CHANNELS[channel] == inputChannel){
            return channel;
        }
    }
    return -1;
}

static int start()
{
    for (unsigned ch = 0; servoMgr && ch < servoMgr->getOutputCnt(); ++ch)
    {
        const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
        servoMgr->setRefreshInterval(ch, servoOutputModeToUs((eServoOutputMode)chConfig->val.mode));
    }

    return DURATION_NEVER;
}

static int event()
{
    if (InForceUnbindMode){
        servosFailsafe();
        return DURATION_NEVER;
    }

#ifdef FRSKY_R9MM
    if (updatePWM && (PWM)pwmCmd == PWM::SET_PWM_VAL){
        uint8_t outputChannel = getOutputChannel(pwmInputChannel);
        if (outputChannel == -1){
            return DURATION_IMMEDIATELY;
        }

        updatePWM = false;
        if (pwmType == 's'){
            servoMgr->writeMicroseconds(outputChannel, pwmValue);
        }
        else if (pwmType == 'd'){
            servoMgr->writeDuty(outputChannel, pwmValue);
        }

    } else if (updatePWM && (PWM)pwmCmd == PWM::SET_PWM_CH){
        updatePWM = false;
        
        // If channel is active then we should stop it first
        if (servoMgr->isPwmActive(pwmOutputChannel)){
            servoMgr->stopPwm(pwmOutputChannel);
        }

        updatePwmChannels(pwmInputChannel, pwmOutputChannel, pwmPin);
    }

#endif // End FRSKY_R9MM

    if (servoMgr == nullptr || connectionState == disconnected)
    {
        // Disconnected should come after failsafe on the RX
        // so it is safe to shut down when disconnected
        return DURATION_NEVER;
    }
    else if (connectionState == wifiUpdate)
    {
        servoMgr->stopAllPwm();
        return DURATION_NEVER;
    }
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
    return servosUpdate(millis());
}

device_t ServoOut_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
};

#endif
