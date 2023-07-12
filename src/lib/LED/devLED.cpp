#include "targets.h"
#include "common.h"
#include "devLED.h"

#include "crsf_protocol.h"
#include "POWERMGNT.h"

#ifdef HAS_LED

/* Set red led to default */
#ifndef GPIO_PIN_LED_RED
#ifdef GPIO_PIN_LED
#define GPIO_PIN_LED_RED GPIO_PIN_LED
#else
#define GPIO_PIN_LED_RED UNDEF_PIN
#endif
#endif
#ifndef GPIO_PIN_LED
#define GPIO_PIN_LED GPIO_PIN_LED_RED
#endif
#ifndef GPIO_PIN_LED_GREEN
#define GPIO_PIN_LED_GREEN UNDEF_PIN
#endif
#ifndef GPIO_PIN_LED_BLUE
#define GPIO_PIN_LED_BLUE UNDEF_PIN
#endif
#ifndef GPIO_LED_RED_INVERTED
#define GPIO_LED_RED_INVERTED 0
#endif
#ifndef GPIO_LED_GREEN_INVERTED
#define GPIO_LED_GREEN_INVERTED 0
#endif
#ifndef GPIO_LED_BLUE_INVERTED
#define GPIO_LED_BLUE_INVERTED 0
#endif

// #ifdef R9m_Ch1
// #define CH1 R9m_Ch1
// #endif
// #ifdef R9m_Ch2
// #define CH2 R9m_Ch2
// #endif

extern bool InBindingMode;
#if defined(TARGET_RX)
extern bool connectionHasModelMatch;
#endif

constexpr uint8_t LEDSEQ_RADIO_FAILED[] = { 20, 100 }; // 200ms on, 1000ms off
constexpr uint8_t LEDSEQ_DISCONNECTED[] = { 50, 50 };  // 500ms on, 500ms off
constexpr uint8_t LEDSEQ_WIFI_UPDATE[] = { 2, 3 };     // 20ms on, 30ms off
constexpr uint8_t LEDSEQ_BINDING[] = { 10, 10, 10, 100 };   // 2x 100ms blink, 1s pause
constexpr uint8_t LEDSEQ_CUSTOM[] = { 10, 10, 10, 10, 100, 100 };   // 2x 100ms blink, 1s pause
constexpr uint8_t LEDSEQ_MODEL_MISMATCH[] = { 10, 10, 10, 10, 10, 100 };   // 3x 100ms blink, 1s pause
constexpr uint8_t LEDSEQ_UPDATE[] = { 20, 5, 5, 5, 5, 40 };   // 200ms on, 2x 50ms off/on, 400ms off

static uint8_t _pin = -1;
static uint8_t _pin_inverted;
static const uint8_t *_durations;
static uint8_t _count;
static uint8_t _counter = 0;
static bool hasRGBLeds = false;
static bool hasGBLeds = false;

static uint16_t updateLED()
{
    if (_pin == UNDEF_PIN)
    {
        return DURATION_NEVER;
    }
    if(_counter % 2 == 1)
        digitalWrite(_pin, LOW ^ _pin_inverted);
    else
        digitalWrite(_pin, HIGH ^ _pin_inverted);
    if (_counter >= _count)
    {
        _counter = 0;
    }
    return _durations[_counter++] * 10;
}

static uint16_t flashLED(uint8_t pin, uint8_t pin_inverted, const uint8_t durations[], uint8_t count)
{
    _counter = 0;
    _pin = pin;
    _pin_inverted = pin_inverted;
    _durations = durations;
    _count = count;
    return updateLED();
}

static void initialize()
{
    // TODO for future PR, remove TARGET_TX, TARGET_RX, and TARGET_TX_FM30 defines.
    #if defined(TARGET_TX)
        if (GPIO_PIN_LED_BLUE != UNDEF_PIN)
        {
            pinMode(GPIO_PIN_LED_BLUE, OUTPUT);
            digitalWrite(GPIO_PIN_LED_BLUE, LOW ^ GPIO_LED_BLUE_INVERTED);
        }
        if (GPIO_PIN_LED_GREEN != UNDEF_PIN)
        {
            pinMode(GPIO_PIN_LED_GREEN, OUTPUT);
            digitalWrite(GPIO_PIN_LED_GREEN, HIGH ^ GPIO_LED_GREEN_INVERTED);
        }
        if (GPIO_PIN_LED_RED != UNDEF_PIN)
        {
            pinMode(GPIO_PIN_LED_RED, OUTPUT);
            digitalWrite(GPIO_PIN_LED_RED, LOW ^ GPIO_LED_RED_INVERTED);
        }
        if (GPIO_PIN_LED_BLUE != UNDEF_PIN && GPIO_PIN_LED_GREEN != UNDEF_PIN && GPIO_PIN_LED_RED != UNDEF_PIN)
        {
            hasRGBLeds = true;
            digitalWrite(GPIO_PIN_LED_GREEN, LOW);
            digitalWrite(GPIO_PIN_LED_RED, HIGH);
            digitalWrite(GPIO_PIN_LED_BLUE, LOW);
        }
        else if (GPIO_PIN_LED_BLUE != UNDEF_PIN && GPIO_PIN_LED_GREEN != UNDEF_PIN && GPIO_PIN_LED_RED == UNDEF_PIN)
        {
            hasGBLeds = true;
        }
        #if defined(TARGET_TX_FM30)
            pinMode(GPIO_PIN_LED_RED_GREEN, OUTPUT); // Green LED on "Red" LED (off)
            digitalWrite(GPIO_PIN_LED_RED_GREEN, HIGH);
            pinMode(GPIO_PIN_LED_GREEN_RED, OUTPUT); // Red LED on "Green" LED (off)
            digitalWrite(GPIO_PIN_LED_GREEN_RED, HIGH);
        #endif
    #endif
    #if defined(TARGET_RX)
        if (GPIO_PIN_LED_GREEN != UNDEF_PIN)
        {
            pinMode(GPIO_PIN_LED_GREEN, OUTPUT);
            digitalWrite(GPIO_PIN_LED_GREEN, LOW ^ GPIO_LED_GREEN_INVERTED);
        }
        if (GPIO_PIN_LED_RED != UNDEF_PIN)
        {
            pinMode(GPIO_PIN_LED_RED, OUTPUT);
            digitalWrite(GPIO_PIN_LED_RED, LOW ^ GPIO_LED_RED_INVERTED);
        }
        if (GPIO_PIN_LED != UNDEF_PIN)
        {
            pinMode(GPIO_PIN_LED, OUTPUT);
            digitalWrite(GPIO_PIN_LED, LOW ^ GPIO_LED_RED_INVERTED);
        }
        // #ifdef CH1
        // pinMode(CH1, OUTPUT);
        // #endif
        // #ifdef CH2
        // pinMode(CH2, OUTPUT);
        // #endif

    #endif
}

static int timeout()
{
    return updateLED();
}

static void setPowerLEDs()
{
    if (hasGBLeds)
    {
        switch (POWERMGNT::currPower())
        {
        case PWR_250mW:
            digitalWrite(GPIO_PIN_LED_BLUE, HIGH);
            digitalWrite(GPIO_PIN_LED_GREEN, HIGH);
            break;
        case PWR_500mW:
            digitalWrite(GPIO_PIN_LED_BLUE, LOW);
            digitalWrite(GPIO_PIN_LED_GREEN, HIGH);
            break;
        case PWR_10mW:
        case PWR_25mW:
        case PWR_50mW:
        case PWR_100mW:
        default:
            digitalWrite(GPIO_PIN_LED_BLUE, HIGH);
            digitalWrite(GPIO_PIN_LED_GREEN, LOW);
            break;
        }
    }
}

static int event()
{
    #if defined(TARGET_RX)
        if (InBindingMode && GPIO_PIN_LED != UNDEF_PIN)
        {
            // #ifdef CH1
            // #ifdef HAL_TIM_MODULE_ENABLED
            // digitalWrite(CH1, HIGH);
            // #else
            // digitalWrite(CH1, LOW);
            // #endif
            // #endif
            // #ifdef CH2
            // digitalWrite(CH2, HIGH);
            // #endif 

            // #ifdef CH1 
            // return flashLED(GPIO_PIN_LED, GPIO_LED_RED_INVERTED, LEDSEQ_CUSTOM, sizeof(LEDSEQ_CUSTOM));
            // #else
            return flashLED(GPIO_PIN_LED, GPIO_LED_RED_INVERTED, LEDSEQ_BINDING, sizeof(LEDSEQ_BINDING));
            // #endif
        }
    #else
        setPowerLEDs();
    #endif
    switch (connectionState)
    {
    case connected:
        #if defined(TARGET_TX)
            if (hasRGBLeds)
            {
                digitalWrite(GPIO_PIN_LED_GREEN, HIGH);
                digitalWrite(GPIO_PIN_LED_RED, LOW);
                digitalWrite(GPIO_PIN_LED_BLUE, LOW);
            }
            else if (GPIO_PIN_LED_RED != UNDEF_PIN)
            {
                digitalWrite(GPIO_PIN_LED_RED, HIGH ^ GPIO_LED_RED_INVERTED);
            }
        #endif
        #if defined(TARGET_RX)
            if (GPIO_PIN_LED_GREEN != UNDEF_PIN)
            {
                digitalWrite(GPIO_PIN_LED_GREEN, HIGH ^ GPIO_LED_GREEN_INVERTED);
            }

            if (GPIO_PIN_LED_RED != UNDEF_PIN)
            {
                digitalWrite(GPIO_PIN_LED_RED, HIGH ^ GPIO_LED_RED_INVERTED);
            }

            if (GPIO_PIN_LED != UNDEF_PIN)
            {
                if (connectionHasModelMatch)
                {
                    digitalWrite(GPIO_PIN_LED, HIGH ^ GPIO_LED_RED_INVERTED); // turn on led
                }
                else
                {
                    return flashLED(GPIO_PIN_LED, GPIO_LED_RED_INVERTED, LEDSEQ_MODEL_MISMATCH, sizeof(LEDSEQ_MODEL_MISMATCH));
                }
            }
        #endif
        return DURATION_NEVER;
    case disconnected:
        #if defined(TARGET_TX)
            if (hasRGBLeds)
            {
                digitalWrite(GPIO_PIN_LED_GREEN, LOW);
                digitalWrite(GPIO_PIN_LED_BLUE, LOW);
                return flashLED(GPIO_PIN_LED_RED, GPIO_LED_RED_INVERTED, LEDSEQ_DISCONNECTED, sizeof(LEDSEQ_DISCONNECTED));
            }
            else if (GPIO_PIN_LED_RED != UNDEF_PIN)
            {
                digitalWrite(GPIO_PIN_LED_RED, LOW ^ GPIO_LED_RED_INVERTED);
            }
        #endif
        #if defined(TARGET_RX)
            if (GPIO_PIN_LED_GREEN != UNDEF_PIN)
            {
                digitalWrite(GPIO_PIN_LED_GREEN, LOW ^ GPIO_LED_GREEN_INVERTED);
            }
            if (GPIO_PIN_LED_RED != UNDEF_PIN)
            {
                digitalWrite(GPIO_PIN_LED_RED, LOW ^ GPIO_LED_RED_INVERTED);
            }
            if (GPIO_PIN_LED != UNDEF_PIN)
            {
                return flashLED(GPIO_PIN_LED, GPIO_LED_RED_INVERTED, LEDSEQ_DISCONNECTED, sizeof(LEDSEQ_DISCONNECTED));
            }
            else if (GPIO_PIN_LED_GREEN != UNDEF_PIN)
            {
                return flashLED(GPIO_PIN_LED_GREEN, GPIO_LED_GREEN_INVERTED, LEDSEQ_DISCONNECTED, sizeof(LEDSEQ_DISCONNECTED));
            }
        #endif
        return DURATION_NEVER;
    case wifiUpdate:
        #if defined(TARGET_TX)
            if (hasRGBLeds)
            {
                digitalWrite(GPIO_PIN_LED_GREEN, LOW);
                digitalWrite(GPIO_PIN_LED_RED, LOW);
                return flashLED(GPIO_PIN_LED_BLUE, GPIO_LED_BLUE_INVERTED, LEDSEQ_WIFI_UPDATE, sizeof(LEDSEQ_WIFI_UPDATE));
            }
            else if (GPIO_PIN_LED_RED != UNDEF_PIN)
            {
                return flashLED(GPIO_PIN_LED_RED, GPIO_LED_RED_INVERTED, LEDSEQ_WIFI_UPDATE, sizeof(LEDSEQ_WIFI_UPDATE));
            }
            return DURATION_NEVER;
        #endif
        #if defined(TARGET_RX)
            if (GPIO_PIN_LED != UNDEF_PIN)
            {
                return flashLED(GPIO_PIN_LED, GPIO_LED_RED_INVERTED, LEDSEQ_WIFI_UPDATE, sizeof(LEDSEQ_WIFI_UPDATE));
            }
        #else
            return DURATION_NEVER;
        #endif
    case radioFailed:
        if (hasRGBLeds)
        {
            digitalWrite(GPIO_PIN_LED_GREEN, LOW);
            digitalWrite(GPIO_PIN_LED_BLUE, LOW);
            return flashLED(GPIO_PIN_LED_RED, GPIO_LED_RED_INVERTED, LEDSEQ_RADIO_FAILED, sizeof(LEDSEQ_RADIO_FAILED));
        }
        else if (GPIO_PIN_LED_GREEN != UNDEF_PIN)
        {
            digitalWrite(GPIO_PIN_LED_GREEN, LOW ^ GPIO_LED_GREEN_INVERTED);
        }
        if (GPIO_PIN_LED_RED != UNDEF_PIN)
        {
            return flashLED(GPIO_PIN_LED_RED, GPIO_LED_RED_INVERTED, LEDSEQ_RADIO_FAILED, sizeof(LEDSEQ_RADIO_FAILED));
        }
        else if (GPIO_PIN_LED != UNDEF_PIN)
        {
            return flashLED(GPIO_PIN_LED, GPIO_LED_RED_INVERTED, LEDSEQ_RADIO_FAILED, sizeof(LEDSEQ_RADIO_FAILED));
        }
        return DURATION_NEVER;
    case noCrossfire:
        if (GPIO_PIN_LED_RED != UNDEF_PIN)
        {
            // technically nocrossfire is {10,100} but {20,100} is close enough
            return flashLED(GPIO_PIN_LED_RED, GPIO_LED_RED_INVERTED, LEDSEQ_RADIO_FAILED, sizeof(LEDSEQ_RADIO_FAILED));
        }
    case serialUpdate:
        if (GPIO_PIN_LED_RED != UNDEF_PIN)
        {
            return flashLED(GPIO_PIN_LED_RED, GPIO_LED_RED_INVERTED, LEDSEQ_UPDATE, sizeof(LEDSEQ_UPDATE));
        }
    default:
        return DURATION_NEVER;
    }
}

static int test(int type=0)
{   
    if(type == 0){
        // Blink GREEN LED on FrSky R9MM mini twice then HIGH for 2 sec
        digitalWrite(GPIO_PIN_LED_GREEN, HIGH);
        delay(200);
        digitalWrite(GPIO_PIN_LED_GREEN, LOW);
        delay(200);
        digitalWrite(GPIO_PIN_LED_GREEN, HIGH);
        delay(200);
        digitalWrite(GPIO_PIN_LED_GREEN, LOW);
        delay(200);
        digitalWrite(GPIO_PIN_LED_GREEN, HIGH);
        delay(2000);
        digitalWrite(GPIO_PIN_LED_GREEN, LOW);
        delay(100);
    }
    else if (type == 1)
    {
        // Write Green LED on FrSky R9MM mini HIGH for 5 sec then LOW for 1 sec
        digitalWrite(GPIO_PIN_LED_GREEN, HIGH);
        delay(5000);
        digitalWrite(GPIO_PIN_LED_GREEN, LOW);
        delay(1000);
    }
    else if (type == 2){
        // Blink RED LED on FrSky R9MM mini twice then HIGH for 1 sec
        digitalWrite(GPIO_PIN_LED_RED, HIGH);
        delay(100);
        digitalWrite(GPIO_PIN_LED_RED, LOW);
        delay(100);
        digitalWrite(GPIO_PIN_LED_RED, HIGH);
        delay(100);
        digitalWrite(GPIO_PIN_LED_RED, LOW);
        delay(100);
        digitalWrite(GPIO_PIN_LED_RED, HIGH);
        delay(1000);
        digitalWrite(GPIO_PIN_LED_RED, LOW);
        delay(100);

    }
    return 0;
}

device_t LED_device = {
    .initialize = initialize,
    .start = event,
    .event = event,
    .timeout = timeout,
    .test = test,
};

#endif