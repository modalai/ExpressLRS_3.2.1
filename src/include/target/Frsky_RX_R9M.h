/*
Credit to Jacob Walser (jaxxzer) for the pinout!!!
https://github.com/jaxxzer
*/
#if !defined(TARGET_R9SLIM_RX)
    #define TARGET_USE_EEPROM               1
    #define TARGET_EEPROM_ADDR              0x50
#endif

#define GPIO_PIN_SDA            PB7  // EEPROM
#define GPIO_PIN_SCL            PB6  // EEPROM

// /*  Radio 1
#define GPIO_PIN_NSS            PA4  // RADIO 1 - SPI1
#define GPIO_PIN_DIO0           PB5  // RADIO 1 - SPI1
#define GPIO_PIN_MOSI           PA7  // RADIO 1 - SPI1
#define GPIO_PIN_MISO           PA6  // RADIO 1 - SPI1
#define GPIO_PIN_SCK            PA5  // RADIO 1 - SPI1
#define GPIO_PIN_RST            PB2  // RADIO 1 - SPI1
// */

/*  Radio 2
#define GPIO_PIN_NSS            PB12  // RADIO 2 - SPI2
#define GPIO_PIN_DIO0           PB4   // RADIO 2 - SPI2
#define GPIO_PIN_MOSI           PB15  // RADIO 2 - SPI2
#define GPIO_PIN_MISO           PB14  // RADIO 2 - SPI2
#define GPIO_PIN_SCK            PB13  // RADIO 2 - SPI2
#define GPIO_PIN_RST            PA15  // RADIO 2 - SPI2
*/

#if defined(TARGET_R9SLIM_RX)
    #define GPIO_PIN_RCSIGNAL_RX    PA3  // RX1 PIN OF CONNECTOR 1 ON SLIM
    #define GPIO_PIN_RCSIGNAL_TX    PA2  // TX1 PIN OF CONNECTOR 1 ON SLIM
    #define DEVICE_NAME "FrSky R9SLIM RX"
#elif defined(TARGET_R9SLIMPLUS_RX)      // R9SLIMPLUS USES DUAL UART CONFIGURATION FOR TX1/RX1
    #define GPIO_PIN_RCSIGNAL_RX    PB11 // RX1 PIN OF CONNECTOR 1 ON SLIMPLUS
    #define GPIO_PIN_RCSIGNAL_TX    PA9  // TX1 PIN OF CONNECTOR 1 ON SLIMPLUS
    #define DEVICE_NAME "FrSky R9SLIM+"
#elif defined(TARGET_R900MINI_RX)
    #define GPIO_PIN_RCSIGNAL_RX    PA3 // convinient pin for direct chip solder
    #define GPIO_PIN_RCSIGNAL_TX    PA2 // convinient pin for direct chip solder
    #define DEVICE_NAME "Jumper R900 MINI"
#else
    #define GPIO_PIN_RCSIGNAL_RX        PA10
    #define GPIO_PIN_RCSIGNAL_TX        PA9
    #define GPIO_PIN_DEBUG_RX           PA3
    #define GPIO_PIN_DEBUG_TX           PA2
    // #define GPIO_PIN_RCSIGNAL_RX_SBUS   PA3
    // #define GPIO_PIN_RCSIGNAL_TX_SBUS   PA2  // Turning this off so we can use pin for PWM
    #ifndef DEVICE_NAME
        #define DEVICE_NAME "ModalAI M0139"
        // #define DEVICE_NAME "FrSky R9MM"
    #endif
#endif

#if defined(TARGET_R9MX_RX)
    #define GPIO_PIN_LED_RED        PB2 // Red
    #define GPIO_PIN_LED_GREEN      PB3 // Green
    #define GPIO_PIN_BUTTON         PB0  // pullup e.g. LOW when pressed
#elif defined(TARGET_R9SLIM_RX)
    #define GPIO_PIN_LED_RED        PA11 // Red
    #define GPIO_PIN_LED_GREEN      PA12 // Green
    #define GPIO_PIN_BUTTON         PC13  // pullup e.g. LOW when pressed
    /* PB3: RX = HIGH, TX = LOW */
    #define GPIO_PIN_RX_ENABLE      PB3
#elif defined(TARGET_R9SLIMPLUS_RX)
    #define GPIO_PIN_LED_RED        PA11 // Red
    #define GPIO_PIN_LED_GREEN      PA12 // Green
    #define GPIO_PIN_BUTTON         PC13  // pullup e.g. LOW when pressed
    /* PB3: RX = HIGH, TX = LOW */
    #define GPIO_PIN_RX_ENABLE      PB3
    /* PB9: antenna 1 (left) = HIGH, antenna 2 (right) = LOW
     * Note: Right Antenna is selected by default, LOW */
    #define GPIO_PIN_ANT_CTRL PB9
#elif defined(TARGET_R900MINI_RX)
    #define GPIO_PIN_LED_RED        PA11 // Red
    #define GPIO_PIN_LED_GREEN      PA12 // Green
    #define GPIO_PIN_BUTTON         PC13 // pullup e.g. LOW when pressed
    // RF Switch: HIGH = RX, LOW = TX
    #define GPIO_PIN_RX_ENABLE      PB3
#else //R9MM_R9MINI
    #define GPIO_PIN_LED_RED        PA12  // Red
    #define GPIO_PIN_LED_GREEN      PB3   // Green
    #define GPIO_PIN_BUTTON         PA1   // pullup e.g. LOW when pressed
#endif

#define POWER_OUTPUT_FIXED 15 //MAX power for 900 RXes
// External pads
#define R9m_Ch1    PB0     // TIM3 CH3
#define R9m_Ch2    PB1     // TIM3 CH4 
#define R9m_Ch3    PA8     // TIM1 CH1 
#define R9m_Ch4    PA11    // TIM1 CH4 
// #define R9m_Ch3    PA9
// #define R9m_Ch4    PA10
// #define R9m_sbus   PA2
// #define R9m_sport  PA5
// #define R9m_isport PB11

#define M0139
//method to set HSE and clock speed correctly//
// #if defined(HSE_VALUE)
// /* Redefine the HSE value; it's equal to 8 MHz on the STM32F4-DISCOVERY Kit */
// #undef HSE_VALUE
// #define HSE_VALUE ((uint32_t)16000000).
// #define HSE_VALUE    25000000U
// #endif /* HSE_VALUE */
#define SYSCLK_FREQ_72MHz


// Output Power - Default to SX1272 max output
