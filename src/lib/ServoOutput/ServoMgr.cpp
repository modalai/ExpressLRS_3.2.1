#if defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32) || defined(FRSKY_R9MM) || defined(PLATFORM_STM32)

#include "ServoMgr.h"
#include "logging.h"
#include "waveform_8266.h"
#include <math.h>

#ifdef FRSKY_R9MM
    #include "variant_R9MM.h"
#endif

/* Private variables */
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* Private function prototypes */
static void GPIO_Init(void);
static void TIM1_Init(void);
static void TIM2_Init(void);

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
static void PWM_Error_Handler()
{
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
//   while (1)
//   {
//     /*Write both GREEN and RED LED HIGH to indicate error*/
//     digitalWrite(GPIO_PIN_LED_GREEN, HIGH);
//     digitalWrite(GPIO_PIN_LED, HIGH);
//   }
}

ServoMgr::ServoMgr(const uint8_t *const pins, const uint8_t outputCnt, uint32_t defaultInterval)
    : _pins(pins), _outputCnt(outputCnt), _refreshInterval(new uint16_t[outputCnt]), _activePwmChannels(0), _resolution_bits(new uint8_t[outputCnt])
{
    for (uint8_t ch = 0; ch < _outputCnt; ++ch)
    {
        _refreshInterval[ch] = defaultInterval;
    }
}

#if defined(PLATFORM_ESP32)
#include "driver/ledc.h"

#ifdef SOC_LEDC_SUPPORT_XTAL_CLOCK
#define LEDC_DEFAULT_CLK LEDC_USE_XTAL_CLK
#else
#define LEDC_DEFAULT_CLK LEDC_AUTO_CLK
#endif

extern uint8_t channels_resolution[];

/*
 * Modified versions of the ledcSetup/ledcAttachPin from Arduino ESP32 Hal which allows
 * control of the timer to use rather than being directly associated with the channel. This
 * allows multiple channels to use the same timer within a LEDC group (high/low speed) when
 * they are set to the same frequency.
 */
static void ledcSetupEx(uint8_t chan, ledc_timer_t timer, uint32_t freq, uint8_t bit_num)
{
    ledc_mode_t group = (ledc_mode_t)(chan / 8);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = group,
        .duty_resolution = (ledc_timer_bit_t)bit_num,
        .timer_num = timer,
        .freq_hz = freq,
        .clk_cfg = LEDC_DEFAULT_CLK};
    if (ledc_timer_config(&ledc_timer) != ESP_OK)
    {
        log_e("ledc setup failed!");
        return;
    }
    channels_resolution[chan] = bit_num;
}

static void ledcAttachPinEx(uint8_t pin, uint8_t chan, ledc_timer_t timer)
{
    uint8_t group = (chan / 8), channel = (chan % 8);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = pin,
        .speed_mode = (ledc_mode_t)group,
        .channel = (ledc_channel_t)channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = (ledc_timer_t)timer,
        .duty = 0,
        .hpoint = 0};
    ledc_channel_config(&ledc_channel);
}

void ServoMgr::allocateLedcChn(uint8_t ch, uint16_t intervalUs, uint8_t pin)
{
    uint32_t target_freq = 1000000U / intervalUs;
    _resolution_bits[ch] = (uint16_t)(log2f(80000000.0f / target_freq)); // no matter high speed timer or low speed timer, the clk src can be 80Mhz
    if (_resolution_bits[ch] > 16)
    {
        _resolution_bits[ch] = 16;
    }

    uint8_t group = ch / 8;
    for (int i = 0; i <= 3; i++)
    {
        uint8_t timer_idx = group * 4 + i;
        if (_timerConfigs[timer_idx] == 0)
        {
            _timerConfigs[timer_idx] = target_freq;
            ledcSetupEx(ch, (ledc_timer_t)i, target_freq, _resolution_bits[ch]);
        }
        if (_timerConfigs[timer_idx] == target_freq)
        {
            ledcAttachPinEx(pin, ch, (ledc_timer_t)i);
            DBGLN("allocate ledc_ch %d on pin %d using group: %d, ledc_tim: %d, bits: %d", ch, pin, group, i, _resolution_bits[ch]);
            return;
        }
    }
    DBGLN("Could not allocate timer for channel %d", ch);
}
#endif

void ServoMgr::initialize()
{
#ifdef FRSKY_R9MM
    /* Init GPIOs, TIM1, TIM2*/
    GPIO_Init();
    TIM1_Init();
    TIM2_Init();

    /* Start PWM on each CH at 900us so LEDs on LED Bar are all off */
    for (uint8_t ch = 0; ch < _outputCnt; ++ch)
    {
        const uint8_t pin = _pins[ch];
        if (pin != PIN_DISCONNECTED)
        {
            if (ch == 0)        // R9m_Ch1 -> PA8 -> TIM1_CH1
            {
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                HAL_Delay(10);
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0U);  // ALL LEDs OFF
            } 
            else if (ch == 1)   // R9m_Ch2 -> PA11 -> TIM1_CH4
            {
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
                HAL_Delay(10);
                __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0U);  // ALL LEDs OFF
            }
            else if (ch == 2)   // R9m_Ch3 -> PA2 -> TIM2_CH3
            {
                HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
                HAL_Delay(10);
                __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, 0U);  // ALL LEDs OFF
            }
        }
    }
#elif defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32)
    for (uint8_t ch = 0; ch < _outputCnt; ++ch)
    {
        const uint8_t pin = _pins[ch];
        if (pin != PIN_DISCONNECTED)
        {
            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
        }
    }
#endif

}

void ServoMgr::writeMicroseconds(uint8_t ch, uint16_t valueUs)
{
    const uint8_t pin = _pins[ch];
    if (pin == PIN_DISCONNECTED)
    {
        return;
    }
    _activePwmChannels |= (1 << ch);
#if defined(PLATFORM_ESP32)
    ledcWrite(ch, map(valueUs, 0, _refreshInterval[ch], 0, (1 << _resolution_bits[ch]) - 1));
#elif defined(PLATFORM_ESP8266)
    startWaveform8266(pin, valueUs, _refreshInterval[ch] - valueUs);
#elif defined(FRSKY_R9MM)
    if (ch == 0)        // R9m_Ch1 -> PA8 -> TIM1_CH1
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, valueUs);
    } 
    else if (ch == 1)   // R9m_Ch2 -> PA11 -> TIM1_CH4
    {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, valueUs);
    }
    else if (ch == 2)   // R9m_Ch3 -> PA2 -> TIM2_CH3
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, valueUs);
    }
#endif
}

void ServoMgr::writeDuty(uint8_t ch, uint16_t duty)
{
    const uint8_t pin = _pins[ch];
    if (pin == PIN_DISCONNECTED)
    {
        return;
    }
    _activePwmChannels |= (1 << ch);
#if defined(PLATFORM_ESP32)
    ledcWrite(ch, map(duty, 0, 1000, 0, (1 << _resolution_bits[ch]) - 1));
#elif defined(PLATFORM_ESP8266)
    uint16_t high = map(duty, 0, 1000, 0, _refreshInterval[ch]);
    startWaveform8266(pin, high, _refreshInterval[ch] - high);
#elif defined(FRSKY_R9MM)
    if (ch == 0)        // R9m_Ch1 -> PA8 -> TIM1_CH1
    {
        uint32_t valueUs = htim1.Init.Period * (duty/100);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, valueUs);
    } 
    else if (ch == 1)   // R9m_Ch2 -> PA11 -> TIM1_CH4
    {
        uint32_t valueUs = htim1.Init.Period * (duty/100);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, valueUs);
    }
    else if (ch == 2)   // R9m_Ch3 -> PA2 -> TIM2_CH3
    {
        uint32_t valueUs = htim2.Init.Period * (duty/100);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, valueUs);
    }
#endif
}

void ServoMgr::setRefreshInterval(uint8_t ch, uint16_t intervalUs)
{
    if (intervalUs != 0)
    {
        _refreshInterval[ch] = intervalUs;
#if defined(PLATFORM_ESP32)
        const uint8_t pin = _pins[ch];
        if (pin == PIN_DISCONNECTED)
        {
            return;
        }
        allocateLedcChn(ch, intervalUs, pin);
#endif
    }
}

void ServoMgr::stopPwm(uint8_t ch)
{
    const uint8_t pin = _pins[ch];
    if (pin == PIN_DISCONNECTED)
    {
        return;
    }
    _activePwmChannels &= ~(1 << ch);
#if defined(PLATFORM_ESP32)
    ledcDetachPin(pin);
#elif defined(PLATFORM_ESP8266)
    stopWaveform8266(pin);
#elif defined(FRSKY_R9MM)
    if (ch == 0)        // R9m_Ch1 -> PA8 -> TIM1_CH1
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    } 
    else if (ch == 1)   // R9m_Ch2 -> PA11 -> TIM1_CH4
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    }
    else if (ch == 2)   // R9m_Ch3 -> PA2 -> TIM2_CH3
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    }
#endif
    digitalWrite(pin, LOW);
}

void ServoMgr::stopAllPwm()
{
    for (uint8_t ch = 0; ch < _outputCnt; ++ch)
    {
        stopPwm(ch);
    }
    _activePwmChannels = 0;
}

void ServoMgr::writeDigital(uint8_t ch, bool value)
{
    const uint8_t pin = _pins[ch];
    if (pin == PIN_DISCONNECTED)
    {
        return;
    }
    if (isPwmActive(ch))
    {
        stopPwm(ch);
        // Wait for the last edge, which is at most 1 cycle from now
        delay((_refreshInterval[ch] / 1000U) + 1);
    }
    digitalWrite(pin, value);
}

/**
  * @brief TIM1 Initialization Function
  *         ARR = 2068 (Max 2100 us period, 32 clock cycle delay so we use 2068)
  *         900  us HIGH -> 886  us (ARR * DUTY/100)... DUTY is ~42.84% (900/2100)
  *         1400 us HIGH -> 1379 us (ARR * DUTY/100)... DUTY is ~66.66% (1400/2100)
  *         2100 us HIGH -> 2068 us (ARR * DUTY/100)... DUTY is 100% (2100/2100)
  *
  * @param None
  * @retval None
  */
static void TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72;        // 72MHz APB2 CLK -> 1MHz APB2 CLK
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2068;         // 2100 us at 1 MHz 
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    PWM_Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    PWM_Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    PWM_Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    PWM_Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    PWM_Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    PWM_Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    PWM_Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function. 
  *         ARR = 2068 (Max 2100 us period, 32 clock cycle delay so we use 2068)
  *         900  us HIGH -> 886  us (ARR * DUTY/100)... DUTY is ~42.84% (900/2100)
  *         1400 us HIGH -> 1379 us (ARR * DUTY/100)... DUTY is ~66.66% (1400/2100)
  *         2100 us HIGH -> 2068 us (ARR * DUTY/100)... DUTY is 100% (2100/2100)
  * 
  * @param None
  * @retval None
  */
static void TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72;        // 72MHz APB1 CLK -> 1MHz APB1 CLK
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2068;         // 2100 us at 1 MHz 
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    PWM_Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    PWM_Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    PWM_Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    PWM_Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    PWM_Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim->Instance==TIM1)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA8     ------> TIM1_CH1
    PA11    ------> TIM1_CH4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if(htim->Instance==TIM2)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM2 GPIO Configuration
    PA2     ------> TIM2_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

#endif