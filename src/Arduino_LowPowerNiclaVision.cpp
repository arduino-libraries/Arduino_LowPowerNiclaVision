/*
********************************************************************************
*                      Arduino_LowPowerNiclaVision Library
*
*                 Copyright 2024 Arduino SA. http://arduino.cc
*
*                Original Author: A. Vidstrom (info@arduino.cc)
*
*         An API for the management of Sleep, Deep Sleep and Standby
*         Mode for the STM32H747 microcontroller on the Nicla Vision
*
*                    SPDX-License-Identifier: MPL-2.0
*
*      This Source Code Form is subject to the terms of the Mozilla Public
*      License, v. 2.0. If a copy of the MPL was not distributed with this
*      file, you can obtain one at: http://mozilla.org/MPL/2.0/
*
********************************************************************************
*/

/*
********************************************************************************
*                           Included header files
********************************************************************************
*/

#include "Arduino_LowPowerNiclaVision.h"

/*
********************************************************************************
*                               NMI handling
********************************************************************************
*/

extern "C" void NMI_Handler(void)
{
    // Make sure that the NMI doesn't trigger again and again when the HSE
    // clock has failed. If there's some other kind of NMI, we want to enter an
    // infinite loop. But we can't check the RCC_CIFR_HSECSSF flag in RCC->CIFR,
    // because there seems to be a problem with the uC where it sometimes
    // triggers an NMI for HSE failure without setting the flag. Then we would
    // go into an infinite loop by mistake, so we just check RCC_CR_CSSHSEON
    // instead.
    if ((RCC->CR) & RCC_CR_CSSHSEON)
    {
        LL_RCC_ClearFlag_HSECSS();
    }
    else
    {
        for ( ; ; )
        {
            // See the C++11 standard 1.10 point 24 (as well as point 1 for
            // implementations with only one thread of execution still being
            // considered executing a thread)
            volatile int dummy = 0;
            (void) dummy;
        }
    }
}

/*
********************************************************************************
*                           Overloaded operators
********************************************************************************
*/

RTCWakeupDelay operator+(const RTCWakeupDelay d1, const RTCWakeupDelay d2)
{
    return RTCWakeupDelay(d1.value + d2.value);
}

RTCWakeupDelay operator""_s(const unsigned long long int seconds)
{
    return RTCWakeupDelay(seconds);
}

RTCWakeupDelay operator""_min(const unsigned long long int minutes)
{
    return RTCWakeupDelay(minutes * 60);
}

RTCWakeupDelay operator""_h(const unsigned long long int hours)
{
    return RTCWakeupDelay(hours * 60 * 60);
}

/*
********************************************************************************
*                              Instantiations
********************************************************************************
*/

const LowPowerNiclaVision& LowPower = LowPowerNiclaVision::getInstance();

/*
********************************************************************************
*                             Member functions
********************************************************************************
*/

void LowPowerNiclaVision::allowDeepSleep() const
{
  // Turn off USB
  USBPhy * const phy = get_usb_phy();
  phy->deinit();
  // Turn off the micros() timer
  getTimer(TIMER).stop();
}

bool LowPowerNiclaVision::canDeepSleep() const
{
    return sleep_manager_can_deep_sleep();
}

LowPowerReturnCode LowPowerNiclaVision::checkOptionBytes() const
{
    FLASH_OBProgramInitTypeDef flashOBProgramInit{};

    flashOBProgramInit.Banks = FLASH_BANK_1;
    HAL_FLASHEx_OBGetConfig(&flashOBProgramInit);
    if (OB_STDBY_RST_D1 & flashOBProgramInit.USERConfig)
    {
        return LowPowerReturnCode::obNotPrepared;
    }
    if (OB_STDBY_RST_D2 & flashOBProgramInit.USERConfig)
    {
        return LowPowerReturnCode::obNotPrepared;
    }
    if (OB_BCM4_ENABLE & flashOBProgramInit.USERConfig)
    {
        return LowPowerReturnCode::obNotPrepared;
    }
    return LowPowerReturnCode::success;
}

// This function uses undocumented features of Mbed to retrieve the number
// of active deep sleep locks. It is experimental and may break at any time,
// but can be handy for some users to debug deep sleep lock problems.
// It uses features of the compiled machine code to find the number of locks.
uint16_t LowPowerNiclaVision::numberOfDeepSleepLocks() const
{
    // clang-format off
    return *((volatile uint16_t*) *((volatile uint32_t*) ((((volatile uint32_t)
            &sleep_manager_can_deep_sleep) & 0xfffffffe) + 0x10)));
    // clang-format on
}

LowPowerReturnCode LowPowerNiclaVision::prepareOptionBytes() const
{
    FLASH_OBProgramInitTypeDef flashOBProgramInit{};

    flashOBProgramInit.Banks        = FLASH_BANK_1;
    flashOBProgramInit.OptionType   = OPTIONBYTE_USER;
    flashOBProgramInit.USERType     = OB_USER_NRST_STDBY_D1 |
                                      OB_USER_NRST_STDBY_D2 |
                                      OB_USER_BCM4;
    flashOBProgramInit.USERConfig   = OB_STDBY_NO_RST_D1 |
                                      OB_STDBY_NO_RST_D2 |
                                      OB_BCM4_DISABLE;
    if (HAL_OK != HAL_FLASH_Unlock())
    {
        return LowPowerReturnCode::flashUnlockFailed;
    }
    if (HAL_OK != HAL_FLASH_OB_Unlock())
    {
        return LowPowerReturnCode::obUnlockFailed;
    }
    if (HAL_OK != HAL_FLASHEx_OBProgram(&flashOBProgramInit))
    {
        HAL_FLASH_OB_Lock();
        HAL_FLASH_Lock();
        return LowPowerReturnCode::obProgramFailed;
    }
    HAL_FLASH_OB_Launch();
    // The board should reset at this point, so anything beyond this point
    // is a failure
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
    return LowPowerReturnCode::obLaunchFailed;
}

void LowPowerNiclaVision::resetPreviousCPUModeFlags() const
{
    PWR->CPUCR |= PWR_CPUCR_CSSF;
}

LowPowerReturnCode LowPowerNiclaVision::standbyM4() const
{
    // Prevent Mbed from changing things
    core_util_critical_section_enter();

    waitForFlashReady();

    // Clear all but the reserved bits in these registers to mask out external
    // interrupts -->
    EXTI->C2IMR1 = 0;
    // Bit 13 of IMR is reserved and must always be 1
    EXTI->C2IMR2 = 1 << 13;
    // Bits 31:25, 19, and 18 of IMR are reserved and must be preserved
    EXTI->C2IMR3 &= ~0x1f5ffff;
    // <--

    // Set all but the reserved bits in these registers to clear pending
    // external interrupts -->
    // Bits 31:22 in PR1 are reserved and the original value must be preserved
    EXTI->C2PR1 |= 0x3fffff;
    // All bits except 17 and 19 in PR2 are reserved and the original value must
    // be preserved
    EXTI->C2PR2 |= ((1 << 17) | (1 << 19));
    // All bits except 18, 20, 21, and 22 in PR3 are reserved and the original
    // value must be preserved
    EXTI->C2PR3 |= ((1 << 18) | (1 << 20) | (1 << 21) | (1 << 22));
    // <--

    // Disable and clear all pending interrupts in the NVIC. There are 8
    // registers in the Cortex-M4.
    for (auto i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xffffffff;
        NVIC->ICPR[i] = 0xffffffff;
    }

    HAL_PWREx_EnterSTANDBYMode(PWR_D3_DOMAIN);

    HAL_PWREx_EnterSTANDBYMode(PWR_D2_DOMAIN);

    return LowPowerReturnCode::m4StandbyFailed;
}

LowPowerReturnCode LowPowerNiclaVision::standbyM7(RTCWakeupDelay delay) const
{
    const unsigned long long int wakeupDelay = delay.value;

    if ((wakeupDelay >= (2ULL << 17)) && (RTCWakeupDelay::infinite != wakeupDelay))
    {
        return LowPowerReturnCode::wakeupDelayTooLong;
    }

    // Prevent Mbed from changing things
    core_util_critical_section_enter();

    waitForFlashReady();

    // Make the D3 domain follow the CPU subsystem modes. This also applies to
    // Standby Mode according to the Reference Manual, even though the constant
    // is called PWR_D3_DOMAIN_STOP.
    HAL_PWREx_ConfigD3Domain(PWR_D3_DOMAIN_STOP);

    // Make sure that the voltage scaling isn't in VOS0 by setting it to VOS1.
    // While troubleshooting, change this to PWR_REGULATOR_VOLTAGE_SCALE3 to
    // better differentiate the states of the uC while measuring VCORE at the
    // VCAP pins.
    if (HAL_OK != HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1))
    {
        return LowPowerReturnCode::voltageScalingFailed;
    }

    // Clear all but the reserved bits in these registers to mask out external
    // interrupts -->
    EXTI->IMR1 = 0;
    // Bit 13 in IMR2 is reserved and must always be 1
    EXTI->IMR2 = 1 << 13;
    // Bits 31:25, 19, and 18 in IMR3 are reserved and must be preserved
    EXTI->IMR3 &= ~0x1f5ffff;
    // <--

    if (RTCWakeupDelay::infinite != wakeupDelay)
    {
        // Enable RTC wakeup in IMR
        HAL_EXTI_D1_EventInputConfig(EXTI_LINE19, EXTI_MODE_IT, ENABLE);
    }

    // Set all but the reserved bits in these registers to clear pending
    // external interrupts -->
    // Bits 31:22 in PR1 are reserved and the original value must be preserved
    EXTI->PR1 |= 0x3fffff;
    // All bits except 17 and 19 in PR2 are reserved and the original value must
    // be preserved
    EXTI->PR2 |= ((1 << 17) | (1 << 19));
    // All bits except 18, 20, 21, and 22 in PR3 are reserved and the original
    // value must be preserved
    EXTI->PR3 |= ((1 << 18) | (1 << 20) | (1 << 21) | (1 << 22));
    // <--

    if (RTCWakeupDelay::infinite != wakeupDelay)
    {
        RCC_OscInitTypeDef oscInit{};
        oscInit.OscillatorType = RCC_OSCILLATORTYPE_LSE;
        oscInit.LSEState = RCC_LSE_ON;
        if (HAL_OK != HAL_RCC_OscConfig(&oscInit))
        {
            return LowPowerReturnCode::enableLSEFailed;
        }

        RCC_PeriphCLKInitTypeDef periphClkInit{};
        periphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
        periphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
        if (HAL_OK != HAL_RCCEx_PeriphCLKConfig(&periphClkInit))
        {
            return LowPowerReturnCode::selectLSEFailed;
        }

        // This enables the RTC. It must not be called before the RTC input
        // clock source is selected above.
        __HAL_RCC_RTC_ENABLE();

        LL_RTC_DisableWriteProtection(RTC);

        // Enter init mode. We're doing this at the register level because of
        // a bug in the LL that ships with the current version of Mbed, where,
        // among other things, reserved bits are overwritten. Bit 7 is the INIT
        // bit.
        RTC->ISR |= 1 << 7;
        while (1U != LL_RTC_IsActiveFlag_INIT(RTC))
            ;

        LL_RTC_SetHourFormat(RTC, LL_RTC_HOURFORMAT_24HOUR);
        // LSE at 32767 Hz / (127+1) / (255 + 1) = 1 Hz for the RTC
        LL_RTC_SetAsynchPrescaler(RTC, 127);
        LL_RTC_SetSynchPrescaler(RTC, 255);

        // Exit init mode
        RTC->ISR &= ~(1 << 7);
        // This is probably not necessary, but included just in case
        while (0U != LL_RTC_IsActiveFlag_INIT(RTC))
            ;

        LL_RTC_DisableIT_WUT(RTC);
        LL_RTC_WAKEUP_Disable(RTC);
        while (1 != LL_RTC_IsActiveFlag_WUTW(RTC))
            ;

        if (wakeupDelay < (2ULL << 16)) {
            LL_RTC_WAKEUP_SetAutoReload(RTC, wakeupDelay);
            LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE);
        }
        else {
            LL_RTC_WAKEUP_SetAutoReload(RTC, wakeupDelay - (2ULL << 16));
            LL_RTC_WAKEUP_SetClock(RTC, LL_RTC_WAKEUPCLOCK_CKSPRE_WUT);
        }

        LL_RTC_WAKEUP_Enable(RTC);
        LL_RTC_EnableIT_WUT(RTC);
        __HAL_RTC_WAKEUPTIMER_EXTI_ENABLE_RISING_EDGE();
        LL_RTC_ClearFlag_WUT(RTC);

        LL_RTC_EnableWriteProtection(RTC);
    }

    // Set all but the reserved bits in these registers to clear pending
    // interrupts -->
    // Bits 31:22 in PR1 are reserved and the original value must be preserved
    EXTI->PR1 |= 0x3fffff;
    // All bits except 17 and 19 in PR2 are reserved and the original value must
    // be preserved
    EXTI->PR2 |= ((1 << 17) | (1 << 19));
    // All bits except 18, 20, 21, and 22 in PR3 are reserved and the original
    // value must be preserved
    EXTI->PR3 |= ((1 << 18) | (1 << 20) | (1 << 21) | (1 << 22));
    // <--

    // Disable and clear all pending interrupts in the NVIC. There are 8
    // registers in the Cortex-M7.
    for (auto i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xffffffff;
        NVIC->ICPR[i] = 0xffffffff;
    }

    if (RTCWakeupDelay::infinite != wakeupDelay)
    {
        HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0x0, 0);
        HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);
    }

    // When we reset the peripherals below, the OSCEN line will no longer enable
    // the MEMS oscillator for the HSE. This creates a race condition, where the
    // HSE sometimes stops before we enter Standby Mode, and sometimes keeps
    // going until Standby Mode is reached. If the HSE stops before Standby
    // Mode is reached, the STM32H747 goes into a frozen state where the SMPS
    // step-down converter never enters OPEN mode, the LDO voltage regulator
    // stays on, and NRST stops working. One solution to this is to enable the
    // Clock Security System (CSS), which makes the uC automatically switch over
    // to HSI when it detects an HSE failure. It also triggers an NMI, which
    // must be handled correctly.
    HAL_RCC_EnableCSS();

    // Reset peripherals to prepare for entry into Standby Mode
    __HAL_RCC_AHB3_FORCE_RESET();
    __HAL_RCC_AHB3_RELEASE_RESET();
    __HAL_RCC_AHB1_FORCE_RESET();
    __HAL_RCC_AHB1_RELEASE_RESET();
    __HAL_RCC_AHB2_FORCE_RESET();
    __HAL_RCC_AHB2_RELEASE_RESET();
    __HAL_RCC_APB3_FORCE_RESET();
    __HAL_RCC_APB3_RELEASE_RESET();
    __HAL_RCC_APB1L_FORCE_RESET();
    __HAL_RCC_APB1L_RELEASE_RESET();
    __HAL_RCC_APB1H_FORCE_RESET();
    __HAL_RCC_APB1H_RELEASE_RESET();
    __HAL_RCC_APB2_FORCE_RESET();
    __HAL_RCC_APB2_RELEASE_RESET();
    __HAL_RCC_APB4_FORCE_RESET();
    __HAL_RCC_APB4_RELEASE_RESET();
    __HAL_RCC_AHB4_FORCE_RESET();
    __HAL_RCC_AHB4_RELEASE_RESET();

    // Make sure that the M7 core takes the M4 core's state into account before
    // turning off the power to the flash memory. The normal way to do this
    // would be with __HAL_RCC_FLASH_C2_ALLOCATE(), but there's a bug in GCC
    // versions before 7.5.0, where the load instruction may sometimes be
    // discarded when a volatile variable is cast to void and there are extra
    // parentheses around the volatile variable's name. Extra parentheses should
    // not make a difference - see the C++14 standard, section 5, paragraph 11
    // - but we are currently running GCC 7.2.1. The data synchronization
    // barrier instruction below replaces the void-casted register read in
    // __HAL_RCC_FLASH_C2_ALLOCATE().
    RCC_C2->AHB3ENR |= RCC_AHB3ENR_FLASHEN;
    __DSB();

    // Clean the entire data cache if we're running on the M7 core. We must
    // make sure we're compiling for the CM7 core with conditional compilation,
    // or we won't get this through the first phase of template compilation.
#if defined CORE_CM7
    SCB_CleanDCache();
#endif

    HAL_PWREx_EnterSTANDBYMode(PWR_D1_DOMAIN);

    return LowPowerReturnCode::m7StandbyFailed;
}

uint64_t LowPowerNiclaVision::timeSinceBoot() const
{
    mbed_stats_cpu_t stats{};
    mbed_stats_cpu_get(&stats);
    return stats.uptime;
}

uint64_t LowPowerNiclaVision::timeSpentIdle() const
{
    mbed_stats_cpu_t stats{};
    mbed_stats_cpu_get(&stats);
    return stats.idle_time;
}

uint64_t LowPowerNiclaVision::timeSpentInDeepSleep() const
{
    mbed_stats_cpu_t stats{};
    mbed_stats_cpu_get(&stats);
    return stats.deep_sleep_time;
}

uint64_t LowPowerNiclaVision::timeSpentInSleep() const
{
    mbed_stats_cpu_t stats{};
    mbed_stats_cpu_get(&stats);
    return stats.sleep_time;
}

void LowPowerNiclaVision::waitForFlashReady() const
{
    // Make sure the flash controller isn't busy before we continue, since
    // that would block standby mode.
    //
    // 0x07 = QW, WBNE, and BSY flags
    while ((FLASH->SR1 & 0x07) || (FLASH->SR2 & 0x07))
      ;
}

bool LowPowerNiclaVision::wasInCPUMode(CPUMode mode) const
{
    switch (mode)
    {
        case CPUMode::d1DomainStandby:
            return PWR->CPUCR & PWR_CPUCR_SBF_D1;
        case CPUMode::d2DomainStandby:
            return PWR->CPUCR & PWR_CPUCR_SBF_D2;
        case CPUMode::standby:
            return PWR->CPUCR & PWR_CPUCR_SBF;
        case CPUMode::stop:
            return PWR->CPUCR & PWR_CPUCR_STOPF;
        default:
            return false;
    }
}
