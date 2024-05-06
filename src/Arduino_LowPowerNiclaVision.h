/**
* @file
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

#ifndef LowPowerNiclaVision_H
#define LowPowerNiclaVision_H

/*
********************************************************************************
*                           Included header files
********************************************************************************
*/

#include <mbed.h>
#include <usb_phy_api.h>
#include <limits>

/*
********************************************************************************
*                   Enumerations to be exposed to the sketch
********************************************************************************
*/

enum class LowPowerReturnCode
{
    success,                    ///< The call was successful
    flashUnlockFailed,          ///< Unable to unlock flash to set option bytes
    obUnlockFailed,             ///< Unable to unlock option bytes before set
    obProgramFailed,            ///< Unable to program option bytes
    obLaunchFailed,             ///< Unable to reset board with new option bytes
    obNotPrepared,              ///< Option bytes not correct for Standby Mode
    m7StandbyFailed,            ///< M7 core unable to enter Standby Mode
    m4StandbyFailed,            ///< M4 core unable to enter Standby Mode
    wakeupDelayTooLong,         ///< RTC delay longer than supported by hardware
    enableLSEFailed,            ///< Unable to enable external 32 kHz oscillator
    selectLSEFailed,            ///< Unable to select external 32 kHz oscillator
    voltageScalingFailed,       ///< Unable to set appropriate voltage scaling
};

/*
********************************************************************************
*                                 Classes
********************************************************************************
*/

class RTCWakeupDelay {
    public:
        /**
        * @brief Create a delay object for the RTC wakeup.
        * @param hours Hours to wait before wakeup.
        * @param minutes Minutes to wait before wakeup.
        * @param seconds Seconds to wait before wakeup.
        */
        RTCWakeupDelay(const unsigned long long int hours,
                       const unsigned long long int minutes,
                       const unsigned long long int seconds) :
            value(hours * 60 * 60 + minutes * 60 + seconds)
        {
        }

    private:
        // To wait forever before waking up (used in combination with NRST)
        static const unsigned long long int infinite = ULONG_LONG_MAX;
        // We don't really need this large type, but we must use this specific
        // type for user-defined literals to work.
        unsigned long long int value;
        RTCWakeupDelay(const unsigned long long int delay) : value(delay)
        {
        }

        friend RTCWakeupDelay operator""_s(const unsigned long long int seconds);
        friend RTCWakeupDelay operator""_min(const unsigned long long int seconds);
        friend RTCWakeupDelay operator""_h(const unsigned long long int seconds);
        friend RTCWakeupDelay operator+(const RTCWakeupDelay d1,
                                        const RTCWakeupDelay d2);

        friend class LowPowerNiclaVision;
};

class LowPowerNiclaVision {
    private:
        LowPowerNiclaVision()    = default;
        ~LowPowerNiclaVision()   = default;

        void waitForFlashReady() const;

    public:
        static LowPowerNiclaVision& getInstance() noexcept {
            static LowPowerNiclaVision instance;
            return instance;
        }
        LowPowerNiclaVision(const LowPowerNiclaVision&)               = delete;
        LowPowerNiclaVision(LowPowerNiclaVision&&)                    = delete;
        LowPowerNiclaVision& operator=(const LowPowerNiclaVision&)    = delete;
        LowPowerNiclaVision& operator=(LowPowerNiclaVision&&)         = delete;

        /**
        * @brief Make Deep Sleep possible in the default case.
        */
        void allowDeepSleep() const;
        /**
        * @brief Check if Deep Sleep is possible or not at the moment.
        * @return Possible: true. Not possible: false.
        */
        bool canDeepSleep() const;
        /**
        * @brief Check if the option bytes are correct to enter Standby Mode.
        * @return A constant from the LowPowerReturnCode enum.
        */
        LowPowerReturnCode checkOptionBytes() const;
        /**
        * @brief Check if the D1 domain was in Standby Mode or not.
        * @return Was: true. Was not: false;
        */
        bool modeWasD1Standby() const;
        /**
        * @brief Check if the D2 domain was in Standby Mode or not.
        * @return Was: true. Was not: false;
        */
        bool modeWasD2Standby() const;
        /**
        * @brief Check if the whole microcontroller was in Standby Mode or not.
        * @return Was: true. Was not: false;
        */
        bool modeWasStandby() const;
        /**
        * @brief Check if the whole microcontroller was in Stop Mode or not.
        * @return Was: true. Was not: false;
        */
        bool modeWasStop() const;
        // -->
        // The deprecated attribute is used here because we only want this
        // warning to be shown if the user actually calls the function
        /**
        * @brief Check how many Deep Sleep locks are held at the moment.
        * @return The number held.
        */
        __attribute__ ((deprecated("The numberOfDeepSleepLocks() function"
            " is experimental and should not be used in production code"))) 
        uint16_t numberOfDeepSleepLocks() const;
        // <--
        /**
        * @brief Prepare the option bytes for entry into Standby Mode.
        * @return A constant from the LowPowerReturnCode enum.
        */
        LowPowerReturnCode prepareOptionBytes() const;
        /**
        * @brief Reset the flags behind the modeWas...() functions.
        */
        void resetPreviousMode() const;
        /**
        * @brief Make the M4 core enter Standby Mode.
        * @return A constant from the LowPowerReturnCode enum.
        */
        LowPowerReturnCode standbyM4() const;
        // -->
        /**
        * @brief Make the M7 core enter Standby Mode.
        * @param delay The delay before waking up again.
        * @return A constant from the LowPowerReturnCode enum.
        */
        LowPowerReturnCode standbyM7(RTCWakeupDelay delay
                                        = RTCWakeupDelay::infinite) const;
        // <--
        /**
        * @brief Time since the board was booted.
        * @return Number of microseconds.
        */
        uint64_t timeSinceBoot() const;
        /**
        * @brief Time spent in idle.
        * @return Number of microseconds.
        */
        uint64_t timeSpentIdle() const;
        /**
        * @brief Time spent in Sleep Mode.
        * @return Number of microseconds.
        */
        uint64_t timeSpentInSleep() const;
        /**
        * @brief Time spent in Deep Sleep Mode.
        * @return Number of microseconds.
        */
        uint64_t timeSpentInDeepSleep() const;
};

/*
********************************************************************************
*                                   Externs
********************************************************************************
*/

extern const LowPowerNiclaVision& LowPower;

/*
********************************************************************************
*                           Overloaded operators
********************************************************************************
*/

RTCWakeupDelay operator+(const RTCWakeupDelay d1, const RTCWakeupDelay d2);
RTCWakeupDelay operator""_s(const unsigned long long int seconds);
RTCWakeupDelay operator""_min(const unsigned long long int minutes);
RTCWakeupDelay operator""_h(const unsigned long long int minutes);

#endif  // End of header guard
