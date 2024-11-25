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

/**
 * @enum LowPowerReturnCode
 * @brief Provides the return codes for the library API functions.
 * The codes indicate the success or failure of the operations.
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

/**
 * @enum CPUMode
 * @brief Provides the different modes of the CPU.
 * Those can be used to determine in which mode the CPU was before waking up.
*/
enum class CPUMode
{
    d1DomainStandby,        ///< Standby mode for the D1 domain
    d2DomainStandby,        ///< Standby mode for the D2 domain
    standby,                ///< Standby mode for the whole microcontroller
    stop                    ///< Stop mode for the whole microcontroller
};

/*
********************************************************************************
*                                 Classes
********************************************************************************
*/

/**
 * @brief The RTCWakeupDelay class represents a delay before waking up from Standby Mode.
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
        /**
        * @brief Private constructor to create a delay object with a specific delay value.
        * @param delay The delay value in seconds.
        */
        RTCWakeupDelay(const unsigned long long int delay) : value(delay)
        {
        }

        friend RTCWakeupDelay operator""_s(const unsigned long long int seconds);
        friend RTCWakeupDelay operator""_min(const unsigned long long int minutes);
        friend RTCWakeupDelay operator""_h(const unsigned long long int hours);
        friend RTCWakeupDelay operator+(const RTCWakeupDelay d1,
                                        const RTCWakeupDelay d2);

        friend class LowPowerNiclaVision;
};

/**
 * @class LowPowerNiclaVision
 * @brief A class that provides low power functionality for the Nicla Vision board.
 * 
 * The LowPowerPortentaH7 class allows the microcontroller on the Nicla Vision board
 * to enter low power modes such as Standby Mode and Deep Sleep Mode. It provides
 * functions to check the mode before start up, prepare the option bytes for entering Standby Mode,
 * and control the M4 and M7 cores independently. It also provides functions to measure
 * the time since boot, time spent in Idle, Sleep, and Deep Sleep modes.
 * 
 * This class is a singleton and shall always be accessed through the global LowPower object.
 * 
 * @note This class is specific to the Nicla Vision board.
 */
class LowPowerNiclaVision {
    private:
        LowPowerNiclaVision()    = default;
        ~LowPowerNiclaVision()   = default;

        void waitForFlashReady() const;

    public:
        /// @cond DEV
        /**
         * Returns the singleton instance of the LowPowerNiclaVision class.
         * Due to the way the low power modes are configured, only one instance
         * of this class can exist at a time.
         *
         * @return The singleton instance of the LowPowerNiclaVision class.
         */
        static LowPowerNiclaVision& getInstance() noexcept {
            static LowPowerNiclaVision instance;
            return instance;
        }
        LowPowerNiclaVision(const LowPowerNiclaVision&)               = delete;
        LowPowerNiclaVision(LowPowerNiclaVision&&)                    = delete;
        LowPowerNiclaVision& operator=(const LowPowerNiclaVision&)    = delete;
        LowPowerNiclaVision& operator=(LowPowerNiclaVision&&)         = delete;

        /// @endcond

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
        * @brief Reset the flags behind the wasInCPUMode() function.
        */
        void resetPreviousCPUModeFlags() const;
        /**
        * @brief Make the M4 core and domain D2 enter standby mode.
        * @return A constant from the LowPowerReturnCode enum.
        */
        LowPowerReturnCode standbyM4() const;
        // -->
        /**
        * @brief Make the M7 core and D2 domain enter standby mode, and make it possible for the D3 domain to do so.
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
        /**
         * Checks if the microcontroller was in the given CPU mode before starting.
         * Note: It's possible that the microcontroller was in more than one of these modes
         * before starting. Call this function multiple times to check for each mode.
         * Important: When you're done checking, call resetStandbyModeFlags() to reset the flags
         * so they are reported correctly the next time the microcontroller starts.
         * @param mode The CPU mode to check.
         * @return True if the microcontroller was in the given mode, false otherwise.
         */
        bool wasInCPUMode(CPUMode mode) const;
};

/*
********************************************************************************
*                                   Externs
********************************************************************************
*/

/**
 * @brief The global LowPower singleton object provides access to low power features of the Portenta H7 board.
 */
extern const LowPowerNiclaVision& LowPower;

/*
********************************************************************************
*                           Overloaded operators
********************************************************************************
*/

/**
 * @brief Literals operator to add multiple delays together. e.g. 5_s + 10_min + 2_h
 * @param d1 The first delay.
 * @param d2 The second delay.
 * @return The sum of the two delays.
*/
RTCWakeupDelay operator+(const RTCWakeupDelay d1, const RTCWakeupDelay d2);

/**
 * @brief Literals operator to create a delay in seconds.
 * @param seconds The number of seconds to wait before waking up.
 * @return The delay object.
*/
RTCWakeupDelay operator""_s(const unsigned long long int seconds);

/**
 * @brief Literals operator to create a delay in minutes.
 * @param minutes The number of minutes to wait before waking up.
 * @return The delay object.
*/
RTCWakeupDelay operator""_min(const unsigned long long int minutes);

/**
 * @brief Literals operator to create a delay in hours.
 * @param hours The number of hours to wait before waking up.
 * @return The delay object.
*/
RTCWakeupDelay operator""_h(const unsigned long long int hours);

#endif  // End of header guard
