## 💻 Usage

Start by including the library header file:

```cpp
#include "Arduino_LowPowerNiclaVision.h"
```

What you do next depends on what you want to achieve. Using only one of Deep Sleep Mode and Standby Mode might be sufficient, or you might want to combine them. If you combine them, start by enabling Deep Sleep Mode, perform any necessary work, and let the board go into Standby Mode until you need to perform more work.

### Deep Sleep Mode

The only function necessary to enable automatic Deep Sleep Mode is `allowDeepSleep()`. You might also find the following functions helpful: `canDeepSleep()`, `timeSinceBoot()`, `timeSpentIdle()`, `timeSpentInSleep()`, and `timeSpentInDeepSleep()`. Another function that can be useful when debugging Deep Sleep Locks, but not for production code, is `numberOfDeepSleepLocks()`.

> [!CAUTION]
> The `numberOfDeepSleepLocks()` function should never be used in production code because it relies on undocumented functionality in Mbed.

### Standby Mode

To use Standby Mode, you need the following functions: `checkOptionBytes()`, `prepareOptionBytes()`, `standbyM7()`, and `standbyM4()`. The option byte functions are necessary to ensure that the flash option bytes in the microcontroller are correctly set for going into Standby Mode. 
Additionally, you can use `wasInCPUMode()` to check what the board was doing before it started:
By passing one of the modes `d1DomainStandby`, `d2DomainStandby`, `standby` or `stop` as parameter you can determine in which of these modes the CPU was before it started. It's possible that the CPU was in more than one of these modes so you need to check for each mode separately to get the complete picture.
Only if `wasInCPUMode(CPUMode::standby)` returns true, Standby Mode was entered correctly. The other functions can be handy for troubleshooting. Remember to clear the mode flags by calling `resetPreviousCPUModeFlags()` when you are done checking them.

The microcontroller has three power domains: one for the M7 core (D1), one for the M4 core (D2), and a separate third domain (D3) for some other functionality. `wasInCPUMode(CPUMode::d1DomainStandby)` returns true if D1 was in standby, but all three weren't at the same time, while `wasInCPUMode(CPUMode::d2DomainStandby)()` returns true if D2 was in standby, but all three weren't at the same time. Both functions can return true simultaneously if D1 and D2 were in standby mode while D3 was still awake. When all three domains are in standby mode simultaneously, the microcontroller as a whole enters Standby Mode automatically, and `wasInCPUMode(CPUMode::standby)` returns true when it wakes up again. The `wasInCPUMode(CPUMode::stop)` function should never return true but can be helpful for further troubleshooting if you encounter any issues with this library.

All configuration of Standby Mode is done when calling `standbyM7()`. It takes one or zero parameters, depending on the conditions you want to set for waking up. The single parameter's preferred format is `2_h + 30_min + 45_s`. You can use any combination of hours, minutes, and seconds. For example, `15_h`, or `1_h + 30_min`, or just `90_s`. If you first have to calculate the delay in your sketch, you can also pass something like this: `RTCWakeupDelay(1, 20, 30)`. The first number is hours, the second minutes, and the third seconds. But, the preferred option is to use _h, _min, and _s since that's more explicit.

To enter Standby Mode indefinitely, just call the function without any parameter at all. In this case, you have to pull NRST low (located at the P5 fin, and no external pull-up resistor is necessary) to wake up from Standby Mode. Notice that this - contrary to using a true wakeup pin on other boards - resets the microcontroller even if it is not in Standby Mode at the time. To prevent this, you can set an I/O pin to high or low as soon as your sketch starts running, and keep it in that state indefinitely. Connect the pin to an external pull-up or pull-down resistor to make it go to the opposite state when the microcontroller goes into Standby Mode, as the I/O pin itself will float (go into HiZ state) in Standby Mode. Then, you can design an external circuit that uses the I/O pin, plus resistor, to block the wakeup signal to the NRST fin.

> [!IMPORTANT]
> You must always upload a sketch to the M4 core, in which you call `standbyM4()`, even if you don't intend to use the M4 core. If you don't, the microcontroller won't enter Standby Mode as a whole, even if you call `standbyM7()`.

## 👀 Examples

- [Standby](../examples/Standby_Example): This example demonstrates how to enter Standby Mode for a few seconds and then wake up again. It's also possible to wake up early by pulling the NRST pin low.
- [AllowDeepSleep](../examples/AllowDeepSleep_Example): This example demonstrates how to enable Deep Sleep Mode.
- [DeepSleepLockDebug](../examples/DeepSleepLockDebug_Example): This example demonstrates how to debug Deep Sleep Lock problems.