# LowPowerButtons
Dual button low power Mysensors sensor.

Interrupt driven binary switch example

This example demonstrates several concepts that my be useful for switch sensors:

- Pin change interrupts: MySensors does not support pin change interrupts
  currently. This sketch initializes pin change interrupts and combines it with
  MySensors sleep functions. Some hacking is required to get out of the
  hwInternalSleep loop.
- Implements sleep for STM32 bluepill (stm32f103c8t6). This requires some
  modifications to the MySensors library (hal part) as can be found here:
  https://github.com/freynder/MySensors/tree/stm32f1_sleep
- Demonstrates how state machines can be leveraged to coordinate between states
  and events (sleep state, button states). Uses the excellent Automaton library:
  https://github.com/tinkerspy/Automaton/wiki

Tested with atmega328p standalone but should work with Arduino Nano/Pro MiniCore. Also tested with STM32 bluepill (stm32f103c8t6).

Uses RFM69 for transport.
