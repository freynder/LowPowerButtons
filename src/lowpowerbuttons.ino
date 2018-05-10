/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2018 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * DESCRIPTION
 *
 * Interrupt driven binary switch example
 * Author: Francis Reynders
 * Keywords: mysensors sensor binary switch automaton state machine
 * This example demonstrates several concepts that my be useful for switch sensors:
 * - Pin change interrupts: MySensors does not support pin change interrupts
 *   currently. This sketch initializes pin change interrupts and combines it with
 *   MySensors sleep functions. Some hacking is required to get out of the
 *   hwInternalSleep loop.
 * - Implements sleep for STM32 bluepill (stm32f103c8t6). This requires some
 *   modifications to the MySensors library (hal part) as can be found here:
 *   https://github.com/freynder/MySensors/tree/stm32f1_sleep
 * - Demonstrates how state machines can be leveraged to coordinate between states
 *   and events (sleep state, button states). Uses the excellent Automaton library:
 *   https://github.com/tinkerspy/Automaton/wiki
 *
 * Tested with atmega328p standalone but should work with Arduino Nano/Pro
 * MiniCore. Also tested with STM32 bluepill (stm32f103c8t6).
 *
 * Uses RFM69 for transport.
 *
 * Based on original work by:
 * Author: Patrick 'Anticimex' Fallberg
 * Connect one button or door/window reed switch between
 * digitial I/O pin 3 (BUTTON_PIN below) and GND and the other
 * one in similar fashion on digital I/O pin 2.
 * This example is designed to fit Arduino Nano/Pro Mini
 *
 */

// Enable debug prints to serial monitor
#define MY_DEBUG

#define MY_NODE_ID						        (3)

// RFM69
#define MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER
//#define MY_RFM69_ATC_MODE_DISABLED
#define MY_RFM69_ATC_TARGET_RSSI_DBM  (-50)  // target RSSI -70dBm
#define MY_RFM69_MAX_POWER_LEVEL_DBM  (10)   // max. TX power 10dBm = 10mW
//#define MY_RFM69_TX_POWER_DBM (10)
// Lower bitrate to increase reliability
#define MY_RFM69_BITRATE_MSB (RFM69_BITRATEMSB_9600)
#define MY_RFM69_BITRATE_LSB (RFM69_BITRATELSB_9600)
#define MY_RFM69_ENABLE_ENCRYPTION
#define MY_RFM69_FREQUENCY            RFM69_433MHZ
//#define MY_IS_RFM69HW
#define MY_DEBUG_VERBOSE_RFM69

// Pins
#if defined(ARDUINO_ARCH_AVR)
  #define BUTTON_1_PIN                (3)
  #define BUTTON_2_PIN                (4)
#elif defined(ARDUINO_ARCH_STM32F1)
  #define MY_RFM69_IRQ_PIN            (PA3)
  #define MY_RF69_IRQ_NUM             MY_RFM69_IRQ_PIN
  //#define MY_RFM69_RST_PIN_CUST       (PB6)
  #define BUTTON_1_PIN                (PA1)
  #define BUTTON_2_PIN                (PA2)
#endif

#include <MySensors.h>
#include <Automaton.h>
#include "SleepyMachine.h"
#include "MysensorsSender.h"

#define SKETCH_NAME             "Binary Automaton Sensor"
#define SKETCH_MAJOR_VER        "4"
#define SKETCH_MINOR_VER        "0"

#define CHILD_ID_BUTTON_1       (0)
#define CHILD_ID_BUTTON_2       (1)

#define SLEEP_TIME              (60 * 60 * 1000ul) // 1 hour

#define BATTERY_MAX_MVOLT       (2900) // 2 Eneloop AAA batteries
#define BATTERY_MIN_MVOLT       (2300)
#define SEND_BATTERY_THRESHOLD  (10) // Send battery level every 10 actions

// State machines for automaton
SleepyMachine sleepyMachine;
Atm_button button1, button2;
// We define an individual sender per button to make it easier to
// run the confirmation routines concurrently. Each sender can processes
// 1 send/confirmation cycle at a time.
MysensorsSender button1Sender, button2Sender;

MyMessage msg1(CHILD_ID_BUTTON_1, V_TRIPPED);
MyMessage msg2(CHILD_ID_BUTTON_2, V_TRIPPED);

uint8_t batteryLevelCounter = SEND_BATTERY_THRESHOLD + 1;

enum wakeup_t {
  WAKE_BY_TIMER,
  WAKE_BY_PCINT0,
  WAKE_BY_PCINT1,
  WAKE_BY_PCINT2,
  UNDEFINED
};

volatile wakeup_t wakeupReason = UNDEFINED;
volatile bool activePCINT = false;

#if defined(ARDUINO_ARCH_AVR)

  // Pin change interrupt service routines
  ISR (PCINT0_vect) // handle pin change interrupt for PCINT[7:0]
  {
    if(activePCINT) {
      wakeupReason = WAKE_BY_PCINT0;
      _wokeUpByInterrupt = 0xFE; // Dirty hack to get out of MySensors sleep loop
    }
  }

  ISR (PCINT1_vect) // handle pin change interrupt for PCINT[14:8]
  {
    if(activePCINT) {
      wakeupReason = WAKE_BY_PCINT1;
      _wokeUpByInterrupt = 0xFE; // Dirty hack to get out of MySensors sleep loop
    }
  }

  ISR (PCINT2_vect) // handle pin change interrupt for PCINT[23:16]
  {
    if(activePCINT) {
      wakeupReason = WAKE_BY_PCINT2;
      _wokeUpByInterrupt = 0xFE; // Dirty hack to get out of MySensors sleep loop
    }
  }

  void pciSetup(byte pin)
  {
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
  }

#elif defined(ARDUINO_ARCH_STM32F1)

  // Mysensors callback
  void preHwInit() {
    #if defined(MY_RFM69_RST_PIN_CUST)
      // Reset RFM69
      pinMode(MY_RFM69_RST_PIN_CUST, OUTPUT);
      digitalWrite(MY_RFM69_RST_PIN_CUST, HIGH);
      delay(100);
      digitalWrite(MY_RFM69_RST_PIN_CUST, LOW);
      delay(100);
    #endif
    // Power saving
    adc_disable_all();
    setGPIOModeToAllPins(GPIO_INPUT_ANALOG);
  }

#endif

uint8_t getBatteryLevel()
{
  static uint16_t voltage;
  static uint8_t batteryPct;

  CORE_DEBUG(PSTR("Checking Battery BEGIN\n"));

  #if defined(ARDUINO_ARCH_STM32F1)
    adc_enable(ADC1);
  #endif

  voltage  = hwCPUVoltage();

  #if defined(ARDUINO_ARCH_STM32F1)
    adc_disable(ADC1);
  #endif

  CORE_DEBUG(PSTR("Voltage: %d\n"), voltage);

  if(voltage < BATTERY_MIN_MVOLT) {
    batteryPct = 0;
  } else {
    batteryPct = 100 * (voltage - BATTERY_MIN_MVOLT) / (BATTERY_MAX_MVOLT - BATTERY_MIN_MVOLT);
  }
  return batteryPct;
}

/*------------------------------------------------------------------------------
*
* Callbacks for state machines
*
*******************************************************************************/

void enterSleepCB( int idx, int v, int up ) {
  CORE_DEBUG(PSTR("Going to sleep...\n"));
  activePCINT = true;
  sleep(SLEEP_TIME);
  activePCINT = false;
}

void enterWakeupCB( int idx, int v, int up ) {
  CORE_DEBUG(PSTR("Woken up...\n"));
  #if defined(ARDUINO_ARCH_AVR)
    // AVR sleep function continously loops with 16ms sleep times to achieve
    // the total desired value. We interrupted this with a dirty hack by
    // setting _wokeUpByInterrupt = 0xFE in the PC interrupt handler. We reset
    // it back here so next sleep will not return immediately.
    _wokeUpByInterrupt = INVALID_INTERRUPT_NUM;
    //wakeupReason = UNDEFINED;
  #endif
}

void onPressCallback(int idx, int v, int up) {
  // We detected a button press, allow additional processing time before sleep
  sleepyMachine.trigger(sleepyMachine.EVT_ACTIVITY_DETECTED);
  // Send communications
  Serial.println("Sending!!");
  if(idx == 1) {
    button1Sender.doSend(msg1.set((uint8_t) 1));
  } else if(idx == 2) {
    button2Sender.doSend(msg2.set((uint8_t) 1));
  }
}

void onSuccessSendCB(int idx, int v, int up) {
  // Determine if we need to send battery level
  if(batteryLevelCounter++ > SEND_BATTERY_THRESHOLD) {
    batteryLevelCounter = 0u;
    sendBatteryLevel(getBatteryLevel());
  }
  sleepyMachine.trigger(sleepyMachine.EVT_ALL_DONE);
}

void setup()
{
  CORE_DEBUG(PSTR("Started --\n"));

  // interrupts
  #if defined(ARDUINO_ARCH_AVR)
    // Set up Pin change interrupt
    pciSetup(BUTTON_1_PIN);
    pciSetup(BUTTON_2_PIN);
  #elif defined(ARDUINO_ARCH_STM32F1)
    attachInterrupt(BUTTON_1_PIN, [] {}, FALLING); // Just wakeup
    attachInterrupt(BUTTON_2_PIN, [] {}, FALLING);
  #endif

  // automaton
  sleepyMachine.begin()
    .trace(Serial)
    .onReadyforsleep(enterSleepCB, 0)
    .onWakeup(enterWakeupCB, 0);
  button1.begin(BUTTON_1_PIN)
    .onPress(onPressCallback, 1);
  button2.begin(BUTTON_2_PIN)
    .onPress(onPressCallback, 2);
  button1Sender.begin()
    .onSuccess(onSuccessSendCB, 1)
    .onFailure(sleepyMachine, sleepyMachine.EVT_ALL_DONE)
    .trace(Serial);
  button2Sender.begin()
    .onSuccess(onSuccessSendCB, 2)
    .onFailure(sleepyMachine, sleepyMachine.EVT_ALL_DONE)
    .trace(Serial);
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

	// Register binary input sensor to sensor_node (they will be created as child devices)
	// You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage.
	// If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
  // Last parameter is request for ACK
  present(CHILD_ID_BUTTON_1, S_MOTION, "BUTTON 1");
	present(CHILD_ID_BUTTON_2, S_MOTION, "BUTTON 2");
}

void loop()
{
  // The Automaton framework runs within the mysensors framework and takes
  // care of all states and transitions.
  automaton.run();
}

// MySensors callback for processing ack messages
void receive(const MyMessage &message) {
  if (message.isAck()){
    button1Sender.ackReceived(message);
    button2Sender.ackReceived(message);
  }
}
