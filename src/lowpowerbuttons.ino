/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
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
 * Interrupt driven binary switch example with dual pin change interrupts
 * Author: Francis Reynders
 * MySensors does not support pin change interrupts currently. This sketch
 * initializes pin change interrupts and combines it with MySensors sleep
 * functions. Some hacking is required to get out of the hwInternalSleep loop.
 * Tested with atmega328p standalone but should work with Arduino Nano/Pro MiniCore
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

// RFM69
#define MY_RADIO_RFM69
#define MY_RFM69_NEW_DRIVER   // ATC on RFM69 works only with the new driver (not compatible with old=default driver)
#define MY_RFM69_ATC_TARGET_RSSI_DBM (-70)  // target RSSI -70dBm
#define MY_RFM69_MAX_POWER_LEVEL_DBM (10)   // max. TX power 10dBm = 10mW
#define MY_RFM69_ENABLE_ENCRYPTION
#define MY_RFM69_FREQUENCY RFM69_433MHZ
#ifdef ARDUINO_ARCH_AVR
  #define MY_RF69_IRQ_PIN 2
  #define MY_RF69_IRQ_NUM 0
  #define MY_RF69_SPI_CS 10
  #define PRIMARY_BUTTON_PIN      (3)
  #define SECONDARY_BUTTON_PIN    (4)
#endif

#ifdef ARDUINO_ARCH_STM32F1
  // Workaround for STM32 support
  #define ADC_CR2_TSVREFE  (1 << 23) // from libopencm3
  #define digitalPinToInterrupt(x) (x)
  // HW version of RFM69
  #define MY_IS_RFM69HW

  #define PRIMARY_BUTTON_PIN      (PA2)
  #define SECONDARY_BUTTON_PIN    (PA1)
#endif

//#define MY_NODE_ID 4

#include <MySensors.h>

#define SKETCH_NAME "Binary Sensor"
#define SKETCH_MAJOR_VER "2"
#define SKETCH_MINOR_VER "0"

#define CHILD_ID_BUTTON_1       (0)
#define CHILD_ID_BUTTON_2       (1)

// ID of the sensor child
#define CHILD_ID_UPLINK_QUALITY (2)
#define CHILD_ID_TX_LEVEL       (3)
#define CHILD_ID_TX_PERCENT     (4)
#define CHILD_ID_TX_RSSI        (5)
#define CHILD_ID_RX_RSSI        (7)
#define CHILD_ID_TX_SNR         (8)
#define CHILD_ID_RX_SNR         (9)



#define DEBOUNCE_INTERVAL 100
#define DEBOUNCE_COUNT_THRESHOLD 15 // required consecutive positive readings
#define PREVENT_DOUBLE_INTERVAL 400

#define SLEEP_TIME (6 * 60 * 60 * 1000ul) // Check battery every 6 hours

#define BATTERY_MAX_MVOLT 2900
#define BATTERY_MIN_MVOLT 2300

// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage msg(CHILD_ID_BUTTON_1, V_LIGHT);
MyMessage msg2(CHILD_ID_BUTTON_2, V_LIGHT);

// Initialize general message
MyMessage msgTxRSSI(CHILD_ID_TX_RSSI, V_CUSTOM);
MyMessage msgRxRSSI(CHILD_ID_RX_RSSI, V_CUSTOM);
MyMessage msgTxSNR(CHILD_ID_TX_SNR, V_CUSTOM);
MyMessage msgRxSNR(CHILD_ID_RX_SNR, V_CUSTOM);
MyMessage msgTxLevel(CHILD_ID_TX_LEVEL, V_CUSTOM);
MyMessage msgTxPercent(CHILD_ID_TX_PERCENT, V_CUSTOM);
MyMessage msgUplinkQuality(CHILD_ID_UPLINK_QUALITY, V_CUSTOM);

bool triggered = false;
uint32_t lastWakeup = 0;
uint16_t lastBatteryVoltage = 0u;

enum wakeup_t {
  WAKE_BY_TIMER,
  WAKE_BY_PCINT0,
  WAKE_BY_PCINT1,
  WAKE_BY_PCINT2,
  UNDEFINED
};

volatile wakeup_t wakeupReason = UNDEFINED;

#ifdef ARDUINO_ARCH_AVR

// Pin change interrupt service routines
ISR (PCINT0_vect) // handle pin change interrupt for PCINT[7:0]
{
  wakeupReason = WAKE_BY_PCINT0;
  _wokeUpByInterrupt = 0xFE; // Dirty hack to get out of MySensors sleep loop
}

ISR (PCINT1_vect) // handle pin change interrupt for PCINT[14:8]
{
  wakeupReason = WAKE_BY_PCINT1;
  _wokeUpByInterrupt = 0xFE; // Dirty hack to get out of MySensors sleep loop
}

ISR (PCINT2_vect) // handle pin change interrupt for PCINT[23:16]
{
  wakeupReason = WAKE_BY_PCINT2;
  _wokeUpByInterrupt = 0xFE; // Dirty hack to get out of MySensors sleep loop
}

void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

#endif


void setup()
{
  CORE_DEBUG(PSTR("Started\n"));

  pinMode(PRIMARY_BUTTON_PIN, INPUT);           // set pin to input
  digitalWrite(PRIMARY_BUTTON_PIN, INPUT_PULLUP);       // turn on pullup resistors

  pinMode(SECONDARY_BUTTON_PIN, INPUT);           // set pin to input
  digitalWrite(SECONDARY_BUTTON_PIN, INPUT_PULLUP);       // turn on pullup resistors

#ifdef ARDUINO_ARCH_AVR
  // Set up Pin change interrupt
  pciSetup(PRIMARY_BUTTON_PIN);
  pciSetup(SECONDARY_BUTTON_PIN);
#endif
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

	// Register binary input sensor to sensor_node (they will be created as child devices)
	// You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage.
	// If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
	present(CHILD_ID_BUTTON_1, S_LIGHT, "BUTTON 1");
	present(CHILD_ID_BUTTON_2, S_LIGHT, "BUTTON 2");
  // Register all sensors to gw (they will be created as child devices)
  present(CHILD_ID_UPLINK_QUALITY, S_CUSTOM, "UPLINK QUALITY RSSI");
  present(CHILD_ID_TX_LEVEL, S_CUSTOM, "TX LEVEL DBM");
  present(CHILD_ID_TX_PERCENT, S_CUSTOM, "TX LEVEL PERCENT");
  present(CHILD_ID_TX_RSSI, S_CUSTOM, "TX RSSI");
  present(CHILD_ID_RX_RSSI, S_CUSTOM, "RX RSSI");
  present(CHILD_ID_TX_SNR, S_CUSTOM, "TX SNR");
  present(CHILD_ID_RX_SNR, S_CUSTOM, "RX SNR");
}

void handleButtons()
{
  static uint8_t button1Count;
  static uint8_t button2Count;
  static uint32_t started, ended, delta;

  CORE_DEBUG(PSTR("Detecting buttons START\n"));

  button1Count = 0;
  button2Count = 0;

  // Try and detect which key during max DEBOUNCE_INTERVAL
  started = millis();
  while(millis() - started < DEBOUNCE_INTERVAL) {
    if(digitalRead(PRIMARY_BUTTON_PIN) == LOW) {
      button1Count++;
    } else {
      button1Count=0;
    }
    if(digitalRead(SECONDARY_BUTTON_PIN) == LOW) {
      button2Count++;
    } else {
      button2Count=0;
    }
    if(button1Count > DEBOUNCE_COUNT_THRESHOLD) {
      CORE_DEBUG(PSTR("Button 1 pressed\n"));
      send(msg.set((uint8_t) 1));
      break;
    }
    if(button2Count > DEBOUNCE_COUNT_THRESHOLD) {
      CORE_DEBUG(PSTR("Button 2 pressed\n"));
      send(msg2.set((uint8_t) 1));
      break;
    }
  }
  CORE_DEBUG(PSTR("Detecting buttons END\n"));

  // This section prevents detecting additional bounces
  ended = millis();
  if(ended > started) {
    delta = ended - started;
    if(delta < PREVENT_DOUBLE_INTERVAL) {
      CORE_DEBUG(PSTR("Waiting: %d \n"), PREVENT_DOUBLE_INTERVAL - delta);
      wait(PREVENT_DOUBLE_INTERVAL - delta); // In case the signal still is not stable after detection
    }
  }
}

void handleBatteryLevel()
{
  static uint16_t voltage;
  static uint8_t batteryPct;

  CORE_DEBUG(PSTR("Checking Battery BEGIN\n"));
  //voltage  = hwCPUVoltage();
  voltage = 0;
  CORE_DEBUG(PSTR("Voltage: %d\n"), voltage);

  // Process change in battery level
  if(lastBatteryVoltage == 0 || lastBatteryVoltage != voltage) {
    lastBatteryVoltage = voltage;
    if(voltage < BATTERY_MIN_MVOLT) {
      batteryPct = 0;
    } else {
      batteryPct = 100 * (voltage - BATTERY_MIN_MVOLT) / (BATTERY_MAX_MVOLT - BATTERY_MIN_MVOLT);
    }
    sendBatteryLevel(batteryPct);
    // send messages to GW
  	send(msgUplinkQuality.set(transportGetSignalReport(SR_UPLINK_QUALITY)));
  	send(msgTxLevel.set(transportGetSignalReport(SR_TX_POWER_LEVEL)));
  	send(msgTxPercent.set(transportGetSignalReport(SR_TX_POWER_PERCENT)));
  	// retrieve RSSI / SNR reports from incoming ACK
  	send(msgTxRSSI.set(transportGetSignalReport(SR_TX_RSSI)));
  	send(msgRxRSSI.set(transportGetSignalReport(SR_RX_RSSI)));
  	send(msgTxSNR.set(transportGetSignalReport(SR_TX_SNR)));
  	send(msgRxSNR.set(transportGetSignalReport(SR_RX_SNR)));
  } else {
    CORE_DEBUG(PSTR("No Change\n"));
  }

  CORE_DEBUG(PSTR("Checking Battery END\n"));
}

void loop()
{
#ifdef ARDUINO_ARCH_AVR
  // Unset value from dirty hack to get out of sleep loop (set in interrupt)
  _wokeUpByInterrupt = INVALID_INTERRUPT_NUM;
#endif

  CORE_DEBUG(PSTR("Woken up\n"));
  if(wakeupReason == WAKE_BY_PCINT2) {
    wakeupReason = UNDEFINED;
    handleButtons();
  }
  //handleBatteryLevel();

  CORE_DEBUG(PSTR("Going to sleep...\n"));
  sleep(SLEEP_TIME);
}
