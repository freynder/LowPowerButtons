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
 * Interrupt driven binary switch example with dual interrupts
 * Author: Patrick 'Anticimex' Fallberg
 * Connect one button or door/window reed switch between
 * digitial I/O pin 3 (BUTTON_PIN below) and GND and the other
 * one in similar fashion on digital I/O pin 2.
 * This example is designed to fit Arduino Nano/Pro Mini
 *
 */


// Enable debug prints to serial monitor
#define MY_DEBUG

// Enable and select radio type attached
//#define MY_RADIO_NRF24
#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RF69_433MHZ // Set your frequency here
#define MY_IS_RFM69HW // Omit if your RFM is not "H"
//#define MY_RFM69_NETWORKID 100  // Default is 100 in lib. Uncomment it and set your preferred network id if needed
#define RF69_IRQ_PIN PD2  // Default in lib is using D2 for common Atmel 328p (mini pro, nano, uno etc.). Uncomment it and set the pin you're using. Note for Atmel 328p, Mysensors, and regarding Arduino core implementation D2 or D3 are only available. But for advanced mcus like Atmel SAMD (Arduino Zero etc.), Esp8266 you will need to set this define for the corresponding pin used for IRQ
#define MY_RF69_IRQ_NUM 0 // Temporary define (will be removed in next radio driver revision). Needed if you want to change the IRQ pin your radio is connected. So, if your radio is connected to D3/INT1, value is 1 (INT1). For others mcu like Atmel SAMD, Esp8266, value is simply the same as your RF69_IRQ_PIN
#define RF69_SPI_CS PB2 // If using a different CS pin for the SPI bus

#define MY_NODE_ID 2

#include <MySensors.h>
#include <Bounce2.h>
#include "PinChangeInterrupt.h"


#define SKETCH_NAME "Binary Sensor"
#define SKETCH_MAJOR_VER "1"
#define SKETCH_MINOR_VER "0"

#define PRIMARY_CHILD_ID 3 // PD3
#define SECONDARY_CHILD_ID 4 // PD4

#define PRIMARY_BUTTON_PIN PD3   // Arduino Digital I/O pin for button/reed switch
#define SECONDARY_BUTTON_PIN PD4 // Arduino Digital I/O pin for button/reed switch

Bounce debouncer1 = Bounce();
Bounce debouncer2 = Bounce();

// Change to V_LIGHT if you use S_LIGHT in presentation below
MyMessage msg(PRIMARY_CHILD_ID, V_LIGHT);
MyMessage msg2(SECONDARY_CHILD_ID, V_LIGHT);

bool interrupted = false;

void wakeUp() {
    interrupted = true;
}

void setup()
{
  Serial.println("Started");

  pinMode(PRIMARY_BUTTON_PIN, INPUT);           // set pin to input
  digitalWrite(PRIMARY_BUTTON_PIN, INPUT_PULLUP);       // turn on pullup resistors

  pinMode(SECONDARY_BUTTON_PIN, INPUT);           // set pin to input
  digitalWrite(SECONDARY_BUTTON_PIN, INPUT_PULLUP);       // turn on pullup resistors


  // After setting up the button, setup debouncer
  debouncer1.attach(PRIMARY_BUTTON_PIN);
  debouncer1.interval(5);
  debouncer2.attach(SECONDARY_BUTTON_PIN);
  debouncer2.interval(5);

  // Attach the new PinChangeInterrupt and enable event function below
  attachPCINT(digitalPinToPCINT(PRIMARY_BUTTON_PIN), wakeUp, FALLING);
  attachPCINT(digitalPinToPCINT(SECONDARY_BUTTON_PIN), wakeUp, FALLING);
}

void goToSleep(){
	//Custom Function that puts ATMega and NRF24L01 into sleep mode.
	//This was necessary because PinChange Interrupts have been used.
	//RF69::powerDown();
	//Serial.flush();	// although there should be nothing in the Serial cue, Let serial prints finish (debug, log etc)
	//powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //power everything down, wake up only by (PinChange) interrupts

  #if defined(MY_SENSOR_NETWORK)
  	CORE_DEBUG(PSTR("MCO:SLP:TPD\n"));	// sleep, power down transport
  	transportPowerDown();
  #endif

	// sleep until ext interrupt triggered
	hwPowerDown(SLEEP_FOREVER);
}

void presentation()
{
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo(SKETCH_NAME, SKETCH_MAJOR_VER "." SKETCH_MINOR_VER);

	// Register binary input sensor to sensor_node (they will be created as child devices)
	// You can use S_DOOR, S_MOTION or S_LIGHT here depending on your usage.
	// If S_LIGHT is used, remember to update variable type you send in. See "msg" above.
	present(PRIMARY_CHILD_ID, S_LIGHT);
	present(SECONDARY_CHILD_ID, S_LIGHT);
}

// Loop will iterate on changes on the BUTTON_PINs
void loop()
{
	static uint8_t value;
	static uint8_t lastValue=2;
	static uint8_t lastValue2=2;

  if(interrupted) {
    Serial.println("Interrupted!");
    interrupted = false;
  }

  debouncer1.update();
  debouncer2.update();

	// Short delay to allow buttons to properly settle
	sleep(5);

  debouncer1.update();
  debouncer2.update();

	value = debouncer1.read();

	if (value != lastValue) {
    Serial.println("Change detected for button 1");
		// Value has changed from last transmission, send the updated value
    if(value == LOW) {
      send(msg.set(1));
    }
		lastValue = value;
	}

	value = debouncer2.read();

	if (value != lastValue2) {
    Serial.println("Change detected for button 2");
    // Value has changed from last transmission, send the updated value
    if(value == LOW) {
      send(msg2.set(1));
    }
		lastValue2 = value;
	}

  goToSleep();
}
