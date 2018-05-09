#pragma once

#include <Automaton.h>

#define WAKEUP_TIMEOUT (50)   // 50 milliseconds
#define AWAKE_TIMEOUT  (5000) // 5 seconds

class SleepyMachine: public Machine {

 public:
  enum { AWAKE, SLEEP, WAKEUP }; // STATES
  enum { EVT_SLEEP_CYCLE, EVT_ACTIVITY_DETECTED, EVT_WAKEUP_TIMEOUT, EVT_AWAKE_TIMEOUT, EVT_ALL_DONE, ELSE }; // EVENTS
  SleepyMachine( void ) : Machine() {};
  SleepyMachine& begin( void );
  SleepyMachine& trace( Stream & stream );
  SleepyMachine& trigger( int event );
  int state( void );
  SleepyMachine& onReadyforsleep( Machine& machine, int event = 0 );
  SleepyMachine& onReadyforsleep( atm_cb_push_t callback, int idx = 0 );
  SleepyMachine& onWakeup( Machine& machine, int event = 0 );
  SleepyMachine& onWakeup( atm_cb_push_t callback, int idx = 0 );
  SleepyMachine& sleep_cycle( void );
  SleepyMachine& activity_detected( void );
  SleepyMachine& all_done( void );

 private:
  enum { ENT_SLEEP, ENT_WAKEUP }; // ACTIONS
  enum { ON_READYFORSLEEP, ON_WAKEUP, CONN_MAX }; // CONNECTORS
  atm_connector connectors[CONN_MAX];
  atm_timer_millis wakeup_timer, awake_timer;
  int event( int id );
  void action( int id );

};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="SleepyMachine">
    <states>
      <AWAKE index="0">
        <EVT_AWAKE_TIMEOUT>SLEEP</EVT_AWAKE_TIMEOUT>
        <EVT_ALL_DONE>SLEEP</EVT_ALL_DONE>
      </AWAKE>
      <SLEEP index="1" on_enter="ENT_SLEEP">
        <EVT_WAKEUP>WAKEUP</EVT_WAKEUP>
      </SLEEP>
      <WAKEUP index="2" on_enter="ENT_WAKEUP">
        <EVT_SLEEP_CYCLE>SLEEP</EVT_SLEEP_CYCLE>
        <EVT_ACTIVITY_DETECTED>AWAKE</EVT_ACTIVITY_DETECTED>
        <EVT_WAKEUP_TIMEOUT>SLEEP</EVT_WAKEUP_TIMEOUT>
      </WAKEUP>
    </states>
    <events>
      <EVT_WAKEUP index="0" access="PRIVATE"/>
      <EVT_SLEEP_CYCLE index="1" access="MIXED"/>
      <EVT_ACTIVITY_DETECTED index="2" access="PUBLIC"/>
      <EVT_WAKEUP_TIMEOUT index="3" access="PRIVATE"/>
      <EVT_AWAKE_TIMEOUT index="4" access="PRIVATE"/>
      <EVT_ALL_DONE index="5" access="MIXED"/>
    </events>
    <connectors>
      <READYFORSLEEP autostore="0" broadcast="0" dir="PUSH" slots="1"/>
      <WAKEUP autostore="0" broadcast="0" dir="PUSH" slots="1"/>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/
