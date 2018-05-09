#include "SleepyMachine.h"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

SleepyMachine& SleepyMachine::begin() {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*             ON_ENTER  ON_LOOP  ON_EXIT  EVT_SLEEP_CYCLE  EVT_ACTIVITY_DETECTED  EVT_WAKEUP_TIMEOUT  EVT_AWAKE_TIMEOUT  EVT_ALL_DONE  ELSE */
    /*  AWAKE */         -1,      -1,      -1,              -1,                    -1,                 -1,             SLEEP,        SLEEP,   -1,
    /*  SLEEP */  ENT_SLEEP,      -1,      -1,              -1,                    -1,                 -1,                -1,           -1,   WAKEUP,
    /* WAKEUP */ ENT_WAKEUP,      -1,      -1,           SLEEP,                 AWAKE,              SLEEP,                -1,           -1,   -1,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  wakeup_timer.set(WAKEUP_TIMEOUT);
  awake_timer.set(AWAKE_TIMEOUT);
  return *this;
}

/* Add C++ code for each internally handled event (input)
 * The code must return 1 to trigger the event
 */

int SleepyMachine::event( int id ) {
  switch ( id ) {
    case EVT_SLEEP_CYCLE:
      return 0;
    case EVT_WAKEUP_TIMEOUT:
      return wakeup_timer.expired(this);
    case EVT_AWAKE_TIMEOUT:
      return awake_timer.expired(this);
    case EVT_ALL_DONE:
      return 0;
  }
  return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 *
 * Available connectors:
 *   push( connectors, ON_READYFORSLEEP, 0, <v>, <up> );
 *   push( connectors, ON_WAKEUP, 0, <v>, <up> );
 */

void SleepyMachine::action( int id ) {
  switch ( id ) {
    case ENT_SLEEP:
      push(connectors, ON_READYFORSLEEP, 0, 0, 0);
      return;
    case ENT_WAKEUP:
      push(connectors, ON_WAKEUP, 0, 0, 0);
      return;
  }
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

SleepyMachine& SleepyMachine::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int SleepyMachine::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line
 ************************************************************************************************
*/

/* Public event methods
 *
 */

SleepyMachine& SleepyMachine::sleep_cycle() {
  trigger( EVT_SLEEP_CYCLE );
  return *this;
}

SleepyMachine& SleepyMachine::activity_detected() {
  trigger( EVT_ACTIVITY_DETECTED );
  return *this;
}

SleepyMachine& SleepyMachine::all_done() {
  trigger( EVT_ALL_DONE );
  return *this;
}

/*
 * onReadyforsleep() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

SleepyMachine& SleepyMachine::onReadyforsleep( Machine& machine, int event ) {
  onPush( connectors, ON_READYFORSLEEP, 0, 1, 1, machine, event );
  return *this;
}

SleepyMachine& SleepyMachine::onReadyforsleep( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_READYFORSLEEP, 0, 1, 1, callback, idx );
  return *this;
}

/*
 * onWakeup() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

SleepyMachine& SleepyMachine::onWakeup( Machine& machine, int event ) {
  onPush( connectors, ON_WAKEUP, 0, 1, 1, machine, event );
  return *this;
}

SleepyMachine& SleepyMachine::onWakeup( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_WAKEUP, 0, 1, 1, callback, idx );
  return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

SleepyMachine& SleepyMachine::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "SLEEPYMACHINE\0EVT_SLEEP_CYCLE\0EVT_ACTIVITY_DETECTED\0EVT_WAKEUP_TIMEOUT\0EVT_AWAKE_TIMEOUT\0EVT_ALL_DONE\0ELSE\0AWAKE\0SLEEP\0WAKEUP" );
  return *this;
}
