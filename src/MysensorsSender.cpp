#include "MysensorsSender.h"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

MysensorsSender& MysensorsSender::begin() {
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*                   ON_ENTER  ON_LOOP  ON_EXIT  EVT_SEND  EVT_RETRY  EVT_SUCCESS  EVT_FAILURE  ELSE */
    /*      IDLE */      ENT_IDLE,      -1,      -1, TRY_SEND,        -1,          -1,          -1,   -1,
    /*  TRY_SEND */  ENT_TRY_SEND,      -1,      -1,       -1,  TRY_SEND,   COMPLETED,      FAILED,   -1,
    /*    FAILED */    ENT_FAILED,      -1,      -1,       -1,        -1,          -1,          -1, IDLE,
    /* COMPLETED */ ENT_COMPLETED,      -1,      -1,       -1,        -1,          -1,          -1, IDLE,
  };
  // clang-format on
  Machine::begin( state_table, ELSE );
  retryTimer.set(ACK_WAIT_TIME);
  return *this;
}

/* Add C++ code for each internally handled event (input)
 * The code must return 1 to trigger the event
 */

int MysensorsSender::event( int id ) {
  switch ( id ) {
    case EVT_RETRY:
      return retryTimer.expired(this) && !retryCounter.expired();
    case EVT_FAILURE:
      return retryTimer.expired(this) && retryCounter.expired();
  }
  return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 *
 * Available connectors:
 *   push( connectors, ON_FAILURE, 0, <v>, <up> );
 *   push( connectors, ON_SUCCESS, 0, <v>, <up> );
 */

void MysensorsSender::action( int id ) {
  switch ( id ) {
    case ENT_IDLE:
      retryCounter.set(SEND_RETRIES);
      return;
    case ENT_TRY_SEND:
      retryCounter.decrement();
      send(myMsg, true); // Send message and request ack
      return;
    case ENT_FAILED:
      push(connectors, ON_FAILURE, 0, 0, 0);
      return;
    case ENT_COMPLETED:
      push(connectors, ON_SUCCESS, 0, 0, 0);
      return;
   }
 }

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

MysensorsSender& MysensorsSender::trigger( int event ) {
  Machine::trigger( event );
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int MysensorsSender::state( void ) {
  return Machine::state();
}

/* Nothing customizable below this line
 ************************************************************************************************
*/

/* Public event methods
 *
 */

 MysensorsSender& MysensorsSender::doSend(MyMessage& msg) {
   myMsg = msg;
   trigger(EVT_SEND);
   return *this;
 }

 MysensorsSender& MysensorsSender::ackReceived(const MyMessage& ack) {
   if(myMsg.sensor == ack.sensor) {
     trigger(EVT_SUCCESS);
   }
   return *this;
 }

/*
 * onFailure() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

MysensorsSender& MysensorsSender::onFailure( Machine& machine, int event ) {
  onPush( connectors, ON_FAILURE, 0, 1, 1, machine, event );
  return *this;
}

MysensorsSender& MysensorsSender::onFailure( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_FAILURE, 0, 1, 1, callback, idx );
  return *this;
}

/*
 * onSuccess() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

MysensorsSender& MysensorsSender::onSuccess( Machine& machine, int event ) {
  onPush( connectors, ON_SUCCESS, 0, 1, 1, machine, event );
  return *this;
}

MysensorsSender& MysensorsSender::onSuccess( atm_cb_push_t callback, int idx ) {
  onPush( connectors, ON_SUCCESS, 0, 1, 1, callback, idx );
  return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

MysensorsSender& MysensorsSender::trace( Stream & stream ) {
  Machine::setTrace( &stream, atm_serial_debug::trace,
    "MYSENSORSSENDER\0EVT_SEND\0EVT_RETRY\0EVT_SUCCESS\0EVT_FAILURE\0ELSE\0IDLE\0TRY_SEND\0FAILED\0COMPLETED" );
  return *this;
}
