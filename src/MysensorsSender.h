#pragma once

#include <Automaton.h>
#include <core/MySensorsCore.h>

#define SEND_RETRIES 3
#define ACK_WAIT_TIME 1000  // 1 second

class MysensorsSender: public Machine {

 public:
  enum { IDLE, TRY_SEND, FAILED, COMPLETED }; // STATES
  enum { EVT_SEND, EVT_RETRY, EVT_SUCCESS, EVT_FAILURE, ELSE }; // EVENTS
  MysensorsSender( void ) : Machine() {};
  MysensorsSender& begin( void );
  MysensorsSender& trace( Stream & stream );
  MysensorsSender& trigger( int event );
  int state( void );
  MysensorsSender& onFailure( Machine& machine, int event = 0 );
  MysensorsSender& onFailure( atm_cb_push_t callback, int idx = 0 );
  MysensorsSender& onSuccess( Machine& machine, int event = 0 );
  MysensorsSender& onSuccess( atm_cb_push_t callback, int idx = 0 );
  MysensorsSender& doSend(MyMessage& msg);
  MysensorsSender& ackReceived(const MyMessage& ack);

 private:
  enum { ENT_IDLE, ENT_TRY_SEND, ENT_FAILED, ENT_COMPLETED }; // ACTIONS
  enum { ON_FAILURE, ON_SUCCESS, CONN_MAX }; // CONNECTORS
  atm_connector connectors[CONN_MAX];
  int event( int id );
  void action( int id );
  MyMessage myMsg;
  atm_counter retryCounter;
  atm_timer_millis retryTimer;
};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="MysensorsSender">
    <states>
      <IDLE index="0" on_enter="ENT_IDLE">
        <EVT_SEND>TRY_SEND</EVT_SEND>
      </IDLE>
      <TRY_SEND index="1" on_enter="ENT_TRY_SEND">
        <EVT_RETRY>TRY_SEND</EVT_RETRY>
        <EVT_SUCCESS>COMPLETED</EVT_SUCCESS>
        <EVT_FAILURE>FAILED</EVT_FAILURE>
      </TRY_SEND>
      <FAILED index="2" on_enter="ENT_FAILED">
        <ELSE>IDLE</ELSE>
      </FAILED>
      <COMPLETED index="3" on_enter="ENT_COMPLETED">
        <ELSE>IDLE</ELSE>
      </COMPLETED>
    </states>
    <events>
      <EVT_SEND index="0" access="PUBLIC"/>
      <EVT_RETRY index="1" access="PRIVATE"/>
      <EVT_SUCCESS index="2" access="PRIVATE"/>
      <EVT_FAILURE index="3" access="PRIVATE"/>
    </events>
    <connectors>
      <FAILURE autostore="0" broadcast="0" dir="PUSH" slots="1"/>
      <SUCCESS autostore="0" broadcast="0" dir="PUSH" slots="1"/>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/
