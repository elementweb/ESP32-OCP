using namespace std;

#pragma once

#include <driver/uart.h>
#include "definitions.h"
#include "dataManager.class.h"
#include "opticalInterface.class.h"

#define UART_PORT_BAUD        (115200)

class uartInterface {
  private: bool command_flag = false;
  private: bool new_data = false;

  private: char buffer_return_command[24] PROGMEM       =   "ATOP+COM=BUFFER.RETURN;";
  private: char buffer_length_command[24] PROGMEM       =   "ATOP+COM=BUFFER.LENGTH;";
  private: char buffer_available_command[27] PROGMEM    =   "ATOP+COM=BUFFER.REMAINING;";
  private: char buffer_size_command[22] PROGMEM         =   "ATOP+COM=BUFFER.SIZE;";
  private: char buffer_flush_command[23] PROGMEM        =   "ATOP+COM=BUFFER.CLEAR;";
  private: char buffer_transmit_command[26] PROGMEM     =   "ATOP+COM=BUFFER.TRANSMIT;";
  private: char buffer_stop_command[22] PROGMEM         =   "ATOP+COM=BUFFER.STOP;";

  private: char inc_buffer_pull_command[24] PROGMEM     =   "ATOP+COM=INCOMING.PULL;";
  private: char inc_buffer_length_command[26] PROGMEM   =   "ATOP+COM=INCOMING.LENGTH;";
  private: char inc_buffer_flush_command[25] PROGMEM    =   "ATOP+COM=INCOMING.CLEAR;";

  private: char system_reboot_command[27] PROGMEM       =   "ATOP+COM=REBOOT;";

  public: void initialize() {
    size_t UART_DEPTH = 4096;

    Serial2.begin(UART_PORT_BAUD);
    Serial2.setRxBufferSize(UART_DEPTH);

    delay(100);
  }

  public: void sendData(const char* data) {
    Serial2.write(data);
  }

  public: void processData(dataManager &dataManager) {
    bool data_available = false;

    // int available = Serial2.available();

    while(Serial2.available()) {
      dataManager.outgoingBufferPush(Serial2.read());
      delayMicroseconds(100);
      data_available = true;
      // if(available > dataManager.bufferRemaining()) {
      //   Serial.print(PROGMEM "buffer overflow");
      //   Serial2.flush();

      //   return;
      // }

      // for(i=0; i<available; i++) {
        // t = ;
      // }
    }

    if(data_available) {
      this->newDataAvailable(true);
      // dataManager.returnBuffer();

      dataManager.reportOutgoingBufferStats();

      // dataManager.updateFrontOutgoingBuffer();
      // Serial.println((char*) dataManager.frontBuffer);
    }
  }

  public: void newDataAvailable(bool condition) {
    this->new_data = condition;
  }

  public: bool isNewDataAvailable() {
    return this->new_data;
  }

  private: void returnSuccessOrFailMsg(HardwareSerial HS, bool condition) {
    if(condition) {
      HS.println(PROGMEM "OK");

      return;
    }

    HS.println(PROGMEM "FAIL");
  }

  public: void processCommands(dataManager &dataManager, opticalInterface &opticalInterface) {
    if(this->isNewDataAvailable()) {
      dataManager.updateFrontOutgoingBuffer();

      /** 
       * Return outgoing buffer
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->buffer_return_command)) {
        dataManager.removeFromOutgoingBuffer(loc, strlen(this->buffer_return_command));

        dataManager.returnOutgoingBuffer();
        Serial2.println();

        Serial.println(this->buffer_return_command);
      }

      /**
       * Return outgoing buffer length
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->buffer_length_command)) {
        dataManager.removeFromOutgoingBuffer(loc, strlen(this->buffer_length_command));

        print_uint64_t(Serial2, dataManager.outgoingBufferLength());
        Serial2.println();

        Serial.println(this->buffer_length_command);
      }

      /** 
       * Return remaining available outgoing buffer bytes
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->buffer_available_command)) {
        dataManager.removeFromOutgoingBuffer(loc, strlen(this->buffer_available_command));

        print_uint64_t(Serial2, dataManager.outgoingBufferRemaining());
        Serial2.println();

        Serial.println(this->buffer_available_command);
      }

      /**
       * Return total buffer size (this applies to both incoming and outgoing buffers as sizes are the same)
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->buffer_size_command)) {
        dataManager.removeFromOutgoingBuffer(loc, strlen(this->buffer_size_command));

        print_uint64_t(Serial2, dataManager.bufferSize());
        Serial2.println();

        Serial.println(this->buffer_size_command);
      }

      /**
       * Clear outgoing buffer
       */
      if(strstr((char*) dataManager.frontBuffer, this->buffer_flush_command)) {
        dataManager.outgoingBufferFlush();
        this->returnSuccessOrFailMsg(Serial2, true);

        Serial.println(this->buffer_flush_command);
      }

      /** 
       * Transmit data
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->buffer_transmit_command)) {
        dataManager.removeFromOutgoingBuffer(loc, strlen(this->buffer_transmit_command));

        this->returnSuccessOrFailMsg(Serial2, opticalInterface.startTransmission(dataManager));

        Serial.println(this->buffer_transmit_command);
      }

      /** 
       * Stop data transmission
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->buffer_stop_command)) {
        dataManager.removeFromOutgoingBuffer(loc, strlen(this->buffer_stop_command));

        this->returnSuccessOrFailMsg(Serial2, opticalInterface.stopTransmission());

        Serial.println(this->buffer_stop_command);
      }

      /** 
       * Pull the incoming buffer
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->inc_buffer_pull_command)) {
        dataManager.removeFromOutgoingBuffer(loc, strlen(this->inc_buffer_pull_command));

        dataManager.returnIncomingBuffer();
        Serial2.println();

        Serial.println(this->inc_buffer_pull_command);
      }

      /** 
       * Return incoming buffer length
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->inc_buffer_length_command)) {
        dataManager.removeFromOutgoingBuffer(loc, strlen(this->inc_buffer_length_command));

        print_uint64_t(Serial2, dataManager.incomingBufferLength());
        Serial2.println();

        Serial.println(this->inc_buffer_length_command);
      }

      /** 
       * Clear the incoming buffer
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->inc_buffer_flush_command)) {
        dataManager.removeFromOutgoingBuffer(loc, strlen(this->inc_buffer_flush_command));

        dataManager.incomingBufferFlush();
        this->returnSuccessOrFailMsg(Serial2, true);

        Serial.println(this->inc_buffer_flush_command);
      }

      /** 
       * Reboot
       */
      if(strstr((char*) dataManager.frontBuffer, this->system_reboot_command)) {
        Serial.println(PROGMEM "Rebooting...");
        this->returnSuccessOrFailMsg(Serial2, true);

        delay(100);

        ESP.restart();
      }

      /**
       * Set flag to false
       */
      this->newDataAvailable(false);
    }
  }
};
