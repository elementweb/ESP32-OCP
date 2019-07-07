using namespace std;

#pragma once

#include <driver/uart.h>
#include "definitions.h"
#include "dataManager.class.h"
#include "opticalInterface.class.h"

#define UART_BUFFER_DEPTH     (_1KB * 4)
#define UART_PORT_NUM         (2)
#define UART_PORT_BAUD        (115200)

class uartInterface {
  private: uart_t* interface;
  private: bool command_flag = false;
  private: bool new_data = false;

  private: char buffer_return_command[26] PROGMEM =         "ATOP+COM=BUFFER.RETURN();";
  private: char buffer_length_command[24] PROGMEM =         "ATOP+COM=BUFFER.LENGTH;";
  private: char buffer_available_command[27] PROGMEM =      "ATOP+COM=BUFFER.REMAINING;";
  private: char buffer_size_command[22] PROGMEM =           "ATOP+COM=BUFFER.SIZE;";
  private: char buffer_flush_command[25] PROGMEM =          "ATOP+COM=BUFFER.CLEAR();";
  private: char buffer_transmit_command[28] PROGMEM =       "ATOP+COM=BUFFER.TRANSMIT();";
  private: char buffer_stop_command[24] PROGMEM =           "ATOP+COM=BUFFER.STOP();";

  public: void initialize() {
    this->interface = uartBegin(2, UART_PORT_BAUD, SERIAL_8N1, GPIO_NUM_16, GPIO_NUM_17, 10, false);
    
    uartResizeRxBuffer(this->interface, UART_BUFFER_DEPTH);
  }

  public: void sendData(const char* data) {
    uartWriteBuf(this->interface, (const uint8_t *)data, strlen(data));
  }

  public: void processData(dataManager &dataManager) {
    unsigned int i;
    char t;

    size_t available = uartAvailable(this->interface);

    if(available) {      
      if(available > dataManager.bufferRemainingReal()) {
        Serial.print(PROGMEM "buffer overflow");
        
        uartFlush(this->interface);

        return;
      }

      for(i=0; i<available; i++) {
        t = uartRead(this->interface);
        
        dataManager.bufferPush(t);
      }

      this->newDataAvailable(true);
    }
  }

  public: void newDataAvailable(bool condition) {
    this->new_data = condition;
  }

  public: bool isNewDataAvailable() {
    return this->new_data;
  }

  public: void processCommands(dataManager &dataManager, opticalInterface &opticalInterface) {
    char conversion_buffer[16];

    if(this->isNewDataAvailable()) {
      /** 
       * Return buffer
       */
      if(char *loc = strstr(dataManager.buffer(), this->buffer_return_command)) {
        dataManager.removeFromBuffer(loc, strlen(this->buffer_return_command));

        this->sendData(dataManager.buffer());
        this->sendData("\n");

        Serial.println(this->buffer_return_command);
      }

      /**
       * Return buffer length
       */
      if(char *loc = strstr(dataManager.buffer(), this->buffer_length_command)) {
        dataManager.removeFromBuffer(loc, strlen(this->buffer_length_command));

        char * buffer_length = itoa(dataManager.bufferLength(), conversion_buffer, 10);

        this->sendData(buffer_length);
        this->sendData("\n");

        // debug; TODO: remove
        Serial.println(opticalInterface.packetCount(dataManager));

        Serial.println(this->buffer_length_command);
      }

      /** 
       * Return remaining available buffer bytes
       */
      if(char *loc = strstr(dataManager.buffer(), this->buffer_available_command)) {
        dataManager.removeFromBuffer(loc, strlen(this->buffer_available_command));

        char * buffer_remaining = itoa(dataManager.bufferRemaining(), conversion_buffer, 10);

        this->sendData(buffer_remaining);
        this->sendData("\n");

        Serial.println(this->buffer_available_command);
      }

      /**
       * Return total buffer size
       */
      if(char *loc = strstr(dataManager.buffer(), this->buffer_size_command)) {
        dataManager.removeFromBuffer(loc, strlen(this->buffer_size_command));

        char * buffer_size = itoa(dataManager.bufferSizeLimit(), conversion_buffer, 10);

        this->sendData(buffer_size);
        this->sendData("\n");

        Serial.println(this->buffer_size_command);
      }

      /**
       * Clear buffer
       */
      if(strstr(dataManager.buffer(), this->buffer_flush_command)) {
        dataManager.bufferFlush();

        Serial.println(this->buffer_flush_command);
      }

      /** 
       * Transmit data
       */
      if(char *loc = strstr(dataManager.buffer(), this->buffer_transmit_command)) {
        dataManager.removeFromBuffer(loc, strlen(this->buffer_transmit_command));

        opticalInterface.activate();

        Serial.println(this->buffer_transmit_command);
      }

      /** 
       * Stop data transmission
       */
      if(char *loc = strstr(dataManager.buffer(), this->buffer_stop_command)) {
        dataManager.removeFromBuffer(loc, strlen(this->buffer_stop_command));

        opticalInterface.deactivate();

        Serial.println(this->buffer_stop_command);
      }
      
      /**
       * Report free heap space
       */
      Serial.print("Heap: ");
      Serial.println(ESP.getFreeHeap());

      /**
       * Set flag to false
       */
      this->newDataAvailable(false);
    }
  }
};
