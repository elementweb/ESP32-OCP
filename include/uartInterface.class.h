using namespace std;

#pragma once

#include <driver/uart.h>
#include "definitions.h"
#include "dataManager.class.h"

#define UART_BUFFER_DEPTH     (_1KB * 4)
#define UART_PORT_NUM         (2)
#define UART_PORT_BAUD        (115200)

class uartInterface {
  private: uart_t* interface;
  private: bool command_flag = false;
  private: bool new_data = false;

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
        Serial.print("buffer overflow");
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

  public: void processCommands(dataManager &dataManager) {
    char conversion_buffer[16];

    if(this->isNewDataAvailable()) {
      /** 
       * Return buffer
       */
      char buffer_return_command[] = "ATOP+COM=BUFFER.RETURN();";
      if(char *loc = strstr(dataManager.buffer(), buffer_return_command)) {
        dataManager.removeFromBuffer(loc, strlen(buffer_return_command));

        this->sendData(dataManager.buffer());
        this->sendData("\n");

        Serial.println(buffer_return_command);
      }

      /**
       * Return buffer length
       */
      char buffer_length_command[] = "ATOP+COM=BUFFER.LENGTH;";
      if(char *loc = strstr(dataManager.buffer(), buffer_length_command)) {
        dataManager.removeFromBuffer(loc, strlen(buffer_length_command));

        char * buffer_length = itoa(dataManager.bufferLength(), conversion_buffer, 10);
        this->sendData(buffer_length);
        this->sendData("\n");

        Serial.println(buffer_length_command);
      }

      /** 
       * Return remaining available buffer bytes
       */
      char buffer_available_command[] = "ATOP+COM=BUFFER.REMAINING;";
      if(char *loc = strstr(dataManager.buffer(), buffer_available_command)) {
        dataManager.removeFromBuffer(loc, strlen(buffer_available_command));

        char * buffer_remaining = itoa(dataManager.bufferRemaining(), conversion_buffer, 10);
        this->sendData(buffer_remaining);
        this->sendData("\n");

        Serial.println(buffer_available_command);
      }

      /**
       * Return total buffer size
       */
      char buffer_size_command[] = "ATOP+COM=BUFFER.SIZE;";
      if(char *loc = strstr(dataManager.buffer(), buffer_size_command)) {
        dataManager.removeFromBuffer(loc, strlen(buffer_size_command));

        char * buffer_size = itoa(dataManager.bufferSizeLimit(), conversion_buffer, 10);
        this->sendData(buffer_size);
        this->sendData("\n");

        Serial.println(buffer_size_command);
      }

      /**
       * Clear buffer
       */
      char buffer_flush_command[] = "ATOP+COM=BUFFER.CLEAR();";
      if(strstr(dataManager.buffer(), buffer_flush_command)) {
        dataManager.bufferFlush();

        Serial.println(buffer_flush_command);
      }

      /** 
       * Transmit data
       */
      char buffer_transmit_command[] = "ATOP+COM=BUFFER.TRANSMIT();";
      if(char *loc = strstr(dataManager.buffer(), buffer_transmit_command)) {
        dataManager.removeFromBuffer(loc, strlen(buffer_transmit_command));

        // not implemented yet

        Serial.println(buffer_transmit_command);
      }

      /**
       * Set flag to false
       */
      this->newDataAvailable(false);
    }
  }
};
