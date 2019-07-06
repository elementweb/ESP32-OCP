using namespace std;

#pragma once

#include "definitions.h"
#include <driver/uart.h>
#include "dataManager.class.h"

#define UART_BUFFER_DEPTH     (_1KB * 4)
#define UART_PORT_NUM         (2)
#define UART_PORT_BAUD        (115200)

class uartInterface {
  private: uart_t* interface;
  private: bool command_flag = false;

  public: void initialize() {
    this->interface = uartBegin(2, UART_PORT_BAUD, SERIAL_8N1, GPIO_NUM_16, GPIO_NUM_17, 10, false);
    
    uartResizeRxBuffer(this->interface, UART_BUFFER_DEPTH);
  }

  public: void sendData(const char* data) {
    uartWriteBuf(this->interface, (const uint8_t *)data, strlen(data));
  }

  public: void processData(dataManager &dataManager) {
    int i;
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

      dataManager.newDataAvailable(true);
    }
  }

  public: void processCommands(dataManager &dataManager) {
    if(dataManager.isNewDataAvailable()) {
      /**
       * Return buffer length
       */
      char buffer_length_command[] = "ATOP+COM=BUFFER.LENGTH;";
      if(strstr(dataManager.buffer(), "ATOP+COM=BUFFER.LENGTH;")) {
        dataManager.removeFromBuffer((char *)"ATOP+COM=BUFFER.LENGTH;");
        
        this->sendData((char *)dataManager.bufferLength());

        
      }

      /** 
       * Return remaining available buffer bytes
       */
      if(strstr(dataManager.buffer(), (char*)"ATOP+COM=BUFFER.REMAINING;")) {
        Serial.println("contains a command");
      }

      /**
       * Return total buffer size
       */
      if(strstr(dataManager.buffer(), (char*)"ATOP+COM=BUFFER.SIZE;")) {
        Serial.println("contains a command");
      }

      /**
       * Clear buffer
       */
      if(strstr(dataManager.buffer(), (char*)"ATOP+COM=BUFFER.CLEAR();")) {
        Serial.println("contains a command");
      }

      /** 
       * Transmit data
       */
      if(strstr(dataManager.buffer(), (char*)"ATOP+COM=BUFFER.TRANSMIT();")) {
        Serial.println("contains a command");
      }

      dataManager.newDataAvailable(false);
    }
  }
};
