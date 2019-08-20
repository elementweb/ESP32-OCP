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
  private: char buffer_flush_command[25] PROGMEM        =   "ATOP+COM=BUFFER.CLEAR();";
  private: char buffer_transmit_command[28] PROGMEM     =   "ATOP+COM=BUFFER.TRANSMIT();";
  private: char buffer_stop_command[24] PROGMEM         =   "ATOP+COM=BUFFER.STOP();";
  private: char system_reboot_command[29] PROGMEM       =   "ATOP+COM=REBOOT();";

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
      dataManager.bufferPush(Serial2.read());
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

      dataManager.reportStats();

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

  private: void print_uint64_t(HardwareSerial HS, uint64_t num) {
    char rev[128]; 
    char *p = rev+1;

    while (num > 0) {
      *p++ = '0' + ( num % 10);
      num/= 10;
    }

    p--;

    while (p > rev) {
      HS.print(*p--);
    }
  }

  public: void processCommands(dataManager &dataManager, opticalInterface &opticalInterface) {
    if(this->isNewDataAvailable()) {
      dataManager.updateFrontOutgoingBuffer();

      /** 
       * Return buffer
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->buffer_return_command)) {
        dataManager.removeFromBuffer(loc, strlen(this->buffer_return_command));
        dataManager.returnBuffer();
        Serial2.println();

        Serial.println(this->buffer_return_command);
      }

      /**
       * Return buffer length
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->buffer_length_command)) {
        dataManager.removeFromBuffer(loc, strlen(this->buffer_length_command));

        if(uint64_t buffer_length = dataManager.bufferLength()) {
          this->print_uint64_t(Serial2, buffer_length);
        } else {
          Serial2.print("0");
        }

        Serial2.println();

        Serial.println(this->buffer_length_command);
      }

      /** 
       * Return remaining available buffer bytes
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->buffer_available_command)) {
        dataManager.removeFromBuffer(loc, strlen(this->buffer_available_command));
        this->print_uint64_t(Serial2, dataManager.bufferRemaining());
        Serial2.println();

        Serial.println(this->buffer_available_command);
      }

      /**
       * Return total buffer size
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->buffer_size_command)) {
        dataManager.removeFromBuffer(loc, strlen(this->buffer_size_command));
        this->print_uint64_t(Serial2, dataManager.bufferSize());
        Serial2.println();

        Serial.println(this->buffer_size_command);
      }

      /**
       * Clear buffer
       */
      if(strstr((char*) dataManager.frontBuffer, this->buffer_flush_command)) {
        dataManager.bufferFlush();
        Serial2.println("OK");

        Serial.println(this->buffer_flush_command);
      }

      /** 
       * Transmit data
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->buffer_transmit_command)) {
        dataManager.removeFromBuffer(loc, strlen(this->buffer_transmit_command));

        if(opticalInterface.startTransmission(dataManager)) {
          Serial2.println("OK");
        } else {
          Serial2.println("FAIL");
        }

        Serial.println(this->buffer_transmit_command);
      }

      /** 
       * Stop data transmission
       */
      if(char *loc = strstr((char*) dataManager.frontBuffer, this->buffer_stop_command)) {
        dataManager.removeFromBuffer(loc, strlen(this->buffer_stop_command));

        if(opticalInterface.stopTransmission()) {
          Serial2.println("OK");
        } else {
          Serial2.println("FAIL");
        }

        Serial.println(this->buffer_stop_command);
      }

      /** 
       * Reboot
       */
      if(strstr((char*) dataManager.frontBuffer, this->system_reboot_command)) {
        Serial.println(PROGMEM "Rebooting...");

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
