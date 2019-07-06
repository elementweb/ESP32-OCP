using namespace std;

#pragma once

#include <string.h>
#include "definitions.h"

#define BUFFER_SIZE         _64KBw64
#define BUFFER_SIZE_LIMIT   _64KB

class dataManager {
  private: char _buffer[BUFFER_SIZE];
  private: bool new_data = false;

  public: size_t bufferLength() {
    return strlen(this->_buffer);
  }

  public: size_t bufferSize() {
    return BUFFER_SIZE;
  }

  public: size_t bufferSizeLimit() {
    return BUFFER_SIZE_LIMIT;
  }

  public: size_t bufferRemaining() {
    return BUFFER_SIZE_LIMIT - this->bufferLength();
  }

  public: size_t bufferRemainingReal() {
    return BUFFER_SIZE - this->bufferLength();
  }

  public: void bufferFlush() {
    int i;
    for(i=0; i<BUFFER_SIZE; i++) {
      this->_buffer[i] = 0x00;
    }
  }

  public: char * buffer() {
    return this->_buffer;
  }

  public: void bufferPush(char data) {
    this->_buffer[strlen(this->_buffer)] = data;
  }

  public: void newDataAvailable(bool condition) {
    this->new_data = condition;
  }

  public: bool isNewDataAvailable() {
    return this->new_data;
  }

  public: void removeFromBuffer(char * data) {
    char command[] = "asd";

    int pos = (int) strpbrk(this->_buffer, command);
    int len = strlen(command);
    
    if(pos == NULL) {
      return;
    }

    int i;
    for(i=0; i<len; i++) {
      this->_buffer[i + pos] = this->buffer[i + pos + (int)len];
    }

    // char *pos = strchr(this->_buffer, (char)'dfsdf');

    // Serial.print('[');
    // Serial.print(pos);
    // Serial.println(']');
  }
};
