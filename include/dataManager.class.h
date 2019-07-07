using namespace std;

#pragma once

#include <string.h>
#include "definitions.h"
#include <MD5Builder.h>

#define BUFFER_SIZE         _64KBw64
#define BUFFER_SIZE_LIMIT   _64KB

MD5Builder _md5;

class dataManager {
  private: char _buffer[BUFFER_SIZE];

  public: unsigned int bufferLength() {
    return strlen(this->_buffer);
  }

  public: unsigned int bufferSize() {
    return BUFFER_SIZE;
  }

  public: unsigned int bufferSizeLimit() {
    return BUFFER_SIZE_LIMIT;
  }

  public: unsigned int bufferRemaining() {
    return BUFFER_SIZE_LIMIT - this->bufferLength();
  }

  public: unsigned int bufferRemainingReal() {
    return BUFFER_SIZE - this->bufferLength();
  }

  public: void bufferFlush() {
    unsigned int i;

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

  public: void removeFromBuffer(char * pointer, int length) {
    uint16_t buffer_length = strlen(this->_buffer),
             position = pointer - this->_buffer,
             placements = buffer_length - position - length,
             i;
    
    for(i=0; i<placements; i++) {
      this->_buffer[position + i] = this->_buffer[position + length + i];
    }

    for(i=1; i<=length; i++) {
      this->_buffer[buffer_length - i] = 0x00;
    }
  }

  public: String md5(char * data) {
    _md5.begin();
    
    _md5.add(data);

    _md5.calculate();

    return _md5.toString();
  }
};
