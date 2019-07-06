using namespace std;

#pragma once

#include <string.h>
#include "definitions.h"

#define BUFFER_SIZE         _64KBw64
#define BUFFER_SIZE_LIMIT   _64KB

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

  public: void removeFromBuffer(char * com_pointer, int com_len) {
    uint16_t buf_len = strlen(this->_buffer),
             com_pos = com_pointer - this->_buffer,
             placements = buf_len - com_pos - com_len,
             i;
    
    for(i=0; i<placements; i++) {
      this->_buffer[com_pos + i] = this->_buffer[com_pos + com_len + i];
    }

    for(i=1; i<=com_len; i++) {
      this->_buffer[buf_len - i] = 0x00;
    }
  }
};
