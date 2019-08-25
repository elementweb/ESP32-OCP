using namespace std;

#pragma once

#include <string.h>
#include "definitions.h"
#include <MD5Builder.h>
#include <EEPROM.h>

#define BUFFER_BLOCK_SIZE_BYTES   (512)
#define BUFFER_OUTGOING_START     (300)
#define BUFFER_MAX_SIZE_BLOCKS    (8000000)

MD5Builder _md5;
const size_t buffer_length = BUFFER_BLOCK_SIZE_BYTES;
const size_t buffer_length_excess = 768;

// PIN definitions
#define SDCS_PIN 14

class dataManager {
  private: SdFat uSD;

  // outgoing block buffer
  private: uint8_t _block1[buffer_length_excess];
  public: uint8_t _block2[buffer_length_excess];

  // buffer for SD card operations
  private: uint8_t _exchange[buffer_length_excess];

  // outgoing block pointers
  public: const uint32_t outgoingBlockStart = BUFFER_OUTGOING_START;
  public: uint32_t outgoingBlockPointer = BUFFER_OUTGOING_START;
  public: size_t outgoingBytePointer = 0;

  public: void initialize(SdFat &uSD) {
    this->uSD = uSD;

    SPI_OP_BEGIN();

    while(!this->uSD.cardBegin(SDCS_PIN, SD_SCK_MHZ(20))) {
      Serial.println(PROGMEM "uSD card failed to initialize. Trying again");
      ring(5, 2, 50);

      delay(1000);
    }

    long blocks = this->uSD.card()->cardCapacity();

    SPI_OP_END();

    Serial.println(PROGMEM "uSD card initialized with total size of " + (String) blocks + " blocks");
  }

  public: uint64_t bufferSize() {
    return (uint64_t) BUFFER_MAX_SIZE_BLOCKS * (uint64_t) BUFFER_BLOCK_SIZE_BYTES;
  }

  public: uint64_t outgoingBufferLength() {
    return (this->outgoingBlockPointer - BUFFER_OUTGOING_START) * BUFFER_BLOCK_SIZE_BYTES
      + this->outgoingBytePointer;
  }

  public: uint16_t outgoingBufferBlockCount() {
    return (this->outgoingBlockPointer - BUFFER_OUTGOING_START);
  }

  public: void outgoingBufferFlush() {
    this->outgoingBlockPointer = this->outgoingBlockStart;
    
    this->frontBufferFlush();
  }

  public: void frontBufferFlush() {
    this->outgoingBytePointer = 0;

    memset(this->_block1, 0, (size_t) BUFFER_BLOCK_SIZE_BYTES);
  }

  public: uint8_t * returnOutgoingBlock(uint32_t block) {
    uint32_t uSD_block_count = this->outgoingBufferBlockCount();

    if(block <= (this->outgoingBlockStart + uSD_block_count)) {
      SPI_OP_BEGIN();

      #ifdef DEBUG
      Serial.print(PROGMEM "Reading block: ");
      Serial.println(block);
      #endif

      this->uSD.card()->readBlock(block, this->_block2);
      SPI_OP_END();

      return this->_block2;
    }

    return this->returnOutgoingDataExcess();
  }

  public: uint8_t * returnOutgoingDataExcess() {
    return this->_block1;
  }

  private: uint32_t returnOutgoingBlockPointer() {
    this->outgoingBlockPointer++;
    
    if(this->outgoingBlockPointer > (this->outgoingBlockStart + BUFFER_MAX_SIZE_BLOCKS)) {
      this->outgoingBlockPointer = this->outgoingBlockStart;
    }

    return this->outgoingBlockPointer;
  }

  public: void outgoingBufferPush(char data) {
    uint32_t pointer;
    size_t buffer_len = BUFFER_BLOCK_SIZE_BYTES;
    bool match = false;
    bool last_matched = true;
    
    memset(this->_exchange, 0, buffer_len);

    this->_block1[this->outgoingBytePointer++] = (uint8_t) data;

    if(this->outgoingBytePointer >= buffer_len) {
      match = false;

      pointer = this->returnOutgoingBlockPointer();

      while(!match) {
        SPI_OP_BEGIN();
        this->uSD.card()->writeBlock(pointer, this->_block1);
        this->uSD.card()->readBlock(pointer, this->_exchange);
        SPI_OP_END();

        this->_exchange[buffer_len] = (uint8_t) 0;

        match = this->md5((char*) this->_exchange).toString() == this->md5((char*) this->_block1).toString();

        #ifdef DEBUG
        // Serial.print(PROGMEM "Push _block1: ");
        // Serial.println(this->md5((char*) this->_block1).toString());
        // Serial.print(PROGMEM "Push _exchange: ");
        // Serial.println(this->md5((char*) this->_exchange).toString());
        Serial.print(match ? (last_matched ? '+' : '*') : '!');
        #endif

        last_matched = match;

        if(!match) {
          delay(100);
        }
      }

      this->outgoingBytePointer = 0;

      memset(this->_block1, 0, buffer_len);
    }
  }

  public: void reportOutgoingBufferStats() {
    Serial.println(PROGMEM "outgoingBytePointer: " + (String) this->outgoingBytePointer);
    Serial.println(PROGMEM "outgoingBlockPointer: " + (String) this->outgoingBlockPointer);

    Serial.print(PROGMEM "buffer length: ");
    print_uint64_t(Serial, this->outgoingBufferLength());
    Serial.println();
  }

  public: void copy(uint8_t* src, uint8_t* dst, int len) {
    memcpy(dst, src, sizeof(src[0])*len);
  }

  public: MD5Builder md5(char * data) {
    _md5.begin();
    _md5.add(data);
    _md5.calculate();

    return _md5;
  }
};
