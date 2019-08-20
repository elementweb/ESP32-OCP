using namespace std;

#pragma once

#include <string.h>
#include "definitions.h"
#include <MD5Builder.h>
#include <EEPROM.h>


#define BUFFER_OUTGOING_START     (300)
#define BUFFER_INCOMING_START     (16000300)
#define BUFFER_BLOCK_SIZE_BYTES   (512)

#define BUFFER_MAX_SIZE_BLOCKS    (8000000)

// #define MODE_DATA     0x11;
// #define MODE_COMMAND  0x00;

MD5Builder _md5;
const size_t buffer_length = BUFFER_BLOCK_SIZE_BYTES;

// PIN definitions
#define SDCS_PIN 14

class dataManager {
  private: SdFat uSD;

  // data/command mode flag
  // private: byte mode = MODE_COMMAND;

  // outgoing
  private: uint8_t _block1[buffer_length];
  private: uint8_t _block2[buffer_length];

  // buffer for SD card operations
  private: uint8_t _exchange1[buffer_length];
  private: uint8_t _exchange2[buffer_length];

  // incoming
  private: uint8_t _block3[buffer_length];
  private: uint8_t _block4[buffer_length];

  // outgoing block pointers
  public: const uint32_t outgoingBlockStart = BUFFER_OUTGOING_START;
  public: uint32_t outgoingBlockPointer = BUFFER_OUTGOING_START;
  public: size_t outgoingBytePointer = 0;

  // incoming block pointers
  private: const uint32_t incomingBlockStart = BUFFER_INCOMING_START;
  private: uint32_t incomingBlockPointer = BUFFER_INCOMING_START;
  private: int incomingBytePointer = 0;

  public: bool block2_full = false;

  public: uint8_t frontBuffer[(size_t) 1024];

  public: void initialize(SdFat &uSD) {
    this->uSD = uSD;

    SPI_OP_START();

    while(!this->uSD.cardBegin(SDCS_PIN, SD_SCK_MHZ(20))) {
      Serial.println(PROGMEM "uSD card failed to initialize. Trying again");
      ring(5, 2, 50);

      delay(1000);
    }

    long blocks = this->uSD.card()->cardCapacity();

    SPI_OP_STOP();

    Serial.println(PROGMEM "uSD card initialized with total size of " + (String) blocks + " blocks");
  }

  public: void updateFrontOutgoingBuffer() {
    size_t block_len = 512;

    this->frontBuffer[0] = (uint8_t) 0;

    // Serial.print("Block 1 length: ");
    // Serial.println(strlen((char*) this->_block1));

    // Serial.print("Block 2 length: ");
    // Serial.println(strlen((char*) this->_block2));

    if(this->block2_full) {
      this->copy(this->_block2, this->frontBuffer, block_len);
      this->frontBuffer[block_len] = (uint8_t) 0;
    }

    if(this->outgoingBytePointer > 0) {
      this->copy(this->_block1, this->frontBuffer + strlen((char*) this->frontBuffer), strlen((char*) this->_block1));
      this->frontBuffer[strlen((char*) this->_block1) + strlen((char*) this->_block2)] = (uint8_t) 0;
    }

    // Serial.print("Buffer length: ");
    // Serial.println(strlen((char*) buffer));

    // if(this->block2_full) {
    //   memcpy(buffer, this->_block2, strlen((char*) this->_block2));
    //   memcpy(buffer + 512, this->_block1, strlen((char*) this->_block1));
    // }
    // memcpy(buffer, this->_block1, strlen((char*) this->_block1));
    // Serial.println((char*) buffer);
  }

  public: uint64_t bufferSize() {
    return (uint64_t) BUFFER_MAX_SIZE_BLOCKS * (uint64_t) BUFFER_BLOCK_SIZE_BYTES;
  }

  public: uint64_t bufferLength() {
    return (this->outgoingBlockPointer - BUFFER_OUTGOING_START) * BUFFER_BLOCK_SIZE_BYTES
      + strlen((char*) this->_block2)
      + this->outgoingBytePointer;
  }

  public: uint64_t bufferRemaining() {
    return this->bufferSize() - this->bufferLength();
  }

  public: void bufferFlush() {
    this->outgoingBlockPointer = this->outgoingBlockStart;
    this->outgoingBytePointer = 0;

    memset(this->_block1, 0, (size_t) 512);
    memset(this->_block2, 0, (size_t) 512);
    memset(this->frontBuffer, 0, (size_t) 1024);

    this->block2_full = false;
  }

  public: void returnBuffer() {
    uint32_t block;
    size_t buffer_len = 512;
    uint8_t local[buffer_len];

    for(block = this->outgoingBlockStart; block < this->outgoingBlockPointer; block++) {
      local[0] = 0x00;

      SPI_OP_START();
      this->uSD.card()->readBlock(block, local);
      SPI_OP_STOP();

      local[buffer_len] = (uint8_t) 0x00;

      // Serial.print(PROGMEM "Return: ");
      // Serial.println(this->md5((char*) local));

      Serial2.print((char*) local);
      delayMicroseconds(100);
    }

    if(this->block2_full) {
      Serial2.print((char*) this->_block2);
      delayMicroseconds(100);
    }

    Serial2.print((char*) this->_block1);
  }

  private: void copy(uint8_t* src, uint8_t* dst, int len) {
    memcpy(dst, src, sizeof(src[0])*len);
  }

  public: void bufferPush(char data) {
    // size_t byte;
    
    uint32_t pointer;
    size_t buffer_len = 512;
    uint8_t local[buffer_len];
    bool match = false;
    
    local[0] = 0x00;

    this->_block1[this->outgoingBytePointer++] = (uint8_t) data;

    if(this->outgoingBytePointer >= buffer_len) {
      if(this->block2_full) {
        match = false;

        while(!match) {
          // for(byte = 0; byte < buffer_len; byte++) {
          //   this->_exchange2[byte] = (char) this->_block2[byte];
          // }
          this->copy(this->_block2, this->_exchange2, 512);

          pointer = this->outgoingBlockPointer++

          SPI_OP_START();
          this->uSD.card()->writeBlock(pointer, this->_exchange2);
          this->uSD.card()->readBlock(pointer, local);
          SPI_OP_STOP();

          local[buffer_len] = (uint8_t) 0x00;

          match = this->md5((char*) local) == this->md5((char*) this->_exchange2);

          // Serial.print(PROGMEM "Push: ");
          // Serial.println(this->md5((char*) local));

          if(!match) {
            Serial.println(PROGMEM "DATA MISMATCH.");
          }
        }

        // Serial.println("SD write successful!");
      }

      this->copy(this->_block1, this->_block2, 512);
      // for(byte = 0; byte < buffer_len; byte++) {
      //   this->_block2[byte] = this->_block1[byte];
      // }

      this->block2_full = true;
      this->outgoingBytePointer = 0;

      memset(this->_block1, 0, buffer_len);
    }
  }

  public: void reportStats() {
    // Serial.println("buffer length: " + (String) this->bufferLength());
    Serial.println(PROGMEM "outgoingBytePointer: " + (String) this->outgoingBytePointer);
    Serial.println(PROGMEM "outgoingBlockPointer: " + (String) this->outgoingBlockPointer);
    
    if(this->block2_full) {
      Serial.println(PROGMEM "block2_full = true");
    } else {
      Serial.println(PROGMEM "block2_full = false");
    }
  }

  public: void removeFromBuffer(char * pointer, int length) {
    uint16_t buffer_length = strlen((char*) this->frontBuffer),
             position = pointer - (char*) this->frontBuffer,
             placements = buffer_length - position - length,
             i;
    
    for(i=0; i<placements; i++) {
      this->frontBuffer[position + i] = this->frontBuffer[position + length + i];
    }

    for(i=1; i<=length; i++) {
      this->frontBuffer[buffer_length - i] = (uint8_t) 0;
    }

    size_t frontBufferLength = strlen((char*) this->frontBuffer);

    if(frontBufferLength + length < 512) {
      Serial.println(PROGMEM "CMD CASE: 1");

      this->_block1[frontBufferLength] = (uint8_t) 0;
      this->outgoingBytePointer = frontBufferLength;
    }

    if(frontBufferLength >= 512) {
      Serial.println(PROGMEM "CMD CASE: 2");

      this->_block1[frontBufferLength - 512] = (uint8_t) 0;
      this->outgoingBytePointer = frontBufferLength - 512;
    }

    if(frontBufferLength < 512 && (frontBufferLength + length) >= 512) {
      Serial.println(PROGMEM "CMD CASE: 3");

      for(size_t byte = 0; byte < frontBufferLength; byte++) {
        this->_block1[byte] = this->frontBuffer[byte];
      }

      this->_block1[frontBufferLength] = (uint8_t) 0;
      this->outgoingBytePointer = frontBufferLength;

      memset(this->_block2, 0, (size_t) 512);

      this->block2_full = false;
    }
  }

  public: String md5(char * data) {
    _md5.begin();
    _md5.add(data);
    _md5.calculate();

    return _md5.toString();
  }
};
