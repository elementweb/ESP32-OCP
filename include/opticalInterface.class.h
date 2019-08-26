using namespace std;

#pragma once

// Pin allocations
#define DATA_PIN                    (32)
#define LASER_PIN                   (27)
#define GAIN_PIN                    (33)
#define LOAD_PIN                    (25)

// Define optical interface clock frequency in Hz. Baud rate = frequency*2
#define FREQUENCY                   (100000)
#define INCOMING_BUFFER_DEPTH       (16384)

// Signal pulse bounds
#define LOWER_DETECTABLE            period/2 - period/16;
#define UPPER_DETECTABLE            period/2 + period/16;
#define LOWER_VALID                 period/4 + period/16;
#define UPPER_VALID                 period/2 + (3*period)/16;

// Operational modes
#define OP_MODE_IDLE                (0) // Idle (default)
#define OP_MODE_TRANSMITTING        (1) // Tranmission mode
#define OP_MODE_PENDING             (2) // Checking for subsequent packets
#define OP_MODE_RECEIVING           (3) // Receiving mode

// Transmission modes/stages (in order)
#define MODE_IDLE                   (0) // Idle (default)
#define MODE_BEACON                 (1) // Stream square wave beacon
#define MODE_STREAM                 (2) // Stream packet and await verification

// Response types
#define RESPONSE_BEACON             (0x55)
#define RESPONSE_VERIFICATION       (0xA7)
#define PRE_PACKET                  (0x5E)
#define POST_PACKET                 (0x7C)

// Packet sizing
#define PACKET_DATA_SIZE_BYTES      (512)
#define PACKET_WRAPPER_SIZE_BYTES   (256)

// Packet pulsing
#define TRANS_DELAY_MS              (1000)
#define BEACON_TIMEOUT_MS           (500)
#define PACKET_TIMEOUT_MS           (100)
#define PULSE_TIMEOUT_MS            (20)
#define RESPONSE_TIMEOUT_MS         (30000)
#define PRE_POST_PACKET_DURATION_MS (5)

long pulse1, pulse2;

class opticalInterface {
  private: char flag_packet_header[7] PROGMEM         = "[flag]";
  private: char data_packet_header[7] PROGMEM         = "[data]";
  private: char checksum_packet_header[11] PROGMEM    = "[checksum]";
  private: char length_packet_header[9] PROGMEM       = "[length]";
  private: char packet_footer[9] PROGMEM              = "[footer]";
  private: char packet_reset[6] PROGMEM               = "[rst]";

  private: uint8_t operational_mode = OP_MODE_IDLE;
  private: uint8_t transmission_mode = MODE_IDLE;
  private: uint64_t completion = 0;

  private: double period = 1000000 / FREQUENCY;
  private: long baud = FREQUENCY * 2;

  private: double lower = LOWER_DETECTABLE;
  private: double upper = UPPER_DETECTABLE;
  private: double lower_valid = LOWER_VALID;
  private: double upper_valid = UPPER_VALID;

  private: size_t packet_buffer_size = (size_t) (PACKET_DATA_SIZE_BYTES + PACKET_WRAPPER_SIZE_BYTES + 512);
  private: size_t data_buffer_size = (size_t) 768;
  private: uint8_t packet_buffer[(size_t) (PACKET_DATA_SIZE_BYTES + PACKET_WRAPPER_SIZE_BYTES + 512)];
  private: uint8_t data_buffer[(size_t) 768];

  public: uint32_t outgoingBlockPointer;
  private: uint8_t _outgoingPacketFlag = 0;
  private: uint8_t _incomingPacketFlag = 0;
  private: uint8_t _expectingIncomingPacketFlag = 1;

  private: bool remote_unit_responding = false;
  private: long last_remote_unit_response = 0;
  private: bool received_packet_verification = false;
  private: bool incoming_packet_detected = false;
  private: bool expecting_incoming_packet = false;

  private: bool data_ready = false;
  private: String incomingData;

  private: bool notified = true;
  private: bool _reset = false;

  public: void initialize(dataManager &dataManager, uartInterface &portUart) {
    // Optical interface pins
    pinMode(DATA_PIN, INPUT);
    pinMode(LASER_PIN, OUTPUT);

    // Digital potentiometers
    pinMode(GAIN_PIN, OUTPUT);
    pinMode(LOAD_PIN, OUTPUT);

    this->outgoingBlockPointer = dataManager.outgoingBlockPointer;

    size_t buffer_depth = INCOMING_BUFFER_DEPTH;

    // Initialize optical interface
    opticalLink.begin(this->baud, SERIAL_8N1, DATA_PIN, LASER_PIN, true);
    opticalLink.setRxBufferSize(buffer_depth);

    // Allow cooldown time before continuing
    delay(100);
  }

  private: bool dataAvailableForTransmission(dataManager &dataManager) {
    return dataManager.outgoingBytePointer > 0
        || dataManager.outgoingBlockPointer != this->outgoingBlockPointer;
  }

  private: bool dataAvailableBufferBlocks(dataManager &dataManager) {
    return dataManager.outgoingBlockPointer != this->outgoingBlockPointer;
  }

  public: void processOutgoing(dataManager &dataManager, uartInterface &portUart) {
    bool packet_verification_detected;

    if(this->operational_mode == OP_MODE_RECEIVING) {
      return;
    }

    if(this->operational_mode == OP_MODE_IDLE || this->operational_mode == OP_MODE_PENDING) {
      if(portUart.data_available && ((millis() - portUart.last_data_available) > TRANS_DELAY_MS)) {
        if(this->dataAvailableForTransmission(dataManager)) {
          this->activateTransmission(dataManager, portUart);
        } else {
          this->operational_mode = OP_MODE_IDLE;
          this->transmission_mode = MODE_IDLE;
        }
      } else {
        this->operational_mode = OP_MODE_IDLE;
        this->transmission_mode = MODE_IDLE;
      }
    }
    
    if(this->operational_mode != OP_MODE_TRANSMITTING) {
      return;
    }

    switch (this->transmission_mode) {
      case MODE_IDLE:
        this->transmission_mode = MODE_BEACON;
      break;
    
      case MODE_BEACON:
        #ifdef DEBUG
        Serial.println(PROGMEM "T: emitting beacon");
        #endif
        
        this->emitBeacon();

        this->transmission_mode = this->searchBeacon() ? MODE_STREAM : MODE_BEACON;
      break;

      case MODE_STREAM:
        #ifdef DEBUG
        Serial.print(PROGMEM "T: Streaming packet (" + (String) this->_outgoingPacketFlag + ")");
        #endif

        packet_verification_detected = false;

        while(!packet_verification_detected) {
          this->streamPacket();

          delayMicroseconds(50);

          packet_verification_detected = this->expectPacketVerification();
        }

        #ifdef DEBUG
        Serial.println(PROGMEM " -- received verification");
        #endif

        if(this->_reset) {
          this->reset();
          
          return;
        }

        this->operational_mode = OP_MODE_PENDING;
        this->transmission_mode = MODE_IDLE;
      break;
    }
  }

  private: void emitBeacon(uint length = 7000) {
    for(uint n=0; n<length; n++) {
      opticalLink.write((char) RESPONSE_BEACON);
    }
  }

  public: void processIncoming(dataManager &dataManager, uartInterface &portUart) {
    char read;

    bool packet_complete = false;
    bool packet_detected = false;
    bool pre_packet_detected = false;

    size_t buffer_pointer = 0;
    
    if(this->operational_mode == OP_MODE_TRANSMITTING || this->operational_mode == OP_MODE_PENDING) {
      return;
    }

    if(this->operational_mode == OP_MODE_IDLE) {
      if(this->searchBeacon()) {
        #ifdef DEBUG
        Serial.println(PROGMEM "R: Incoming beacon detected");
        #endif

        while(!packet_detected) {
          this->emitBeacon(21000);

          packet_detected = this->detectIncomingPacket();
        }

        #ifdef DEBUG
        Serial.println(PROGMEM "R: Detected incoming packet");
        #endif

        this->operational_mode = OP_MODE_RECEIVING;

        #ifdef DEBUG
        Serial.println(PROGMEM "R: Reception mode activated");
        #endif
      }
    }

    if(this->operational_mode == OP_MODE_RECEIVING) {
      this->flush();

      packet_complete = false;
      pre_packet_detected = false;

      memset(this->packet_buffer, 0, this->packet_buffer_size);
      buffer_pointer = (size_t) 0;

      #ifdef DEBUG
      Serial.println(PROGMEM "R: Looking up for new packet");
      #endif

      if(this->_incomingPacketFlag > 0) {
        while(!opticalLink.available()) {
          this->streamPacketVerification(5);
        }
      }

      while(!packet_complete) {
        if(opticalLink.available()) {
          read = opticalLink.read();

          if(read == PRE_PACKET) {
            pre_packet_detected = true;
          }

          if(pre_packet_detected) {
            this->packet_buffer[buffer_pointer++] = read;
          }

          if(pre_packet_detected && read == POST_PACKET) {
            packet_complete = true;
          }
        }
      }

      #ifdef DEBUG
      Serial.println(PROGMEM "R: Packet reception complete");
      #endif
      
      this->parsePacketAndValidateIntegrity(dataManager);

      if(this->_reset) {
        this->streamPacketVerification(100);
        this->reset();

        return;
      }

      if(this->_incomingPacketFlag > 0) {
        this->streamPacketVerification(5);
      }
    }
  }

  private: void reset() {
    this->operational_mode = OP_MODE_IDLE;
    this->transmission_mode = MODE_IDLE;
    this->_reset = false;

    this->_outgoingPacketFlag = 0;
    this->_incomingPacketFlag = 0;
    this->_expectingIncomingPacketFlag = 1;

    #ifdef DEBUG
    Serial.println("reseting...");
    #endif

    red(true);
    ring(3, 2);
    red(false);
    delay(1000);
    blue(false);
  }

  private: void streamPacketVerification(uint times) {
    for(int e=0; e<=times; e++) {
      opticalLink.write(RESPONSE_VERIFICATION);
      delayMicroseconds(50);

      opticalLink.write(this->_incomingPacketFlag);
      delayMicroseconds(50);
    }

    delayMicroseconds(50);
  }

  private: bool expectPacketVerification() {
    char read, prev = 0x00;

    bool verification_detected = false;

    long start = millis();
    
    while((millis() - start) < 5) {
      if(opticalLink.available()) {
        read = opticalLink.read();

        if(read == (char) this->_outgoingPacketFlag) {
          if(prev == (char) RESPONSE_VERIFICATION) {
            verification_detected = true;
          }
        }

        prev = read;
      }
    }

    if(verification_detected) {
      this->notified = false;
    }

    return verification_detected;
  }

  private: bool parsePacketAndValidateIntegrity(dataManager &dataManager) {
    char data_buffer[513];

    memset(data_buffer, 0, (size_t) 513);

    if(strlen((char*) this->packet_buffer) <= 0) {
      return false;
    }

    String packet = (char*) this->packet_buffer;
    String flag = dataManager.midString(packet, this->flag_packet_header, this->checksum_packet_header);
    
    if(flag.toInt() != this->_expectingIncomingPacketFlag) {
      return false;
    }

    this->_incomingPacketFlag = (uint8_t) flag.toInt();
    this->expectIncomingPacketFlag(this->_incomingPacketFlag);

    this->incomingData = dataManager.midString(packet, this->data_packet_header, this->packet_reset);

    String length = dataManager.midString(packet, this->length_packet_header, this->data_packet_header);

    if(length.toInt() != this->incomingData.length()) {
      return false;
    }

    String checksum = dataManager.midString(packet, this->checksum_packet_header, this->length_packet_header);
    this->incomingData.toCharArray(data_buffer, 513);

    String md5 = dataManager.md5((char*) data_buffer).toString();

    if(checksum != md5) {
      return false;
    }

    this->_reset = dataManager.midString(packet, this->packet_reset, this->packet_footer) == "1";

    while(this->data_ready);
    this->data_ready = true;

    return true;
  }

  private: bool detectIncomingPacket() {
    char read;
    uint pre_packet_count = 0;

    this->flush();

    long start = millis();

    while((millis() - start) < PACKET_TIMEOUT_MS) {
      if(opticalLink.available()) {
        read = opticalLink.read();

        pre_packet_count = read == PRE_PACKET ? (pre_packet_count + 1) : 0;
      }

      if(pre_packet_count >= 4) {
        this->flush();

        return true;
      }
    }

    return false;
  }

  private: bool detectIncomingPulse() {
    pulse1 = pulseIn(DATA_PIN, HIGH, period*2);
    pulse2 = pulseIn(DATA_PIN, HIGH, period*2);

    if(pulse1 <= 0 || pulse2 <= 0) {
      return false;
    }
    
    return pulse1 > lower && pulse1 < upper
        && pulse2 > lower && pulse2 < upper;
  }

  public: bool detectValidPulse() {
    pulse1 = pulseIn(DATA_PIN, HIGH, period*2);
    pulse2 = pulseIn(DATA_PIN, HIGH, period*2);

    if(pulse1 <= 0 || pulse2 <= 0) {
      return false;
    }
    
    return pulse1 > lower_valid && pulse1 < upper_valid
        && pulse2 > lower_valid && pulse2 < upper_valid;
  }

  private: void setGain(uint8_t gain) {
    SPI_OP_BEGIN();

    digitalWrite(GAIN_PIN, LOW);

    SPI.transfer(0x00);
    SPI.transfer(gain);
    
    digitalWrite(GAIN_PIN, HIGH);
    
    SPI_OP_END();
  }

  private: void setLoad(uint8_t load) {
    SPI_OP_BEGIN();

    digitalWrite(LOAD_PIN, LOW);

    SPI.transfer(0x11);
    SPI.transfer(load);

    digitalWrite(LOAD_PIN, HIGH);

    SPI_OP_END();
  }

  public: void runAGC() {
    int long_gain = 0;
    int long_load = 0;
    int long_pulse = 1;

    int pulse_count = 0;
    int e, i;
    bool pulse;

    int load_increments = FREQUENCY >= 250000 ? map(FREQUENCY, 250000, 400000, 4, 1) : 5;

    #ifdef DEBUG
    long start, end;
    start = millis();
    #endif

    for(e=0; e<=255; e+=load_increments) { // for frequencies >250kHz load increments should be as low as 1
      pulse_count = 0;
      this->setLoad(e);

      for(i=128; i>=0; i-=1) {
        this->setGain(i);

        pulse = this->detectIncomingPulse();

        if(!pulse) {
          pulse_count = 0;
        } else {
          pulse_count++;
          
          if(pulse_count >= long_pulse) {
            long_pulse = pulse_count;
            long_gain = i + (pulse_count*4)/2 + 4;
            long_load = e;
          }
        }
      }
    }

    if(long_pulse > 1) {
      setLoad(long_load);
      setGain(long_gain);

      #ifdef DEBUG
      end = millis() - start;
      Serial.print(PROGMEM "R: AGC has resolved optimum load (");
      Serial.print(long_load);
      Serial.print(PROGMEM ") and gain (");
      Serial.print(long_gain);
      Serial.print(PROGMEM ") in ");
      Serial.print(end);
      Serial.println(PROGMEM "ms");
      #endif

      ring(1, 1);
    }
  }

  private: bool searchBeacon() {
    bool signal = this->detectValidPulse();

    if(signal) {
      blue(true);

      return true;
    }

    blue(false);

    long start = millis();

    while(!signal && (start + BEACON_TIMEOUT_MS) > millis()) {
      this->runAGC();

      signal = this->detectValidPulse();
    }

    blue(signal);

    return signal;
  }

  private: bool perhapsWeShouldReset(dataManager &dataManager, uartInterface &portUart) {
    if(!this->dataAvailableForTransmission(dataManager)) {
      portUart.data_available = false;
      this->_reset = true;
      
      #ifdef DEBUG
      Serial.println(PROGMEM "T: We should reset");
      #endif

      return true;
    }

    return false;
  }

  private: bool activateTransmission(dataManager &dataManager, uartInterface &portUart) {
    if(this->buildDataPacket(dataManager)) {
      #ifdef DEBUG
      Serial.print(PROGMEM "data_buffer: ");
      Serial.println((char*) this->data_buffer);
      #endif

      if(this->buildPacket(dataManager, this->perhapsWeShouldReset(dataManager, portUart))) {
        #ifdef DEBUG
        Serial.print(PROGMEM "packet_buffer: ");
        Serial.println((char*) this->packet_buffer);
        #endif

        this->transmission_mode = this->operational_mode == OP_MODE_PENDING ? MODE_STREAM : MODE_IDLE;
        this->operational_mode = OP_MODE_TRANSMITTING;

        #ifdef DEBUG
        Serial.println(PROGMEM "T: Transmission mode activated");
        #endif

        return true;
      }
    }

    return false;
  }

  private: uint32_t returnOutgoingBlockPointer(dataManager &dataManager) {
    this->outgoingBlockPointer++;
    
    if(this->outgoingBlockPointer > (dataManager.outgoingBlockStart + BUFFER_MAX_SIZE_BLOCKS)) {
      this->outgoingBlockPointer = dataManager.outgoingBlockStart;
    }

    return this->outgoingBlockPointer;
  }

  private: uint32_t peekOutgoingBlockPointer(dataManager &dataManager) {
    uint32_t pointer = this->outgoingBlockPointer + 1;
    
    if(pointer > (dataManager.outgoingBlockStart + BUFFER_MAX_SIZE_BLOCKS)) {
      pointer = dataManager.outgoingBlockStart;
    }

    return pointer;
  }

  private: bool buildDataPacket(dataManager &dataManager) {
    uint32_t block;

    memset(this->data_buffer, 0, this->data_buffer_size);

    if(this->peekOutgoingBlockPointer(dataManager) <= dataManager.outgoingBlockPointer) {
      DATA_OP_BEGIN();
      block = this->returnOutgoingBlockPointer(dataManager);
      dataManager.copy(dataManager.returnOutgoingBlock(block), this->data_buffer, (int) PACKET_DATA_SIZE_BYTES);
      DATA_OP_END();

      this->data_buffer[(size_t) PACKET_DATA_SIZE_BYTES] = (uint8_t) 0x00;

      return true;
    }

    if(dataManager.outgoingBytePointer > 0) {
      dataManager.copy(dataManager.returnOutgoingDataExcess(), this->data_buffer, (int) PACKET_DATA_SIZE_BYTES);
      this->data_buffer[(size_t) dataManager.outgoingBytePointer] = (uint8_t) 0x00;

      dataManager.frontBufferFlush();

      return true;
    }

    return false;
  }

  private: bool buildPacket(dataManager &dataManager, bool reset = false) {
    char checksum[32], flag_buf[4], length_buf[4];

    memset(this->packet_buffer, 0, this->packet_buffer_size);

    dataManager.md5((char*) this->data_buffer).getChars(checksum);

    String flag = (String) this->outgoingPacketFlag();
    flag.toCharArray(flag_buf, 4);

    String data_length = (String) strlen((char*) this->data_buffer);
    data_length.toCharArray(length_buf, 4);

    dataManager.copy((uint8_t*) this->flag_packet_header, this->packet_buffer, 6);
    dataManager.copy((uint8_t*) flag_buf, this->packet_buffer + strlen((char*) this->packet_buffer), strlen((char*) flag_buf));

    dataManager.copy((uint8_t*) this->checksum_packet_header, this->packet_buffer + strlen((char*) this->packet_buffer), 10);
    dataManager.copy((uint8_t*) checksum, this->packet_buffer + strlen((char*) this->packet_buffer), 32);

    dataManager.copy((uint8_t*) this->length_packet_header, this->packet_buffer + strlen((char*) this->packet_buffer), 8);
    dataManager.copy((uint8_t*) length_buf, this->packet_buffer + strlen((char*) this->packet_buffer), strlen((char*) length_buf));

    dataManager.copy((uint8_t*) this->data_packet_header, this->packet_buffer + strlen((char*) this->packet_buffer), 6);
    dataManager.copy(this->data_buffer, this->packet_buffer + strlen((char*) this->packet_buffer), strlen((char*) this->data_buffer));

    dataManager.copy((uint8_t*) this->packet_reset, this->packet_buffer + strlen((char*) this->packet_buffer), 5);
    dataManager.copy((uint8_t*) (reset ? "1" : "0"), this->packet_buffer + strlen((char*) this->packet_buffer), 1);

    dataManager.copy((uint8_t*) this->packet_footer, this->packet_buffer + strlen((char*) this->packet_buffer), 8);

    return strlen((char*) this->packet_buffer) > 0;
  }

  private: uint8_t outgoingPacketFlag() {
    this->_outgoingPacketFlag++;
    
    if(this->_outgoingPacketFlag > 128) {
      this->_outgoingPacketFlag = 1;
    }

    return this->_outgoingPacketFlag;
  }

  private: uint8_t expectIncomingPacketFlag(uint8_t current) {
    this->_expectingIncomingPacketFlag = current + 1;
    
    if(this->_expectingIncomingPacketFlag > 128) {
      this->_expectingIncomingPacketFlag = 1;
    }

    return this->_expectingIncomingPacketFlag;
  }

  private: void streamPacket() {
    long start = millis();

    while((millis() - start) < PRE_POST_PACKET_DURATION_MS) {
      opticalLink.write(PRE_PACKET);
      
      delayMicroseconds(100);
    }

    delayMicroseconds(100);
    
    opticalLink.write((char*) this->packet_buffer);

    start = millis();

    while((millis() - start) < PRE_POST_PACKET_DURATION_MS) {
      opticalLink.write(POST_PACKET);
      
      delayMicroseconds(100);
    }
  }

  public: void emitIncomingData() {
    if(this->data_ready) {
      platformInterface.print(this->incomingData);

      this->data_ready = false;

      redToggle();
      ringMicroseconds(1, 50);
    }

    if(!this->notified) {
      redToggle();
      ringMicroseconds(1, 50);

      this->notified = true;
    }
  }

  private: void flush() {
    while(opticalLink.available() > 0) {
      opticalLink.read();
    }
  }

  private: void remoteUnitResponds(bool condition = true) {
    // this->remote_unit_responding = condition;
    
    // this->last_remote_unit_response = condition ? millis() : 0;
  }

  private: bool remoteUnitResponding() {
    // if(!this->remote_unit_responding) {
    //   return false;
    // }

    // if(this->last_remote_unit_response > millis()) {
    //   return false;
    // }

    // return (millis() - this->last_remote_unit_response) < RESPONSE_TIMEOUT_MS;
  }
  
};
