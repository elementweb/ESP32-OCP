using namespace std;

#pragma once

// Pin allocations
#define DATA_PIN                    (32)
#define LASER_PIN                   (27)
#define GAIN_PIN                    (33)
#define LOAD_PIN                    (25)

// Define optical interface clock frequency in Hz. Baud rate = frequency*2
#define FREQUENCY                   (150000)
#define INCOMING_BUFFER_DEPTH       (16384)

// Signal pulse bounds
#define LOWER_DETECTABLE            period/2 - period/16;
#define UPPER_DETECTABLE            period/2 + period/16;
#define LOWER_VALID                 period/4 + period/16;
#define UPPER_VALID                 period/2 + (3*period)/16;

// Operational modes
#define OP_MODE_IDLE                (0) // Idle (default)
#define OP_MODE_TRANSMITTING        (1) // Tranmission mode
#define OP_MODE_RECEIVING           (2) // Receiving mode
#define OP_MODE_PENDING             (3) // Checking for subsequent packets

// Transmission modes/stages (in order)
#define MODE_IDLE                   (0) // Idle (default)
#define MODE_BEACON                 (1) // Stream square wave beacon
#define MODE_STREAM                 (2) // Stream packet and await verification
#define MODE_SUB_PENDING            (3) // Outgoing packet verified, looking up for subsequent packets

// Response types
#define RESPONSE_BEACON             (0x55)
#define RESPONSE_VERIFICATION       (0xCC)
#define PRE_PACKET                  (0x99)

// Packet sizing
#define PACKET_DATA_SIZE_BYTES      (512)
#define PACKET_WRAPPER_SIZE_BYTES   (256)

// Packet pulsing
#define TRANS_DELAY_MS              (1000)
#define BEACON_TIMEOUT_MS           (500)
#define PULSE_TIMEOUT_MS            (20)
#define RESPONSE_TIMEOUT_MS         (30000)
#define PRE_PACKET_DURATION_MS      (5)

long pulse1, pulse2;

class opticalInterface {
  private: char flag_packet_header[7] PROGMEM         = "[flag]";
  private: char data_packet_header[14] PROGMEM        = "[data-header]";
  private: char checksum_packet_header[18] PROGMEM    = "[checksum-header]";
  private: char length_packet_header[9] PROGMEM       = "[length]";
  private: char packet_footer[9] PROGMEM              = "[footer]";

  private: uint8_t operational_mode = OP_MODE_IDLE;
  private: uint8_t transmission_mode = MODE_IDLE;
  private: uint64_t completion = 0;

  private: double period = 1000000 / FREQUENCY;
  private: long baud = FREQUENCY * 2;

  private: double lower = LOWER_DETECTABLE;
  private: double upper = UPPER_DETECTABLE;
  private: double lower_valid = LOWER_VALID;
  private: double upper_valid = UPPER_VALID;

  private: size_t packet_buffer_size = (size_t) (PACKET_DATA_SIZE_BYTES + PACKET_WRAPPER_SIZE_BYTES);
  private: size_t data_buffer_size = (size_t) PACKET_DATA_SIZE_BYTES;
  private: uint8_t packet_buffer[(size_t) (PACKET_DATA_SIZE_BYTES + PACKET_WRAPPER_SIZE_BYTES + 256)];
  private: uint8_t data_buffer[(size_t) 768];

  public: uint32_t outgoingBlockPointer;
  private: uint8_t _outgoingPacketFlag = 0;

  private: bool remote_unit_responding = false;
  private: long last_remote_unit_response = 0;
  private: bool received_packet_verification = false;
  private: bool incoming_packet_detected = false;
  private: bool expecting_incoming_packet = false;

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

    if(this->operational_mode != OP_MODE_TRANSMITTING/* || this->operational_mode == OP_MODE_PENDING*/) {
      if(portUart.data_available && ((millis() - portUart.last_data_available) > TRANS_DELAY_MS || this->dataAvailableBufferBlocks(dataManager))) {
        // Serial.println(PROGMEM "()");
        
        if(this->dataAvailableForTransmission(dataManager)) {
          this->activateTransmission(dataManager);
        }
      }
    }
    
    if(!this->isTransmissionActive()) {
      this->operational_mode = OP_MODE_IDLE;

      return;
    }

    switch (this->transmission_mode) {
      case MODE_IDLE:
        this->transmission_mode = MODE_BEACON;
      break;
    
      case MODE_BEACON:
        Serial.println(PROGMEM "T: emitting beacon");
        
        this->emitBeacon();

        this->transmission_mode = this->searchBeacon() ? MODE_STREAM : MODE_BEACON;
      break;

      case MODE_STREAM:
        Serial.print(PROGMEM "T: Streaming packet (" + (String) this->_outgoingPacketFlag + ")");

        while(!packet_verification_detected) {
          this->streamPacket();

          packet_verification_detected = this->expectPacketVerification();
        }

        Serial.println(PROGMEM " -- received verification");

        this->operational_mode = OP_MODE_PENDING;
        this->transmission_mode = MODE_IDLE;
      break;
    }
  }

  private: void emitBeacon() {
    for(uint n=0; n<7000; n++) {
      opticalLink.write((char) RESPONSE_BEACON);
    }
  }

  public: void processIncoming(dataManager &dataManager, uartInterface &portUart) {
    char read;
    bool packet_complete = false;
    bool packet_detected = false;
    uint8_t buffer_pointer = 0;
    
    if(this->operational_mode == OP_MODE_TRANSMITTING || this->operational_mode == OP_MODE_PENDING) {
      return;
    }

    if(this->operational_mode == OP_MODE_IDLE) {
      if(this->searchBeacon()) {
        Serial.println(PROGMEM "R: Incoming beacon detected");

        opticalLink.flush();

        while(!packet_detected) {
          this->emitBeacon();

          packet_detected = this->detectIncomingPacket();
        }

        Serial.println(PROGMEM "R: Detected incoming packet");

        this->operational_mode = OP_MODE_RECEIVING;
      }
    }

    if(this->operational_mode == OP_MODE_RECEIVING) {
      while(!opticalLink.available()) {
        delayMicroseconds(100);
      }

      while(!this->detectIncomingPacket());

      while((read = opticalLink.read()) == (char) PRE_PACKET);

      this->packet_buffer[buffer_pointer++] = opticalLink.read();

      while(!packet_complete) {
        if(opticalLink.available()) {
          this->packet_buffer[buffer_pointer++] = opticalLink.read();

          if(strstr((char*) this->packet_buffer, this->packet_footer)) {
            packet_complete = true;
          }
        }
      }
    }
  }

  public: void outboundHousekeepingRunner() {
    // if(this->operational_mode == OP_MODE_TRANSMITTING && !this->transmission_response_beacon) {
    //   this->transmission_response_beacon = this->searchBeacon();
    // }

    // if(this->operational_mode == OP_MODE_TRANSMITTING && !this->received_packet_verification) {
    //   this->received_packet_verification = this->expectPacketVerification();
    // }

    // if(this->expecting_incoming_packet && !this->incoming_packet_detected) {
    //   this->incoming_packet_detected = this->detectIncomingPacket();
    // }

    // delayMicroseconds(100);
  }

  private: bool detectIncomingPacket() {
    char read;
    uint pre_packet_count = 0;

    opticalLink.flush();

    long start = millis();

    while((millis() - start) < 1000) {
      if(opticalLink.available()) {
        read = opticalLink.read();

        pre_packet_count = read == (char) PRE_PACKET ? (pre_packet_count + 1) : 0;
      }

      if(pre_packet_count >= 8) {
        opticalLink.flush();

        return true;
      }
    }

    return false;
  }

  private: bool expectPacketVerification() {
    bool incoming_verification = false;
    char read;

    opticalLink.flush();

    long start = millis();

    while((millis() - start) < PULSE_TIMEOUT_MS) {
      if(opticalLink.available()) {
        read = opticalLink.read();

        delayMicroseconds(100);

        if(incoming_verification && read == (char) this->_outgoingPacketFlag) {
          return true;
        }

        incoming_verification = false;

        if(read == (char) RESPONSE_VERIFICATION) {
          incoming_verification = true;

          continue;
        }
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
    long start, end;
    int e, i;
    bool pulse;

    int load_increments = FREQUENCY >= 250000 ? map(FREQUENCY, 250000, 400000, 4, 1) : 5;

    start = millis();

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

      end = millis() - start;

      Serial.print(PROGMEM "R: AGC has resolved optimum load (");
      Serial.print(long_load);
      Serial.print(PROGMEM ") and gain (");
      Serial.print(long_gain);
      Serial.print(PROGMEM ") in ");
      Serial.print(end);
      Serial.println(PROGMEM "ms");

      ring(1, 1);
    }
  }

  private: bool searchBeacon() {
    bool signal = this->detectValidPulse();

    if(signal) {
      return true;
    }

    long start = millis();

    while(!signal && (start + BEACON_TIMEOUT_MS) > millis()) {
      this->runAGC();

      signal = this->detectValidPulse();
    }

    return signal;
  }

  private: void remoteUnitResponds(bool condition = true) {
    this->remote_unit_responding = condition;
    this->last_remote_unit_response = condition ? millis() : 0;
  }

  private: bool remoteUnitResponding() {
    if(!this->remote_unit_responding) {
      return false;
    }

    if(this->last_remote_unit_response > millis()) {
      return false;
    }

    return (millis() - this->last_remote_unit_response) < RESPONSE_TIMEOUT_MS;
  }

  private: bool activateTransmission(dataManager &dataManager) {
    if(this->buildDataPacket(dataManager)) {
      #ifdef DEBUG
      Serial.print(PROGMEM "data_buffer: ");
      Serial.println((char*) this->data_buffer);
      #endif

      if(this->buildPacket(dataManager)) {
        #ifdef DEBUG
        Serial.print(PROGMEM "packet_buffer: ");
        Serial.println((char*) this->packet_buffer);
        #endif

        this->operational_mode = OP_MODE_TRANSMITTING;
        this->transmission_mode = MODE_IDLE;

        Serial.println(PROGMEM "T: Transmission mode activated");

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

  private: bool buildDataPacket(dataManager &dataManager) {
    uint32_t block;

    memset(this->data_buffer, 0, this->data_buffer_size);

    if(this->outgoingBlockPointer != dataManager.outgoingBlockPointer) {
      DATA_OP_BEGIN();
      block = this->returnOutgoingBlockPointer(dataManager);
      dataManager.copy(dataManager.returnOutgoingBlock(block), this->data_buffer, (int) 512);
      DATA_OP_END();

      this->data_buffer[(size_t) 512] = (uint8_t) 0x00;

      return true;
    }

    if(dataManager.outgoingBytePointer > 0) {
      dataManager.copy(dataManager.returnOutgoingDataExcess(), this->data_buffer, (int) 512);
      this->data_buffer[(size_t) dataManager.outgoingBytePointer] = (uint8_t) 0x00;

      dataManager.frontBufferFlush();

      return true;
    }

    return false;
  }

  private: bool buildPacket(dataManager &dataManager) {
    char checksum[32], flag_buf[4], length_buf[4];

    memset(this->packet_buffer, 0, this->packet_buffer_size);

    dataManager.md5((char*) this->data_buffer).getChars(checksum);

    String flag = (String) this->outgoingPacketFlag();
    flag.toCharArray(flag_buf, 4);

    String data_length = (String) strlen((char*) this->data_buffer);
    data_length.toCharArray(length_buf, 4);

    dataManager.copy((uint8_t*) this->flag_packet_header, this->packet_buffer, 6);
    dataManager.copy((uint8_t*) flag_buf, this->packet_buffer + strlen((char*) this->packet_buffer), strlen((char*) flag_buf));

    dataManager.copy((uint8_t*) this->checksum_packet_header, this->packet_buffer + strlen((char*) this->packet_buffer), 17);
    dataManager.copy((uint8_t*) checksum, this->packet_buffer + strlen((char*) this->packet_buffer), 32);

    dataManager.copy((uint8_t*) this->length_packet_header, this->packet_buffer + strlen((char*) this->packet_buffer), 8);
    dataManager.copy((uint8_t*) length_buf, this->packet_buffer + strlen((char*) this->packet_buffer), strlen((char*) length_buf));

    dataManager.copy((uint8_t*) this->data_packet_header, this->packet_buffer + strlen((char*) this->packet_buffer), 13);
    dataManager.copy(this->data_buffer, this->packet_buffer + strlen((char*) this->packet_buffer), strlen((char*) this->data_buffer));

    dataManager.copy((uint8_t*) this->packet_footer, this->packet_buffer + strlen((char*) this->packet_buffer), 8);
  }

  private: uint8_t outgoingPacketFlag() {
    this->_outgoingPacketFlag++;
    
    if(this->_outgoingPacketFlag >= 128) {
      this->_outgoingPacketFlag = 0;
    }

    return this->_outgoingPacketFlag;
  }

  private: void streamPacket() {
    long start = millis();

    while((millis() - start) < PRE_PACKET_DURATION_MS) {
      opticalLink.write((char) PRE_PACKET);
      
      delayMicroseconds(100);
    }
    
    opticalLink.write((char*) this->packet_buffer);
  }

  public: bool isTransmissionActive() {
    return this->operational_mode == OP_MODE_TRANSMITTING;
  }
};
