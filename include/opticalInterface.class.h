using namespace std;

#pragma once

#define DATA_PIN 32
#define LASER_PIN 27
#define GAIN_PIN 33
#define LOAD_PIN 25

#define PACKET_DATA_SIZE 6

// Define optical interface clock frequency in Hz. Baud rate = frequency*2
#define FREQUENCY (150000)

// Signal pulse bounds
#define LOWER_DETECTABLE period/2 - period/16;
#define UPPER_DETECTABLE period/2 + period/16;
#define LOWER_VALID period/4 + period/16;
#define UPPER_VALID period/2 + (3*period)/16;

// Operational modes
#define OP_MODE_IDLE                (0) // Idle (default)
#define OP_MODE_TRANSMITTING        (1) // Tranmission mode
#define OP_MODE_RECEIVING           (2) // Receiving mode

// Receiver modes
#define REC_MODE_LISTENING          (0) // Seaching for beacon (default)
#define REC_MODE_ACKNOWLEDGE        (1) // Acknowledge beacon by streaming beacon back
#define REC_MODE_RECEIVING          (2) // Beacon detected, ready to receive packets

// Transmission modes/stages (in order)
#define MODE_IDLE                   (0) // Idle (default)
#define MODE_BEACON                 (1) // Stream square wave beacon
#define MODE_ACKNOWLEDGE            (2) // Wait for incoming beacon acknowledgement
#define MODE_STREAM                 (3) // Stream packet
#define MODE_STREAM_VERIFY          (4) // Listen for incoming verification pulse
#define MODE_COMPLETE               (5) // Stream completion message
#define MODE_COMPLETE_ACKNOWLEDGE   (6) // Wait for completion message acknowledgement

// Response types
#define RESPONSE_BEACON             (0x55)
#define RESPONSE_VERIFICATION       (0xCC)

// Packet sizing
#define PACKET_DATA_SIZE_BYTES      (512)                 
#define PACKET_WRAPPER_SIZE_BYTES   (256)   

// Packet pulsing
#define TRANS_DELAY_MS              (1000)
#define BEACON_TIMEOUT_MS           (500)
#define RESPONSE_TIMEOUT_MS         (10)

long pulse1, pulse2;

class opticalInterface {
  private: unsigned int packet_body_size = PACKET_DATA_SIZE;

  private: char flag_packet_header[7] PROGMEM       = "[flag]";
  private: char data_packet_header[14] PROGMEM      = "[data-header]";
  private: char checksum_packet_header[18] PROGMEM  = "[checksum-header]";
  private: char length_packet_header[9] PROGMEM     = "[length]";
  private: char packet_footer[9] PROGMEM            = "[footer]";

  private: uint8_t operational_mode = OP_MODE_IDLE;
  private: uint8_t transmission_mode = MODE_IDLE;
  private: uint8_t receiving_mode = REC_MODE_LISTENING;
  private: uint64_t completion = 0;

  private: double period = 1000000 / FREQUENCY;
  private: long baud = FREQUENCY * 2;

  private: double lower = LOWER_DETECTABLE;
  private: double upper = UPPER_DETECTABLE;
  private: double lower_valid = LOWER_VALID;
  private: double upper_valid = UPPER_VALID;

  private: uint16_t packet_count = 0;
  private: uint16_t packet_current = 0;
  private: uint16_t packet_completion = 0;

  private: size_t packet_buffer_size = (size_t) (PACKET_DATA_SIZE_BYTES + PACKET_WRAPPER_SIZE_BYTES);
  private: size_t data_buffer_size = (size_t) PACKET_DATA_SIZE_BYTES;
  private: uint8_t packet_buffer[(size_t) (PACKET_DATA_SIZE_BYTES + PACKET_WRAPPER_SIZE_BYTES)];
  private: uint8_t data_buffer[(size_t) PACKET_DATA_SIZE_BYTES];

  public: uint32_t outgoingBlockPointer;
  private: uint8_t _outgoingPacketFlag = 0;

  public: void initialize(dataManager &dataManager, uartInterface &portUart) {
    // Optical interface pins
    pinMode(DATA_PIN, INPUT);
    pinMode(LASER_PIN, OUTPUT);

    // Digital potentiometers
    pinMode(GAIN_PIN, OUTPUT);
    pinMode(LOAD_PIN, OUTPUT);

    this->outgoingBlockPointer = dataManager.outgoingBlockPointer;

    // Initialize optical interface
    Serial1.begin(this->baud, SERIAL_8N1, DATA_PIN, LASER_PIN, true);

    delay(100);
    
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

      Serial.print(PROGMEM "AGC has resolved optimum load (");
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

  private: bool expect(char pulse) {
    int sequence = 0;
    char last_char = pulse,
         read;

    Serial1.flush();

    long start = millis();

    while((start + RESPONSE_TIMEOUT_MS) > millis()) {
      if(Serial1.available()) {
        read = Serial1.read();

        if(read == pulse == last_char) {
          sequence++;
        }

        last_char = read;
      }

      if(sequence >= 10) {
        return true;
      }
    }

    return false;
  }

  public: void processIncoming(dataManager &dataManager, uartInterface &portUart) {
    if(this->operational_mode == OP_MODE_TRANSMITTING) {
      return;
    }

    if(this->receiving_mode == REC_MODE_LISTENING) {
      if(this->searchBeacon()) {
        this->operational_mode = OP_MODE_RECEIVING;
        this->receiving_mode = REC_MODE_ACKNOWLEDGE;
      }
    }

    if(this->operational_mode == OP_MODE_RECEIVING) {
      if(this->receiving_mode == REC_MODE_ACKNOWLEDGE) {
        this->emitBeacon();
        this->receiving_mode = REC_MODE_RECEIVING;
      }

      if(this->receiving_mode == REC_MODE_RECEIVING) {
        Serial1.flush();

        // TODO
      }
    }
  }

  private: bool dataAvailableForTransmission(dataManager &dataManager) {
    return dataManager.outgoingBytePointer > 0
        || dataManager.outgoingBlockPointer > dataManager.outgoingBlockStart;
  }

  private: bool activateTransmission(dataManager &dataManager) {
    if(this->buildDataPacket(dataManager)) {
      #ifdef DEBUG
      Serial.print("data_buffer: ");
      Serial.println((char*) this->data_buffer);
      #endif

      if(this->buildPacket(dataManager)) {
        #ifdef DEBUG
        Serial.print("packet_buffer: ");
        Serial.println((char*) this->packet_buffer);
        #endif

        this->operational_mode = OP_MODE_TRANSMITTING;
        this->transmission_mode = MODE_IDLE;

        return true;
      }
    }

    return false;
  }

  private: bool buildDataPacket(dataManager &dataManager) {
    uint32_t block;

    memset(this->data_buffer, 0, this->data_buffer_size);

    if(dataManager.outgoingBlockPointer > this->outgoingBlockPointer) {
      block = this->outgoingBlockPointer++;
      dataManager.copy(dataManager.returnOutgoingBlock(block), this->data_buffer, (int) 512);
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

  public: void processOutgoing(dataManager &dataManager, uartInterface &portUart) {
    if(this->operational_mode == OP_MODE_RECEIVING) {
      return;
    }

    if(this->operational_mode != OP_MODE_TRANSMITTING) {
      DATA_OP_BEGIN();

      if(portUart.data_available && (portUart.last_data_available + TRANS_DELAY_MS < millis() || millis() < portUart.last_data_available)) {
        if(this->dataAvailableForTransmission(dataManager)) {
          this->activateTransmission(dataManager);
        }
      }

      DATA_OP_END();
    }
    
    if(!this->isTransmissionActive()) {
      return;
    }

    switch (this->transmission_mode) {
      case MODE_IDLE:
        this->transmission_mode = MODE_BEACON;
      break;
    
      case MODE_BEACON:
        this->emitBeacon();
        this->transmission_mode = MODE_ACKNOWLEDGE;
      break;

      case MODE_ACKNOWLEDGE:
        this->transmission_mode = this->searchBeacon() ? MODE_STREAM : MODE_BEACON;
      break;

      case MODE_STREAM:
        Serial1.write((char*) this->packet_buffer);

        this->transmission_mode = MODE_STREAM_VERIFY;
      break;

      case MODE_STREAM_VERIFY:
        if(this->expect(RESPONSE_VERIFICATION)) {
          this->operational_mode = OP_MODE_IDLE;
          this->transmission_mode = MODE_IDLE;

        } else {
          this->transmission_mode = MODE_STREAM;

        }
      break;
    }
  }

  private: void emitBeacon() {
    for(int n=0; n<7000; n++) {
      Serial1.write(RESPONSE_BEACON);
    }
  }

  public: bool isTransmissionActive() {
    return this->operational_mode == OP_MODE_TRANSMITTING;
  }
};
