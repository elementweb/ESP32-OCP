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

// Packet sizing
#define PACKET_DATA_SIZE_BYTES      (512)                 
#define PACKET_WRAPPER_SIZE_BYTES   (256)                 

long pulse1, pulse2;

class opticalInterface {
  private: unsigned int packet_body_size = PACKET_DATA_SIZE;

  private: char data_packet_header[14] PROGMEM      = "[data-header]";
  private: char checksum_packet_header[18] PROGMEM  = "[checksum-header]";
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

  private: uint8_t packet_buffer[(size_t) (PACKET_DATA_SIZE_BYTES + PACKET_WRAPPER_SIZE_BYTES)];
  private: uint8_t data_buffer[(size_t) PACKET_DATA_SIZE_BYTES];

  public: uint32_t outgoingBlockPointer;

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

  public: void AGC() {
    int long_gain = 0;
    int long_load = 0;
    int long_pulse = 1;

    int pulse_count = 0;
    long start, end;
    int e, i;
    bool pulse;

    int load_increments = FREQUENCY >= 250000 ? map(FREQUENCY, 250000, 400000, 4, 1) : 5;

    start = millis();

    for(e=0; e<=255; e+=load_increments) { // for frequencies >250kHz increments should be 1
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

  private: bool listen() {
    bool signal = this->detectValidPulse();

    long start = millis();

    while(!signal && (start + 500) > millis()) {
      this->AGC();

      signal = this->detectValidPulse();
    }

    return signal;
  }

  public: void processIncoming(dataManager &dataManager) {
    if(this->operational_mode == OP_MODE_TRANSMITTING) {
      return;
    }

    if(this->receiving_mode == REC_MODE_LISTENING) {
      if(this->listen()) {
        this->operational_mode = OP_MODE_RECEIVING;
        this->receiving_mode = REC_MODE_ACKNOWLEDGE;
      }
    }

    if(this->operational_mode == OP_MODE_RECEIVING) {
      if(this->receiving_mode == REC_MODE_ACKNOWLEDGE) {
        this->beacon();
        this->receiving_mode = REC_MODE_RECEIVING;
      }

      if(this->receiving_mode == REC_MODE_RECEIVING) {
        // listen on Serial1 here
      }
    }
  }

  private: bool dataAvailableForTransmission(dataManager &dataManager) {
    return dataManager.outgoingBytePointer > 0
        || dataManager.outgoingBlockPointer > dataManager.outgoingBlockStart;
  }

  private: void activateTransmission(dataManager &dataManager) {
    if(this->buildDataPacket(dataManager)) {
      // this->buildPacket(dataManager)
      Serial.println((char*) this->data_buffer);
    }

    this->operational_mode = OP_MODE_TRANSMITTING;
    this->transmission_mode = MODE_IDLE;
  }

  private: bool buildDataPacket(dataManager &dataManager) {
    uint32_t block;

    memset(this->data_buffer, 0, (size_t) 512);

    if(dataManager.outgoingBlockPointer > this->outgoingBlockPointer) {
      block = this->outgoingBlockPointer++;
      dataManager.copy(dataManager.returnOutgoingBlock(block), this->data_buffer, (int) 512);

      return true;
    }

    if(dataManager.outgoingBytePointer > 0) {
      dataManager.copy(dataManager.returnOutgoingBlock(block), this->data_buffer, (int) 512);
      dataManager.frontBufferFlush();

      return true;
    }

    return false;
  }

  public: void processOutgoing(dataManager &dataManager, uartInterface &portUart) {
    if(this->operational_mode == OP_MODE_RECEIVING) {
      return;
    }

    if(portUart.data_available && (portUart.last_data_available + 500 < millis() || millis() < portUart.last_data_available)) {
      if(this->dataAvailableForTransmission(dataManager)) {
        this->activateTransmission(dataManager);
      }
    }

    




    bool packet_verified;

    if(!this->isTransmissionActive()) {
      return;
    }

    switch (this->transmission_mode) {
      case MODE_IDLE:
        this->transmission_mode = MODE_BEACON;
      break;
    
      case MODE_BEACON:
        this->beacon();
        this->transmission_mode = MODE_ACKNOWLEDGE;
      break;

      case MODE_ACKNOWLEDGE:
        this->transmission_mode = this->listen() ? MODE_STREAM : MODE_BEACON;
      break;

      case MODE_STREAM:
        // build packet if not built yet
        if(this->packet_current == this->packet_completion) {
          // this->buildPacket(dataManager, ++this->packet_current);
        }

        // Serial1.write((char*) this->packet_buffer);

        this->transmission_mode = MODE_STREAM_VERIFY;
      break;

      case MODE_STREAM_VERIFY:
        packet_verified = false;

        // RECEIVING VERIFICATION CHARACTER PULSE?
        // ALL LOGIC GOES HERE.

        this->transmission_mode = MODE_STREAM;

        if(packet_verified) {
          this->packet_completion++;
          this->transmission_mode = this->packet_completion == this->packet_count ? MODE_COMPLETE : MODE_STREAM;         
        }
      break;

      case MODE_COMPLETE:
        //
      break;

      case MODE_COMPLETE_ACKNOWLEDGE:
        //
      break;
    }
  }

  private: void beacon() {
    for(int n=0; n<7000; n++) {
      Serial1.write(0x55);
    }
  }

  public: bool isTransmissionActive() {
    return this->operational_mode == OP_MODE_TRANSMITTING;
  }

  public: bool startTransmission(dataManager &dataManager) {
    if(this->isTransmissionActive()) {
      return false;
    }

    if(this->operational_mode != OP_MODE_IDLE) {
      Serial.println(PROGMEM "Interface is not idling. Transmission won't start.");
      
      return false;
    }

    // this->packet_count = this->packetCount(dataManager);

    if(this->packet_count <= 0) {
      Serial.println(PROGMEM "No data have been loaded into the buffer. Transmission won't start.");

      return false;
    }

    this->packet_current = 0;
    this->packet_completion = 0;

    Serial.println(PROGMEM "Packet count: " + (String) this->packet_count);

    this->operational_mode = OP_MODE_TRANSMITTING;
    this->transmission_mode = MODE_IDLE;

    Serial.println(PROGMEM "Data transmission started.");

    return true;
  }
};
