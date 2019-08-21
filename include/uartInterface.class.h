using namespace std;

#pragma once

#define UART_PORT_BAUD        (115200)

class dataManager;

class uartInterface {
  public: bool data_available = true;
  public: unsigned long last_data_available = 0;

  public: void initialize() {
    size_t uart_depth = 8192;

    Serial2.begin(UART_PORT_BAUD);
    Serial2.setRxBufferSize(uart_depth);

    delay(100);
  }

  public: void sendData(const char* s) {
    Serial2.write(s);
  }

  public: void processOutgoingData(dataManager &dataManager) {
    bool data_available = false;

    while(Serial2.available()) {
      DATA_OP_BEGIN();
      dataManager.outgoingBufferPush(Serial2.read());
      DATA_OP_END();

      data_available = true;

      delayMicroseconds(100);
    }

    if(data_available) {
      ring(1, 1);

      this->data_available = true;
      this->last_data_available = millis();

      #ifdef DEBUG
      dataManager.reportOutgoingBufferStats();
      #endif
    }
  }
  
};
