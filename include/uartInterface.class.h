using namespace std;

#pragma once

#define UART_PORT_BAUD        (460800)
#define UART_PORT_DEPTH       (4096)

class uartInterface {
  public: bool data_available = false;
  public: unsigned long last_data_available = 0;

  public: void initialize() {
    size_t uart_depth = UART_PORT_DEPTH;

    platformInterface.begin(UART_PORT_BAUD);
    platformInterface.setRxBufferSize(uart_depth);

    delay(100);
  }

  public: void sendData(const char* s) {
    platformInterface.write(s);
  }

  public: void flush() {
    while(platformInterface.available() > 0) {
      platformInterface.read();
    }
  }

  public: void processOutgoingData(dataManager &dataManager) {
    bool _data_available = false;

    while(platformInterface.available()) {
      DATA_OP_BEGIN();
      dataManager.outgoingBufferPush(platformInterface.read());
      DATA_OP_END();

      _data_available = true;
    }

    if(_data_available) {
      this->data_available = this->data_available ? true : _data_available;
      this->last_data_available = millis();

      #ifdef DEBUG
      // dataManager.reportOutgoingBufferStats();
      #endif
    }
  }
  
};
