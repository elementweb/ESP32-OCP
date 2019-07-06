using namespace std;

#include "uartInterface.class.h"
#include "dataManager.class.h"

class outboundController {
  public: void loop(uartInterface &portUart, dataManager &dataManager) {
    /**
     * Process incoming UART data
     */
    portUart.processData(dataManager);

    /**
     * Check for incomming commands
     */
    portUart.processCommands(dataManager);

    /**
     * Packetize data and module over optical beam
     */
    // optical_transmit();

  }
};
