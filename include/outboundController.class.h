using namespace std;

#include "uartInterface.class.h"
#include "dataManager.class.h"
#include "opticalInterface.class.h"

class outboundController {
  public: void run(uartInterface &portUart, dataManager &dataManager, opticalInterface &opticalInterface) {
    /**
     * Process incoming UART data
     */
    portUart.processData(dataManager);

    /**
     * Check for incomming commands
     */
    portUart.processCommands(dataManager, opticalInterface);

    /**
     * Packetize data and module over optical beam
     */
    opticalInterface.packetizeAndTransmit(dataManager);

  }
};
