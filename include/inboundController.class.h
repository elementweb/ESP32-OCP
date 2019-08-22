using namespace std;

// #include "uartInterface.class.h"
// #include "dataManager.class.h"
// #include "opticalInterface.class.h"

class inboundController {
  public: void run(uartInterface &portUart, dataManager &dataManager, opticalInterface &opticalInterface) {
    /**
     * Packetize data and module over optical beam
     */
    opticalInterface.processOutgoing(dataManager, portUart);

    /**
     * Listen for incoming data
     */
    opticalInterface.processIncoming(dataManager, portUart);

  }
};
