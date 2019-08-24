using namespace std;

#include "uartInterface.class.h"
#include "dataManager.class.h"
#include "opticalInterface.class.h"

class outboundController {
  public: void run(uartInterface &portUart, dataManager &dataManager, opticalInterface &opticalInterface) {
    /**
     * Process incoming UART data
     */
    portUart.processOutgoingData(dataManager);

    /**
     * Optical interface housekeeping activities
     */
    opticalInterface.outboundHousekeepingRunner();

  }
};
