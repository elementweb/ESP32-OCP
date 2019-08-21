// Toggle debug code: un-comment to enable
#define DEBUG 1

#include <Arduino.h>
#include <SdFat.h>
#include "includes.h"
#include "definitions.h"
#include "peripherals.h"
#include "dataManager.class.h"
#include "uartInterface.class.h"
#include "canInterface.class.h"
#include "opticalInterface.class.h"
#include "outboundController.class.h"
#include "inboundController.class.h"

TaskHandle_t outboundTaskHandler;
TaskHandle_t inboundTaskHandler;

outboundController outbound;
inboundController inbound;

uartInterface portUart;
canInterface portCan;
dataManager dataManagerObject;
opticalInterface opticalInterfaceObject;

SdFat uSD;

typedef uartInterface uartT;

// Software version, title
#define SOFTWARE_TITLE          PROGMEM "ESP32-OCP"
#define SOFTWARE_VERSION        PROGMEM "v1.0.1dev"

// Task manager memory allocations
#define OUTBOUND_STACK_DEPTH    (_1KB * 8)
#define INBOUND_STACK_DEPTH     (_1KB * 8)

void outboundTask(void * parameter) {
  // Log the core number that task is initialized on
  Serial.println(PROGMEM "outbound task initialized on core " + (String) xPortGetCoreID());
  ring(1, 5);
  delay(100);
  
  // Signalize beeKit that OCP is ready
  portUart.sendData((char*)"READY\n");

  while(true) {
    outbound.run(portUart, dataManagerObject, opticalInterfaceObject);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void inboundTask(void * parameter) {
  // Log the core number that task is initialized on
  Serial.println(PROGMEM "inbound task initialized on core " + (String) xPortGetCoreID());
  ring(1, 5);

  while(true) {
    inbound.run(portUart, dataManagerObject, opticalInterfaceObject);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // Run CPU at 240Mhz
  setCpuFrequencyMhz(240);

  // Initialize debug port
  Serial.begin(115200);

  // Initialize UART port to communicate with beeKit
  portUart.initialize();

  // Software version
  Serial.print(SOFTWARE_TITLE + (String) " ");
  Serial.println(SOFTWARE_VERSION);
  Serial.println("CPU running at " + (String) getCpuFrequencyMhz() + "MHz");

  // Initialize SPI
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  // Initialize peripheral methods
  initializePeripherals();

  // Initialize uSD card buffer
  dataManagerObject.initialize(uSD);

  // Initialize optical interface
  opticalInterfaceObject.initialize(dataManagerObject, portUart);

  // Initialize core tasks
  xTaskCreatePinnedToCore(outboundTask, "outbound_controller", OUTBOUND_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, &outboundTaskHandler, CORE0);
  delay(100);
  xTaskCreatePinnedToCore(inboundTask, "inbound_controller", INBOUND_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, &inboundTaskHandler, CORE1);
}

void loop() {
  // Intentionally left empty:
  // Tasks allocated to separate cores running infinite loops
}
