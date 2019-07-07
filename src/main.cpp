#include <Arduino.h>
#include <esp32-hal-uart.c>
#include "definitions.h"
#include "uartInterface.class.h"
#include "opticalInterface.class.h"
#include "outboundController.class.h"
#include "inboundController.class.h"
#include "dataManager.class.h"

TaskHandle_t outboundTaskHandler;
TaskHandle_t inboundTaskHandler;

outboundController outbound;
inboundController inbound;

uartInterface portUart;
dataManager dataManagerObject;
opticalInterface opticalInterfaceObject;

#define SOFTWARE_TITLE          PROGMEM "ESP32-OCP"
#define SOFTWARE_VERSION        PROGMEM "v1.0.1dev"
#define OUTBOUND_STACK_DEPTH    (_1KB * 8)
#define INBOUND_STACK_DEPTH     (_1KB * 8)

void outboundTask(void * parameter) {
  while(1) {
    // Serial.print("outbound core ");
    // Serial.println(xPortGetCoreID());

    outbound.run(portUart, dataManagerObject, opticalInterfaceObject);

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void inboundTask(void * parameter) {
  while(1) {
    // Serial.print("inbound core ");
    // Serial.println(xPortGetCoreID());

    inbound.run();

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);

  portUart.initialize();

  xTaskCreatePinnedToCore(outboundTask, "outbound_controller", OUTBOUND_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, &outboundTaskHandler, CORE0);
  xTaskCreatePinnedToCore(inboundTask, "inbound_controller", INBOUND_STACK_DEPTH, NULL, configMAX_PRIORITIES - 1, &inboundTaskHandler, CORE1);

  Serial.print(SOFTWARE_TITLE + (String) " ");
  Serial.println(SOFTWARE_VERSION);
  Serial.println(PROGMEM "Ready.");
}

void loop() {
  //
}
