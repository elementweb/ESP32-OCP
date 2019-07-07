using namespace std;

#include "dataManager.class.h"

#pragma once

#define PACKET_DATA_SIZE 6

class opticalInterface {
  private: bool tranmsission_active = false;

  private: unsigned int packet_body_size = PACKET_DATA_SIZE;

  private: char data_packet_header[14] = PROGMEM "[data-header]";
  private: char checksum_packet_header[18] = PROGMEM "[checksum-header]";
  private: char packet_footer[9] = PROGMEM "[footer]";

  private: uint64_t completion = 0;

  public: void packetizeAndTransmit(dataManager &dataManager) {
    if(!this->isActive()) {
      return;
    }

    //
  }

  private: char * packet(dataManager &dataManager, unsigned int number) {
    const char *packet;

    if(number > this->packetCount(dataManager)) {
      return 0x00;
    }

    return (char *)packet;
  }

  public: unsigned int packetCount(dataManager &dataManager) {
    float packets = float(dataManager.bufferLength()) / float(this->packet_body_size);

    return ceil(packets);
  }

  public: bool isActive() {
    return this->tranmsission_active;
  }

  public: void activate() {
    this->tranmsission_active = true;
  }

  public: void deactivate() {
    this->tranmsission_active = false;
  }
};
