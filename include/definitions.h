#pragma once

#define CORE0               0
#define CORE1               1

#define _1KB                1000
#define _16KB               16000
#define _1MB                1000000
#define BLOCK_KB            512
#define _1KBw64             1088
#define _1MBw64             1048640
#define _64KB               64000
#define _64KBw64            64064

#define SPI_OP_BEGIN();     while(_spi_in_use){delayMicroseconds(100);}_spi_in_use=true;
#define SPI_OP_END();       _spi_in_use=false;

#define DATA_OP_BEGIN();    while(_data_manager_in_operation){delayMicroseconds(100);}_data_manager_in_operation=true;
#define DATA_OP_END();      _data_manager_in_operation=false;
#define WAIT_DATA_OP_END(); while(_data_manager_in_operation);

HardwareSerial opticalLink(1);
HardwareSerial platformInterface(2);
