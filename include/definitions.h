#pragma once

#define CORE0         0
#define CORE1         1
#define _1KB          1000
#define _16KB         16000
#define _1MB          1000000
#define BLOCK_KB      512

#define _1KBw64       1088
#define _1MBw64       1048640

#define _64KB         64000
#define _64KBw64      64064

#define SPI_OP_START(); while(SPI_IN_USE);SPI_IN_USE=true;
#define SPI_OP_STOP(); SPI_IN_USE=false;
