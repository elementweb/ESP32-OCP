bool _spi_in_use = false;
bool _data_manager_in_operation = false;

void print_uint64_t(HardwareSerial HS, uint64_t num) {
  char rev[128]; 
  char *p = rev+1;
  bool not_null = false;

  while (num > 0) {
    *p++ = '0' + (num % 10);
    num/= 10;
  }

  p--;

  while (p > rev) {
    HS.print(*p--);
    not_null = true;
  }
  
  if(!not_null) {
    HS.print(0);
  }
}