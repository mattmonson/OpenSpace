// This code comes from http://www.excamera.com/sphinx/article-crc.html
// It was generated by pycrc (http://www.tty1.net/pycrc/)

#include <avr/pgmspace.h>

uint32_t crc32(const uint8_t* data, size_t size);

uint32_t crc32_init();
uint32_t crc32_update(uint32_t crc, uint8_t data);
uint32_t crc32_finish(uint32_t crc);

