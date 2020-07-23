/*
 * Arduino sketch for reading data from serial port and writing it out to SPI.
 *
 * Serial transfer is at 115200 baud.
 *
 * Digital pin 8 acts as a "clear to send" (CTS) pin for writing SPI data. SPI
 * data is only written if CTS is high. If CTS is low, incoming serial data is
 * buffered.
 *
 * SPI data output is on the MOSI pin which is pin 11. SPI clock is on SCK which
 * is pin 13.
 *
 * We have no SS pin since the console is always listening out (i.e. it is
 * always selected).
 */
#include <SPI.h>

// CTS pin is pin 8
#define PIN_CTS     PINB
#define DDR_CTS     DDRB
#define CTS_PIN     0

void setup() {
  SPI.begin();
  Serial.begin(115200);

  // Usually, beginTransaction is paired with endTransaction. In this case we
  // never *stop* sending SPI data so we begin and never end a transaction.
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
}

void loop() {
  // A simple ring buffer implementation. This implementation is so simple that
  // we don't even check for overflow :(. We'd better hope the console is fast
  // enough to consume data as we send it.
  const int buf_log2_len = 3; // 2^3 = 8 bytes
  const int buf_len = 1 << buf_log2_len;
  static uint8_t buf[buf_len];
  static uint8_t buf_read_idx = 0, buf_write_idx = 0;

  if(Serial.available() && (((buf_write_idx+1) & (buf_len-1)) != buf_read_idx)) {
    buf[buf_write_idx] = Serial.read();
    buf_write_idx = (buf_write_idx+1) & (buf_len-1);
  }

  if((buf_read_idx != buf_write_idx) && (PIN_CTS & _BV(CTS_PIN))) {
    SPI.transfer(buf[buf_read_idx]);
    buf_read_idx = (buf_read_idx+1) & (buf_len-1);
  }
}
