/*
 * TV-out console. Based on some aspects of the Arduino TV-out library at
 * https://github.com/Avamander/arduino-tvout.
 *
 * THIS CODE IS VERY, VERY EXPERIMENTAL.
 *
 * Listens over hardware SPI for incoming 8-bit characters and provides a simple
 * ASCII terminal over composite video.
 *
 * SS (digital "10") must be tied low for hardware SPI to work.
 * SCK (digital "13") and MOSI (digital "11") should be connected to
 * corresponding pins on the serial to SPI translation Arduino.
 *
 * PB0 (digital "8") is used as a "clear to send" signal. This is low during
 * critical sections of the render loop when the Ardunio is not free to consume
 * incoming SPI data. The SPI terminal should avoid sending new data while CTS
 * is low.
 */
#include <math.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/cpufunc.h>

#include "video_timing.h"
#include "font.h"

#define TV_MODE_PAL 0
#define TV_MODE_NTSC 1
#define TV_MODE TV_MODE_PAL

// SYNC     is OC1A (digital "9" on Nano)
// VIDEO    is TX and *must* be inverted
// CTS      is PB0 (digital "10" on Nano)

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)
#define PORT_SYNC   PORTB
#define DDR_SYNC    DDRB
#define SYNC_PIN    1

#define PORT_VIDEO  PORTD
#define DDR_VIDEO   DDRD
#define VIDEO_PIN   1
#else
# error "No pin definitions for this microcontroller specified."
#endif

#define PORT_CTS    PORTB
#define DDR_CTS     DDRB
#define CTS_PIN     0

// Render state
static int display_scan_line;
static uint16_t display_frame_count;

#if TV_MODE == TV_MODE_PAL
// Configuration for PAL
const int display_active_line_count = 240;
const int display_vsync_end = _PAL_LINE_STOP_VSYNC;
const int display_lines_per_frame= _PAL_LINE_FRAME;
const int display_start_active_line = _PAL_LINE_MID -
  (display_active_line_count>>1) + 8;
const int display_overscan_cycles = 64;
const int display_start_cycle = _PAL_CYCLES_OUTPUT_START +
  display_overscan_cycles;
#elif TV_MODE == TV_MODE_NTSC
// Configuration for NTSC
const int display_active_line_count = 224;
const int display_vsync_end = _NTSC_LINE_STOP_VSYNC;
const int display_lines_per_frame= _NTSC_LINE_FRAME;
const int display_start_active_line = _NTSC_LINE_MID -
  (display_active_line_count>>1) + 8;
const int display_overscan_cycles = 48;
const int display_start_cycle = _NTSC_CYCLES_OUTPUT_START +
  display_overscan_cycles;
#else
# error "TV_MODE must be set"
#endif

const int display_chars_per_row = 40;
const int display_rows_per_frame = display_active_line_count >> 3;

const int input_ring_buf_log2_len = 7;
const int input_ring_buf_len = 1<<input_ring_buf_log2_len;
uint8_t input_ring_buf[input_ring_buf_len];
uint8_t input_ring_buf_write_idx = 0;
uint8_t input_ring_buf_read_idx = 0;

uint8_t char_buf[display_chars_per_row * display_rows_per_frame];

static uint16_t cursor_idx = 0;
const uint16_t char_buf_len = display_chars_per_row*display_rows_per_frame;

void (*line_handler)();

void vsync_line();
void blank_line();
void active_line();

static void inline wait_until(uint8_t time) {
  __asm__ __volatile__ (
      "subi %[time], 10\n"
      "sub  %[time], %[tcnt1l]\n\t"
    "100:\n\t"
      "subi %[time], 3\n\t"
      "brcc 100b\n\t"
      "subi %[time], 0-3\n\t"
      "breq 101f\n\t"
      "dec  %[time]\n\t"
      "breq 102f\n\t"
      "rjmp 102f\n"
    "101:\n\t"
      "nop\n" 
    "102:\n"
    :
    : [time] "a" (time),
    [tcnt1l] "a" (TCNT1L)
  );
}

ISR(SPI_STC_vect) {
  uint8_t next_idx = (
      (input_ring_buf_write_idx + 1) & (input_ring_buf_len-1)
  );
  if(next_idx != input_ring_buf_read_idx) {
    input_ring_buf[input_ring_buf_write_idx] = SPDR;
    input_ring_buf_write_idx = next_idx;
  }
}

static inline void send_usart(uint8_t v) {
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = v;
}

void vsync_line() {
  if (display_scan_line >= display_lines_per_frame) {
    OCR1A = _CYCLES_VIRT_SYNC;
    display_scan_line = 0;
    display_frame_count++;
  } else if (display_scan_line == display_vsync_end) {
    OCR1A = _CYCLES_HORZ_SYNC;
    line_handler = &blank_line;
  }

  display_scan_line++;
}

void blank_line() {
  if(display_scan_line == display_start_active_line) {
    line_handler = &active_line;
    PORT_CTS &= ~_BV(CTS_PIN);
  } else if (display_scan_line == display_lines_per_frame) {
    line_handler = &vsync_line;
  }

  display_scan_line++;
}

void active_line() {
  uint8_t disp_line = (display_scan_line - display_start_active_line - 1);
  uintptr_t font = (font_data + ((disp_line & 0x7)<<8));
  uint8_t *p = char_buf + (display_chars_per_row * (disp_line >> 3));

  wait_until(display_start_cycle);

  for(uint8_t x=display_chars_per_row; x; --x, ++p) {
    // Can use bitwise OR since font is aligned to 256 byte boundary.
    UDR0 = pgm_read_byte(font | *p);

    // We don't have enough cycles left to check for the UDRE0 bit in UCSR0A so
    // just nop it out for the right period. This entire look should really be
    // replaced with a cycle-counted asm implementation(!)
    _NOP();
    _NOP();
    _NOP();
  }

  if(display_scan_line == display_start_active_line + display_active_line_count) {
    line_handler = &blank_line;
  }

  display_scan_line++;
}

void setup_video_signals() {
  display_frame_count = 0;

  // Set SYNC pin as output and initially high.
  DDR_SYNC |= _BV(SYNC_PIN);
  PORT_SYNC |= _BV(SYNC_PIN);

  // Set VIDEO as output and initially high
  DDR_VIDEO |= _BV(VIDEO_PIN);
  PORT_VIDEO |= _BV(VIDEO_PIN);

  // Inverted fast PWM on timer 1
  TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);

  // Configure SYNC timer
# if TV_MODE == TV_MODE_PAL
  ICR1 = _PAL_CYCLES_SCANLINE;
# elif TV_MODE == TV_MODE_NTSC
  ICR1 = _NTSC_CYCLES_SCANLINE;
# else
# error "TV_MODE must be set"
# endif
  OCR1A = _CYCLES_HORZ_SYNC;

  // Master SPI USART0
  UCSR0C |= _BV(UMSEL01) | _BV(UMSEL00);
  UCSR0C &= ~_BV(UDORD0);
  UBRR0 = 0;
  UCSR0B |= _BV(TXEN0);

  // Configure line interrupt handler and enable interrupts.
  display_scan_line = display_lines_per_frame + 1;
  line_handler = &vsync_line;
  TIMSK1 = _BV(TOIE1);

  sei();
}

// Interrupt handler called once per line
ISR(TIMER1_OVF_vect) {
  // Set CTS low during the line handler so that serial terminal does not
  // attempt to send us new data in a critical section.
  PORT_CTS &= ~_BV(CTS_PIN);
  line_handler();
  PORT_CTS |= _BV(CTS_PIN);
}

void display_clear() {
  memset(char_buf, '\0', char_buf_len);
  cursor_idx = 0;
}

void display_scroll_up() {
  memmove(
    char_buf, char_buf+display_chars_per_row,
    char_buf_len-display_chars_per_row
  );
  memset(
    char_buf+char_buf_len-display_chars_per_row,
    '\0', display_chars_per_row
  );
}

void display_putc(uint8_t c) {
  if(c == '\r') {
    // Carriage return
    cursor_idx = cursor_idx - (cursor_idx % display_chars_per_row);
  } else if(c == '\n') {
    // New line
    cursor_idx += display_chars_per_row;
  } else if(c == 0xc) {
    // New page
    display_clear();
    cursor_idx = 0;
  } else if(c == 0x8) {
    // Backspace
    if(cursor_idx != 0) { --cursor_idx; }
  } else {
    char_buf[cursor_idx] = c;
    ++cursor_idx;
  }

  if(cursor_idx >= char_buf_len) {
    display_scroll_up();
    cursor_idx -= display_chars_per_row;
  }
}

static bool cursor_is_on = false;
static uint8_t cursor_cache = '\0';

void cursor_off() {
  if(cursor_is_on) {
    char_buf[cursor_idx] = cursor_cache;
    cursor_is_on = false;
  }
}

void cursor_on() {
  if(!cursor_is_on) {
    cursor_cache = char_buf[cursor_idx];
    char_buf[cursor_idx] = 219;
    cursor_is_on = true;
  }
}

void setup() {
  DDR_CTS |= _BV(CTS_PIN);
  PORT_CTS |= _BV(CTS_PIN);

  SPCR = _BV(SPE) | _BV(SPIE);
  display_clear();
  setup_video_signals();
}

void loop() {
  while(input_ring_buf_write_idx != input_ring_buf_read_idx) {
    cursor_off();
    display_putc(input_ring_buf[input_ring_buf_read_idx]);
    input_ring_buf_read_idx = (
        (input_ring_buf_read_idx + 1) & (input_ring_buf_len-1)
    );
  }

  if(display_frame_count & 0x10) {
    cursor_on();
  } else {
    cursor_off();
  }
}
