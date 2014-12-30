#include <Arduino.h>
#include <avr/pgmspace.h>
// 2400 for the authentic experience, 9600 for something more enjoyable
#define BAUD 2400
#include <util/setbaud.h>
#include "rom.h"

#define CLOCK 53
#define RESET 52
#define RW 51

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

byte ram[0x1000];
byte pia[0x14];

#define ADDR ((addrh << 8) | addrl)

static uint8_t memRead() {
  const uint8_t addrh = PINC;
  const uint8_t addrl = PINA;
  const uint16_t addr = ADDR;
  uint8_t val;
  switch (addr >> 12) {
    default:
      // nothing in the address space, just return zero.
      return 0x00;
    case 0x0:
      val = ram[addr];
      return val;
    case 0xd:
      // fake 6821
      switch (addrl) {
        case 0x10:
          val = 0x80 | UDR0; 
          break;
        case 0x11:
          val = UCSR0A & _BV(RXC0);
          break;
        case 0x12:
          val = UCSR0A & _BV(UDRE0) ? 0x00 : 0xFF;
          break;
        default:
          printf("pia: invalid read at %04x\n", addr);
          val = 0;
          break;
      }
      return val;
    case 0xe:
      val = pgm_read_byte(erom + (addr & 0x0fff));
      return val;
    case 0xf:
      val = pgm_read_byte(from + (addr & 0x0fff));
      return val;
  }
}

static void memWrite() {
  const uint8_t addrh = PINC;
  const uint8_t addrl = PINA;
  const uint16_t addr = ADDR;
  const uint8_t val = PINL;
  switch (addr >> 12) {
    default:
      // nothing in the address space ignore the write;
      break;
    case 0x0:
      ram[addr] = val;
      break;
    case 0xd:
      switch (addrl) {
        default:
          printf("pia: invalid write at %04x: %02x\n", addr, val);
          break;
        case 0x12:
          putchar(val & 0x7f);
          break;
        case 0x11:
          // ignore write to $D011, PIA is not configurable.
        case 0x13:
          // ignore write to $D013, PIA is not configurable.
          break;
      }
      break;
  }
}

int uart_putchar(char c, FILE *stream) {
 if (c == '\r') {
    uart_putchar('\n', stream);
  }
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
  return 1;
}

static FILE uartout = {0} ;

void uart_init(void) {
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;

#if USE_2X
  UCSR0A |= _BV(U2X0);
#else
  UCSR0A &= ~(_BV(U2X0));
#endif

  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout ;
}

void setup() {
  uart_init();
  printf("RESET\n");

  pinMode(CLOCK, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(RW, INPUT);
  digitalWrite(RESET, HIGH);

  // setup PORTC to read the low byte of the address
  DDRC = 0x00;
  PORTC = 0x00;
  // setup PORTA to do the same for the high byte
  DDRA = 0x00;
  PORTA = 0x00;

  // send reset pulse
  digitalWrite(RESET, LOW);
  digitalWrite(CLOCK, HIGH);
  digitalWrite(CLOCK, LOW);
  digitalWrite(CLOCK, HIGH);
  digitalWrite(CLOCK, LOW);
  digitalWrite(CLOCK, HIGH);
  digitalWrite(CLOCK, LOW);
  digitalWrite(CLOCK, HIGH);
  digitalWrite(CLOCK, LOW);
  digitalWrite(CLOCK, HIGH);
  digitalWrite(CLOCK, LOW);
  digitalWrite(CLOCK, HIGH);
  digitalWrite(RESET, HIGH);
}

void loop() {
  while (true) {
    cli();
    cbi(PORTB, 0);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    sbi(PORTB, 0);
    sei();
    if (PINB & _BV(PB2)) {
      // read cycle
      DDRL = 0xff;
      PORTL = memRead();
    } else {
      // write cycle
      DDRL = 0x00;
      PORTL = 0;
      memWrite();
    }
  }
}
