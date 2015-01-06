#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "rom.h"

//#define DEBUG true

uint8_t ram[0x2000];

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define ADDR (((uint16_t)addrh << 8) | (uint16_t)PINA)

static void memRead() {
  DDRC = 0xff;
  const uint8_t addrh = PINB;
  if (addrh & 0x80) {
    switch (addrh >> 4) {
    case 0xd:
      // fake 6821
      switch (PINA) {
      case 0x10:
        PORTC = 0x80 | UDR0;
        break;
      case 0x11:
        PORTC = UCSR0A & _BV(RXC0);
        break;
      case 0x12:
        PORTC = UCSR0A & _BV(UDRE0) ? 0x00 : 0xFF;
        break;
      default:
#ifdef DEBUG
        printf("pia: invalid read at %04x\n", ADDR);
#endif
        PORTC = 0;
      }
      break;
    case 0xe:
      PORTC = pgm_read_byte(
          erom + ((((uint16_t)addrh << 8) | (uint16_t)PINA) - 0xe000));
      break;
    case 0xf:
      PORTC = pgm_read_byte(
          from + ((((uint16_t)addrh << 8) | (uint16_t)PINA) - 0xf000));
      break;
    }
  } else {
    PORTC = ram[(((uint16_t)addrh << 8) | (uint16_t)PINA)];
#ifdef DEBUG
    printf("memRead: %04x: %02x\r", ADDR, val);
#endif
  }
}

static void memWrite() {
  DDRC = 0x00;
  PORTC = 0;
  const uint8_t addrh = PINB;
  const uint8_t addrl = PINA;
  if (addrh == 0xd0) {
    if (addrl == 0x12)
      putchar(PINC & 0x7f);
  } else {
    const uint16_t addr = (((uint16_t)addrh << 8) | (uint16_t)addrl);
    ram[addr] = PINC;
#ifdef DEBUG
    printf("memWrite: %04x: %02x\r", addr, PINC);
#endif
  }
}

void uart_init(void);

#define CLOCK 5
#define RESET 6
#define RW 7

void setup() {
  uart_init();
  printf("%s\n", "RESET");

  // setup PORTA to read the low byte of the address
  DDRA = 0x00;
  PORTA = 0x00;
  // setup PORTB to do the same for the high byte
  DDRB = 0x00;
  PORTB = 0x00;

  // setup PORTC for the data bus
  DDRC = 0x00;
  PORTC = 0x00;

  // setup service ports
  DDRD |= _BV(CLOCK) | _BV(RESET);
  cbi(DDRD, RW);
  cbi(PORTD, RW);
  sbi(PORTD, RESET);

  // send reset pulse
  cbi(PORTD, RESET);
  uint8_t i;
  for (i = 0; i < 6; i++) {
    sbi(PORTD, CLOCK);
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    cbi(PORTD, CLOCK);
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"); // 4 * 62.5ns delay @ 16mhz
  }
  sbi(PORTD, RESET);
}

void delay(uint16_t ms) {
  for (; ms > 0; ms--) {
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"); // 4 * 62.5ns delay @ 16mhz
  }
}

void loop() {
  while (1) {
    cli();
    cbi(PORTD, CLOCK);
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    sbi(PORTD, CLOCK);
    sei();
    // must pause for at two clocks after raising CLOCK to allow PIND:RW to
    // latch correctly.
    __asm__("nop\n\t"
            "nop\n\t"
            "nop\n\t"
            "nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    if (bit_is_set(PIND, RW)) {
      memRead();
    } else {
      memWrite();
    }
#ifdef DEBUG
    // printf("%02x%02x: %02x %02x\r", PINB, PINA, PINC, PIND
    // &(_BV(RW)|_BV(CLOCK)|_BV(RESET)));
    delay(10000);
#endif
  }
}

int main() {
  setup();
  while (1)
    loop();
}
