#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "rom.h"

//#define DEBUG true

uint8_t ram[0x2000];

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define ADDR (((uint16_t)addrh << 8) | (uint16_t)addrl)

static uint8_t memRead() {
  const uint16_t addr = (((uint16_t)PINB << 8) | (uint16_t)PINA);
  uint8_t val;
  switch (PINB >> 4) {
  default:
    // nothing in the address space, just return zero.
    return 0x00;
  case 0x0:
    val = ram[addr];
#ifdef DEBUG
    printf("memRead: %04x: %02x\r", addr, val);
#endif
    return val;
  case 0xd:
    // fake 6821
    switch (PINA) {
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
#ifdef DEBUG
      printf("pia: invalid read at %04x\n", addr);
#endif
      val = 0;
      break;
    }
    return val;
  case 0xe:
    val = pgm_read_byte(erom + (addr - 0xe000));
    return val;
  case 0xf:
    val = pgm_read_byte(from + (addr - 0xf000));
    return val;
  }
  return 0x00;
}

static void memWrite() {
  const uint16_t addr = (((uint16_t)PINB << 8) | (uint16_t)PINA);
  const uint8_t val = PINC;
  switch (PINB >> 4) {
  default:
    // nothing in the address space ignore the write;
    break;
  case 0x0:
    ram[addr] = val;
#ifdef DEBUG
    printf("memWrite: %04x: %02x\r", addr, val);
#endif
    break;
  case 0xd:
    switch (PINA) {
    default:
#ifdef DEBUG
      printf("pia: invalid write at %04x: %02x\n", addr, val);
#endif
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
    if (bit_is_set(PIND,RW)) {
      // read cycle
      DDRC = 0xff;
      PORTC = memRead();
    } else {
      // write cycle
      DDRC = 0x00;
      PORTC = 0;
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
