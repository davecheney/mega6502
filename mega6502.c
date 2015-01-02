#include <stdio.h>
// 2400 for the authentic experience, 9600 for something more enjoyable
#define BAUD 2400
#include <util/setbaud.h>
#include <avr/pgmspace.h>
#include "rom.h"

//#define DEBUG true

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

uint8_t ram[0x2000];

#define ADDR (((uint16_t)addrh << 8) | (uint16_t)addrl)

static uint8_t memRead() {
  const uint8_t addrh = PINB;
  const uint8_t addrl = PINA;
  const uint16_t addr = (((uint16_t)addrh << 8) | (uint16_t)addrl);
  uint8_t val;
  switch (addr >> 12) {
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
      //val = pgm_read_byte(from + (addr - 0xf000));
      val = pgm_read_byte(wozrom + addrl);
      return val;
  }
  return 0x00;
}

static void memWrite() {
  const uint8_t addrh = PINB;
  const uint8_t addrl = PINA;
  const uint16_t addr = (((uint16_t)addrh << 8) | (uint16_t)addrl);
  const uint8_t val = PINC;
  switch (addr >> 12) {
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
      switch (addrl) {
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

int uart_putchar(char c, FILE *stream) {
 if (c == '\r') {
    uart_putchar('\n', stream);
  }
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;
  return 0;
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
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    cbi(PORTD, CLOCK);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); // 4 * 62.5ns delay @ 16mhz
  }
  sbi(PORTD, RESET);
}

void delay(uint16_t ms) {
    for (;ms > 0; ms--) {
	__asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); // 4 * 62.5ns delay @ 16mhz
	__asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); // 4 * 62.5ns delay @ 16mhz
	__asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); // 4 * 62.5ns delay @ 16mhz
	__asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    }
}

void loop() {
  while (1) {
    __asm__("cli\n\t");
    cbi(PORTD, CLOCK);
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    sbi(PORTD, CLOCK);
    __asm__("sei\n\t");
    // must pause for at two clocks after raising CLOCK to allow PIND:RW to latch correctly.
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t"); // 4 * 62.5ns delay @ 16mhz
    if (PIND & _BV(RW)) {
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
    //printf("%02x%02x: %02x %02x\r", PINB, PINA, PINC, PIND &(_BV(RW)|_BV(CLOCK)|_BV(RESET)));
    delay(10000);
    #endif
  }
}

int main() {
	setup();
	while(1) 
		loop();
}
