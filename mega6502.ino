#include <Arduino.h>
#include <avr/pgmspace.h>
#include <LiquidCrystal.h>
#include "woz.h"

// configuration and debugging parameters
//
// set to false to disable LCD debugging
#define LCD false
// 2400 for the authentic experience, 9600 for something more enjoyable
#define BAUD 2400
// these constants won't change.  But you can change the size of
// your LCD using them:
const int numRows = 2;
const int numCols = 16;

#define CLOCK 53
#define RESET 52
#define RW 51

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Freetronics
LiquidCrystal lcd( 8, 9, 4, 5, 6, 7 );

byte ram[0x1000];
byte pia[0x14];

const byte hex[0x10] {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

void printAddr(const uint16_t addr, const uint8_t col, const uint8_t row) {
  lcd.setCursor(col, row);
  lcd.write(hex[(addr >> 12) & 0x0f]);
  lcd.write(hex[(addr >> 8) & 0x0f]);
  lcd.write(hex[(addr >> 4) & 0x0f]);
  lcd.write(hex[addr & 0x0f]);
}

void printByte(const uint8_t val, const uint8_t col, const uint8_t row) {
  lcd.setCursor(col, row);
  lcd.write(hex[(val >> 4) & 0x0f]);
  lcd.write(hex[val & 0x0f]);
}

const byte rom[] PROGMEM = {0xd8, 0x58, 0xa0, 0x7f, 0x8c, 0x12, 0xd0, 0xa9, 0xa7, 0x8d, 0x11, 0xd0, 0x8d, 0x13, 0xd0, 0xc9, 0xdf, 0xf0, 0x13, 0xc9, 0x9b, 0xf0, 0x3, 0xc8, 0x10, 0xf, 0xa9, 0xdc, 0x20, 0xef, 0xff, 0xa9, 0x8d, 0x20, 0xef, 0xff, 0xa0, 0x1, 0x88, 0x30, 0xf6, 0xad, 0x11, 0xd0, 0x10, 0xfb, 0xad, 0x10, 0xd0, 0x99, 0x0, 0x2, 0x20, 0xef, 0xff, 0xc9, 0x8d, 0xd0, 0xd4, 0xa0, 0xff, 0xa9, 0x0, 0xaa, 0xa, 0x85, 0x2b, 0xc8, 0xb9, 0x0, 0x2, 0xc9, 0x8d, 0xf0, 0xd4, 0xc9, 0xae, 0x90, 0xf4, 0xf0, 0xf0, 0xc9, 0xba, 0xf0, 0xeb, 0xc9, 0xd2, 0xf0, 0x3b, 0x86, 0x28, 0x86, 0x29, 0x84, 0x2a, 0xb9, 0x0, 0x2, 0x49, 0xb0, 0xc9, 0xa, 0x90, 0x6, 0x69, 0x88, 0xc9, 0xfa, 0x90, 0x11, 0xa, 0xa, 0xa, 0xa, 0xa2, 0x4, 0xa, 0x26, 0x28, 0x26, 0x29, 0xca, 0xd0, 0xf8, 0xc8, 0xd0, 0xe0, 0xc4, 0x2a, 0xf0, 0x97, 0x24, 0x2b, 0x50, 0x10, 0xa5, 0x28, 0x81, 0x26, 0xe6, 0x26, 0xd0, 0xb5, 0xe6, 0x27, 0x4c, 0x44, 0xff, 0x6c, 0x24, 0x0, 0x30, 0x2b, 0xa2, 0x2, 0xb5, 0x27, 0x95, 0x25, 0x95, 0x23, 0xca, 0xd0, 0xf7, 0xd0, 0x14, 0xa9, 0x8d, 0x20, 0xef, 0xff, 0xa5, 0x25, 0x20, 0xdc, 0xff, 0xa5, 0x24, 0x20, 0xdc, 0xff, 0xa9, 0xba, 0x20, 0xef, 0xff, 0xa9, 0xa0, 0x20, 0xef, 0xff, 0xa1, 0x24, 0x20, 0xdc, 0xff, 0x86, 0x2b, 0xa5, 0x24, 0xc5, 0x28, 0xa5, 0x25, 0xe5, 0x29, 0xb0, 0xc1, 0xe6, 0x24, 0xd0, 0x2, 0xe6, 0x25, 0xa5, 0x24, 0x29, 0x7, 0x10, 0xc8, 0x48, 0x4a, 0x4a, 0x4a, 0x4a, 0x20, 0xe5, 0xff, 0x68, 0x29, 0xf, 0x9, 0xb0, 0xc9, 0xba, 0x90, 0x2, 0x69, 0x6, 0x2c, 0x12, 0xd0, 0x30, 0xfb, 0x8d, 0x12, 0xd0, 0x60, 0x0, 0x0, 0x0, 0xf, 0x0, 0xff, 0x0, 0x0, };

extern byte ram[];
extern byte pia[];

#define ADDR ((addrh << 8) | addrl)

uint8_t memRead(const uint8_t addrh, const uint8_t addrl) {
  const uint16_t addr = ADDR;
  uint8_t val;
  switch (addr >> 12) {
    default:
      // nothing in the address space, just return zero.
      return 0x00;
    case 0x0:
      val = ram[addr];
      if (LCD) printAddr(addr, 0, 0);
      if (LCD) printByte(val, 5 , 0);
      return val;
    case 0xd:
      // fake 6821
      switch (addrl & 0xff) {
        case 0x10:
          val = 0x80 | (uint8_t)Serial.read();
          break;
        case 0x11:
          if (Serial.available()) {
            val = 0x80;
          } else {
            val = 0x00;
          }
          break;
        default:
          val = pia[addrl & 0xff];
      }
      if (LCD) printAddr(addr, 8, 0);
      if (LCD) printByte(val, 14, 0);
      return val;
    case 0xf:
      // monitor rom is mirrored at every page in $Fxxx
      val = pgm_read_byte(rom + addrl);
      if (LCD) printAddr(addr, 8, 0);
      if (LCD) printByte(val, 14, 0);
      return val;
  }
}

void memWrite(const uint8_t addrh, const uint8_t addrl, const uint8_t val) {
  const uint16_t addr = ADDR;
  switch (addr >> 12) {
    default:
      // nothing in the address space ignore the write;
      break;
    case 0x0:
      if (LCD) printAddr(addr, 0, 1);
      if (LCD) printByte(val, 5, 1);
      ram[addr] = val;
      break;
    case 0xd:
      if (LCD) printAddr(addr, 8, 1);
      if (LCD) printByte(val, 14, 1);
      switch (addr & 0xff) {
        default:
          pia[addr & 0xff] = val;
          break;
        case 0x12:
          Serial.write(val & 0x7f);
          if (val == ('\r' | 0x80))
            Serial.write('\n');
          break;
      }
      break;
  }
}

void setup() {
  // set up the LCD's number of columns and rows:
  if (LCD)
    lcd.begin(numCols, numRows);
  Serial.begin(BAUD);
  Serial.println("RESET");

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
    __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");
    sbi(PORTB, 0);
    sei();
    if (PINB & _BV(PB2)) {
      // read cycle
      DDRL = 0xff;
      PORTL = memRead(PINC, PINA);
    } else {
      // write cycle
      DDRL = 0x00;
      PORTL = 0;
      memWrite(PINC, PINA, PINL);
    }
  }
}
