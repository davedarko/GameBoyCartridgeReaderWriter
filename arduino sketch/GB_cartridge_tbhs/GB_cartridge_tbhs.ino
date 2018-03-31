// inspired by https://github.com/insidegadgets/GBCartRead
// I took the liberty to get some ideas like the define PIN toggle stuff,
// also RAM and ROM bank switching was written more elegant than I could bash into an arduino
//

#include <math.h>
#include "hadLogo.h"

// PORTC lower OUT
// PORTD higher OUT
// PORTF DATA
// GB_RD E1
// GB_WR E0 // also called mreqPin in insideGadgets Version
// GB_RST E7

#define start_switch 19 // B0 lila
#define analyze_switch 23 // B4 yellow
#define led_blue 24 // B5
#define led_red 25 // B6
#define led_green 26 // B7
#define GB_WR 0 //E0
#define GB_RD 1 //E1
#define GB_RST 27 //E7
#define CS_SRAM 18 //E6
#define GB_AUD 47 //E5


#define wrPin_high    PORTE |= (1<<PE0);
#define wrPin_low     PORTE &= ~(1<<PE0);
#define rdPin_high    PORTE |= (1<<PE1);
#define rdPin_low     PORTE &= ~(1<<PE1);
#define GB_AUD_high  PORTE |= (1<<PE5);
#define GB_AUD_low   PORTE &= ~(1<<PE5);
#define mreqPin_high  PORTE |= (1<<PE6);
#define mreqPin_low   PORTE &= ~(1<<PE6);
#define reset_high  PORTE |= (1<<PE7);
#define reset_low   PORTE &= ~(1<<PE7);

long start_time = 0;
long restartTIME = 0;

char buff[64];
int mbc_cartridge = 0;

uint8_t flunk = 0;

uint16_t cartridgeType = 0;
uint16_t romSize = 0;
uint16_t romBanks = 0;
uint16_t ramSize = 0;
uint16_t ramBanks = 0;
uint16_t ramEndAddress = 0;

void setup() {

  restartTIME = millis();
  
  pinMode(start_switch, INPUT_PULLUP);
  pinMode(analyze_switch, INPUT_PULLUP);

  pinMode(led_blue, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);

  digitalWrite(led_blue, HIGH);
  digitalWrite(led_red, HIGH);
  digitalWrite(led_green, HIGH);

  pinMode(GB_WR, OUTPUT);
  pinMode(GB_RD, OUTPUT);
  pinMode(CS_SRAM, OUTPUT);
  pinMode(GB_RST, OUTPUT);
  pinMode(GB_AUD, OUTPUT);


  digitalWrite(GB_WR, HIGH);
  digitalWrite(GB_RD, HIGH);
  digitalWrite(GB_RST, HIGH);
  digitalWrite(CS_SRAM, HIGH);

  digitalWrite(led_green, LOW);

  Serial.begin(115200);
  delay(1000);
  Serial.println("ddarko GBcartridge");

  // PORTC lower OUT
  PORTC = 0b00000000; // set port values
  DDRC = 0b11111111;  // set port direction to output

  // PORTD higher OUT
  PORTD = 0b00000000; // set port values
  DDRD = 0b11111111;  // set port direction to output

  // PORTF DATA
  PORTF = 0b00000000; // set port to no pullups
  DDRF = 0b00000000;  // set port direction to input

}
// 1000
uint16_t input_addr = 0x0000;
uint8_t inByte = 0;

void loop() {
  // put your main code here, to run repeatedly:
  // http://sean.voisen.org/blog/2011/10/breathing-led-with-arduino/

  
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == 'a')
    {
      /// Serial.write("hey, ho, let");
      digitalWrite(led_red, LOW);
      digitalWrite(led_green, HIGH);
      analyze_cartridge(false);
      digitalWrite(led_red, HIGH);
      digitalWrite(led_green, LOW);
    }
    if (c == 'r')
    {
      Serial.println("Reading");
      analyze_cartridge(true);
      read_ROM();
      Serial.println("done");
    }
    if (c == 'w')
    {
      Serial.println("deleting EEPROM");
      deleteEEPROM();

      analogWrite(led_green, 127);
      analogWrite(led_red, 127);
      digitalWrite(led_blue, HIGH);

      Serial.println("writing EEPROM");
      for (int i = 0; i < 3631; i++)
      {
        GB_write_byte_audio(i, game[i]);
        uint8_t in = GB_read_byte(i);

        if (game[i] != in)
        {
          Serial.print(i);
          Serial.print(": ");
          Serial.print(game[i]);
          Serial.print(" != ");
          Serial.print(in);
          Serial.println();
        }
        if (i % 100 == 0)
        {
          Serial.print('.');
        }
      }
      Serial.println("done writing to EEPROM");

      analogWrite(led_blue, 127);
      digitalWrite(led_red, HIGH);
      digitalWrite(led_green, HIGH);
    }
  }
}

void deleteEEPROM ()
{
  digitalWrite(led_blue, HIGH);
  digitalWrite(led_green, HIGH);
  analogWrite(led_red, 127);

  // delete
  input_addr = 0x0000;
  GB_write_byte_audio(0x5555, 0xAA);
  GB_write_byte_audio(0x2AAA, 0x55);
  GB_write_byte_audio(0x5555, 0x80);
  GB_write_byte_audio(0x5555, 0xAA);
  GB_write_byte_audio(0x2AAA, 0x55);
  GB_write_byte_audio(0x5555, 0x10);
  delay(2000);

  digitalWrite(led_green, HIGH);
  digitalWrite(led_red, HIGH);
  analogWrite(led_blue, 127);
}
void read_ROM()
{
  Serial.write("ROM_START");
  Serial.flush();
  rd_wr_mreq_reset();
  uint16_t romAddress = 0;

  // Read number of banks and switch banks
  for (uint16_t bank = 1; bank < romBanks; bank++) {
    if (cartridgeType >= 5) { // MBC2 and above
      GB_write_byte(0x2100, bank); // Set ROM bank
    }
    else { // MBC1
      GB_write_byte(0x6000, 0); // Set ROM Mode
      GB_write_byte(0x4000, bank >> 5); // Set bits 5 & 6 (01100000) of ROM bank
      GB_write_byte(0x2000, bank & 0x1F); // Set bits 0 & 4 (00011111) of ROM bank
    }
    if (bank > 1) {
      romAddress = 0x4000;
    }

    // Read up to 7FFF per bank
    while (romAddress <= 0x7FFF) {
      uint8_t readData[64];
      for (uint8_t i = 0; i < 64; i++) {
        readData[i] = GB_read_byte(romAddress + i);
      }

      Serial.write(readData, 64); // Send the 64 byte chunk
      romAddress += 64;
    }
  }
  Serial.write("ROM_END");
}

void analyze_cartridge(bool silent)
{

  cartridgeType = GB_read_byte(0x0147);
  romSize = GB_read_byte(0x0148);
  ramSize = GB_read_byte(0x0149);

  // ROM banks
  romBanks = 2; // Default 32K
  if (romSize >= 1) { // Calculate rom size
    romBanks = 2 << romSize;
  }

  // RAM banks
  ramBanks = 0; // Default 0K RAM
  if (cartridgeType == 6) {
    ramBanks = 1;
  }
  if (ramSize == 2) {
    ramBanks = 1;
  }
  if (ramSize == 3) {
    ramBanks = 4;
  }
  if (ramSize == 4) {
    ramBanks = 16;
  }
  if (ramSize == 5) {
    ramBanks = 8;
  }

  // RAM end address
  if (cartridgeType == 6) {
    ramEndAddress = 0xA1FF;  // MBC2 512bytes (nibbles)
  }
  if (ramSize == 1) {
    ramEndAddress = 0xA7FF;  // 2K RAM
  }
  if (ramSize > 1) {
    ramEndAddress = 0xBFFF;  // 8K RAM
  }

  if (!silent)
  {
    Serial.print("Game Title: ");
    for (uint16_t i = 0x0134; i < 0x013F; i++)
    {
      uint8_t a = GB_read_byte(i);
      char c = (char) a;
      Serial.print(c);
    }
    // readCartridge(0x0134, 0x013E, false);
    Serial.println();

    Serial.print("Cartridge Type: ");
    Serial.print(cartridgeType, HEX);
    Serial.print(" - ");
    printCartridgeType(cartridgeType);

    Serial.print("ROM Size: ");
    Serial.print(romSize, HEX);
    Serial.print(" - ");
    printRomSize(romSize);

    Serial.print("RAM Size: ");
    Serial.print(ramSize, HEX);
    Serial.print(" - ");
    printRamSize(ramSize);

    Serial.println();
    // NINTENDO LOGO
    // CE ED 66 66 CC 0D 00 0B 03 73 00 83 00 0C 00 0D
    // 00 08 11 1F 88 89 00 0E DC CC 6E E6 DD DD D9 99
    // BB BB 67 63 6E 0E EC CC DD DC 99 9F BB B9 33 3E
    //
    // 0100-0103 - Entry Point
    // 0104-0133 - Nintendo Logo
    // 0134-0143 - Title
    // 013F-0142 - Manufacturer Code
    // 0143 - CGB Flag
    // 0144-0145 - New Licensee Code
    // 0146 - SGB Flag
    // 0147 - Cartridge Type
    // 0148 - ROM Size
    // 0149 - RAM Size
    // 014A - Destination Code
    // 014B - Old Licensee Code
    // 014C - Mask ROM Version number
    // 014D - Header Checksum
    // 014E-014F - Global Checksum
  }
}

// Turn RD, WR and MREQ to high so they are deselected (reset state)
void rd_wr_mreq_reset(void) {
  rdPin_high; // RD off
  wrPin_high; // WR off
  mreqPin_high; // MREQ off
  reset_high;
}

// Turn RD, WR and MREQ off as no power should be applied to GB Cart
void rd_wr_mreq_off(void) {
  rdPin_low;
  wrPin_low;
  mreqPin_low;
  reset_low;
}

uint8_t GB_read_byte(int addr)
{
  int high = addr >> 8;
  int low = addr & 0x00FF;

  PORTD = high;
  PORTC = low;

  // neat "Makros" copied from inside Gadgets
  //mreqPin_low; // gets ignored when in ROM mode
  rdPin_low;
  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");

  uint8_t bPortF = PINF;

  rdPin_high;
  //mreqPin_high; // gets ignored when in ROM mode

  return bPortF;
}

uint8_t GB_write_byte(uint16_t addr, uint8_t data)
{
  int high = addr >> 8;
  int low = addr & 0x00FF;

  // PORTF DATA
  DDRF = 0b11111111;  // set port direction to output
  // PORTF = 0b00000000;

  PORTD = high;
  PORTC = low;

  // set outputs to the data variable
  PORTF = data;

  // Pulse WR
  wrPin_low;
  asm volatile("nop");
  wrPin_high;

  uint8_t bPortF;
  bPortF = PINF;

  // Set pins as inputs
  DDRF = 0b00000000;  // set port direction to input
}

uint8_t GB_write_byte_audio(uint16_t addr, uint8_t data)
{

  int high = addr >> 8;
  high &= ~(1 << 8); // make sure it's low so !CE is enabled
  int low = addr & 0x00FF;

  DDRF = 0b11111111;  // set port direction to output

  PORTD = high;
  PORTC = low;
  PORTF = data;
  // Pulse WR
  GB_AUD_low;
  asm volatile("nop");
  GB_AUD_high;

  asm volatile("nop");
  asm volatile("nop");
  asm volatile("nop");

  //  uint8_t bPortF;
  //  bPortF = PINF;
  //  DDRF = 0b00000000;  // set port direction to input
  delay(20);
}
//
//uint8_t GB_write_byte_audio(uint16_t addr, uint8_t data)
//{
//  int high = addr>>8;
//  int low = addr & 0x00FF;
//
//  DDRF = 0b11111111;
//
//  PORTD = high;
//  PORTC = low;
//
//  // set outputs to the data variable
//  PORTF = data;
//
//  // Pulse WR
//  GB_AUD_low;
//  asm volatile("nop"); // 8MHz nop equals 125ns
//  asm volatile("nop"); // 8MHz nop equals 125ns
//  asm volatile("nop"); // 8MHz nop equals 125ns
//  asm volatile("nop"); // 8MHz nop equals 125ns
//  GB_AUD_high;
//  asm volatile("nop"); // 8MHz nop equals 125ns
//  asm volatile("nop"); // 8MHz nop equals 125ns
//  asm volatile("nop"); // 8MHz nop equals 125ns
//  asm volatile("nop"); // 8MHz nop equals 125ns
//
//  DDRF = 0b00000000;  // set port direction to input
//  delayMicroseconds(200);
//}

void printCartridgeType(uint8_t type)
{
  if (type == 0x00) Serial.println(F("ROM ONLY"));
  if (type == 0x01) Serial.println(F("MBC1"));
  if (type == 0x02) Serial.println(F("MBC1+RAM"));
  if (type == 0x03) Serial.println(F("MBC1+RAM+BATTERY"));
  if (type == 0x05) Serial.println(F("MBC2"));
  if (type == 0x06) Serial.println(F("MBC2+BATTERY"));
  if (type == 0x08) Serial.println(F("ROM+RAM"));
  if (type == 0x09) Serial.println(F("ROM+RAM+BATTERY"));
  if (type == 0x0B) Serial.println(F("MMM01"));
  if (type == 0x0C) Serial.println(F("MMM01+RAM"));
  if (type == 0x0D) Serial.println(F("MMM01+RAM+BATTERY"));
  if (type == 0x0F) Serial.println(F("MBC3+TIMER+BATTERY"));
  if (type == 0x10) Serial.println(F("MBC3+TIMER+RAM+BATTERY"));
  if (type == 0x11) Serial.println(F("MBC3"));
  if (type == 0x12) Serial.println(F("MBC3+RAM"));
  if (type == 0x13) Serial.println(F("MBC3+RAM+BATTERY"));
  if (type == 0x15) Serial.println(F("MBC4"));
  if (type == 0x16) Serial.println(F("MBC4+RAM"));
  if (type == 0x17) Serial.println(F("MBC4+RAM+BATTERY"));
  if (type == 0x19) Serial.println(F("MBC5"));
  if (type == 0x1A) Serial.println(F("MBC5+RAM"));
  if (type == 0x1B) Serial.println(F("MBC5+RAM+BATTERY"));
  if (type == 0x1C) Serial.println(F("MBC5+RUMBLE"));
  if (type == 0x1D) Serial.println(F("MBC5+RUMBLE+RAM"));
  if (type == 0x1E) Serial.println(F("MBC5+RUMBLE+RAM+BATTERY"));
  if (type == 0xFC) Serial.println(F("POCKET CAMERA"));
  if (type == 0xFD) Serial.println(F("BANDAI TAMA5"));
  if (type == 0xFE) Serial.println(F("HuC3"));
  if (type == 0xFF) Serial.println(F("HuC1+RAM+BATTERY"));
}

void printRomSize(uint8_t Rsize)
{
  if (Rsize == 0x00) Serial.println(F("32KByte (no ROM banking)"));
  if (Rsize == 0x01) Serial.println(F("64KByte (4 banks)"));
  if (Rsize == 0x02) Serial.println(F("128KByte (8 banks)"));
  if (Rsize == 0x03) Serial.println(F("256KByte (16 banks)"));
  if (Rsize == 0x04) Serial.println(F("512KByte (32 banks)"));
  if (Rsize == 0x05) Serial.println(F("1MByte (64 banks) - only 63 banks used by MBC1"));
  if (Rsize == 0x06) Serial.println(F("2MByte (128 banks) - only 125 banks used by MBC1"));
  if (Rsize == 0x07) Serial.println(F("4MByte (256 banks)")) ;
  if (Rsize == 0x08) Serial.println(F("8MByte (512 banks)")) ;
  if (Rsize == 0x52) Serial.println(F("1.1MByte (72 banks)"));
  if (Rsize == 0x53) Serial.println(F("1.2MByte (80 banks)"));
  if (Rsize == 0x54) Serial.println(F("1.5MByte (96 banks)"));
}

void printRamSize(uint8_t Rsize)
{
  if (Rsize == 0x00) Serial.println(F("None"));
  if (Rsize == 0x01) Serial.println(F("2 KBytes (1 Bank)"));
  if (Rsize == 0x02) Serial.println(F("8 Kbytes (1 Bank)"));
  if (Rsize == 0x03) Serial.println(F("32 KBytes (4 banks of 8KBytes each)"));
  if (Rsize == 0x04) Serial.println(F("128 KBytes (16 Banks)"));
  if (Rsize == 0x05) Serial.println(F("64 KBytes (8 banks of 8KBytes each)"));
}
