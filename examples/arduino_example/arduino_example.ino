/*
  The MIT License (MIT)

  Copyright (c) 2016 Ivor Wanders

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <Arduino.h>
#include <SPI.h>
#include <mfrc630.h>

/*

  This example shows how to use the library on an Arduino-compatible platform, in this case the testing has been done
  on a Teensy 3.0.

  In the setup() function, there are some custom register settings which are you probably have to uncomment or change
  such that they are in the correct configuration for your hardware.
  The hardware I used had three switchable antenna's, so modification of there parameters is likely to get it to work.

  If all goes well, a card with the factory default key of 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF will print something like:

    UID of 4 bytes (SAK: 0): 95 84 E1 4B 
    Yay! We are authenticated!
    Read block 0x10 : 95 84 E1 4B BB 08 04 00 62 63 64 65 66 67 68 69 
    Read block 0x10 : 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
    Read block 0x10 : 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
    Read block 0x10 : 00 00 00 00 00 00 FF 07 80 69 FF FF FF FF FF FF 

  Onto the serial port.

*/


// Pin to select the hardware, the NSS pin.
#define CHIP_SELECT 10

// Pins MOSI, MISO and SCK are connected to the default pins, and are manipulated through the SPI object.
// By default that means MOSI=11, MISO=12, SCK=13.


// Implement the HAL functions on an Arduino compatible system.
void mfrc630_SPI_transfer(const uint8_t* tx, uint8_t* rx, uint16_t len) {
  for (uint16_t i=0; i < len; i++){
    rx[i] = SPI.transfer(tx[i]);
  }
}

// Select the chip and start an SPI transaction.
void mfrc630_SPI_select() {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));  // gain control of SPI bus
  digitalWrite(CHIP_SELECT, LOW);
}

// Unselect the chip and end the transaction.
void mfrc630_SPI_unselect() {
  digitalWrite(CHIP_SELECT, HIGH);
  SPI.endTransaction();    // release the SPI bus
}

// Hex print for blocks without printf.
void print_block(uint8_t * block,uint8_t length){
    for (uint8_t i=0; i<length; i++){
        if (block[i] < 16){
          Serial.print("0");
          Serial.print(block[i], HEX);
        } else {
          Serial.print(block[i], HEX);
        }
        Serial.print(" ");
    }
}

// The example dump function adapted such that it prints with Serial.print.
void mfrc630_MF_example_dump_arduino() {
  uint16_t atqa = mfrc630_iso14443a_REQA();
  if (atqa != 0) {  // Are there any cards that answered?
    uint8_t sak;
    uint8_t uid[10] = {0};  // uids are maximum of 10 bytes long.

    // Select the card and discover its uid.
    uint8_t uid_len = mfrc630_iso14443a_select(uid, &sak);

    if (uid_len != 0) {  // did we get an UID?
      Serial.print("UID of ");
      Serial.print(uid_len);
      Serial.print(" bytes (SAK: ");
      Serial.print(sak);
      Serial.print("): ");
      print_block(uid, uid_len);
      Serial.print("\n");

      // Use the manufacturer default key...
      uint8_t FFkey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

      mfrc630_cmd_load_key(FFkey);  // load into the key buffer

      // Try to athenticate block 0.
      if (mfrc630_MF_auth(uid, MFRC630_MF_AUTH_KEY_A, 0)) {
        Serial.println("Yay! We are authenticated!");

        // Attempt to read the first 4 blocks.
        uint8_t readbuf[16] = {0};
        uint8_t len;
        for (uint8_t b=0; b < 4 ; b++) {
          len = mfrc630_MF_read_block(b, readbuf);
          Serial.print("Read block 0x");
          print_block(&len,1);
          Serial.print(": ");
          print_block(readbuf, len);
          Serial.println();
        }
        mfrc630_MF_deauth();  // be sure to call this after an authentication!
      } else {
        Serial.print("Could not authenticate :(\n");
      }
    } else {
      Serial.print("Could not determine UID, perhaps some cards don't play");
      Serial.print(" well with the other cards? Or too many collisions?\n");
    }
  } else {
    Serial.print("No answer to REQA, no cards?\n");
  }
}


void setup(){
  // Start serial communication.
  Serial.begin(9600);

  // Set the chip select pin to output.
  pinMode(CHIP_SELECT, OUTPUT);

  // Start the SPI bus.
  SPI.begin();

  // Set the registers of the MFRC630 into the default.
  mfrc630_AN1102_recommended_registers(MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER);

  // This are register required for my development platform, you probably have to change (or uncomment) them.
  mfrc630_write_reg(0x28, 0x8E);
  mfrc630_write_reg(0x29, 0x15);
  mfrc630_write_reg(0x2A, 0x11);
  mfrc630_write_reg(0x2B, 0x06);
}

void loop(){
  // call the above function until infinity.
  mfrc630_MF_example_dump_arduino();
  delay(1000);
}
