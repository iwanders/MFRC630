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


  This example shows how to use the library on Arduino platform, it was tested with Arduino Nano


  In the setup() function, there are some custom register settings which are you probably have to uncomment or change
  such that they are in the correct configuration for your hardware.
  The hardware I used had three switchable antenna's, so modification of there parameters is likely to get it to work.

  If all goes well, a ISO15693 Tag/Card will print something like:

 	 Tag with valid UID was found! UID: UID:00 02 F4 CF 31 1E 66 24 16 E0  ..waiting 1s for next read


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


void mfrc630_ISO15693_example_dump_arduino(){
	uint8_t uid[10]={0};	//variable for 10byte UID
	uint8_t status = mfrc630_ISO15693_readTag(uid);

	if(status==10){
		Serial.print("Tag with valid UID was found! UID: ");
		print_block(uid,10);
		Serial.println(" ..waiting 1s for next read");
	}
	else{
		Serial.println("Failure, No Tag found or Reader Problem!");
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
  mfrc630_AN1102_recommended_registers(MFRC630_PROTO_ISO15693_1_OF_4_SSC);

}

void loop(){
  // call the above function until infinity.
  mfrc630_ISO15693_example_dump_arduino();
  delay(1000);
}
