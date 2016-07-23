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


/*
  This isn't really an example, just some pointers to use the library on an STM32 platform with STM's Cube HAL.

  I used an STM32F072CBT6 microcontroller, with the MFRC630 connected to SPI2.
*/

/*
  You should have an SPI port enabled, the following settings worked for me, so this is somewhere at system start:
*/
// SPI_HandleTypeDef SpiHandle;
SpiHandle.Instance               = SPIx;
SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; // Ensure that it's not above 1MHz.
SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
SpiHandle.Init.CRCPolynomial     = 7;
SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
SpiHandle.Init.NSS               = SPI_NSS_SOFT;
SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
SpiHandle.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
SpiHandle.Init.CRCLength         = SPI_CRC_LENGTH_8BIT;


GPIO_InitStruct.Pin       = SPIx_NSS_PIN;
GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;


HAL_GPIO_Init(SPIx_NSS_GPIO_PORT, &GPIO_InitStruct);
HAL_GPIO_WritePin (SPIx_NSS_GPIO_PORT, SPIx_NSS_PIN, GPIO_PIN_SET);

// The other pins (SCK, MOSI, MISO) should be done by HAL_SPI_MspInit.

SpiHandle.Init.Mode = SPI_MODE_MASTER;
// init the SPI port.
if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
{
  /* Initialization Error */
  Error_Handler();
}



/*
  The the HAL functions should be defined, something like the following will work.
*/

void mfrc630_SPI_transfer(const uint8_t* tx, uint8_t* rx, uint16_t len){
  switch(HAL_SPI_TransmitReceive(&SpiHandle, tx, rx, len, 5000)){
      case HAL_OK:
        // Communication is completed, dont do anything.
        break;

      case HAL_TIMEOUT:
        // VCP_printf("Timeout\n");
      case HAL_ERROR:
        // VCP_printf("Some error\n");
        Error_Handler();
        break;
      default:
        break;
  }
}
void mfrc630_SPI_select(){
  HAL_GPIO_WritePin(SPIx_NSS_GPIO_PORT, SPIx_NSS_PIN, GPIO_PIN_RESET);
}
void mfrc630_SPI_unselect(){
  HAL_GPIO_WritePin(SPIx_NSS_GPIO_PORT, SPIx_NSS_PIN, GPIO_PIN_SET);
}

/*
  After this you can copy the function from the Arduino example, or use mfrc630_MF_example_dump() as a starting point.
*/
