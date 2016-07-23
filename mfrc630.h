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

#ifndef MFRC630_H_
#define MFRC630_H_
#include <stdint.h>
#include "mfrc630_def.h"

#ifdef __cplusplus
extern "C" {
#endif

// debug print statement.
#ifdef MFRC630_DEBUG_PRINTF
#define MFRC630_PRINTF(...) MFRC630_DEBUG_PRINTF(__VA_ARGS__)
#else
#define MFRC630_PRINTF(...)
#endif

// ---------------------------------------------------------------------------
// HAL functions.
// ---------------------------------------------------------------------------
/*! \defgroup hal Hardware Abstraction Layer
    \brief Functions that interact with the SPI hardware.

    These functions should provide all the neccessary interaction with the hardware. Only the SPI bus is used;
    two functions to select and deselect the chip and a function to transmit and receive bytes over the SPI bus.

    No interrupt pins are required, nor is a delay function.

    \note These functions are marked as `extern` and it is up to the user to ensure that they exist and work correctly.
  @{
*/

/*!
  @brief Transmit and receive bytes via SPI.
  
    This function should provide the main SPI transfer functionality. SPI always reads a byte at the same time as it
    transmits a byte, the function should read from the `tx` array to transmit this data over MOSI and it should write
    the data received via MISO to the `rx` array.


  \param [in] len This is the number of bytes to be transfered.
  \param [in] tx The bytes from this array are transmitted, `len` bytes are always read from this argument. (MOSI)
  \param [out] rx The bytes received during transmission are written into this array, `len` bytes are always written.
               (MISO)
 */
extern void mfrc630_SPI_transfer(const uint8_t* tx, uint8_t* rx, uint16_t len);

/*!
  @brief Selects the MFRC630 to accept data from the SPI bus.
  
    This function should set the Chip Select (NSS) line to the appropriate level such that the chip acceps the data
    on the SPI bus. For the MFRC630 this means setting the Chip Select line to a LOW logic level.

 */
extern void mfrc630_SPI_select();

/*!
  @brief Deselects the MFRC630 to stop accepting data from the SPI bus.
  
    This function should set the Chip Select (NSS) line to the appropriate level such that the chip does not accept the
    data on the SPI bus. For the MFRC630 this means setting the Chip Select line to a HIGH logic level.

 */
extern void mfrc630_SPI_unselect();


//! @}
// ---------------------------------------------------------------------------
// SPI interface functions.
// ---------------------------------------------------------------------------

/*! \defgroup interface Interface
    \brief Manipulate the chip's registers.

    These functions use the SPI communication functions to interact with the hardware. All interaction with the chip
    goes through these functions. In case another interface than SPI is to be used, these functions can be adapted.
  @{
*/

/*!
  @brief Reads a register.
  
  \param [in] reg Specifies which register to read.
  \return the value of the register to be read.
 */
uint8_t mfrc630_read_reg(uint8_t reg);

/*!
  @brief Write a register.
  
  Sets a single register to the provided value.
  \param [in] reg Specifies which register to write.
  \param [in] value Specifies the value to write to that register.
 */
void mfrc630_write_reg(uint8_t reg, uint8_t value);


/*!
  @brief Write multiple registers.
  
  Sets consecutive registers to the provided values.
  \param [in] reg Specifies at which register writing starts.
  \param [in] values An array of the values to write to the register starting from `reg`.
              The first value (`values[0]`) gets written to `reg`, the second (`values[1]`) to `reg+1`, and so on.
  \param [in] len The number of register to write.
 */
void mfrc630_write_regs(uint8_t reg, const uint8_t* values, uint8_t len);

/*!
  @brief Write data to FIFO.
  
  The FIFO is located at register `MFRC630_REG_FIFODATA`. 
  Writes to this register do not automatically increment the write pointer in the chip and multiple bytes may be written
  to this register to place them into the FIFO buffer.

  This function does not clear the FIFO beforehand, it only provides the raw transfer functionality.
  \param [in] data The data to be written into the FIFO.
  \param [in] len The number of bytes to be written into the FIFO.
 */

void mfrc630_write_fifo(const uint8_t* data, uint16_t len);

/*!
  @brief Read data from FIFO.
  
  This function reads data from the FIFO into an array on the microcontroller.

  \param [out] rx The data read from the FIFO is placed into this array.
  \param [in] len The number of bytes to be read from the FIFO.

  \warning This reads regardless of MFRC630_REG_FIFOLENGTH, if there aren't enough bytes present in the FIFO, they are
           read from the chip anyway, these bytes should not be used. (The returned bytes from an empty FIFO are often
           identical to the last valid byte that was read from it.)
 */
void mfrc630_read_fifo(uint8_t* rx, uint16_t len);
//! @}

// ---------------------------------------------------------------------------
// Command functions
// ---------------------------------------------------------------------------

/*! \defgroup commands Commands
    \brief These activate the various commands of the chip with the right arguments.

    The chip has various commands it can execute, these commands often have arguments which should be transferred to
    the FIFO before the command is initiated. These functions provide this functionality of transferring the arguments
    to the FIFO and then initating the commands.
  @{
*/

/*!
  @brief Read data from EEPROM into the FIFO buffer.
  
  This instruction transfers data from the EEPROM (section 2) at the given address locaction into the FIFO buffer.

  \param [in] address The start address in the EEPROM to start reading from.
  \param [in] length The number of bytes to read from the EEPROM into the FIFO buffer.
 */
void mfrc630_cmd_read_E2(uint16_t address, uint16_t length);

/*!
  @brief Read data from EEPROM into the registers.
  
  This instruction transfers data from the EEPROM (section 2) at the provided address into the registers..

  \param [in] address The start address in the EEPROM to start reading from.
  \param [in] regaddr The start address of the register to start writing into.
  \param [in] length The number of bytes to read and registers to write consecutively from `regaddr`.
 */
void mfrc630_cmd_load_reg(uint16_t address, uint8_t regaddr, uint16_t length);


// Loads a protocol according to the protocol definitions. See protocol defines
// in the _def.h file.
// rx: The index for the rx protocol values.
// tx: The index for the tx protocol values.
void mfrc630_cmd_load_protocol(uint8_t rx, uint8_t tx);

// Transmit the data and enter receive mode after transmission.
// data: pointer to the data to be transmitted.
// len: the number of bytes to be read from this pointer and transmitted.
void mfrc630_cmd_transceive(const uint8_t* data, uint16_t len);

// Set the device into idle mode.
void mfrc630_cmd_idle();

// Load a key from the EEPROM into the key buffer.
// key_nr: Specify which key to load into the key buffer.
void mfrc630_cmd_load_key_E2(uint8_t key_nr);

// Load a key into the keybuffer.
// key: Pointer to the 6 byte key that is to be loaded into the keybuffer.
void mfrc630_cmd_load_key(const uint8_t* key);

// Performs the MIFARE standard authentication.
// block_address: The block number on which to authenticate.
// key_type: The MIFARE key A or B (0x60 or 0x61) to use.
// block_address: The block number on which to authenticate.
// card_uid: The first four bytes of the UID are used.
void mfrc630_cmd_auth(uint8_t key_type, uint8_t block_address, const uint8_t* card_uid);
//! @}
// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

// Flush the FIFO.
void mfrc630_flush_fifo();

// Returns the length of the fifo.
uint16_t mfrc630_fifo_length();

// Interrupt functions, these are used so often it's useful to have a function.
void mfrc630_clear_irq0(); // clear irq0
void mfrc630_clear_irq1(); // clear irq1
uint8_t mfrc630_irq0();  // retrieve irq0
uint8_t mfrc630_irq1();  // retrieve irq0

// Debug function, it prints n bytes from data in hex format.
void mfrc630_print_block(const uint8_t* data, uint16_t len);

/*!
  @brief Copy a page from EEPROM into an array on the MCU.
  
  This instruction transfers a page from the EEPROM into the FIFO and then transfers this data from the FIFO
  into an array. It always transfers 64 bytes, as such `dest` must be (atleast) 64 bytes long.

  This basically calls mfrc630_cmd_read_E2() and then transfers the FIFO with mfrc630_read_fifo().

  \param [out] dest The array to write the data into.
  \param [in] page The page to read from the EEPROM. (This gets multiplied by 64 to obtain the start address).
  \return The number of bytes transmitted from the FIFO into `dest`.
 */
uint8_t mfrc630_transfer_E2_page(uint8_t* dest, uint8_t page);

// ---------------------------------------------------------------------------
// Timer functions
// ---------------------------------------------------------------------------

// The timers can be treated uniformly, so making functions for them makes
// sense. The first argument 'timer' specifies which timer to use (0-4).

// Control whether a timer is running without affecting the other timers.
// Seems to trigger timer reload?
void mfrc630_activate_timer(uint8_t timer, uint8_t active);

// Set the timer control field of the timer.
// value: the value to set the timer's control field to.
void mfrc630_timer_set_control(uint8_t timer, uint8_t value);
// MFRC630_TCONTROL_CLK_211KHZ, results in 4.52e-06 ~ 5 uSec ticks.
// MFRC630_TCONTROL_CLK_13MHZ, results in 7.69e-08 ~ 76 nSec ticks.

// Set the reload value of a timer, it counts down from here.
void mfrc630_timer_set_reload(uint8_t timer, uint16_t value);

// Set the value of a timer, it counts downwards.
void mfrc630_timer_set_value(uint8_t timer, uint16_t value);
uint16_t mfrc630_timer_get_value(uint8_t timer);
// From datasheet;
// If the counter value has reached a value of 0000h and the interrupts are
// enabled for this specific timer, an interrupt will be generated as soon as
// the next clock is received.
// So an underflow IRQ is reached when the timers reaches 0.

// Timer4 is special and has different fields for the control!


// ---------------------------------------------------------------------------
// From documentation
// ---------------------------------------------------------------------------

// From Application Note 11145:
//      CLRC663, MFRC631, MFRC 630, SLRC610 Low Power Card Detection
//      http://www.nxp.com/documents/application_note/AN11145.pdf

//      IQ measurement, section 3.2.1
void mfrc630_AN11145_start_IQ_measurement();
//      wait about 50ms between them.
void mfrc630_AN11145_stop_IQ_measurement();
//      then retrieve them with 
//      mfrc630_read_reg(MFRC630_REG_LPCD_I_RESULT) & 0x3F
//      mfrc630_read_reg(MFRC630_REG_LPCD_Q_RESULT) & 0x3F

// From Application Note 11022:
//      CLRC663 Quickstart Guide
//      http://www.nxp.com/documents/application_note/AN11022.pdf

// Set registers to recommended values for a certain protocol.
// protocol: Specify which protocol to configure the default settings for.
void mfrc630_AN1102_recommended_registers(uint8_t protocol);
// Only copied the following protocols:
//      MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER
//      MFRC630_PROTO_ISO14443A_212_MILLER_BPSK
//      MFRC630_PROTO_ISO14443A_424_MILLER_BPSK
//      MFRC630_PROTO_ISO14443A_848_MILLER_BPSK
//      There are more in the Application Note!

// Set registers to recommended values for a certain protocol, skipping those
// before the MFRC630_REG_TXCRCPRESET register.
// This can be useful because those before that register require hardware
// dependent customization.
void mfrc630_AN1102_recommended_registers_no_transmitter(uint8_t protocol);

// Sets recommended registers, skipping an arbitrary amount at the start of the
// specification.
void mfrc630_AN1102_recommended_registers_skip(uint8_t protocol, uint8_t skip);
// This actually is used by the methods above.

// ---------------------------------------------------------------------------
// ISO 14443A
// ---------------------------------------------------------------------------

// Perform an REQA transmission, response is returned.
// This instruction modifies the registers.
// Returns 0 on failure.
uint16_t mfrc630_iso14443a_REQA();

// Perform the SELECT procedure to determine the UID.
// This instruction modifies the registers.
// uid: The UID that is selected and discovered is written to this pointer.
// sak: The last SAK byte retrieved for this UID is stored here.
// returns: the length of the uid in bytes, or 0 in case of failure.
uint8_t mfrc630_iso14443a_select(uint8_t* uid, uint8_t* sak);
/*
  The select procedure is explained quite complex in the ISO norm.
  Conceptually it is not quite hard though...
    - The cascade level can be seen as a prefix to ensure both the PICC and PCD
      are working on identifying the same part of the UID.
    - Then, the entire anti-collision scheme is more of a binary search, the
      PICC sends the CASCADE level prefix, then the NVB byte, this field deter-
      mines how many bits of the UID will follow, this allows the PICC's to
      listen to this and respond if their UID's match these first bits with the
      UID that is transmitted. After this all PICC's (that have matched the UID
      bits already sent) respond with the remainder of their UIDS. This results
      in either a complete UID, or in case two PICC's share a few bits but then
      differ a bit, a collision occurs on this bit. This collision is detected
      by the PCD, at which point it can chose either to pursue the PICC(s) that
      has a 0b1 at that position, or pursue the 0b0 at that position.
      The ISO norm states: A typical implementation adds a (1)b. I use the
      bit value that's in the pointer at the same position as the collision, or
      atleast for the first cascade level that works, after that it's off by a
      byte because of the cascade tag, see the actual implementation.
  Returns 0, in case of failure, or the length of the UID in bytes that was
  selected (4, 7 or 10).
*/

// ---------------------------------------------------------------------------
// MIFARE 
// ---------------------------------------------------------------------------

// Performs the MIFARE standard authentication.
// uid: The first four bytes of the UID are used.
// block_address: The block number to authenticate.
// key_type: The MIFARE key A or B (0x60 or 0x61) to use. This is either
//           MFRC630_MF_AUTH_KEY_A or MFRC630_MF_AUTH_KEY_B
// returns: 0 in case of failure, nonzero in case of success.
// 
// This function is a higher-level wrapper around the MF authenticate command.
// internally it calls mfrc630_auth which is described above, the result of the
// authentication is checked to identify whether it appears to have succeeded.
//
// The key must be loaded into the key buffer, by one of the following functions: 
//      void mfrc630_load_key_E2(uint8_t key_nr);
//      void mfrc630_load_key(uint8_t* key);
//
// Once authenticated, the authentication MUST be stopped manually by calling
// the mfrc630_MF_deauth() function or otherwires disabling the Crypto1 ON bit
// in the status register.
uint8_t mfrc630_MF_auth(const uint8_t* uid, uint8_t key_type, uint8_t block);

// Disable the Crypto1 bit from the status register to disable encryption again.
void mfrc630_MF_deauth();

// Read a block of memory from a previously authenticated card.
// block_address: The block index to retrieve.
// dest: The pointer in which to write the bytes read from the card.
// returns: 0 for failure, otherwise the number of bytes received.
// This function writes a maximum of 16 bytes to dest, and cannot deal with data
// that is longer than 16 bytes.
uint8_t mfrc630_MF_read_block(uint8_t block_address, uint8_t* dest);

// Write a block of memory to a previously authenticated card.
// block_address: The block index to write to.
// source: The pointer in which the bytes to be written are.
// returns: 0 for failure, nonzero means success.
uint8_t mfrc630_MF_write_block(uint8_t block_address,const uint8_t* source);


/*

  // Load the correct register settings with:
   mfrc630_cmd_load_protocol(MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER, MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER);
   mfrc630_AN1102_recommended_registers(MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER);

  The normal procedure to read a card would be to wake them up using:
    mfrc630_iso14443a_REQA();

  If there is a response, start the SELECT procedure with 
    uint8_t uid_len = mfrc630_iso14443a_select(uid, &sak);

  If uid_len is nonzero, we have found a card and UID. We can attempt to
  authenticate with it, for example with the default key:
    
      uint8_t FFkey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      mfrc630_cmd_load_key(FFkey); // load into the key buffer

  Then attempt to authenticate block 0 using this FFkey as KEY_A type:
    mfrc630_MF_auth(uid, MFRC630_MF_AUTH_KEY_A, 0)

  Finally, if that succeeds, we may use:
    len = mfrc630_MF_read_block(0, readbuf)

  To read block 0 from the card. After which we call:
    mfrc630_MF_deauth()

  To disable the currently enabled encryption process.
*/
// Implements the above steps and prints output.
void mfrc630_MF_example_dump();

#ifdef __cplusplus
}
#endif

#endif // MFRC630_H_
