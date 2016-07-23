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
// Register interaction functions.
// ---------------------------------------------------------------------------

/*! \defgroup register Register interaction.
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
  
  The FIFO is located at register `#MFRC630_REG_FIFODATA`. 
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

  \warning This reads regardless of `#MFRC630_REG_FIFOLENGTH`, if there aren't enough bytes present in the FIFO, they
           are read from the chip anyway, these bytes should not be used. (The returned bytes from an empty FIFO are
           often identical to the last valid byte that was read from it.)
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

/*!
  @brief Load protocol settings.
  
  Loads register settings for the protocol indicated. Can configure different protocols for rx and tx. The most common
  protocol is `#MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER` which is the default protocol for the SELECT procedure.

  The most common protocols are listed in the datasheet, but the MFRC630 Quickstart Guide AN11022 gives a complete
  description.

  \param [in] rx The protocol number to load for the receiving frontend.
  \param [in] tx The protocol number to load for the tranmitting frontend.

 */
void mfrc630_cmd_load_protocol(uint8_t rx, uint8_t tx);

/*!
  @brief Transmit the bytes provided and go into receive mode.

  This function loads the data from the `data` array into the FIFO, and then issues the `MFRC630_CMD_TRANSCEIVE`
  command, which sends the data in the FIFO and switches to receiving mode afterwards.

  \param [in] data The data to be transmitted.
  \param [in] len The numeber of bytes to be read from `data` and be transmitted..
 */
void mfrc630_cmd_transceive(const uint8_t* data, uint16_t len);

/*!
  @brief Set the device into idle mode.

  Stops the currently active command and return to idle mode.
 */
void mfrc630_cmd_idle();

/*!
  @brief Loads a key from the EEPROM into the key buffer.

  This function can load a key from the MIFARE key area in the EEPROM into the key buffer. This section of the EEPROM
  can only be written. The key buffer is a part of memory in the MFRC630 used for the MIFARE authentication procedure.

  \param [in] key_nr Loads the key stored for this index.
 */
void mfrc630_cmd_load_key_E2(uint8_t key_nr);

/*!
  @brief Loads the provided key into the key buffer.

  This function reads 6 bytes from the `key` array into the FIFO and then loads the key into the key buffer.

  \param [in] key Array which holds the MIFARE key, it is always 6 bytes long.
 */
void mfrc630_cmd_load_key(const uint8_t* key);

/*!
  @brief Perform MIFARE authentication procedure with a card.

  This function attemps to authenticate with the specified card using the key which is currently in the key buffer.
  This function is usually preceded by either the `mfrc630_cmd_load_key_E2()` or `mfrc630_cmd_load_key()` functions.

  \param [in] key_type The MIFARE key A or B (`MFRC630_MF_AUTH_KEY_A` = 0x60 or `MFRC630_MF_AUTH_KEY_B` = 0x61) to use.
  \param [in] block_address The block on which to authenticate.
  \param [in] card_uid The authentication procedure required the first four bytes of the card's UID to authenticate.
 */
void mfrc630_cmd_auth(uint8_t key_type, uint8_t block_address, const uint8_t* card_uid);

//! @}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------
/*! \defgroup utility Utility
    \brief Various utility functions for often performed actions.

  @{
*/


/*!
  @brief Flush the FIFO buffer.

  This function clears all contents that are currently in the FIFO.

 */
void mfrc630_flush_fifo();

/*!
  @brief Get the FIFO length.

  Returns the current number of bytes in the FIFO.

  \warning This function only returns the first 8 bits of the FIFO length, if the 512 byte FIFO is used, only the least
           significant eight bits will be returned.
  
  \return The number of bytes currently in the FIFO.
 */
uint16_t mfrc630_fifo_length();


/*!
  @brief Clear the interrupt0 flags.

  Resets the interrupt 0 register (`MFRC630_REG_IRQ0`).
 */
void mfrc630_clear_irq0();
/*!
  @brief Clear the interrupt1 flags.

  Resets the interrupt 1 register (`MFRC630_REG_IRQ1`).
 */
void mfrc630_clear_irq1();


/*!
  @brief Get the value of the interrupt 0 register.

  \return The value of the `MFRC630_REG_IRQ0` register.
 */
uint8_t mfrc630_irq0();

/*!
  @brief Get the value of the interrupt 1 register.

  \return The value of the `MFRC630_REG_IRQ1` register.
 */
uint8_t mfrc630_irq1();

/*!
  @brief Print an array in hexadecimal format using `MFRC630_PRINTF`.

  Prints the bytes in `data` in hexadecimal format, separated by spaces using the `MFRC630_PRINTF` macro, if defined.

  \param [in] data The array to be printed.
  \param [in] len The number of bytes to print..
 */
void mfrc630_print_block(const uint8_t* data, uint16_t len);

/*!
  @brief Copy a page from EEPROM into an array on the MCU.
  
  This instruction transfers a page from the EEPROM into the FIFO and then transfers this data from the FIFO
  into an array. It always transfers 64 bytes, as such `dest` must be (atleast) 64 bytes long.

  This basically calls mfrc630_cmd_read_E2() and then transfers the FIFO with mfrc630_read_fifo(). This is useful for
  dumping the entire EEPROM.

  \param [out] dest The array to write the data into.
  \param [in] page The page to read from the EEPROM. (This gets multiplied by 64 to obtain the start address).
  \return The number of bytes transmitted from the FIFO into `dest`.
 */
uint8_t mfrc630_transfer_E2_page(uint8_t* dest, uint8_t page);

//!  @}

// ---------------------------------------------------------------------------
// Timer functions
// ---------------------------------------------------------------------------
/*! \defgroup timer Timer
    \brief Functions for manipulating the timers.

    The MFRC630 has 5 timers, the first four can be treated more or less similarly, the last timer `Timer4` has a
    different control register.

    Timer 0-3 can be treated in a similar way, and as such the functions take an argument that specifies which timer
    to manipulate.

    Timer4 is special, read the datasheet on how to use that timer as it has other clock sources and properties.


  @{
*/

/*!
  @brief Activates a timer.

  This sets the the `MFRC630_REG_TCONTROL` register to enable or disable this timer.

  \note Seems to trigger timer reset?

  \param [in] timer Specifies which timer to use (0, 1, 2 or 3).
  \param [in] active Should be `0` to deactivate the timer, `1` to activate it.
 */
void mfrc630_activate_timer(uint8_t timer, uint8_t active);

// Set the timer control field of the timer.
// value: the value to set the timer's control field to.
/*!
  @brief Sets the timer control register.

  This sets the `T[0-3]Control` register to the provided value. The value speficies the propertief of StopRx, Start
  AutoRestart and Clk for this timer.

  \param [in] timer Specifies which timer to use (0, 1, 2 or 3).
  \param [in] value This can be a combination of the defines associated with the Timer controls.
  \see MFRC630_TCONTROL_STOPRX
  \see MFRC630_TCONTROL_START_NOT, MFRC630_TCONTROL_START_TX_END, MFRC630_TCONTROL_START_LFO_WO,
       MFRC630_TCONTROL_START_LFO_WITH
  \see MFRC630_TCONTROL_CLK_13MHZ, MFRC630_TCONTROL_CLK_211KHZ, MFRC630_TCONTROL_CLK_UF_TA1, MFRC630_TCONTROL_CLK_UF_TA2
 */
void mfrc630_timer_set_control(uint8_t timer, uint8_t value);

/*!
  @brief Sets the reload value of the timer.

  This counter starts counting down from this reload value, an underflow occurs when the timer reaches zero.

  \param [in] timer Specifies which timer to use (0, 1, 2 or 3).
  \param [in] value The value from which to start the counter. 
 */
void mfrc630_timer_set_reload(uint8_t timer, uint16_t value);

/*!
  @brief Sets the current value of this timer..

  Sets the current value of this counter, it counts down from this given value.

  \param [in] timer Specifies which timer to use (0, 1, 2 or 3).
  \param [in] value The value to set the counter to. 
 */
void mfrc630_timer_set_value(uint8_t timer, uint16_t value);


/*!
  @brief Retrieve the current value of a timer.

  Reads the current value of the given timer and returns the result.

  \param [in] timer Specifies which timer to use (0, 1, 2 or 3).
  \return The current value of this timer.
 */
uint16_t mfrc630_timer_get_value(uint8_t timer);
//!  @}

// ---------------------------------------------------------------------------
// From documentation
// ---------------------------------------------------------------------------
/*! \defgroup documentation From application notes
    \brief Several functions written based on application notes.

    Two application notes are of particular interest:
      - Application Note 11145:
          CLRC663, MFRC631, MFRC 630, SLRC610 Low Power Card Detection
          http://www.nxp.com/documents/application_note/AN11145.pdf
      - Application Note 11022:
          CLRC663 Quickstart Guide
          http://www.nxp.com/documents/application_note/AN11022.pdf

    The first details how to perform an IQ measurement to determine the thresholds for the Low Power Card Detection
    (LPCD). The second describes default register values for various protocols and the protocol numbers that are
    associated to them.
     
  @{
*/

// From Application Note 11145:
//      CLRC663, MFRC631, MFRC 630, SLRC610 Low Power Card Detection
//      http://www.nxp.com/documents/application_note/AN11145.pdf

//      IQ measurement, section 3.2.1
/*! \brief Start IQ Measurement.

    From Application Note 11145, section 3.2.1, it configures the registers to perform an IQ measurement.
*/
void mfrc630_AN11145_start_IQ_measurement();
//      wait about 50ms between them.
/*! \brief Stop IQ Measurement.

    Stop the previously started IQ measurement. The application note uses a delay of 50 ms between the start and stop.

    The actual vaues can be retrieved with:
    \code
      uint8_t I_value = mfrc630_read_reg(MFRC630_REG_LPCD_I_RESULT) & 0x3F
      uint8_t Q_value = mfrc630_read_reg(MFRC630_REG_LPCD_Q_RESULT) & 0x3F
    \endcode
*/
void mfrc630_AN11145_stop_IQ_measurement();


// From Application Note 11022:
//      CLRC663 Quickstart Guide
//      http://www.nxp.com/documents/application_note/AN11022.pdf

/*! \brief Set the registers to the recommended values.

    This function uses the recommended registers from the datasheets, it should yield identical results to the 
    `mfrc630_cmd_load_protocol()` function.

    \param [in] protocol The protocol index to set the registers to. Only
           `MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER`, `MFRC630_PROTO_ISO14443A_212_MILLER_BPSK`,
           `MFRC630_PROTO_ISO14443A_424_MILLER_BPSK` and `MFRC630_PROTO_ISO14443A_848_MILLER_BPSK` were copied. The
            recommended values for the other protocols can be found in the application note.
*/
void mfrc630_AN1102_recommended_registers(uint8_t protocol);

/*! \brief Set the registers to the recommended values starting from `MFRC630_REG_TXCRCPRESET`.

    Since the transmitter registers can require harware-specific customization to work correctly, this function sets the
    recommended values of the registers after the transmitter settings.

    \param [in] protocol The protocol index to set the registers to. Only
           `MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER`, `MFRC630_PROTO_ISO14443A_212_MILLER_BPSK`,
           `MFRC630_PROTO_ISO14443A_424_MILLER_BPSK` and `MFRC630_PROTO_ISO14443A_848_MILLER_BPSK` were copied. The
            recommended values for the other protocols can be found in the application note.
*/
void mfrc630_AN1102_recommended_registers_no_transmitter(uint8_t protocol);

/*! \brief Set the registers to the recommended values, skipping the first `skip` registers.

    Sets the recommended registers but allows for an arbitrary number of registers to be skipped at the start.

    \param [in] protocol The protocol index to set the registers to. Only
           `MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER`, `MFRC630_PROTO_ISO14443A_212_MILLER_BPSK`,
           `MFRC630_PROTO_ISO14443A_424_MILLER_BPSK` and `MFRC630_PROTO_ISO14443A_848_MILLER_BPSK` were copied. The
            recommended values for the other protocols can be found in the application note.
    \param [in] skip The number of registers to skip from the start.
*/
void mfrc630_AN1102_recommended_registers_skip(uint8_t protocol, uint8_t skip);
//!  @}

// ---------------------------------------------------------------------------
// ISO 14443A
// ---------------------------------------------------------------------------
/*! \defgroup iso14443a ISO14443A
    \brief Functions for card wakeup and UID discovery..

  @{
*/

/*! \brief Sends an Request Command, Type A.

    This sends the ISO14443 REQA request, cards in IDLE mode should answer to this.

    \return The Answer to request A byte (ATQA), or zero in case of no answer.
*/
uint16_t mfrc630_iso14443a_REQA();

/*! \brief Performs the SELECT procedure to discover a card's UID.

  The select procedure is explained quite complex in the ISO norm.

  Conceptually it is not all that complex:
    - The cascade level can be seen as a prefix to ensure both the PICC and PCD
      are working on identifying the same part of the UID.
    - The entire anti-collision scheme is more of a binary search, the
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

  \param [out] uid: The UID of the card will be stored into this array.
  \param [out] sak: The last SAK byte received during the SELECT procedure is placed here, this often holds information
                    about the type of card.
  \return The length of the UID in bytes (4, 7, 10), or 0 in case of failure.
*/
uint8_t mfrc630_iso14443a_select(uint8_t* uid, uint8_t* sak);
//!  @}

// ---------------------------------------------------------------------------
// MIFARE
// ---------------------------------------------------------------------------
/*! \defgroup mifare MIFARE
    \brief Functions to interact with MIFARE RFID tags / cards.


    MIFARE cards have memory blocks of 16 bytes, read and write operations are always done one entire block of 16 bytes
    at a time.
  @{
*/


/*! \brief Perform a MIFARE authentication procedure.

    This function is a higher-level wrapper around the MF authenticate command.
    internally it calls `mfrc630_cmd_auth()` which is described above, the result of the
    authentication is checked to identify whether it appears to have succeeded.

    The key must be loaded into the key buffer, by one of the following functions:
    \code
         void mfrc630_cmd_load_key_E2(uint8_t key_nr);
         void mfrc630_cmd_load_key(uint8_t* key);
    \endcode

    Once authenticated, the authentication MUST be stopped manually by calling
    the `mfrc630_MF_deauth()` function or otherwires disabling the Crypto1 ON bit
    in the status register.

    \param [in] key_type The MIFARE key A or B ( `#MFRC630_MF_AUTH_KEY_A` = 0x60 or `#MFRC630_MF_AUTH_KEY_B` = 0x61) to
                use.
    \param [in] block The block to authenticate.
    \param [in] uid The authentication procedure required the first four bytes of the card's UID to authenticate.
    \return 0 in case of failure, nonzero in case of success.
*/
uint8_t mfrc630_MF_auth(const uint8_t* uid, uint8_t key_type, uint8_t block);

/*! \brief Disables MIFARE authentication.

  Disable the Crypto1 bit from the status register to disable encryption. This bit will never change to 0 by itself, so
  the register must be overwritten by the MCU.
*/
void mfrc630_MF_deauth();

/*! \brief Read a block of memory from an authenticated card.

    Try to read a block of memory from the card with the appropriate timeouts and error checking.


    \param [in] block_address The block to read.
    \param [out] dest The array in which to write the 16 bytes read from the card.
    \return 0 for failure, otherwise the number of bytes received.
*/
uint8_t mfrc630_MF_read_block(uint8_t block_address, uint8_t* dest);

/*! \brief Write a block of memory to an authenticated card.

    Try to write a block of memory from the card with the appropriate timeouts and error checking.

    \param [in] block_address The block to write to.
    \param [in] source The array containing the 16 bytes to be written to this block.
    \return 0 for failure, nonzero means success.
*/
uint8_t mfrc630_MF_write_block(uint8_t block_address, const uint8_t* source);


/*! \brief An example to read the first four blocks.

  Reading from a MIFARE card has the following steps, which are implemented in this function.

  Load the correct register settings with:
    \code
      mfrc630_AN1102_recommended_registers(MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER);
    \endcode

  The normal procedure to read a card would be to wake them up using:
    \code
      mfrc630_iso14443a_REQA();
    \endcode

  If there is a response, start the SELECT procedure with 
    \code
      uint8_t uid_len = mfrc630_iso14443a_select(uid, &sak);
    \endcode

  If uid_len is nonzero, we have found a card and UID. We can attempt to
  authenticate with it, for example with the default key:
    
    \code
      uint8_t FFkey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
      mfrc630_cmd_load_key(FFkey); // load into the key buffer
    \endcode

  Then attempt to authenticate block 0 using this FFkey as KEY_A type:
    \code
      mfrc630_MF_auth(uid, MFRC630_MF_AUTH_KEY_A, 0)
    \endcode

  Finally, if that succeeds, we may use:
    \code
      len = mfrc630_MF_read_block(0, readbuf)
    \endcode

  To read block 0 from the card. After which we call:
    \code
      mfrc630_MF_deauth()
    \endcode
  To disable the currently enabled encryption process.
*/
void mfrc630_MF_example_dump();
//!  @}



#ifdef __cplusplus
}
#endif

#endif  // MFRC630_H_
