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

// Define register name                 Addr    //  comment
#define MFRC630_REG_COMMAND             0x00    // Starts and stops command execution
#define MFRC630_REG_HOSTCTRL            0x01    // Host control register
#define MFRC630_REG_FIFOCONTROL         0x02    // Control register of the FIFO
#define MFRC630_REG_WATERLEVEL          0x03    // Level of the FIFO underflow and overflow warning
#define MFRC630_REG_FIFOLENGTH          0x04    // Length of the FIFO
#define MFRC630_REG_FIFODATA            0x05    // Data In/Out exchange register of FIFO buffer
#define MFRC630_REG_IRQ0                0x06    // Interrupt register 0
#define MFRC630_REG_IRQ1                0x07    // Interrupt register 1
#define MFRC630_REG_IRQ0EN              0x08    // Interrupt enable register 0
#define MFRC630_REG_IRQ1EN              0x09    // Interrupt enable register 1
#define MFRC630_REG_ERROR               0x0A    // Error bits showing the error status of the last command execution
#define MFRC630_REG_STATUS              0x0B    // Contains status of the communication
#define MFRC630_REG_RXBITCTRL           0x0C    // Control for anticoll. adjustments for bit oriented protocols
#define MFRC630_REG_RXCOLL              0x0D    // Collision position register
#define MFRC630_REG_TCONTROL            0x0E    // Control of Timer 0..3
#define MFRC630_REG_T0CONTROL           0x0F    // Control of Timer0
#define MFRC630_REG_T0RELOADHI          0x10    // High register of the reload value of Timer0
#define MFRC630_REG_T0RELOADLO          0x11    // Low register of the reload value of Timer0
#define MFRC630_REG_T0COUNTERVALHI      0x12    // Counter value high register of Timer0
#define MFRC630_REG_T0COUNTERVALLO      0x13    // Counter value low register of Timer0
#define MFRC630_REG_T1CONTROL           0x14    // Control of Timer1
#define MFRC630_REG_T1RELOADHI          0x15    // High register of the reload value of Timer1
#define MFRC630_REG_T1RELOADLO          0x16    // Low register of the reload value of Timer1
#define MFRC630_REG_T1COUNTERVALHI      0x17    // Counter value high register of Timer1
#define MFRC630_REG_T1COUNTERVALLO      0x18    // Counter value low register of Timer1
#define MFRC630_REG_T2CONTROL           0x19    // Control of Timer2
#define MFRC630_REG_T2RELOADHI          0x1A    // High byte of the reload value of Timer2
#define MFRC630_REG_T2RELOADLO          0x1B    // Low byte of the reload value of Timer2
#define MFRC630_REG_T2COUNTERVALHI      0x1C    // Counter value high byte of Timer2
#define MFRC630_REG_T2COUNTERVALLO      0x1D    // Counter value low byte of Timer2
#define MFRC630_REG_T3CONTROL           0x1E    // Control of Timer3
#define MFRC630_REG_T3RELOADHI          0x1F    // High byte of the reload value of Timer3
#define MFRC630_REG_T3RELOADLO          0x20    // Low byte of the reload value of Timer3
#define MFRC630_REG_T3COUNTERVALHI      0x21    // Counter value high byte of Timer3
#define MFRC630_REG_T3COUNTERVALLO      0x22    // Counter value low byte of Timer3
#define MFRC630_REG_T4CONTROL           0x23    // Control of Timer4
#define MFRC630_REG_T4RELOADHI          0x24    // High byte of the reload value of Timer4
#define MFRC630_REG_T4RELOADLO          0x25    // Low byte of the reload value of Timer4
#define MFRC630_REG_T4COUNTERVALHI      0x26    // Counter value high byte of Timer4
#define MFRC630_REG_T4COUNTERVALLO      0x27    // Counter value low byte of Timer4
#define MFRC630_REG_DRVMOD              0x28    // Driver mode register
#define MFRC630_REG_TXAMP               0x29    // Transmitter amplifier register
#define MFRC630_REG_DRVCON              0x2A    // Driver configuration register
#define MFRC630_REG_TXL                 0x2B    // Transmitter register
#define MFRC630_REG_TXCRCPRESET         0x2C    // Transmitter CRC control register, preset value
#define MFRC630_REG_RXCRCCON            0x2D    // Receiver CRC control register, preset value
#define MFRC630_REG_TXDATANUM           0x2E    // Transmitter data number register
#define MFRC630_REG_TXMODWIDTH          0x2F    // Transmitter modulation width register
#define MFRC630_REG_TXSYM10BURSTLEN     0x30    // Transmitter symbol 1 + symbol 0 burst length register
#define MFRC630_REG_TXWAITCTRL          0x31    // Transmitter wait control
#define MFRC630_REG_TXWAITLO            0x32    // Transmitter wait low
#define MFRC630_REG_FRAMECON            0x33    // Transmitter frame control
#define MFRC630_REG_RXSOFD              0x34    // Receiver start of frame detection
#define MFRC630_REG_RXCTRL              0x35    // Receiver control register
#define MFRC630_REG_RXWAIT              0x36    // Receiver wait register
#define MFRC630_REG_RXTHRESHOLD         0x37    // Receiver threshold register
#define MFRC630_REG_RCV                 0x38    // Receiver register
#define MFRC630_REG_RXANA               0x39    // Receiver analog register
#define MFRC630_REG_RFU                 0x3A    // -
#define MFRC630_REG_SERIALSPEED         0x3B    // Serial speed register
#define MFRC630_REG_LFO_TRIMM           0x3C    // Low-power oscillator trimming register
#define MFRC630_REG_PLL_CTRL            0x3D    // IntegerN PLL control register, for mcu clock output adjustment
#define MFRC630_REG_PLL_DIVOUT          0x3E    // IntegerN PLL control register, for mcu clock output adjustment
#define MFRC630_REG_LPCD_QMIN           0x3F    // Low-power card detection Q channel minimum threshold
#define MFRC630_REG_LPCD_QMAX           0x40    // Low-power card detection Q channel maximum threshold
#define MFRC630_REG_LPCD_IMIN           0x41    // Low-power card detection I channel minimum threshold
#define MFRC630_REG_LPCD_I_RESULT       0x42    // Low-power card detection I channel result register
#define MFRC630_REG_LPCD_Q_RESULT       0x43    // Low-power card detection Q channel result register
#define MFRC630_REG_PADEN               0x44    // PIN enable register
#define MFRC630_REG_PADOUT              0x45    // PIN out register
#define MFRC630_REG_PADIN               0x46    // PIN in register
#define MFRC630_REG_SIGOUT              0x47    // Enables and controls the SIGOUT Pin
#define MFRC630_REG_VERSION             0x7F    // Version and subversion register

// Define  command name                 hex     // argument ; comment
#define MFRC630_CMD_IDLE                0x00    // - ; no action, cancels current command execution
#define MFRC630_CMD_LPCD                0x01    // - ; low-power card detection
#define MFRC630_CMD_LOADKEY             0x02    // (keybyte1),(keybyte2), (keybyte3),(keybyte4), (keybyte5),(keybyte6); ; reads a MIFARE key (size of 6 bytes) from FIFO buffer ant puts it into Key buffer
#define MFRC630_CMD_MFAUTHENT           0x03    // 60h or 61h, (block address), (card serial number byte0),(card serial number byte1), (card serial number byte2),(card serial number byte3); performs the MIFARE standard authentication 
#define MFRC630_CMD_RECEIVE             0x05    // - ; activates the receive circuit
#define MFRC630_CMD_TRANSMIT            0x06    // bytes to send: byte1, byte2,....;  transmits data from the FIFO buffer
#define MFRC630_CMD_TRANSCEIVE          0x07    // bytes to send: byte1, byte2,....;  transmits data from the FIFO buffer and automatically activates the receiver after transmission finished
#define MFRC630_CMD_WRITEE2             0x08    // addressH, addressL, data; gets one byte from FIFO buffer and writes it to the internal EEPROM
#define MFRC630_CMD_WRITEE2PAGE         0x09    // (page Address), data0, [data1..data63]; gets up to 64 bytes (one EEPROM page) from the FIFO buffer and writes it to the EEPROM
#define MFRC630_CMD_READE2              0x0A    // addressH, address L, length; reads data from the EEPROM and copies it into the FIFO buffer 
#define MFRC630_CMD_LOADREG             0x0C    // (EEPROM addressH), (EEPROM addressL), RegAdr, (number of Register to be copied); reads data from the internal EEPROM and initializes the MFRC630 registers. EEPROM address needs to be within EEPROM sector 2
#define MFRC630_CMD_LOADPROTOCOL        0x0D    // (Protocol number RX), (Protocol number TX); reads data from the internal EEPROM and initializes the MFRC630 registers needed for a Protocol change
#define MFRC630_CMD_LOADKEYE2           0x0E    // KeyNr; copies a key from the EEPROM into the key buffer
#define MFRC630_CMD_STOREKEYE2          0x0F    // KeyNr, byte1,byte2, byte3, byte4, byte5,byte6; stores a MIFARE key (size of 6 bytes) into the EEPROM
#define MFRC630_CMD_READRNR             0x1C    // - ; Copies bytes from the Random Number generator into the FIFO until the FiFo is full 
#define MFRC630_CMD_SOFTRESET           0x1F    // - ; resets the MFRC630

// Status register, the commtates are more 
#define MFRC630_STATUS_STATE_IDLE           0b000
#define MFRC630_STATUS_STATE_TXWAIT         0b001
#define MFRC630_STATUS_STATE_TRANSMITTING   0b011
#define MFRC630_STATUS_STATE_RXWAIT         0b101
#define MFRC630_STATUS_STATE_WAIT_FOR_DATA  0b110
#define MFRC630_STATUS_STATE_RECEIVING      0b111
#define MFRC630_STATUS_STATE_NOT_USED       0b100
#define MFRC630_STATUS_CRYPTO1_ON           (1<<5)

// Flags for the Timer control register.
#define MFRC630_TCONTROL_STOPRX           (1<<7)
#define MFRC630_TCONTROL_START_NOT        (0b00<<4)
#define MFRC630_TCONTROL_START_TX_END     (0b01<<4)
#define MFRC630_TCONTROL_START_LFO_WO     (0b10<<4)
#define MFRC630_TCONTROL_START_LFO_WITH   (0b11<<4)
#define MFRC630_TCONTROL_AUTO_RESTART     (0b1<<3)
#define MFRC630_TCONTROL_CLK_13MHZ        (0b00)
#define MFRC630_TCONTROL_CLK_211KHZ       (0b01)
#define MFRC630_TCONTROL_CLK_UF_TA1       (0b10)
#define MFRC630_TCONTROL_CLK_UF_TA2       (0b11)


// IRQ0 register fields
#define MFRC630_IRQ0_SET                (1<<7)
#define MFRC630_IRQ0_HIALERT_IRQ        (1<<6)
#define MFRC630_IRQ0_LOALERT_IRQ        (1<<5)
#define MFRC630_IRQ0_IDLE_IRQ           (1<<4)
#define MFRC630_IRQ0_TX_IRQ             (1<<3)
#define MFRC630_IRQ0_RX_IRQ             (1<<2)
#define MFRC630_IRQ0_ERR_IRQ            (1<<1)
#define MFRC630_IRQ0_RXSOF_IRQ          (1<<0)

// IRQ1 register fields
#define MFRC630_IRQ1_SET                (1<<7)
#define MFRC630_IRQ1_GLOBAL_IRQ         (1<<6)
#define MFRC630_IRQ1_LPCD_IRQ           (1<<5)
#define MFRC630_IRQ1_TIMER4_IRQ         (1<<4)
#define MFRC630_IRQ1_TIMER3_IRQ         (1<<3)
#define MFRC630_IRQ1_TIMER2_IRQ         (1<<2)
#define MFRC630_IRQ1_TIMER1_IRQ         (1<<1)
#define MFRC630_IRQ1_TIMER0_IRQ         (1<<0)


// IRQ0EN register fields
#define MFRC630_IRQ0EN_IRQ_INV          (1<<7)
#define MFRC630_IRQ0EN_HIALERT_IRQEN    (1<<6)
#define MFRC630_IRQ0EN_LOALERT_IRQEN    (1<<5)
#define MFRC630_IRQ0EN_IDLE_IRQEN       (1<<4)
#define MFRC630_IRQ0EN_TX_IRQEN         (1<<3)
#define MFRC630_IRQ0EN_RX_IRQEN         (1<<2)
#define MFRC630_IRQ0EN_ERR_IRQEN        (1<<1)
#define MFRC630_IRQ0EN_RXSOF_IRQEN      (1<<0)

// IRQ1EN register fields.
#define MFRC630_IRQ1EN_IRQ_PUSHPULL        (1<<7)
#define MFRC630_IRQ1EN_IRQ_PINEN           (1<<6)
#define MFRC630_IRQ1EN_LPCD_IRQEN          (1<<5)
// notice 1 << timer_id.
#define MFRC630_IRQ1EN_TIMER4_IRQEN        (1<<4)
#define MFRC630_IRQ1EN_TIMER3_IRQEN        (1<<3)
#define MFRC630_IRQ1EN_TIMER2_IRQEN        (1<<2)
#define MFRC630_IRQ1EN_TIMER1_IRQEN        (1<<1)
#define MFRC630_IRQ1EN_TIMER0_IRQEN        (1<<0)

// Error register fields
#define MFRC630_ERROR_EE_ERR            (1<<7)
#define MFRC630_ERROR_FIFOWRERR         (1<<6)
#define MFRC630_ERROR_FIFOOVL           (1<<5)
#define MFRC630_ERROR_MINFRAMEERR       (1<<4)
#define MFRC630_ERROR_NODATAERR         (1<<3)
#define MFRC630_ERROR_COLLDET           (1<<2)
#define MFRC630_ERROR_PROTERR           (1<<1)
#define MFRC630_ERROR_INTEGERR          (1<<0)

// Defines to enable CRC for MFRC630_REG_TXCRCPRESET and  MFRC630_REG_RXCRCCON
#define MFRC630_CRC_ON            1
#define MFRC630_CRC_OFF           0

// MFRC630_REG_TXDATANUM
#define MFRC630_TXDATANUM_DATAEN        (1<<3)

// Protocol defines.
#define MFRC630_PROTO_ISO14443A_106_MILLER_MANCHESTER 0
#define MFRC630_PROTO_ISO14443A_212_MILLER_BPSK 1
#define MFRC630_PROTO_ISO14443A_424_MILLER_BPSK 2
#define MFRC630_PROTO_ISO14443A_848_MILLER_BPSK 3
#define MFRC630_PROTO_ISO14443B_106_NRZ_BPSK 4
#define MFRC630_PROTO_ISO14443B_212_NRZ_BPSK 5
#define MFRC630_PROTO_ISO14443B_424_NRZ_BPSK 6
#define MFRC630_PROTO_ISO14443B_848_NRZ_BPSK 7
#define MFRC630_PROTO_FELICA_212_MANCHESTER_MANCHESTER 8
#define MFRC630_PROTO_FELICA_424_MANCHESTER_MANCHESTER 9
#define MFRC630_PROTO_ISO15693_1_OF_4_SSC 10
#define MFRC630_PROTO_ISO15693_1_OF_4_DSC 11
#define MFRC630_PROTO_ISO15693_1_OF_256_SSC 12
#define MFRC630_PROTO_EPC_UID_UNITRAY_SSC 13
#define MFRC630_PROTO_ISO18000_MODE_3 14

// recommended register values from register 0x28 down.
// From AN11022: CLRC663 Quickstart Guide
// All the other protocols are also in there....
#define MFRC630_RECOM_14443A_CRC 0x18
#define MFRC630_RECOM_14443A_ID1_106 {0x8A, 0x08, 0x21, 0x1A, 0x18, 0x18, 0x0F, 0x27, 0x00, 0xC0, 0x12, 0xCF, 0x00, 0x04, 0x90, 0x32, 0x12, 0x0A}
#define MFRC630_RECOM_14443A_ID1_212 {0x8E, 0x12, 0x11, 0x06, 0x18, 0x18, 0x0F, 0x10, 0x00, 0xC0, 0x12, 0xCF, 0x00, 0x05, 0x90, 0x3F, 0x12, 0x02}
#define MFRC630_RECOM_14443A_ID1_424 {0x8E, 0x12, 0x11, 0x06, 0x18, 0x18, 0x0F, 0x08, 0x00, 0xC0, 0x12, 0xCF, 0x00, 0x06, 0x90, 0x3F, 0x12, 0x0A}
#define MFRC630_RECOM_14443A_ID1_848 {0x8F, 0xDB, 0x11, 0x06, 0x18, 0x18, 0x0F, 0x02, 0x00, 0xC0, 0x12, 0xCF, 0x00, 0x07, 0x90, 0x3F, 0x12, 0x02}

// Defines from ISO14443A
#define MFRC630_ISO14443_CMD_REQA               0x26  // request (idle -> ready)
#define MFRC630_ISO14443_CMD_WUPA               0x52  // wake up type a (idle / halt -> ready)
#define MFRC630_ISO14443_CAS_LEVEL_1            0x93  // Cascade level 1 for select.
#define MFRC630_ISO14443_CAS_LEVEL_2            0x95  // Cascade level 2 for select.
#define MFRC630_ISO14443_CAS_LEVEL_3            0x97  // Cascade level 3 for select.

// Defines for MIFARE
#define MFRC630_MF_AUTH_KEY_A                   0x60  // A key_type for mifare auth.
#define MFRC630_MF_AUTH_KEY_B                   0x61  // A key_type for mifare auth.
#define MFRC630_MF_CMD_READ                     0x30  // To read a block from mifare card.
#define MFRC630_MF_CMD_WRITE                    0xA0  // To write a block to a mifare card.
#define MFRC630_MF_ACK                          0x0A  // Sent by cards to acknowledge an operation.
