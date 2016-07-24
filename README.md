MFRC630
=======

This is a library for NXP's [MFRC630][nxp_mfrc630] NFC and reader IC. It is likely that it also works for other IC's in
the same family, such as the CLRC663, MFRC631, MFRC630 and SLRC610. The library is platform independent and is likely to
work on many different microcontrollers, it is known to work on the Arduino compatible [Teensy][teensy31] and on an
STM32F0xx chip.

Design
------
The library itself is written in C, it requires the chip to be connected to the SPI interface, furthermore, it can only
deal with a single chip that's connected to the microcontroller. The library does *not* require time or delay functions
neither does it need an interrupt pin, all that is required is the SPI bus. The handling of timeouts is done by using
one of the chip's timers.

The library is clearly structured, providing functionality from low level to higher level:

* HAL:  Three functions are to be created to allow the library to interact with the SPI bus.

* Register interaction: The adresses for all registers are provided as define statements, functions for FIFO reading and writing are available, as well as reading and writing normal registers. For various important registers there are additional define statements available that make it easier to interact with these.

* Commands: The chip can be instructed to execute various commands, these functions provide wrappers for these and handle the arguments required for each command.

* ISO14443A: Provides the necessities from ISO14443a to interact with RFID tags: the REQA, WUPA and SELECT procedure (with collision handling) to determine the UID(s).

* MIFARE: Provides functions to authenticate, read and write blocks on MIFARE cards.

Documentation
-------------
The header files are annotated with [Doxygen][doxygen]-style comments, running `make doxygen` in the documentation
folder will output the documentation in html form.  Additionally, running `make` will also generate output using
[Breathe][breathe].

An example for an Arduino compatible board is [provided](examples/arduino_example/arduino_example.ino) as well as some
[pointers](examples/stm32_cube_example/example.c) for use on an STM32 microcontroller.

Miscellaneous
-------------
This chip seems to be significantly less popular than the [MFRC522][nxp_mfrc522], if you are starting with RFID, that
might be a better choice to use, as there is a well-tested [library][arduino_mfrc522] for that chip.

I came across this chip when I was repurposing off-the-shelf hardware to do my bidding; Surprisingly, no library was
available for this chip. Improvements / additional examples are welcome.

License
-------
MIT License, see LICENSE.txt.

Copyright (c) 2016 Ivor Wanders


[nxp_mfrc630]: http://www.nxp.com/products/identification-and-security/nfc-and-reader-ics/nfc-frontend-solutions/high-performance-mifare-and-ntag-frontend:MFRC63002HN
[doxygen]: http://www.doxygen.org
[breathe]: https://breathe.readthedocs.io/en/latest/
[teensy31]: http://www.pjrc.com/teensy/
[stm32f0cube]: http://www.st.com/content/st_com/en/products/embedded-software/mcus-embedded-software/stm32-embedded-software/stm32cube-embedded-software/stm32cubef0.html
[nxp_mfrc522]: http://www.nxp.com/products/identification-and-security/nfc-and-reader-ics/nfc-frontend-solutions/standard-performance-mifare-and-ntag-frontend:MFRC52202HN1
[arduino_mfrc522]: https://github.com/miguelbalboa/rfid