SHELL=/bin/bash

# requires sphinx >= 1.4.5
# breathe >= 4.2.0
# doxygen >= 1.8.7

all: doxygen
	sphinx-build . breathe_output

doxygen:
	doxygen Doxygen.cfg

clean:
	rm -r breathe_output doxygen_output
