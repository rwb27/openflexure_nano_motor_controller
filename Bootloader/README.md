# Bootloader for the Sangaboard, and Arduino IDE integration

The sangaboard works perfectly well with the Ardunio Leonardo bootloader but it is preferable to have a custom one so the board identifies as what it is. This repo allows you to burn the custom bootloader and to have the Sangaboard as a board option in your IDE.

## Installing the Sangaboard into the Arduino IDE

The install process is pretty manual. It has been tested for version 1.8.5 of the Arduino IDE.

1. Locate the directory the Arduino IDE is installed in. We will call this `ArduinoDIR`
1. Copy the file `Sanga.hex` from this directory into `ArduinoDIR/hardware/arduino/avr/bootloaders/caterina`
1. Go to the `ArduinoDIR/hardware/arduino/avr` directory and open up the file boards.txt
1. Copy the contents of the file `boards_add.txt` also in this directory into the bottom of the boards.txt file.
1. Celebrate a job well done.

## Buring the bootloader

We burn the bootloader with an OLIMEX AVR-ISP-MK2.

1. Plug the OLIMEX into the computer.
1. Connect the OLIMEX to the  `ISP` pins of the Sangaboard.
1. Open the Arduino IDE.
1. In the `Tools` menu select `Sangaboard` for the `Board`.
1. In the `Tools` menu select `AVRISP mkII` for the `Programmer`.
1. 1. In the `Tools` menu select `Burn Bootloader`.
1. Wait for the bootloader to finish burning.
