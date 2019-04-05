#What is this file?

This file is to track some of the big changes when refactoring the OpenFlexureStage from a monolithic beast which contains all of the Python, Firmware, and PCB for the Sangaboard as well as the the OpenFlexure motor controler firmware into a few seperate repos.

It will be removed eventually it will probably be helpful to debug problems

## Arduino code rebranded to say Sangaboard

## BasicSerialntrument >> ExtensibleSerialInstrumet?
Why, because it isn't that basic anymore, a bit of a silly change, I hope this doesn't cause problems!

## Stage.board >> Sangaboard.firmware
We have lots of different boards, and they can have different firmware. Changed to be clear this is the firmware version not the board version


## Sangaboard module has core functions like moving, openflexure has more complicated OFS stuff like backlash correction and scans

## Optional modules still living with the sangaboard as they are in the Arduino code.