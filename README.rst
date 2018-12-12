OpenFlexure Motor Controller
============================
This aims to be a super-simple motor controller, based on an Arduino Nano and a couple of Darlington pair ICs.  It controls cheap 28BYJ-48 steppers, as used on the OpenFlexure microscope and stage.  It owes quite a bit to [Fergus Riche's motor board](https://github.com/fr293/motor_board), the hardware developed by [OpenScope](http://2015.igem.org/Team:Cambridge-JIC) and the Arduino-based motor controller used by a number of summer students working with Richard Bowman in Cambridge, particularly James Sharkey.

Contributors
============
Richard Bowman (University of Bath, UK) wrote the initial version of the Arduino and Python code and maintains the module.
Valerian Sanga (STICLab, Tanzania) did the initial PCB design & production, and maintains the PCB design
Fergus Riche (University of Cambridge, UK) hacked the stepper library to make it work with the stepper motors we are using, and is a major contributor to the v3 PCB design.
Boyko Vodenicharski and Filip Ayazi (University of Cambridge, UK) contributed Python 3 support, endstop support, and Arduino code that works on v3 of the hardware.

This project is (c) 2017 by the contributors, and released under CERN Open Hardware License (hardware designs) and GNU GPL v3.0 (software).
