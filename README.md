# OpenFlexure Motor Controller

This aims to be a super-simple motor controller, based on an Arduino Nano and a couple of Darlington pair ICs.  It controls cheap 28BYJ-48 steppers, as used on the OpenFlexure microscope and stage.  It owes quite a bit to [Fergus Riche's motor board](https://github.com/fr293/motor_board), the hardware developed by [OpenScope](http://2015.igem.org/Team:Cambridge-JIC) and the Arduino-based motor controller used by a number of summer students working with Richard Bowman in Cambridge, particularly James Sharkey.  It is currently the motor board used by the [OpenFlexure Microscope](https://github.com/rwb27/openflexure_microscope/).

The repository contains three sets of files; the Arduino firmware in the ``arduino_code`` folder, a Python module in the ``openflexure_stage`` folder, and the PCB design in the ``pcb_design`` folder.

## Documentation
You can now see documentation at [Read the Docs](https://openflexure-stage.readthedocs.io/en/latest/index.html).

## Getting started

The first step is to make and populate the PCB.  You can get in touch with [WaterScope](http://www.waterscope.org/) to order one if they have stock, or alternatively have them made via [Kit Space](http://www.kitspace.org/) ([link to this project](https://kitspace.org/boards/github.com/rwb27/openflexure_nano_motor_controller/)).

Next you will need to download the Arduino sketch and upload it to the Arduino Nano.  You'll need some libraries, see the ``arduino_code`` folder's readme for that.

Lastly, you can control the stage using the ``openflexure_stage`` Python module.  This can be installed via pip, using ``pip install openflexure_stage``.  NB the master branch and the pip released version are currently not in sync, we're working to fix this.  If you want interactive control, you might want to check out the [OpenFlexure Microscope software](https://github.com/rwb27/openflexure_microscope_software/), which will run the stage together with the Raspberry Pi camera module.

## Credits

Firmware/Python code by Richard Bowman, University of Bath, PCB design & production by Sanga Valerian, STICLab.  Development of this board and software was funded by the EPSRC (EP/P029426/1), the Royal Commission for the Exhibition of 1851, and the University of Bath.

(c) The authors, 2017, released under CERN Open Hardware License (hardware designs) and GNU GPL v3.0 (software) 2017.
