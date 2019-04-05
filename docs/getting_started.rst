Getting started
=================
To use the motor controller from Python, you will first need to install this module.  It can be installed using `pip` in the usual way, which will also require the packages that it depends on (:mod:`future` and :mod:`pyserial`).  The simplest way to use the module is this::

   from openflexure_stage import OpenFlexureStage
   with OpenFlexureStage() as stage:
       stage.move_rel([100,0,0])
       print(stage.position)
       stage.move_rel([-100,0,0])
   
By default, it will use the first available serial port - if you are using a Raspberry Pi and you don't have any other USB serial devices connected, this will usually work.  If not, you need to specify the serial port in the constructor::

   OpenFlexureStage("/dev/ttyUSB0")
   
The name of the serial port will depend on your operating system - Linux typically assigns names that look like ``/dev/ttyUSB0`` while Windows will often give it a name like ``COM4``.

Make sure you close the stage after you're finished with it - the best way to do this is using a `with` block, but you can also call :meth:`~.ExtensibleSerialInstrument.close` manually if required.

Moving the stage
-----------------
The most basic thing you are likely to want to do with the stage.  This is done with :meth:`~.OpenFlexureStage.move_rel` most of the time, though it's also possible to make absolute moves.  The Sangaboard keeps track of position in firmware, and will return its position if you query :attr:`~.OpenFlexureStage.position`.

Adjusting settings
--------------------
There are a number of properties of your :class:`~.OpenFlexureStage` object that can be used to change the way it works:

* :attr:`~.OpenFlexureStage.backlash`: software backlash compensation

* :attr:`~.OpenFlexureStage.ramp_time`: acceleration control

* :attr:`~.OpenFlexureStage.step_time`: define the maximum speed

Using optional features
-------------------------
If you compile support for it, you can add a light sensor to the stage, which is accessed as :attr:`~.OpenFlexureStage.light_sensor`.  This returns a :class:`.LightSensor` which allows you to control the gain (if possible) and read the light intensity.

If you compile support for it, you can add mechanical endstops, which are accessed as :attr:`~.OpenFlexureStage.endstops`. This returns :class:`.Endstops` which allows you to home the axes, check endstop status, and control soft endstop position (if enabled).
