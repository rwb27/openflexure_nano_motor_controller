.. Openflexure Stage documentation master file, created by
   sphinx-quickstart on Tue Feb 13 09:49:39 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

The OpenFlexure Stage module
=============================================

This module exposes the functions of the OpenFlexure Nano Motor Controller (AKA
the "Sangaboard") in a
friendly Python class.  It allows the stage to be moved, as well as providing
properties that allow it to be configured.  Various context managers and
generator functions are provided to simplify opening/closing the hardware, and
common operations such as scanning through a list of points.

All of the functionality is accessed through the :class:`openflexure_stage.stage.OpenFlexureStage` class,
which optionally includes a :class:`openflexure_stage.stage.LightSensor` module if the firmware and
hardware are set up to include this and a :class:`openflexure_stage.stage.Endstops` module if the firmware
is compiled with endstop support

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   hardware
   firmware
   getting_started
   API Documentation <source/modules>

.. automodule:: openflexure_stage


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
