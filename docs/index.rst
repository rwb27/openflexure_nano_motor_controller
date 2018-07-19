.. Openflexure Stage documentation master file, created by
   sphinx-quickstart on Tue Feb 13 09:49:39 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Openflexure Stage's documentation!
=============================================

This module exposes the functions of the OpenFlexure Nano Motor Controller in a
friendly Python class.  It allows the stage to be moved, as well as providing
properties that allow it to be easily configured.  Various context managers and
generator functions are provided to simplify opening/closing the hardware, and 
common operations such as scanning through a list of points.

There are two classes provided by this module; :class:`OpenFlexureStage` and 
:class:`OpenFlexureStageWithLightSensor` depending on whether your stage is 
set up to read a light sensor.

.. toctree::
   :maxdepth: 2
   :caption: Contents:

   API


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
