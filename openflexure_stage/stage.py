#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
OpenFlexure Stage module

This Python code deals with the computer (Raspberry Pi) side of communicating
with the OpenFlexure Motor Controller.

It is (c) Richard Bowman 2017 and released under GNU GPL v3
"""
from __future__ import print_function, division
import time
from .extensible_serial_instrument import ExtensibleSerialInstrument, OptionalModule, QueriedProperty, EIGHTBITS, PARITY_NONE, STOPBITS_ONE
import numpy as np
import sys
import re
import warnings

class OpenFlexureStage(ExtensibleSerialInstrument):
    """Class managing serial communications with an Openflexure Motor Controller

    The `OpenFlexureStage` class handles setting up communications with the stage,
    wraps the various serial commands in Python methods, and provides iterators and
    context managers to simplify opening/closing the hardware connection and some
    other tasks like conducting a linear scan.

    Arguments to the constructor are passed to the constructor of
    :class:`openflexure_stage.extensible_serial_instrument.ExtensibleSerialInstrument`,
    most likely the only one necessary is `port` which should be set to the serial port
    you will use to communicate with the motor controller.

    This class can be used as a context manager, i.e. it's encouraged to use it as::

       with OpenFlexureStage() as stage:
           stage.move_rel([1000,0,0])

    In that case, the serial port will automatically be closed at the end of the block,
    even if an error occurs.  Otherwise, be sure to call the
    :meth:`~.ExtensibleSerialInstrument.close()` method to release the serial port.
    """
    port_settings = {'baudrate':115200, 'bytesize':EIGHTBITS, 'parity':PARITY_NONE, 'stopbits':STOPBITS_ONE}
    """These are the settings for the stage's serial port, and can usually be left as default."""
    # position, step time and ramp time are get/set using simple serial
    # commands.
    position = QueriedProperty(get_cmd="p?", response_string=r"%d %d %d",
            doc="Get the position of the stage as a tuple of 3 integers.")
    step_time = QueriedProperty(get_cmd="dt?", set_cmd="dt %d", response_string="minimum step delay %d",
            doc="Get or set the minimum time between steps of the motors in microseconds.\n\n"
                "The step time is ``1000000/max speed`` in steps/second.  It is saved to EEPROM on "
                "the Arduino, so it will be persistent even if the motor controller is turned off.")
    ramp_time = QueriedProperty(get_cmd="ramp_time?", set_cmd="ramp_time %d", response_string="ramp time %d",
            doc="Get or set the acceleration time in microseconds.\n\n"
                "The stage will accelerate/decelerate between stationary and maximum speed over `ramp_time` "
                "microseconds.  Zero means the stage runs at full speed initially, with no accleration "
                "control.  Small moves may last less than `2*ramp_time`, in which case the acceleration "
                "will be the same, but the stage will never reach full speed.  It is saved to EEPROM on "
                "the Arduino, so it will be persistent even if the motor controller is turned off.")
    axis_names = ('x', 'y', 'z')
    """The names of the stage's axes.  NB this also defines the number of axes."""
    board = None
    """Once initialised, `board` is a string that identifies the firmware version."""
    supported_light_sensors = ["TSL2591","ADS1115"]
    """This is a list of the supported light sensor module types."""

    def __init__(self, *args, **kwargs):
        """Create a stage object.

        Arguments are passed to the constructor of
        :class:`openflexure_stage.extensible_serial_instrument.ExtensibleSerialInstrument`,
        most likely the only one necessary is `port` which should be set to the serial port
        you will use to communicate with the motor controller.  That's the first argument so
        it doesn't need to be named.
        """
        super(OpenFlexureStage, self).__init__(*args, **kwargs)
        try:
            # Request version from the board
            self.board = self.query("version").rstrip()
            # The slightly complicated regexp below will match the version string,
            # and store the version number in the "groups" of the regexp.  The version
            # number should be in the format 1.2 and the groups will be "1.2", "1", "2"
            # (for any number of elements).
            match = re.match(r"OpenFlexure Motor Board v(([\d]+)(?:\.([\d]+))+)", self.board)
            version = [int(g) for g in match.groups()[1:]]
            assert match, "Version string \"{}\" not recognised.".format(self.board)
            self.firmware_version = match.group(1)
            assert version[0] == 0, "This version of the Python module requires firmware v0.4"
            assert version[1] == 4, "This version of the Python module requires firmware v0.4"

            #Bit messy: Defining all valid modules as not available, then overwriting with available information if available.
            self.light_sensor = LightSensor(False)

            for module in self.list_modules():
                module_type=module.split(':')[0].strip()
                module_model=module.split(':')[1].strip()
                if module_type.startswith('Light Sensor'):
                    if module_model in self.supported_light_sensors:
                        self.light_sensor = LightSensor(True,parent=self,model=module_model)
                    else:
                        warnings.warn("Light sensor model \"%s\" not recognised."%(module_model),RuntimeWarning)
                elif module_type.startswith('Endstops'):
                    self.endstops = Endstops(True, parent=self, model=module_model)
                else:
                    warnings.warn("Module type \"{}\" not recognised.".format(module_type),RuntimeWarning)
            self.test_mode=False
        except Exception as e:
            # If an error occurred while setting up (e.g. because the board isn't connected or something)
            # make sure we close the serial port cleanly (otherwise it hangs open).
            self.close()
            e.args += ("You may need to update the firmware running on the Arduino.  See " \
                       "https://openflexure-stage.readthedocs.io/en/latest/firmware.html",)
            raise e

    @property
    def n_axes(self):
        """The number of axes this stage has."""
        return len(self.axis_names)

    _backlash = None
    @property
    def backlash(self):
        """The distance used for backlash compensation.

        Software backlash compensation is enabled by setting this property to a value
        other than `None`.  The value can either be an array-like object (list, tuple,
        or numpy array) with one element for each axis, or a single integer if all axes
        are the same.

        The property will always return an array with the same length as the number of
        axes.

        The backlash compensation algorithm is fairly basic - it ensures that we always
        approach a point from the same direction.  For each axis that's moving, the
        direction of motion is compared with ``backlash``.  If the direction is opposite,
        then the stage will overshoot by the amount in ``-backlash[i]`` and then move
        back by ``backlash[i]``.  This is computed per-axis, so if some axes are moving
        in the same direction as ``backlash``, they won't do two moves.
        """
        return self._backlash

    @backlash.setter
    def backlash(self, blsh):
        if blsh is None:
            self._backlash = None
        try:
            assert len(blsh) == self.n_axes
            self._backlash = np.array(blsh, dtype=np.int)
        except:
            self._backlash = np.array([int(blsh)]*self.n_axes, dtype=np.int)

    def move_rel(self, displacement, axis=None, backlash=True):
        """Make a relative move, optionally correcting for backlash.

        displacement: integer or array/list of 3 integers
        axis: None (for 3-axis moves) or one of 'x','y','z'
        backlash: (default: True) whether to correct for backlash.
        """
        if not backlash or self.backlash is None:
            return self._move_rel_nobacklash(displacement, axis=axis)

        if axis is not None:
            # backlash correction is easier if we're always in 3D
            # so this code just converts single-axis moves into all-axis moves.
            assert axis in self.axis_names, "axis must be one of {}".format(self.axis_names)
            move = np.zeros(self.n_axes, dtype=np.int)
            move[np.argmax(np.array(self.axis_names) == axis)] = int(displacement)
            displacement = move

        initial_move = np.array(displacement, dtype=np.int)
        # Backlash Correction
        # This backlash correction strategy ensures we're always approaching the
        # end point from the same direction, while minimising the amount of extra
        # motion.  It's a good option if you're scanning in a line, for example,
        # as it will kick in when moving to the start of the line, but not for each
        # point on the line.
        # For each axis where we're moving in the *opposite*
        # direction to self.backlash, we deliberately overshoot:
        initial_move -= np.where(self.backlash*displacement < 0,
                                 self.backlash,
                                 np.zeros(self.n_axes, dtype=self.backlash.dtype))
        self._move_rel_nobacklash(initial_move)
        if np.any(displacement - initial_move != 0):
            # If backlash correction has kicked in and made us overshoot, move
            # to the correct end position (i.e. the move we were asked to make)
            self._move_rel_nobacklash(displacement - initial_move)

    def _move_rel_nobacklash(self, displacement, axis=None):
        """Just make a move - no messing about with backlash correction!

        Arguments are as for move_rel, but backlash is False
        """
        if axis is not None:
            assert axis in self.axis_names, "axis must be one of {}".format(self.axis_names)
            self.query("mr{} {}".format(axis, int(displacement)))
        else:
            #TODO: assert displacement is 3 integers
            self.query("mr {} {} {}".format(*list(displacement)))

    def release_motors(self):
        """De-energise the stepper motor coils"""
        self.query("release")

    def move_abs(self, final, **kwargs):
        """Make an absolute move to a position

        NB the stage only accepts relative move commands, so this first
        queries the stage for its position, then instructs it to make about
        relative move.
        """
        new_position = final
        rel_mov = np.subtract(new_position, self.position)
        return self.move_rel(rel_mov, **kwargs)

    def focus_rel(self, z):
        """Move the stage in the Z direction by z micro steps."""
        self.move_rel([0, 0, z])

    def scan_linear(self, rel_positions, backlash=True, return_to_start=True):
        """Scan through a list of (relative) positions (generator fn)

        rel_positions should be an nx3-element array (or list of 3 element arrays).
        Positions should be relative to the starting position - not a list of relative moves.

        backlash argument is passed to move_rel

        if return_to_start is True (default) we return to the starting position after a
        successful scan.  NB we always attempt to return to the starting position if an
        exception occurs during the scan..
        """
        starting_position = self.position
        rel_positions = np.array(rel_positions)
        assert rel_positions.shape[1] == 3, ValueError("Positions should be 3 elements long.")
        try:
            self.move_rel(rel_positions[0], backlash=backlash)
            yield 0

            for i, step in enumerate(np.diff(rel_positions, axis=0)):
                self.move_rel(step, backlash=backlash)
                yield i+1
        except Exception as e:
            return_to_start = True # always return to start if it went wrong.
            raise e
        finally:
            if return_to_start:
                self.move_abs(starting_position, backlash=backlash)

    def scan_z(self, dz, **kwargs):
        """Scan through a list of (relative) z positions (generator fn)

        This function takes a 1D numpy array of Z positions, relative to
        the position at the start of the scan, and converts it into an
        array of 3D positions with x=y=0.  This, along with all the
        keyword arguments, is then passed to ``scan_linear``.
        """
        return self.scan_linear([[0,0,z] for z in dz], **kwargs)

    def __enter__(self):
        """When we use this in a with statement, remember where we started."""
        self._position_on_enter = self.position
        return self

    def __exit__(self, type, value, traceback):
        """The end of the with statement.  Reset position if it went wrong.
        NB the instrument is closed when the object is deleted, so we don't
        need to worry about that here.
        """
        if type is not None:
            print("An exception occurred inside a with block, resetting " +
                  "position to its value at the start of the with block")
            try:
                time.sleep(0.5)
                self.move_abs(self._position_on_enter)
            except Exception as e:
                print("A further exception occurred when resetting position: {}".format(e))
            print("Move completed, raising exception...")
            raise value #propagate the exception

    def query(self, message, *args, **kwargs):
        """Send a message and read the response.  See ExtensibleSerialInstrument.query()"""
        time.sleep(0.001) # This is to protect the stage from us talking too fast!
        return ExtensibleSerialInstrument.query(self, message, *args, **kwargs)

    def list_modules(self):
        """Return a list of strings detailing optional modules.

        Each module will correspond to a string of the form ``Module Name: Model``
        """
        modules =  self.query("list_modules",multiline=True,termination_line="--END--\r\n").split('\r\n')[:-2]
        return [str(module) for module in modules]

    def print_help(self):
        """Print the stage's built-in help message."""
        print(self.query("help",multiline=True,termination_line="--END--\r\n"))

    @property
    def test_mode(self):
        """ Get or set test mode

            In test mode
                * Stage may return extra information
                * When homing, the stage will remain at the 0 position
                * Position will not be reset when an endstop is hit
        """
        return self._test_mode

    @test_mode.setter
    def test_mode(self, value):
        self._test_mode=value
        self.query("test_mode "+"on" if value else "off")
        #propagate change to optional modules
        if hasattr(self, 'endstops') and self.endstops:
            self.endstops.test_mode=value
        if hasattr(self, 'light_sensor') and self.light_sensor:
            self.light_sensor.test_mode=value



class LightSensor(OptionalModule):
    """An optional module giving access to the light sensor.

    If a light sensor is enabled in the motor controller's firmware, then
    the :class:`openflexure_stage.OpenFlexureStage` will gain an optional
    module which is an instance of this class.  It can be used to access
    the light sensor (usually via the I2C bus).
    """
    valid_gains = None
    _valid_gains_int = None
    integration_time = QueriedProperty(get_cmd="light_sensor_integration_time?",
                                       set_cmd="light_sensor_integration_time %d",
                                       response_string="light sensor integration time %d ms",
                                       doc="Get or set the integration time of the light sensor in milliseconds.")
    intensity = QueriedProperty(get_cmd="light_sensor_intensity?", response_string="%d",
                                doc="Read the current intensity measured by the light sensor (arbitrary units).")

    def __init__(self,available,parent=None,model="Generic"):
        super(LightSensor, self).__init__(available,parent=parent,module_type="LightSensor",model=model)
        if available:
            self.valid_gains = self.__get_gain_values()
            self._valid_gains_int = [int(g) for g in self.valid_gains]

    @property
    def gain(self):
        """"Get or set the current gain value of the light sensor.

        Valid gain values are defined in the `valid_gains` property, and should be floating-point numbers."""
        self.confirm_available()
        gain = self._parent.query('light_sensor_gain?')
        M = re.search('[0-9\.]+(?=x)',gain)
        assert M is not None, "Cannot read gain string: \"{}\"".format(gain)
        #gain is a float as non integer gains exist but are set with floor of value
        return float(M.group())

    @gain.setter
    def gain(self,val):
        self.confirm_available()
        assert int(val) in self._valid_gains_int, "Gain {} not valid must be one of: {}".format(val,self.valid_gains)
        gain = self._parent.query('light_sensor_gain %d'%(int(val)))
        M = re.search('[0-9\.]+(?=x)',gain)
        assert M is not None, "Cannot read gain string: \"{}\"".format(gain)
        #gain is a float as non integer gains exist but are set with floor of value
        assert int(val) == int(float(M.group())), 'Gain of {} set, \"{}\" returned'.format(val,gain)

    def __get_gain_values(self):
        """Read the allowable values for the light sensor's gain.

        This function will attempt to return a list of floating-point numbers which may
        be used as values of the `gain` property.  If the stage returns non-floating-point
        values, the list will be of strings.
        """
        self.confirm_available()
        gains = self._parent.query('light_sensor_gain_values?')
        try:
            M = re.findall('[0-9\.]+(?=x)',gains)
            return [float(gain) for gain in M]
        except:
            # Fall back to strings if we don't get floats (unlikely)
            gain_strings = gains[20:].split(", ")
            return gain_strings

class Endstops(OptionalModule):
    """An optional module for use with endstops.

    If endstops are installed in the firmware the :class:`openflexure_stage.OpenFlexureStage`
    will gain an optional module which is an instance of this class.  It can be used to retrieve
    the type, state of the endstops, read and write maximum positions, and home.
    """

    installed=[]
    test_mode=False
    """List of installed endstop types (min, max, soft)"""

    def __init__(self,available,parent=None,model="min"):
        super(Endstops, self).__init__(available,parent=parent,model="Endstops")
        self.installed=model.split(' ')

    status = QueriedProperty(get_cmd="endstops?", response_string=r"%d %d %d",
                            doc="Get endstops status as {-1,0,1} for {min,no,max} endstop triggered for each axis")
    maxima = QueriedProperty(get_cmd="max_p?", set_cmd="max %d %d %d", response_string="%d %d %d",
                            doc="Vector of maximum positions, homing to max endstops will measure this, "+
                                "can be set to a known value for use with max only and min+soft endstops")

    def home(self, direction="min", axes=['x','y','z']):
        """ Home given/all axes in the given direction (min/max/both)

            :param direction: one of {min,max,both}
            :param axes: list of axes e.g. ['x','y']
        """
        ax=0
        if 'x' in axes:
            ax+=1
        if 'y' in axes:
            ax+=2
        if 'z' in axes:
            ax+=3

        if direction == "min" or direction == "both":
            if self.test_mode:
                return self._parent.query('home_min {}'.format(ax),multiline=True,termination_line="done.\r\n")
            else:
                self._parent.query('home_min {}'.format(ax))
        if direction == "max" or direction == "both":
            if self.test_mode:
                return self._parent.query('home_max {}'.format(ax),multiline=True,termination_line="done.\r\n")
            else:
                self._parent.query('home_max {}'.format(ax))

