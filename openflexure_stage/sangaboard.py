#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Sangaboard module

This Python code allows control of the Sangaboard

It is (c) Richard Bowman & Julian Stirling 2019 and released under GNU GPL v3
"""
from __future__ import print_function, division
import time
from .extensible_serial_instrument import ExtensibleSerialInstrument, OptionalModule, QueriedProperty, EIGHTBITS, PARITY_NONE, STOPBITS_ONE
import re
import warnings

class Sangaboard(ExtensibleSerialInstrument):
    """Class managing serial communications with a Sangaboard

    The `Sangaboard` class handles setting up communications with the sangaboard,
    wraps the various serial commands in Python methods, and provides iterators and
    context managers to simplify opening/closing the hardware connection and some
    other tasks like conducting a linear scan.

    Arguments to the constructor are passed to the constructor of
    :class:`Sangaboard.extensible_serial_instrument.ExtensibleSerialInstrument`,
    most likely the only one necessary is `port` which should be set to the serial port
    you will use to communicate with the motor controller.

    This class can be used as a context manager, i.e. it's encouraged to use it as::

       with Sangaboard() as sb:
           sb.move_rel([1000,0,0])

    In that case, the serial port will automatically be closed at the end of the block,
    even if an error occurs.  Otherwise, be sure to call the
    :meth:`~.ExtensibleSerialInstrument.close()` method to release the serial port.
    """
    port_settings = {'baudrate':115200, 'bytesize':EIGHTBITS, 'parity':PARITY_NONE, 'stopbits':STOPBITS_ONE}
    """These are the settings for the sangaboards serial port, and can usually be left as default."""
    # position, step time and ramp time are get/set using simple serial
    # commands.
    position = QueriedProperty(get_cmd="p?", response_string=r"%d %d %d",
            doc="Get the position of the axes as a tuple of 3 integers.")
    step_time = QueriedProperty(get_cmd="dt?", set_cmd="dt %d", response_string="minimum step delay %d",
            doc="Get or set the minimum time between steps of the motors in microseconds.\n\n"
                "The step time is ``1000000/max speed`` in steps/second.  It is saved to EEPROM on "
                "the sangaboard, so it will be persistent even if the motor controller is turned off.")
    ramp_time = QueriedProperty(get_cmd="ramp_time?", set_cmd="ramp_time %d", response_string="ramp time %d",
            doc="Get or set the acceleration time in microseconds.\n\n"
                "The motors will accelerate/decelerate between stationary and maximum speed over `ramp_time` "
                "microseconds.  Zero means the motor runs at full speed initially, with no accleration "
                "control.  Small moves may last less than `2*ramp_time`, in which case the acceleration "
                "will be the same, but the motor will never reach full speed.  It is saved to EEPROM on "
                "the sangaboard, so it will be persistent even if the motor controller is turned off.")
    axis_names = ('x', 'y', 'z')
    """The names of the sangaboard's axes.  NB this also defines the number of axes."""
    firmware = None
    """Once initialised, `firmware` is a string that identifies the firmware version."""

    def __init__(self, port=None, **kwargs):
        """Create a sangaboard object.

        Arguments are passed to the constructor of
        :class:`Sangaboard.extensible_serial_instrument.ExtensibleSerialInstrument`,
        most likely the only one necessary is `port` which should be set to the serial port
        you will use to communicate with the motor controller.  That's the first argument so
        it doesn't need to be named.
        """
        super(Sangaboard, self).__init__(port, **kwargs)
        try:
            # Request firmware version from the board
            self.firmware = self.query("version").rstrip()
            # The slightly complicated regexp below will match the version string,
            # and store the version number in the "groups" of the regexp.  The version
            # number should be in the format 1.2 and the groups will be "1.2", "1", "2"
            # (for any number of elements).
            match = re.match(r"Sangaboard Firmware v(([\d]+)(?:\.([\d]+))+)", self.firmware)
            version = [int(g) for g in match.groups()[1:]]
            assert match, "Version string \"{}\" not recognised.".format(self.firmware)
            self.firmware_version = match.group(1)
            assert version[0] == 0, "This version of the Python module requires firmware v0.5"
            assert version[1] == 5, "This version of the Python module requires firmware v0.5"

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
            e.args += ("You may need to update the firmware running on the sangaboard.")
            raise e

    def move_rel(self, displacement, axis=None):
        """Make a relative move.

        displacement: integer or array/list of 3 integers
        axis: None (for 3-axis moves) or one of 'x','y','z'
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

        NB the sangaboard only accepts relative move commands, so this first
        queries the board for its position, then instructs it to make about
        relative move.
        """
        rel_mov = [f_pos-i_pos for f_pos,i_pos in zip(final,self.position)]
        return self.move_rel(rel_mov, **kwargs)

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

class LightSensor(OptionalModule):
    """An optional module giving access to the light sensor.

    If a light sensor is enabled in the motor controller's firmware, then
    the :class:`sangaboard.Sangaboard` will gain an optional
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

    If endstops are installed in the firmware the :class:`sangaboard.Sangaboard`
    will gain an optional module which is an instance of this class.  It can be used to retrieve
    the type, state of the endstops, read and write maximum positions, and home.
    """

    installed=[]
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
            self._parent.query('home_min {}'.format(ax))
        if direction == "max" or direction == "both":
            self._parent.query('home_max {}'.format(ax))

