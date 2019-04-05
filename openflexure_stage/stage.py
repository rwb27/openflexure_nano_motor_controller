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
from .sangaboard import Sangaboard
import numpy as np

class OpenFlexureStage():
    """Class managing serial communications with the motors for an Openflexure stage

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
    
    board = None
    """Once initialised, `board` is is the motor controller board"""
    boardtype = None
    supported_light_sensors = ["TSL2591","ADS1115"]
    """This is a list of the supported light sensor module types."""
    
    SANGABOARD = 0

    def __init__(self, port=None,board='Sangaboard', **kwargs):
        """Create a stage object.

        port: The port on which is used to communicat to the motor board
        
        board: Sets the motor board used in the openflexure stage. Currently Sangaboard is
        the only board supported.
        
        Other arguments are passed to the constructor of the class for the selected board
        most likely the only one necessary is `port` which should be set to the serial port
        you will use to communicate with the motor controller.  That's the first argument so
        it doesn't need to be named.
        """
        
        if board.startswith('Sangaboard'):
            self.board = Sangaboard(port,**kwargs)
            self.boardtype = self.SANGABOARD
        else:
            assert False, "Board type not recognised"
        
        except Exception as e:
            # If an error occurred while setting up (e.g. because the board isn't connected or something)
            # make sure we close the serial port cleanly (otherwise it hangs open).
            self.close()
            e.args += ("You may need to update the firmware running on the motorboard.  See " \
                       "https://openflexure-stage.readthedocs.io/en/latest/firmware.html",)
            raise e
    
    
    def open(self, port=None, quiet=True):
        """Open communications with the serial port.
        
        If no port is specified, it will attempt to autodetect.  If quiet=True
        then we don't warn when ports are opened multiple times.
        """
        self.board.open(port,quiet)
    
    def close(self):
        self.board.close()

    def __del__(self):
        """Close the port when the object is deleted
        
        NB if the object is created in a with statement, this will cause
        the port to be closed at the end of the with block."""
        self.close()

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
    
    
    
    @property
    def n_axes(self):
        """The number of axes this stage has."""
        return len(self.board.axis_names)

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
            return self.board.move_rel(displacement, axis=axis)

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
        self.board.move_rel(initial_move)
        if np.any(displacement - initial_move != 0):
            # If backlash correction has kicked in and made us overshoot, move
            # to the correct end position (i.e. the move we were asked to make)
            self.board.move_rel(displacement - initial_move)

    def release_motors(self):
        """De-energise the stepper motor coils"""
        self.board.release_motors()

    def move_abs(self, final, **kwargs):
        """Make an absolute move to a position
        """
        self.board.move_abs(final, **kwargs)

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

    def query(self, message, *args, **kwargs):
        self.board.query( message, *args, **kwargs)

    def list_modules(self):
        """Return a list of strings detailing optional modules.

        Each module will correspond to a string of the form ``Module Name: Model``
        """
        return self.board.list_modules()

    def print_help(self):
        self.board.print_help()

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

