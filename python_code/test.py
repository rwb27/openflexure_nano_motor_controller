# -*- coding: utf-8 -*-
"""
Created on Sun Jun 18 21:06:04 2017

@author: richa
"""

import nplab
import nplab.instrument.serial_instrument
import time
from basic_serial_instrument import BasicSerialInstrument, QueriedProperty
import numpy as np

class OpenFlexureStage(BasicSerialInstrument):
    port_settings = {'baudrate':115200}
    position = QueriedProperty(get_cmd="p?", response_string=r"%d %d %d")
    step_time = QueriedProperty(get_cmd="dt?", set_cmd="dt %d", response_string="minimum step delay %d")
    ramp_time = QueriedProperty(get_cmd="ramp_time?", set_cmd="ramp_time %d", response_string="ramp time %d")
    
    def move_rel(self, displacement, axis=None):
        if axis is not None:
            assert axis in ['x', 'y', 'z'], "Axis must be x, y, or z"
            self.query("mr{} {}".format(axis, int(displacement)))
        else:
            #TODO: assert displacement is 3 integers
            self.query("mr {} {} {}".format(*list(displacement)))
    
    def release_motors(self):
        """De-energise the stepper motor coils"""
        self.query("release")

s = OpenFlexureStage('COM4')

print s.readline()
time.sleep(1)
#print s.query("mrx 1000")
#time.sleep(1)
#print s.query("mrx -1000")

#first, try a bunch of single-axis moves with and without acceleration
for rt in [-1, 500000]:
    s.ramp_time = rt
    for axis in ['x', 'y', 'z']:
        for move in [-512, 512, 1024, -1024]:
            print "moving {} by {}".format(axis, move)
            qs = "mr{} {}".format(axis, move)
            print qs + ": " + s.query(str(qs))
            print "Position: {}".format(s.position)

time.sleep(0.5)
for i in range(10):
    print s.position
#next, describe a circle with the X and Y motors.  This is a harder test!
radius = 1024;
#print "Setting ramp time: <"+s.query("ramp_time -1")+">" #disable acceleration
#print "Extra Line: <"+s.readline()+">"
s.ramp_time = -1
for a in np.linspace(0, 2*np.pi, 50):
    print "moving to angle {}".format(a)
    oldpos = np.array(s.position)
    print "Position: {}".format(oldpos)
    newpos = np.array([np.cos(a), np.sin(a), 0]) * radius
    displacement = newpos - oldpos
    s.move_rel(list(displacement))
    

s.close()