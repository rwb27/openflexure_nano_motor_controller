# -*- coding: utf-8 -*-
"""
Created on Sun Jun 18 21:06:04 2017

@author: richa
"""

import nplab
import nplab.instrument.serial_instrument
import time

class OpenFlexureStage(nplab.instrument.serial_instrument.SerialInstrument):
    port_settings = {'baudrate':115200}

s = OpenFlexureStage('COM4')

print s.readline()
time.sleep(1)
#print s.query("mrx 1000")
#time.sleep(1)
#print s.query("mrx -1000")

for axis in ['x', 'y', 'z']:
    for move in [-512, 512, 4096, -4096]:
        print "moving {} by {}".format(axis, move)
        qs = "mr{} {}".format(axis, move)
        print qs + ": " + s.query(str(qs))
        print "Position: {}".format(s.query("p?"))
s.close()