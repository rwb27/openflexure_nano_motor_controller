# -*- coding: utf-8 -*-
"""
OpenFlexure Stage module

This Python code deals with the computer (Raspberry Pi) side of communicating
with the OpenFlexure Motor Controller.

It is (c) Richard Bowman 2017 and released under GNU GPL v3
"""
from __future__ import print_function, division
import time
from basic_serial_instrument import QueriedProperty
from .stage import OpenFlexureStage
import numpy as np

class OpenFlexureStageWithLightSensor(OpenFlexureStage):
    """An ``OpenFlexureStage`` with support for a light sensor."""
    light_sensor_gain = QueriedProperty(get_cmd="light_sensor_gain?", set_cmd="light_sensor_gain %f", response_string="light sensor gain %f")
    light_sensor_integration_time = QueriedProperty(get_cmd="light_sensor_integration_time?", set_cmd="light_sensor_integration_time %d", response_string="light sensor integration time %d ms")
    light_sensor_intensity = QueriedProperty(get_cmd="light_sensor_intensity?", response_string="%d")
      
    @property
    def light_sensor_gain_values(self):
        """Allowable values for the light sensor's gain"""
        response = self.query("light_sensor_gain_values?")
        assert response.startswith("light sensor gains: ")
        gain_strings = response[20:].split(", ")
        try:
            return [float(g.strip('x')) for g in gain_strings]
        except:
            return gain_strings
    
