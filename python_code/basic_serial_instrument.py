# -*- coding: utf-8 -*-
"""
Created on Mon Jun 19 11:42:15 2017

This is a chopped-out class from nplab (http://www.github.com/nanophotonics/nplab)

It is a basic serial instrument class for things that talk on serial ports.

@author: richard bowman (c) 2017, released under GNU GPL
"""

import re
from functools import partial
import threading
import serial
import serial.tools.list_ports
from serial import FIVEBITS, SIXBITS, SEVENBITS, EIGHTBITS
from serial import PARITY_NONE, PARITY_EVEN, PARITY_ODD, PARITY_MARK, PARITY_SPACE
from serial import STOPBITS_ONE, STOPBITS_ONE_POINT_FIVE, STOPBITS_TWO
import io

class BasicSerialInstrument(object):
    """
    Basic Serial Instrument
    ======================

    An instrument that communicates by sending strings back and forth over serial

    This base class provides commonly-used mechanisms that support the use of
    serial instruments.  Most interactions with this class involve
    a call to the `query` method.  This writes a message and returns the reply.
    This has been hacked together from the nplab MEssageBusInstrument and SerialInstrument
    classes.
    
    Threading Notes
    ---------------
    
    The message bus protocol includes a property, `communications_lock`.  All
    commands that use the communications bus should be protected by this lock.
    It's also permissible to use it to protect sequences of calls to the bus 
    that must be atomic (e.g. a multi-part exchange of messages).  However, try
    not to hold it too long - or odd things might happen if other threads are 
    blocked for a long time.  The lock is reentrant so there's no issue with
    acquiring it twice.
    """
    termination_character = "\n" #: All messages to or from the instrument end with this character.
    termination_line = None #: If multi-line responses are recieved, they must end with this string
    ignore_echo = False
    port_settings = {}

    def __init__(self, port=None, **kwargs):
        """
        Set up the serial port and so on.
        """
        self.port_settings.update(kwargs)
        self.open(port, False) # Eventually this shouldn't rely on init...

    def open(self, port=None, quiet=True):
        """Open communications with the serial port.
        
        If no port is specified, it will attempt to autodetect.  If quiet=True
        then we don't warn when ports are opened multiple times.
        """
        with self.communications_lock:
            if hasattr(self,'_ser') and self._ser.isOpen():
                if not quiet: print "Warning: attempted to open an already-open port!"
                return
            if port is None: port=self.find_port()
            assert port is not None, "We don't have a serial port to open, meaning you didn't specify a valid port and autodetection failed.  Are you sure the instrument is connected?"
            self._ser = serial.Serial(port,**self.port_settings)
            self._ser_io = io.TextIOWrapper(io.BufferedRWPair(self._ser, self._ser, 1),  
                                           newline = self.termination_character,
                                           line_buffering = True)
            #the block above wraps the serial IO layer with a text IO layer
            #this allows us to read/write in neat lines.  NB the buffer size must
            #be set to 1 byte for maximum responsiveness.
            assert self.test_communications(), "The instrument doesn't seem to be responding.  Did you specify the right port?"
    
    def close(self):
        """Release the serial port"""
        with self.communications_lock:
            try:
                self._ser.close()
            except Exception as e:
                print "The serial port didn't close cleanly:", e
                
    def __del__(self):
        """Close the port when the object is deleted
        
        NB if the object is created in a with statement, this will cause
        the port to be closed at the end of the with block."""
        self.close()

    def __enter__(self):
        """When we use this in a with statement, it should be opened already"""
        self._position_on_enter = self.position
        return self

    def __exit__(self, type, value, traceback):
        """Close down the instrument.  This happens in __del__ though."""
        if type is not None:
            print "An exception occurred inside a with block, resetting "
            "position to its value at the start of the with block"
            self.move_abs(self._position_on_enter)
        
    def write(self,query_string):
        """Write a string to the serial port"""
        with self.communications_lock:
            assert self._ser.isOpen(), "Warning: attempted to write to the serial port before it was opened.  Perhaps you need to call the 'open' method first?"
#            try:        
#                if self._ser.outWaiting()>0: self._ser.flushOutput() #ensure there's nothing waiting
#            except AttributeError:
#                if self._ser.out_waiting>0: self._ser.flushOutput() #ensure there's nothing waiting
            self._ser.write(query_string+self.termination_character)

    def flush_input_buffer(self):
        """Make sure there's nothing waiting to be read, and clear the buffer if there is."""
        with self.communications_lock:
            if self._ser.inWaiting()>0: self._ser.flushInput()
    def readline(self, timeout=None):
        """Read one line from the serial port."""
        with self.communications_lock:
            return self._ser_io.readline().replace(self.termination_character,"\n")

    _communications_lock = None
    @property
    def communications_lock(self):
        """A lock object used to protect access to the communications bus"""
        # This requires initialisation but our init method won't be called - so
        # the property initialises it on first use.
        if self._communications_lock is None:
            self._communications_lock = threading.RLock()
        return self._communications_lock

#    def write(self,query_string):
#        """Write a string to the unerlying communications port"""
#        with self.communications_lock:
#            raise NotImplementedError("Subclasses of MessageBusInstrument must override the write method!")
            
#    def flush_input_buffer(self):
#        """Make sure there's nothing waiting to be read.
#
#        This function should be overridden to make sure nothing's lurking in
#        the input buffer that could confuse a query.
#        """
#        with self.communications_lock:
#            pass
    
#    def readline(self, timeout=None):
#        """Read one line from the underlying bus.  Must be overriden."""
#        with self.communications_lock:
#            raise NotImplementedError("Subclasses of MessageBusInstrument must override the readline method!")
            
    def read_multiline(self, termination_line=None, timeout=None):
        """Read one line from the underlying bus.  Must be overriden.

        This should not need to be reimplemented unless there's a more efficient
        way of reading multiple lines than multiple calls to readline()."""
        with self.communications_lock:
            if termination_line is None:
                termination_line = self.termination_line
            assert isinstance(termination_line, str), "If you perform a multiline query, you must specify a termination line either through the termination_line keyword argument or the termination_line property of the NPSerialInstrument."
            response = ""
            last_line = "dummy"
            while termination_line not in last_line and len(last_line) > 0: #read until we get the termination line.
                last_line = self.readline(timeout)
                response += last_line
            return response
            
    def query(self,queryString,multiline=False,termination_line=None,timeout=None):
        """
        Write a string to the stage controller and return its response.

        It will block until a response is received.  The multiline and termination_line commands
        will keep reading until a termination phrase is reached.
        """
        with self.communications_lock:
            self.flush_input_buffer()
            self.write(queryString)
            if self.ignore_echo == True: # Needs Implementing for a multiline read!
                first_line = self.readline(timeout).strip()
                if first_line == queryString:
                    return self.readline(timeout).strip()
                else:
                    print 'This command did not echo!!!'
                    return first_line
    
            if termination_line is not None:
                multiline = True
            if multiline:
                return self.read_multiline(termination_line)
            else:
                return self.readline(timeout).strip() #question: should we strip the final newline?
    def parsed_query(self, query_string, response_string=r"%d", re_flags=0, parse_function=None, **kwargs):
        """
        Perform a query, returning a parsed form of the response.

        First query the instrument with the given query string, then compare
        the response against a template.  The template may contain text and
        placeholders (e.g. %i and %f for integer and floating point values
        respectively).  Regular expressions are also allowed - each group is
        considered as one item to be parsed.  However, currently it's not
        supported to use both % placeholders and regular expressions at the
        same time.

        If placeholders %i, %f, etc. are used, the returned values are
        automatically converted to integer or floating point, otherwise you
        must specify a parsing function (applied to all groups) or a list of
        parsing functions (applied to each group in turn).
        """

        response_regex = response_string
        noop = lambda x: x #placeholder null parse function
        placeholders = [ #tuples of (regex matching placeholder, regex to replace it with, parse function)
            (r"%c",r".", noop),
            (r"%(\d+)c",r".{\1}", noop), #TODO support %cn where n is a number of chars
            (r"%d",r"[-+]?\d+", int),
            (r"%[eEfg]",r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?", float),
            (r"%i",r"[-+]?(?:0[xX][\dA-Fa-f]+|0[0-7]*|\d+)", lambda x: int(x, 0)), #0=autodetect base
            (r"%o",r"[-+]?[0-7]+", lambda x: int(x, 8)), #8 means octal
            (r"%s",r"\S+",noop),
            (r"%u",r"\d+",int),
            (r"%[xX]",r"[-+]?(?:0[xX])?[\dA-Fa-f]+",lambda x: int(x, 16)), #16 forces hexadecimal
        ]
        matched_placeholders = []
        for placeholder, regex, parse_fun in placeholders:
            response_regex = re.sub(placeholder, '('+regex+')', response_regex) #substitute regex for placeholder
            matched_placeholders.extend([(parse_fun, m.start()) for m in re.finditer(placeholder, response_string)]) #save the positions of the placeholders
        if parse_function is None:
            parse_function = [f for f, s in sorted(matched_placeholders, key=lambda m: m[1])] #order parse functions by their occurrence in the original string
        if not hasattr(parse_function,'__iter__'):
            parse_function = [parse_function] #make sure it's a list.

        reply = self.query(query_string, **kwargs) #do the query
        res = re.search(response_regex, reply, flags=re_flags)
        if res is None:
            raise ValueError("Stage response to '%s' ('%s') wasn't matched by /%s/ (generated regex /%s/" % (query_string, reply, response_string, response_regex))
        try:
            parsed_result= [f(g) for f, g in zip(parse_function, res.groups())] #try to apply each parse function to its argument
            if len(parsed_result) == 1:
                return parsed_result[0]
            else:
                return parsed_result
        except ValueError:
            print "Parsing Error"
            print "Matched Groups:", res.groups()
            print "Parsing Functions:", parse_function
            raise ValueError("Stage response to %s ('%s') couldn't be parsed by the supplied function" % (query_string, reply))
    def int_query(self, query_string, **kwargs):
        """Perform a query and return the result(s) as integer(s) (see parsedQuery)"""
        return self.parsed_query(query_string, "%d", **kwargs)
    def float_query(self, query_string, **kwargs):
        """Perform a query and return the result(s) as float(s) (see parsedQuery)"""
        return self.parsed_query(query_string, "%f", **kwargs)

    #@staticmethod  # this was an attempt at making a property factory - now using a descriptor
    #def queried_property(self, get_cmd, set_cmd, dtype='float', docstring=''):
    #    get_func = self.float_query if dtype=='float' else self.query
    #    return property(fget=partial(get_func, get_cmd), fset=self.write, docstring=docstring)
    def test_communications(self):
        """Check if the device is available on the current port.  
        
        This should be overridden by subclasses.  Assume the port has been
        successfully opened and the settings are as defined by self.port_settings.
        Usually this function sends a command and checks for a known reply."""
        with self.communications_lock:
            return True
            
    def find_port(self):
        """Iterate through the available serial ports and query them to see
        if our instrument is there."""
        with self.communications_lock:
            success = False
            for port_name, _, _ in serial.tools.list_ports.comports(): #loop through serial ports, apparently 256 is the limit?!
                try:
                    print "Trying port",port_name
                    self.open(port_name)
                    success = True
                    print "Success!"
                except:
                    pass
                finally:
                    try:
                        self.close()
                    except:
                        pass #we don't care if there's an error closing the port...
                if success:
                    break #again, make sure this happens *after* closing the port
            if success:
                return port_name
            else:
                return None


class QueriedProperty(object):
    """A Property interface that reads and writes from the instrument on the bus.
    
    This returns a property-like (i.e. a descriptor) object.  You can use it
    in a class definition just like a property.  The property it creates will
    interact with the instrument over the communication bus to set and retrieve
    its value.
    
    Arguments:
    get_cmd: the string sent to the instrument to obtain the value
    set_cmd: the string used to set the value (use {} or % placeholders)
    validate: a list of allowable values
    valrange: a maximum and minimum value
    fdel: a function to call when it's deleted
    doc: the docstring
    response_string: supply a % code (as you would for response_string in a
        ``BasicSerialInstrument.parsed_query``)
    ack_writes: set to "readline" to discard a line of input after writing.
    """
    def __init__(self, get_cmd=None, set_cmd=None, validate=None, valrange=None,
                 fdel=None, doc=None, response_string=None, ack_writes="no"):
        self.response_string = response_string
        self.get_cmd = get_cmd
        self.set_cmd = set_cmd
        self.validate = validate
        self.valrange = valrange
        self.fdel = fdel
        self.ack_writes = ack_writes
        self.__doc__ = doc
        

    # TODO: standardise the return (single value only vs parsed result), consider bool
    def __get__(self, obj, objtype=None):
        #print 'get', obj, objtype
        if obj is None:
            return self
        if self.get_cmd is None:
            raise AttributeError("unreadable attribute")
        # Allow certain "magic" values to set the response string
        for key, val in [('float',r"%f"),
                         ('int',r"%d"),]:
            if self.response_string == key:
                self.response_string = val
        if self.response_string in ['bool', 'raw', None]:
            value = obj.query(self.get_cmd)
            if self.response_string == 'bool':
                value = bool(value)
        else:
            value = obj.parsed_query(self.get_cmd, self.response_string)
        return value

    def __set__(self, obj, value):
        #print 'set', obj, value
        if self.set_cmd is None:
            raise AttributeError("can't set attribute")
        if self.validate is not None:
            if value not in self.validate:
                raise ValueError('invalid value supplied - value must be one of {}'.format(self.validate))
        if self.valrange is not None:
            if value < min(self.valrange) or value > max(self.valrange):
                raise ValueError('invalid value supplied - value must be in the range {}-{}'.format(*self.valrange))
        message = self.set_cmd
        if '{0' in message:
            message = message.format(value)
        elif '%' in message:
            message = message % value
        obj.write(message)
        if self.ack_writes == "readline":
            obj.readline()

    def __delete__(self, obj):
        if self.fdel is None:
            raise AttributeError("can't delete attribute")
        self.fdel(obj)
