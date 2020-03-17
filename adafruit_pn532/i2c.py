# Adafruit PN532 NFC/RFID control library.
# Author: Tony DiCola
#
# The MIT License (MIT)
#
# Copyright (c) 2015-2018 Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
``adafruit_pn532.i2c``
====================================================

This module will let you communicate with a PN532 RFID/NFC shield or breakout
using I2C.

* Author(s): Original Raspberry Pi code by Tony DiCola, CircuitPython by ladyada,
             refactor by Carter Nelson

"""

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_PN532.git"

import time
import adafruit_bus_device.i2c_device as i2c_device
from digitalio import Direction
from micropython import const
from adafruit_pn532.adafruit_pn532 import PN532, BusyError, _reset

# pylint: disable=bad-whitespace
_I2C_ADDRESS = const(0x24)


class PN532_I2C(PN532):
    """Driver for the PN532 connected over I2C."""

    def __init__(self, i2c, *, irq=None, reset=None, req=None, debug=False):
        """Create an instance of the PN532 class using I2C. Note that PN532
        uses clock stretching. Optional IRQ pin (not used),
        reset pin and debugging output.
        """
        self.debug = debug
        self._irq = irq
        self._req = req
        if reset:
            _reset(reset)
        self._i2c = i2c_device.I2CDevice(i2c, _I2C_ADDRESS)
        super().__init__(debug=debug, reset=reset)

    def _wakeup(self):  # pylint: disable=no-self-use
        """Send any special commands/data to wake up PN532"""
        if self._req:
            self._req.direction = Direction.OUTPUT
            self._req.value = True
            time.sleep(0.1)
            self._req.value = False
            time.sleep(0.1)
            self._req.value = True
        time.sleep(0.5)

    def _wait_ready(self, timeout=1):
        """Poll PN532 if status byte is ready, up to `timeout` seconds"""
        status = bytearray(1)
        timestamp = time.monotonic()
        while (time.monotonic() - timestamp) < timeout:
            try:
                with self._i2c:
                    self._i2c.readinto(status)
            except OSError:
                self._wakeup()
                continue
            if status == b"\x01":
                return True  # No longer busy
            time.sleep(0.05)  # lets ask again soon!
        # Timed out!
        return False

    def _read_data(self, count):
        """Read a specified count of bytes from the PN532."""
        # Build a read request frame.
        frame = bytearray(count + 1)
        with self._i2c as i2c:
            i2c.readinto(frame, end=1)  # read status byte!
            if frame[0] != 0x01:  # not ready
                raise BusyError
            i2c.readinto(frame)  # ok get the data, plus statusbyte
        if self.debug:
            print("Reading: ", [hex(i) for i in frame[1:]])
        else:
            time.sleep(0.1)
        return frame[1:]  # don't return the status byte

    def _write_data(self, framebytes):
        """Write a specified count of bytes to the PN532"""
        with self._i2c as i2c:
            i2c.write(framebytes)
