# SPDX-FileCopyrightText: 2015-2018 Tony DiCola for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
``adafruit_pn532.i2c``
====================================================

This module will let you communicate with a PN532 RFID/NFC shield or breakout
using I2C.

* Author(s): Original Raspberry Pi code by Tony DiCola, CircuitPython by ladyada,
             refactor by Carter Nelson

"""

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_PN532.git"

import time
from adafruit_bus_device import i2c_device
from digitalio import Direction
from micropython import const
from adafruit_pn532.adafruit_pn532 import PN532, BusyError

_I2C_ADDRESS = const(0x24)


class PN532_I2C(PN532):
    """Driver for the PN532 connected over I2C."""

    def __init__(self, i2c, *, irq=None, reset=None, req=None, debug=False):
        """Create an instance of the PN532 class using I2C. Note that PN532
        uses clock stretching. Optional IRQ pin (not used),
        resetp pin and debugging output.
        """
        self.debug = debug
        self._req = req
        self._i2c = i2c_device.I2CDevice(i2c, _I2C_ADDRESS)
        super().__init__(debug=debug, irq=irq, reset=reset)

    def _wakeup(self):
        """Send any special commands/data to wake up PN532"""
        if self._reset_pin:
            self._reset_pin.value = True
            time.sleep(0.01)
        if self._req:
            self._req.direction = Direction.OUTPUT
            self._req.value = False
            time.sleep(0.01)
            self._req.value = True
            time.sleep(0.01)
        self.low_power = False
        self.SAM_configuration()  # Put the PN532 back in normal mode

    def _wait_ready(self, timeout=1):
        """Poll PN532 if status byte is ready, up to `timeout` seconds"""
        status = bytearray(1)
        timestamp = time.monotonic()
        while (time.monotonic() - timestamp) < timeout:
            try:
                with self._i2c:
                    self._i2c.readinto(status)
            except OSError:
                continue
            if status == b"\x01":
                return True  # No longer busy
            time.sleep(0.01)  # lets ask again soon!
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
        return frame[1:]  # don't return the status byte

    def _write_data(self, framebytes):
        """Write a specified count of bytes to the PN532"""
        with self._i2c as i2c:
            i2c.write(framebytes)
