# SPDX-FileCopyrightText: 2015-2018 Tony DiCola for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
``adafruit_pn532.uart``
====================================================

This module will let you communicate with a PN532 RFID/NFC shield or breakout
using UART.

* Author(s): Original Raspberry Pi code by Tony DiCola, CircuitPython by ladyada,
             refactor by Carter Nelson

"""

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_PN532.git"


import time
from adafruit_pn532.adafruit_pn532 import PN532, BusyError


class PN532_UART(PN532):
    """Driver for the PN532 connected over Serial UART"""

    def __init__(self, uart, *, reset=None, debug=False):
        """Create an instance of the PN532 class using Serial connection.
        Optional reset pin and debugging output.
        """
        self.debug = debug
        self._uart = uart
        super().__init__(debug=debug, reset=reset)

    def _wakeup(self):
        """Send any special commands/data to wake up PN532"""
        if self._reset_pin:
            self._reset_pin.value = True
            time.sleep(0.01)
        self.low_power = False
        self._uart.write(
            b"\x55\x55\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
        )  # wake up!
        self.SAM_configuration()

    def _wait_ready(self, timeout=1):
        """Wait `timeout` seconds"""
        timestamp = time.monotonic()
        while (time.monotonic() - timestamp) < timeout:
            if self._uart.in_waiting > 0:
                return True  # No Longer Busy
            time.sleep(0.01)  # lets ask again soon!
        # Timed out!
        return False

    def _read_data(self, count):
        """Read a specified count of bytes from the PN532."""
        frame = self._uart.read(count)
        if not frame:
            raise BusyError("No data read from PN532")
        if self.debug:
            print("Reading: ", [hex(i) for i in frame])
        return frame

    def _write_data(self, framebytes):
        """Write a specified count of bytes to the PN532"""
        self._uart.reset_input_buffer()
        self._uart.write(framebytes)
