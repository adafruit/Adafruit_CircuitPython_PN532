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

try:
    from typing import Optional
    from circuitpython_typing import ReadableBuffer
    from digitalio import DigitalInOut
    from busio import UART
except ImportError:
    pass

import time
from adafruit_pn532.adafruit_pn532 import PN532, BusyError


class PN532_UART(PN532):
    """Driver for the PN532 connected over Serial UART"""

    def __init__(
        self, uart: UART, *, reset: Optional[DigitalInOut] = None, debug: bool = False
    ) -> None:
        """Create an instance of the PN532 class using Serial connection.
        Optional reset pin and debugging output.

        :param ~busio.UART uart: The uart object the PN532 is connected to.
        :param digitalio.DigitalInOut reset: board pin the PN532 RSTOUT_N is connected to
        :param bool debug: if True print additional debug statements. Defaults to False

        **Quickstart: Importing and using the device**

            Here is an example of using the :class:`PN532_I2C` class.
            First you will need to import the libraries to use the sensor

            .. code-block:: python

                import board
                import busio
                from digitalio import DigitalInOut
                from adafruit_pn532.uart import PN532_UART

            Once this is done you can define your `busio.UART` object and define your PN532 object

            .. code-block:: python

                uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0.1)
                pn532 = PN532_UART(uart, debug=False)

            Now you have access to the attributes and functions of the PN532 RFID/NFC
            shield or breakout

            .. code-block:: python

                uid = pn532.read_passive_target(timeout=0.5)

        """
        self.debug = debug
        self._uart = uart
        super().__init__(debug=debug, reset=reset)

    def _wakeup(self) -> None:
        """Send any special commands/data to wake up PN532"""
        if self._reset_pin:
            self._reset_pin.value = True
            time.sleep(0.01)
        self.low_power = False
        self._uart.write(
            b"\x55\x55\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
        )  # wake up!
        self.SAM_configuration()

    def _wait_ready(self, timeout: float = 1) -> bool:
        """Wait `timeout` seconds"""
        timestamp = time.monotonic()
        while (time.monotonic() - timestamp) < timeout:
            if self._uart.in_waiting > 0:
                return True  # No Longer Busy
            time.sleep(0.01)  # lets ask again soon!
        # Timed out!
        return False

    def _read_data(self, count: int) -> bytes:
        """Read a specified count of bytes from the PN532."""
        frame = self._uart.read(count)
        if not frame:
            raise BusyError("No data read from PN532")
        if self.debug:
            print("Reading: ", [hex(i) for i in frame])
        return frame

    def _write_data(self, framebytes: ReadableBuffer) -> None:
        """Write a specified count of bytes to the PN532"""
        self._uart.reset_input_buffer()
        self._uart.write(framebytes)
