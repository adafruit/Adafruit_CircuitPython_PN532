# SPDX-FileCopyrightText: 2015-2018 Tony DiCola for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
``adafruit_pn532.spi``
====================================================

This module will let you communicate with a PN532 RFID/NFC shield or breakout
using SPI.

* Author(s): Original Raspberry Pi code by Tony DiCola, CircuitPython by ladyada,
             refactor by Carter Nelson

"""

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_PN532.git"

import time
from adafruit_bus_device import spi_device
from micropython import const
from adafruit_pn532.adafruit_pn532 import PN532

_SPI_STATREAD = const(0x02)
_SPI_DATAWRITE = const(0x01)
_SPI_DATAREAD = const(0x03)
_SPI_READY = const(0x01)


def reverse_bit(num):
    """Turn an LSB byte to an MSB byte, and vice versa. Used for SPI as
    it is LSB for the PN532, but 99% of SPI implementations are MSB only!"""
    result = 0
    for _ in range(8):
        result <<= 1
        result += num & 1
        num >>= 1
    return result


class PN532_SPI(PN532):
    """Driver for the PN532 connected over SPI. Pass in a hardware or bitbang
    SPI device & chip select digitalInOut pin. Optional IRQ pin (not used),
    reset pin and debugging output."""

    def __init__(self, spi, cs_pin, *, irq=None, reset=None, debug=False):
        """Create an instance of the PN532 class using SPI"""
        self.debug = debug
        self._spi = spi_device.SPIDevice(spi, cs_pin)
        super().__init__(debug=debug, irq=irq, reset=reset)

    def _wakeup(self):
        """Send any special commands/data to wake up PN532"""
        if self._reset_pin:
            self._reset_pin.value = True
            time.sleep(0.01)
        with self._spi as spi:
            spi.write(bytearray([0x00]))  # pylint: disable=no-member
            time.sleep(0.01)
        self.low_power = False
        self.SAM_configuration()  # Put the PN532 back in normal mode

    def _wait_ready(self, timeout=1):
        """Poll PN532 if status byte is ready, up to `timeout` seconds"""
        status_cmd = bytearray([reverse_bit(_SPI_STATREAD), 0x00])
        status_response = bytearray([0x00, 0x00])
        timestamp = time.monotonic()
        with self._spi as spi:
            while (time.monotonic() - timestamp) < timeout:
                spi.write_readinto(
                    status_cmd, status_response
                )  # pylint: disable=no-member
                if reverse_bit(status_response[1]) == 0x01:  # LSB data is read in MSB
                    return True  # Not busy anymore!
                time.sleep(0.01)  # pause a bit till we ask again
        # We timed out!
        return False

    def _read_data(self, count):
        """Read a specified count of bytes from the PN532."""
        # Build a read request frame.
        frame = bytearray(count + 1)
        # Add the SPI data read signal byte, but LSB'ify it
        frame[0] = reverse_bit(_SPI_DATAREAD)

        with self._spi as spi:
            spi.write_readinto(frame, frame)  # pylint: disable=no-member
        for i, val in enumerate(frame):
            frame[i] = reverse_bit(val)  # turn LSB data to MSB
        if self.debug:
            print("Reading: ", [hex(i) for i in frame[1:]])
        return frame[1:]

    def _write_data(self, framebytes):
        """Write a specified count of bytes to the PN532"""
        # start by making a frame with data write in front,
        # then rest of bytes, and LSBify it
        rev_frame = [reverse_bit(x) for x in bytes([_SPI_DATAWRITE]) + framebytes]
        if self.debug:
            print("Writing: ", [hex(i) for i in rev_frame])
        with self._spi as spi:
            spi.write(bytes(rev_frame))  # pylint: disable=no-member
