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
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
"""
``adafruit_pn532``
====================================================

This module will let you communicate with a PN532 RFID/NFC shield or breakout
using I2C, SPI or UART.

* Author(s): Original Raspberry Pi code by Tony DiCola, CircuitPython by ladyada

Implementation Notes
--------------------

**Hardware:**

* Adafruit `PN532 Breakout <https://www.adafruit.com/product/364>`_
* Adafruit `PN532 Shield <https://www.adafruit.com/product/789>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases
* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""


import time
from digitalio import Direction
import adafruit_bus_device.i2c_device as i2c_device
import adafruit_bus_device.spi_device as spi_device

from micropython import const

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_PN532.git"

# pylint: disable=bad-whitespace
_PREAMBLE                      = const(0x00)
_STARTCODE1                    = const(0x00)
_STARTCODE2                    = const(0xFF)
_POSTAMBLE                     = const(0x00)

_HOSTTOPN532                   = const(0xD4)
_PN532TOHOST                   = const(0xD5)

# PN532 Commands
_COMMAND_DIAGNOSE              = const(0x00)
_COMMAND_GETFIRMWAREVERSION    = const(0x02)
_COMMAND_GETGENERALSTATUS      = const(0x04)
_COMMAND_READREGISTER          = const(0x06)
_COMMAND_WRITEREGISTER         = const(0x08)
_COMMAND_READGPIO              = const(0x0C)
_COMMAND_WRITEGPIO             = const(0x0E)
_COMMAND_SETSERIALBAUDRATE     = const(0x10)
_COMMAND_SETPARAMETERS         = const(0x12)
_COMMAND_SAMCONFIGURATION      = const(0x14)
_COMMAND_POWERDOWN             = const(0x16)
_COMMAND_RFCONFIGURATION       = const(0x32)
_COMMAND_RFREGULATIONTEST      = const(0x58)
_COMMAND_INJUMPFORDEP          = const(0x56)
_COMMAND_INJUMPFORPSL          = const(0x46)
_COMMAND_INLISTPASSIVETARGET   = const(0x4A)
_COMMAND_INATR                 = const(0x50)
_COMMAND_INPSL                 = const(0x4E)
_COMMAND_INDATAEXCHANGE        = const(0x40)
_COMMAND_INCOMMUNICATETHRU     = const(0x42)
_COMMAND_INDESELECT            = const(0x44)
_COMMAND_INRELEASE             = const(0x52)
_COMMAND_INSELECT              = const(0x54)
_COMMAND_INAUTOPOLL            = const(0x60)
_COMMAND_TGINITASTARGET        = const(0x8C)
_COMMAND_TGSETGENERALBYTES     = const(0x92)
_COMMAND_TGGETDATA             = const(0x86)
_COMMAND_TGSETDATA             = const(0x8E)
_COMMAND_TGSETMETADATA         = const(0x94)
_COMMAND_TGGETINITIATORCOMMAND = const(0x88)
_COMMAND_TGRESPONSETOINITIATOR = const(0x90)
_COMMAND_TGGETTARGETSTATUS     = const(0x8A)

_RESPONSE_INDATAEXCHANGE       = const(0x41)
_RESPONSE_INLISTPASSIVETARGET  = const(0x4B)

_WAKEUP                        = const(0x55)

_SPI_STATREAD                  = const(0x02)
_SPI_DATAWRITE                 = const(0x01)
_SPI_DATAREAD                  = const(0x03)
_SPI_READY                     = const(0x01)

_I2C_ADDRESS                   = const(0x24)

_MIFARE_ISO14443A              = const(0x00)

# Mifare Commands
MIFARE_CMD_AUTH_A                   = const(0x60)
MIFARE_CMD_AUTH_B                   = const(0x61)
MIFARE_CMD_READ                     = const(0x30)
MIFARE_CMD_WRITE                    = const(0xA0)
MIFARE_CMD_TRANSFER                 = const(0xB0)
MIFARE_CMD_DECREMENT                = const(0xC0)
MIFARE_CMD_INCREMENT                = const(0xC1)
MIFARE_CMD_STORE                    = const(0xC2)
MIFARE_ULTRALIGHT_CMD_WRITE         = const(0xA2)

# Prefixes for NDEF Records (to identify record type)
NDEF_URIPREFIX_NONE                 = const(0x00)
NDEF_URIPREFIX_HTTP_WWWDOT          = const(0x01)
NDEF_URIPREFIX_HTTPS_WWWDOT         = const(0x02)
NDEF_URIPREFIX_HTTP                 = const(0x03)
NDEF_URIPREFIX_HTTPS                = const(0x04)
NDEF_URIPREFIX_TEL                  = const(0x05)
NDEF_URIPREFIX_MAILTO               = const(0x06)
NDEF_URIPREFIX_FTP_ANONAT           = const(0x07)
NDEF_URIPREFIX_FTP_FTPDOT           = const(0x08)
NDEF_URIPREFIX_FTPS                 = const(0x09)
NDEF_URIPREFIX_SFTP                 = const(0x0A)
NDEF_URIPREFIX_SMB                  = const(0x0B)
NDEF_URIPREFIX_NFS                  = const(0x0C)
NDEF_URIPREFIX_FTP                  = const(0x0D)
NDEF_URIPREFIX_DAV                  = const(0x0E)
NDEF_URIPREFIX_NEWS                 = const(0x0F)
NDEF_URIPREFIX_TELNET               = const(0x10)
NDEF_URIPREFIX_IMAP                 = const(0x11)
NDEF_URIPREFIX_RTSP                 = const(0x12)
NDEF_URIPREFIX_URN                  = const(0x13)
NDEF_URIPREFIX_POP                  = const(0x14)
NDEF_URIPREFIX_SIP                  = const(0x15)
NDEF_URIPREFIX_SIPS                 = const(0x16)
NDEF_URIPREFIX_TFTP                 = const(0x17)
NDEF_URIPREFIX_BTSPP                = const(0x18)
NDEF_URIPREFIX_BTL2CAP              = const(0x19)
NDEF_URIPREFIX_BTGOEP               = const(0x1A)
NDEF_URIPREFIX_TCPOBEX              = const(0x1B)
NDEF_URIPREFIX_IRDAOBEX             = const(0x1C)
NDEF_URIPREFIX_FILE                 = const(0x1D)
NDEF_URIPREFIX_URN_EPC_ID           = const(0x1E)
NDEF_URIPREFIX_URN_EPC_TAG          = const(0x1F)
NDEF_URIPREFIX_URN_EPC_PAT          = const(0x20)
NDEF_URIPREFIX_URN_EPC_RAW          = const(0x21)
NDEF_URIPREFIX_URN_EPC              = const(0x22)
NDEF_URIPREFIX_URN_NFC              = const(0x23)

_GPIO_VALIDATIONBIT            = const(0x80)
_GPIO_P30                      = const(0)
_GPIO_P31                      = const(1)
_GPIO_P32                      = const(2)
_GPIO_P33                      = const(3)
_GPIO_P34                      = const(4)
_GPIO_P35                      = const(5)

_ACK                           = b'\x00\x00\xFF\x00\xFF\x00'
_FRAME_START                   = b'\x00\x00\xFF'
# pylint: enable=bad-whitespace


def _reset(pin):
    """Perform a hardware reset toggle"""
    pin.direction = Direction.OUTPUT
    pin.value = True
    time.sleep(0.1)
    pin.value = False
    time.sleep(0.5)
    pin.value = True
    time.sleep(0.1)

def reverse_bit(num):
    """Turn an LSB byte to an MSB byte, and vice versa. Used for SPI as
    it is LSB for the PN532, but 99% of SPI implementations are MSB only!"""
    result = 0
    for _ in range(8):
        result <<= 1
        result += (num & 1)
        num >>= 1
    return result



class BusyError(Exception):
    """Base class for exceptions in this module."""
    pass

class PN532:
    """PN532 driver base, must be extended for I2C/SPI/UART interfacing"""

    def __init__(self, *, debug=False, reset=None):
        """Create an instance of the PN532 class
        """
        self.debug = debug
        if reset:
            if debug:
                print("Resetting")
            _reset(reset)

        try:
            self._wakeup()
            self.get_firmware_version() # first time often fails, try 2ce
            return
        except (BusyError, RuntimeError):
            pass
        self.get_firmware_version()

    def _read_data(self, count):
        # Read raw data from device, not including status bytes:
        # Subclasses MUST implement this!
        raise NotImplementedError

    def _write_data(self, framebytes):
        # Write raw bytestring data to device, not including status bytes:
        # Subclasses MUST implement this!
        raise NotImplementedError

    def _wait_ready(self, timeout):
        # Check if busy up to max length of 'timeout' seconds
        # Subclasses MUST implement this!
        raise NotImplementedError

    def _wakeup(self):
        # Send special command to wake up
        raise NotImplementedError

    def _write_frame(self, data):
        """Write a frame to the PN532 with the specified data bytearray."""
        assert data is not None and 1 < len(data) < 255, 'Data must be array of 1 to 255 bytes.'
        # Build frame to send as:
        # - Preamble (0x00)
        # - Start code  (0x00, 0xFF)
        # - Command length (1 byte)
        # - Command length checksum
        # - Command bytes
        # - Checksum
        # - Postamble (0x00)
        length = len(data)
        frame = bytearray(length+8)
        frame[0] = _PREAMBLE
        frame[1] = _STARTCODE1
        frame[2] = _STARTCODE2
        checksum = sum(frame[0:3])
        frame[3] = length & 0xFF
        frame[4] = (~length + 1) & 0xFF
        frame[5:-2] = data
        checksum += sum(data)
        frame[-2] = ~checksum & 0xFF
        frame[-1] = _POSTAMBLE
        # Send frame.
        if self.debug:
            print('Write frame: ', [hex(i) for i in frame])
        self._write_data(bytes(frame))

    def _read_frame(self, length):
        """Read a response frame from the PN532 of at most length bytes in size.
        Returns the data inside the frame if found, otherwise raises an exception
        if there is an error parsing the frame.  Note that less than length bytes
        might be returned!
        """
        # Read frame with expected length of data.
        response = self._read_data(length+8)
        if self.debug:
            print('Read frame:', [hex(i) for i in response])

        # Swallow all the 0x00 values that preceed 0xFF.
        offset = 0
        while response[offset] == 0x00:
            offset += 1
            if offset >= len(response):
                raise RuntimeError('Response frame preamble does not contain 0x00FF!')
        if response[offset] != 0xFF:
            raise RuntimeError('Response frame preamble does not contain 0x00FF!')
        offset += 1
        if offset >= len(response):
            raise RuntimeError('Response contains no data!')
        # Check length & length checksum match.
        frame_len = response[offset]
        if (frame_len + response[offset+1]) & 0xFF != 0:
            raise RuntimeError('Response length checksum did not match length!')
        # Check frame checksum value matches bytes.
        checksum = sum(response[offset+2:offset+2+frame_len+1]) & 0xFF
        if checksum != 0:
            raise RuntimeError('Response checksum did not match expected value: ', checksum)
        # Return frame data.
        return response[offset+2:offset+2+frame_len]

    def call_function(self, command, response_length=0, params=[], timeout=1): # pylint: disable=dangerous-default-value
        """Send specified command to the PN532 and expect up to response_length
        bytes back in a response.  Note that less than the expected bytes might
        be returned!  Params can optionally specify an array of bytes to send as
        parameters to the function call.  Will wait up to timeout seconds
        for a response and return a bytearray of response bytes, or None if no
        response is available within the timeout.
        """
        # Build frame data with command and parameters.
        data = bytearray(2+len(params))
        data[0] = _HOSTTOPN532
        data[1] = command & 0xFF
        for i, val in enumerate(params):
            data[2+i] = val
        # Send frame and wait for response.
        try:
            self._write_frame(data)
        except OSError:
            self._wakeup()
            return None
        if not self._wait_ready(timeout):
            return None
        # Verify ACK response and wait to be ready for function response.
        if not _ACK == self._read_data(len(_ACK)):
            raise RuntimeError('Did not receive expected ACK from PN532!')
        if not self._wait_ready(timeout):
            return None
        # Read response bytes.
        response = self._read_frame(response_length+2)
        # Check that response is for the called function.
        if not (response[0] == _PN532TOHOST and response[1] == (command+1)):
            raise RuntimeError('Received unexpected command response!')
        # Return response data.
        return response[2:]

    def get_firmware_version(self):
        """Call PN532 GetFirmwareVersion function and return a tuple with the IC,
        Ver, Rev, and Support values.
        """
        response = self.call_function(_COMMAND_GETFIRMWAREVERSION, 4, timeout=0.5)
        if response is None:
            raise RuntimeError('Failed to detect the PN532')
        return tuple(response)

    def SAM_configuration(self):   # pylint: disable=invalid-name
        """Configure the PN532 to read MiFare cards."""
        # Send SAM configuration command with configuration for:
        # - 0x01, normal mode
        # - 0x14, timeout 50ms * 20 = 1 second
        # - 0x01, use IRQ pin
        # Note that no other verification is necessary as call_function will
        # check the command was executed as expected.
        self.call_function(_COMMAND_SAMCONFIGURATION, params=[0x01, 0x14, 0x01])

    def read_passive_target(self, card_baud=_MIFARE_ISO14443A, timeout=1):
        """Wait for a MiFare card to be available and return its UID when found.
        Will wait up to timeout seconds and return None if no card is found,
        otherwise a bytearray with the UID of the found card is returned.
        """
        # Send passive read command for 1 card.  Expect at most a 7 byte UUID.
        try:
            response = self.call_function(_COMMAND_INLISTPASSIVETARGET,
                                          params=[0x01, card_baud],
                                          response_length=19,
                                          timeout=timeout)
        except BusyError:
            return None # no card found!
        # If no response is available return None to indicate no card is present.
        if response is None:
            return None
        # Check only 1 card with up to a 7 byte UID is present.
        if response[0] != 0x01:
            raise RuntimeError('More than one card detected!')
        if response[5] > 7:
            raise RuntimeError('Found card with unexpectedly long UID!')
        # Return UID of card.
        return response[6:6+response[5]]

    def mifare_classic_authenticate_block(self, uid, block_number, key_number, key):   # pylint: disable=invalid-name
        """Authenticate specified block number for a MiFare classic card.  Uid
        should be a byte array with the UID of the card, block number should be
        the block to authenticate, key number should be the key type (like
        MIFARE_CMD_AUTH_A or MIFARE_CMD_AUTH_B), and key should be a byte array
        with the key data.  Returns True if the block was authenticated, or False
        if not authenticated.
        """
        # Build parameters for InDataExchange command to authenticate MiFare card.
        uidlen = len(uid)
        keylen = len(key)
        params = bytearray(3+uidlen+keylen)
        params[0] = 0x01  # Max card numbers
        params[1] = key_number & 0xFF
        params[2] = block_number & 0xFF
        params[3:3+keylen] = key
        params[3+keylen:] = uid
        # Send InDataExchange request and verify response is 0x00.
        response = self.call_function(_COMMAND_INDATAEXCHANGE,
                                      params=params,
                                      response_length=1)
        return response[0] == 0x00

    def mifare_classic_read_block(self, block_number):
        """Read a block of data from the card.  Block number should be the block
        to read.  If the block is successfully read a bytearray of length 16 with
        data starting at the specified block will be returned.  If the block is
        not read then None will be returned.
        """
        # Send InDataExchange request to read block of MiFare data.
        response = self.call_function(_COMMAND_INDATAEXCHANGE,
                                      params=[0x01, MIFARE_CMD_READ, block_number & 0xFF],
                                      response_length=17)
        # Check first response is 0x00 to show success.
        if response[0] != 0x00:
            return None
        # Return first 4 bytes since 16 bytes are always returned.
        return response[1:]

    def mifare_classic_write_block(self, block_number, data):
        """Write a block of data to the card.  Block number should be the block
        to write and data should be a byte array of length 16 with the data to
        write.  If the data is successfully written then True is returned,
        otherwise False is returned.
        """
        assert data is not None and len(data) == 16, 'Data must be an array of 16 bytes!'
        # Build parameters for InDataExchange command to do MiFare classic write.
        params = bytearray(19)
        params[0] = 0x01  # Max card numbers
        params[1] = MIFARE_CMD_WRITE
        params[2] = block_number & 0xFF
        params[3:] = data
        # Send InDataExchange request.
        response = self.call_function(_COMMAND_INDATAEXCHANGE,
                                      params=params,
                                      response_length=1)
        return response[0] == 0x0

    def ntag2xx_write_block(self, block_number, data):
        """Write a block of data to the card.  Block number should be the block
        to write and data should be a byte array of length 4 with the data to
        write.  If the data is successfully written then True is returned,
        otherwise False is returned.
        """
        assert data is not None and len(data) == 4, 'Data must be an array of 4 bytes!'
        # Build parameters for InDataExchange command to do NTAG203 classic write.
        params = bytearray(3+len(data))
        params[0] = 0x01  # Max card numbers
        params[1] = MIFARE_ULTRALIGHT_CMD_WRITE
        params[2] = block_number & 0xFF
        params[3:] = data
        # Send InDataExchange request.
        response = self.call_function(_COMMAND_INDATAEXCHANGE,
                                      params=params,
                                      response_length=1)
        return response[0] == 0x00

    def ntag2xx_read_block(self, block_number):
        """Read a block of data from the card.  Block number should be the block
        to read.  If the block is successfully read a bytearray of length 16 with
        data starting at the specified block will be returned.  If the block is
        not read then None will be returned.
        """
        return self.mifare_classic_read_block(block_number)[0:4] # only 4 bytes per page

class PN532_UART(PN532):
    """Driver for the PN532 connected over Serial UART"""
    def __init__(self, uart, *, irq=None, reset=None, debug=False):
        """Create an instance of the PN532 class using Serial connection.
        Optional IRQ pin (not used), reset pin and debugging output.
        """
        self.debug = debug
        self._irq = irq
        self._uart = uart
        super().__init__(debug=debug, reset=reset)

    def _wakeup(self):
        """Send any special commands/data to wake up PN532"""
        #self._write_frame([_HOSTTOPN532, _COMMAND_SAMCONFIGURATION, 0x01])
        self.SAM_configuration()

    def _wait_ready(self, timeout=1):
        """Wait `timeout` seconds"""
        time.sleep(timeout)
        return True

    def _read_data(self, count):
        """Read a specified count of bytes from the PN532."""
        frame = self._uart.read(count)
        if not frame:
            raise BusyError("No data read from PN532")
        if self.debug:
            print("Reading: ", [hex(i) for i in frame])
        else:
            time.sleep(0.1)
        return frame

    def _write_data(self, framebytes):
        """Write a specified count of bytes to the PN532"""
        while self._uart.read(1):  # this would be a lot nicer if we could query the # of bytes
            pass
        self._uart.write('\x55\x55\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00') # wake up!
        self._uart.write(framebytes)

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

    def _wakeup(self): # pylint: disable=no-self-use
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
            if status == b'\x01':
                return True  # No longer busy
            else:
                time.sleep(0.05)  # lets ask again soon!
        # Timed out!
        return False

    def _read_data(self, count):
        """Read a specified count of bytes from the PN532."""
        # Build a read request frame.
        frame = bytearray(count+1)
        with self._i2c as i2c:
            i2c.readinto(frame, end=1) # read status byte!
            if frame[0] != 0x01:             # not ready
                raise BusyError
            i2c.readinto(frame)        # ok get the data, plus statusbyte
        if self.debug:
            print("Reading: ", [hex(i) for i in frame[1:]])
        else:
            time.sleep(0.1)
        return frame[1:]   # don't return the status byte

    def _write_data(self, framebytes):
        """Write a specified count of bytes to the PN532"""
        with self._i2c as i2c:
            i2c.write(framebytes)

class PN532_SPI(PN532):
    """Driver for the PN532 connected over SPI. Pass in a hardware or bitbang
    SPI device & chip select digitalInOut pin. Optional IRQ pin (not used),
    reset pin and debugging output."""
    def __init__(self, spi, cs_pin, *, irq=None, reset=None, debug=False):
        """Create an instance of the PN532 class using SPI"""
        self.debug = debug
        self._irq = irq
        self._spi = spi_device.SPIDevice(spi, cs_pin)
        super().__init__(debug=debug, reset=reset)

    def _wakeup(self):
        """Send any special commands/data to wake up PN532"""
        with self._spi as spi:
            time.sleep(1)
            spi.write(bytearray([0x00])) #pylint: disable=no-member
            time.sleep(1)

    def _wait_ready(self, timeout=1):
        """Poll PN532 if status byte is ready, up to `timeout` seconds"""
        status = bytearray([reverse_bit(_SPI_STATREAD), 0])

        timestamp = time.monotonic()
        while (time.monotonic() - timestamp) < timeout:
            with self._spi as spi:
                time.sleep(0.02)   # required
                spi.write_readinto(status, status) #pylint: disable=no-member
            if reverse_bit(status[1]) == 0x01:  # LSB data is read in MSB
                return True      # Not busy anymore!
            else:
                time.sleep(0.01)  # pause a bit till we ask again
        # We timed out!
        return False

    def _read_data(self, count):
        """Read a specified count of bytes from the PN532."""
        # Build a read request frame.
        frame = bytearray(count+1)
        # Add the SPI data read signal byte, but LSB'ify it
        frame[0] = reverse_bit(_SPI_DATAREAD)

        with self._spi as spi:
            time.sleep(0.02)   # required
            spi.write_readinto(frame, frame) #pylint: disable=no-member
        for i, val in enumerate(frame):
            frame[i] = reverse_bit(val) # turn LSB data to MSB
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
            time.sleep(0.02)   # required
            spi.write(bytes(rev_frame)) #pylint: disable=no-member
