# SPDX-FileCopyrightText: 2015-2018 Tony DiCola for Adafruit Industries
#
# SPDX-License-Identifier: MIT

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

from micropython import const

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_PN532.git"

_PREAMBLE = const(0x00)
_STARTCODE1 = const(0x00)
_STARTCODE2 = const(0xFF)
_POSTAMBLE = const(0x00)

_HOSTTOPN532 = const(0xD4)
_PN532TOHOST = const(0xD5)

# PN532 Commands
_COMMAND_DIAGNOSE = const(0x00)
_COMMAND_GETFIRMWAREVERSION = const(0x02)
_COMMAND_GETGENERALSTATUS = const(0x04)
_COMMAND_READREGISTER = const(0x06)
_COMMAND_WRITEREGISTER = const(0x08)
_COMMAND_READGPIO = const(0x0C)
_COMMAND_WRITEGPIO = const(0x0E)
_COMMAND_SETSERIALBAUDRATE = const(0x10)
_COMMAND_SETPARAMETERS = const(0x12)
_COMMAND_SAMCONFIGURATION = const(0x14)
_COMMAND_POWERDOWN = const(0x16)
_COMMAND_RFCONFIGURATION = const(0x32)
_COMMAND_RFREGULATIONTEST = const(0x58)
_COMMAND_INJUMPFORDEP = const(0x56)
_COMMAND_INJUMPFORPSL = const(0x46)
_COMMAND_INLISTPASSIVETARGET = const(0x4A)
_COMMAND_INATR = const(0x50)
_COMMAND_INPSL = const(0x4E)
_COMMAND_INDATAEXCHANGE = const(0x40)
_COMMAND_INCOMMUNICATETHRU = const(0x42)
_COMMAND_INDESELECT = const(0x44)
_COMMAND_INRELEASE = const(0x52)
_COMMAND_INSELECT = const(0x54)
_COMMAND_INAUTOPOLL = const(0x60)
_COMMAND_TGINITASTARGET = const(0x8C)
_COMMAND_TGSETGENERALBYTES = const(0x92)
_COMMAND_TGGETDATA = const(0x86)
_COMMAND_TGSETDATA = const(0x8E)
_COMMAND_TGSETMETADATA = const(0x94)
_COMMAND_TGGETINITIATORCOMMAND = const(0x88)
_COMMAND_TGRESPONSETOINITIATOR = const(0x90)
_COMMAND_TGGETTARGETSTATUS = const(0x8A)

_RESPONSE_INDATAEXCHANGE = const(0x41)
_RESPONSE_INLISTPASSIVETARGET = const(0x4B)

_WAKEUP = const(0x55)

_MIFARE_ISO14443A = const(0x00)

# Mifare Commands
MIFARE_CMD_AUTH_A = const(0x60)
MIFARE_CMD_AUTH_B = const(0x61)
MIFARE_CMD_READ = const(0x30)
MIFARE_CMD_WRITE = const(0xA0)
MIFARE_CMD_TRANSFER = const(0xB0)
MIFARE_CMD_DECREMENT = const(0xC0)
MIFARE_CMD_INCREMENT = const(0xC1)
MIFARE_CMD_STORE = const(0xC2)
MIFARE_ULTRALIGHT_CMD_WRITE = const(0xA2)

# Prefixes for NDEF Records (to identify record type)
NDEF_URIPREFIX_NONE = const(0x00)
NDEF_URIPREFIX_HTTP_WWWDOT = const(0x01)
NDEF_URIPREFIX_HTTPS_WWWDOT = const(0x02)
NDEF_URIPREFIX_HTTP = const(0x03)
NDEF_URIPREFIX_HTTPS = const(0x04)
NDEF_URIPREFIX_TEL = const(0x05)
NDEF_URIPREFIX_MAILTO = const(0x06)
NDEF_URIPREFIX_FTP_ANONAT = const(0x07)
NDEF_URIPREFIX_FTP_FTPDOT = const(0x08)
NDEF_URIPREFIX_FTPS = const(0x09)
NDEF_URIPREFIX_SFTP = const(0x0A)
NDEF_URIPREFIX_SMB = const(0x0B)
NDEF_URIPREFIX_NFS = const(0x0C)
NDEF_URIPREFIX_FTP = const(0x0D)
NDEF_URIPREFIX_DAV = const(0x0E)
NDEF_URIPREFIX_NEWS = const(0x0F)
NDEF_URIPREFIX_TELNET = const(0x10)
NDEF_URIPREFIX_IMAP = const(0x11)
NDEF_URIPREFIX_RTSP = const(0x12)
NDEF_URIPREFIX_URN = const(0x13)
NDEF_URIPREFIX_POP = const(0x14)
NDEF_URIPREFIX_SIP = const(0x15)
NDEF_URIPREFIX_SIPS = const(0x16)
NDEF_URIPREFIX_TFTP = const(0x17)
NDEF_URIPREFIX_BTSPP = const(0x18)
NDEF_URIPREFIX_BTL2CAP = const(0x19)
NDEF_URIPREFIX_BTGOEP = const(0x1A)
NDEF_URIPREFIX_TCPOBEX = const(0x1B)
NDEF_URIPREFIX_IRDAOBEX = const(0x1C)
NDEF_URIPREFIX_FILE = const(0x1D)
NDEF_URIPREFIX_URN_EPC_ID = const(0x1E)
NDEF_URIPREFIX_URN_EPC_TAG = const(0x1F)
NDEF_URIPREFIX_URN_EPC_PAT = const(0x20)
NDEF_URIPREFIX_URN_EPC_RAW = const(0x21)
NDEF_URIPREFIX_URN_EPC = const(0x22)
NDEF_URIPREFIX_URN_NFC = const(0x23)

_GPIO_VALIDATIONBIT = const(0x80)
_GPIO_P30 = const(0)
_GPIO_P31 = const(1)
_GPIO_P32 = const(2)
_GPIO_P33 = const(3)
_GPIO_P34 = const(4)
_GPIO_P35 = const(5)

_ACK = b"\x00\x00\xFF\x00\xFF\x00"
_FRAME_START = b"\x00\x00\xFF"


class BusyError(Exception):
    """Base class for exceptions in this module."""


class PN532:
    """PN532 driver base, must be extended for I2C/SPI/UART interfacing"""

    def __init__(self, *, debug=False, irq=None, reset=None):
        """Create an instance of the PN532 class"""
        self.low_power = True
        self.debug = debug
        self._irq = irq
        self._reset_pin = reset
        self.reset()
        _ = self.firmware_version

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

    def reset(self):
        """Perform a hardware reset toggle and then wake up the PN532"""
        if self._reset_pin:
            if self.debug:
                print("Resetting")
            self._reset_pin.direction = Direction.OUTPUT
            self._reset_pin.value = False
            time.sleep(0.1)
            self._reset_pin.value = True
            time.sleep(0.1)
        self._wakeup()

    def _write_frame(self, data):
        """Write a frame to the PN532 with the specified data bytearray."""
        assert (
            data is not None and 1 < len(data) < 255
        ), "Data must be array of 1 to 255 bytes."
        # Build frame to send as:
        # - Preamble (0x00)
        # - Start code  (0x00, 0xFF)
        # - Command length (1 byte)
        # - Command length checksum
        # - Command bytes
        # - Checksum
        # - Postamble (0x00)
        length = len(data)
        frame = bytearray(length + 8)
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
            print("Write frame: ", [hex(i) for i in frame])
        self._write_data(bytes(frame))

    def _read_frame(self, length):
        """Read a response frame from the PN532 of at most length bytes in size.
        Returns the data inside the frame if found, otherwise raises an exception
        if there is an error parsing the frame.  Note that less than length bytes
        might be returned!
        """
        # Read frame with expected length of data.
        response = self._read_data(length + 7)
        if self.debug:
            print("Read frame:", [hex(i) for i in response])

        # Swallow all the 0x00 values that preceed 0xFF.
        offset = 0
        while response[offset] == 0x00:
            offset += 1
            if offset >= len(response):
                raise RuntimeError("Response frame preamble does not contain 0x00FF!")
        if response[offset] != 0xFF:
            raise RuntimeError("Response frame preamble does not contain 0x00FF!")
        offset += 1
        if offset >= len(response):
            raise RuntimeError("Response contains no data!")
        # Check length & length checksum match.
        frame_len = response[offset]
        if (frame_len + response[offset + 1]) & 0xFF != 0:
            raise RuntimeError("Response length checksum did not match length!")
        # Check frame checksum value matches bytes.
        checksum = sum(response[offset + 2 : offset + 2 + frame_len + 1]) & 0xFF
        if checksum != 0:
            raise RuntimeError(
                "Response checksum did not match expected value: ", checksum
            )
        # Return frame data.
        return response[offset + 2 : offset + 2 + frame_len]

    def call_function(
        self, command, response_length=0, params=[], timeout=1
    ):  # pylint: disable=dangerous-default-value
        """Send specified command to the PN532 and expect up to response_length
        bytes back in a response.  Note that less than the expected bytes might
        be returned!  Params can optionally specify an array of bytes to send as
        parameters to the function call.  Will wait up to timeout seconds
        for a response and return a bytearray of response bytes, or None if no
        response is available within the timeout.
        """
        if not self.send_command(command, params=params, timeout=timeout):
            return None
        return self.process_response(
            command, response_length=response_length, timeout=timeout
        )

    def send_command(
        self, command, params=[], timeout=1
    ):  # pylint: disable=dangerous-default-value
        """Send specified command to the PN532 and wait for an acknowledgment.
        Will wait up to timeout seconds for the acknowlegment and return True.
        If no acknowlegment is received, False is returned.
        """
        if self.low_power:
            self._wakeup()

        # Build frame data with command and parameters.
        data = bytearray(2 + len(params))
        data[0] = _HOSTTOPN532
        data[1] = command & 0xFF
        for i, val in enumerate(params):
            data[2 + i] = val
        # Send frame and wait for response.
        try:
            self._write_frame(data)
        except OSError:
            return False
        if not self._wait_ready(timeout):
            return False
        # Verify ACK response and wait to be ready for function response.
        if not _ACK == self._read_data(len(_ACK)):
            raise RuntimeError("Did not receive expected ACK from PN532!")
        return True

    def process_response(self, command, response_length=0, timeout=1):
        """Process the response from the PN532 and expect up to response_length
        bytes back in a response.  Note that less than the expected bytes might
        be returned! Will wait up to timeout seconds for a response and return
        a bytearray of response bytes, or None if no response is available
        within the timeout.
        """
        if not self._wait_ready(timeout):
            return None
        # Read response bytes.
        response = self._read_frame(response_length + 2)
        # Check that response is for the called function.
        if not (response[0] == _PN532TOHOST and response[1] == (command + 1)):
            raise RuntimeError("Received unexpected command response!")
        # Return response data.
        return response[2:]

    def power_down(self):
        """Put the PN532 into a low power state. If the reset pin is connected a
        hard power down is performed, if not, a soft power down is performed
        instead. Returns True if the PN532 was powered down successfully or
        False if not."""
        if self._reset_pin:  # Hard Power Down if the reset pin is connected
            self._reset_pin.value = False
            self.low_power = True
        else:
            # Soft Power Down otherwise. Enable wakeup on I2C, SPI, UART
            response = self.call_function(_COMMAND_POWERDOWN, params=[0xB0, 0x00])
            self.low_power = response[0] == 0x00
        time.sleep(0.005)
        return self.low_power

    @property
    def firmware_version(self):
        """Call PN532 GetFirmwareVersion function and return a tuple with the IC,
        Ver, Rev, and Support values.
        """
        response = self.call_function(_COMMAND_GETFIRMWAREVERSION, 4, timeout=0.5)
        if response is None:
            raise RuntimeError("Failed to detect the PN532")
        return tuple(response)

    def SAM_configuration(self):  # pylint: disable=invalid-name
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
        response = self.listen_for_passive_target(card_baud=card_baud, timeout=timeout)
        # If no response is available return None to indicate no card is present.
        if not response:
            return None
        return self.get_passive_target(timeout=timeout)

    def listen_for_passive_target(self, card_baud=_MIFARE_ISO14443A, timeout=1):
        """Send command to PN532 to begin listening for a Mifare card. This
        returns True if the command was received succesfully. Note, this does
        not also return the UID of a card! `get_passive_target` must be called
        to read the UID when a card is found. If just looking to see if a card
        is currently present use `read_passive_target` instead.
        """
        # Send passive read command for 1 card.  Expect at most a 7 byte UUID.
        try:
            response = self.send_command(
                _COMMAND_INLISTPASSIVETARGET, params=[0x01, card_baud], timeout=timeout
            )
        except BusyError:
            return False  # _COMMAND_INLISTPASSIVETARGET failed
        return response

    def get_passive_target(self, timeout=1):
        """Will wait up to timeout seconds and return None if no card is found,
        otherwise a bytearray with the UID of the found card is returned.
        `listen_for_passive_target` must have been called first in order to put
        the PN532 into a listening mode.

        It can be useful to use this when using the IRQ pin. Use the IRQ pin to
        detect when a card is present and then call this function to read the
        card's UID. This reduces the amount of time spend checking for a card.
        """
        response = self.process_response(
            _COMMAND_INLISTPASSIVETARGET, response_length=30, timeout=timeout
        )
        # If no response is available return None to indicate no card is present.
        if response is None:
            return None
        # Check only 1 card with up to a 7 byte UID is present.
        if response[0] != 0x01:
            raise RuntimeError("More than one card detected!")
        if response[5] > 7:
            raise RuntimeError("Found card with unexpectedly long UID!")
        # Return UID of card.
        return response[6 : 6 + response[5]]

    def mifare_classic_authenticate_block(
        self, uid, block_number, key_number, key
    ):  # pylint: disable=invalid-name
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
        params = bytearray(3 + uidlen + keylen)
        params[0] = 0x01  # Max card numbers
        params[1] = key_number & 0xFF
        params[2] = block_number & 0xFF
        params[3 : 3 + keylen] = key
        params[3 + keylen :] = uid
        # Send InDataExchange request and verify response is 0x00.
        response = self.call_function(
            _COMMAND_INDATAEXCHANGE, params=params, response_length=1
        )
        return response[0] == 0x00

    def mifare_classic_read_block(self, block_number):
        """Read a block of data from the card.  Block number should be the block
        to read.  If the block is successfully read a bytearray of length 16 with
        data starting at the specified block will be returned.  If the block is
        not read then None will be returned.
        """
        # Send InDataExchange request to read block of MiFare data.
        response = self.call_function(
            _COMMAND_INDATAEXCHANGE,
            params=[0x01, MIFARE_CMD_READ, block_number & 0xFF],
            response_length=17,
        )
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
        assert (
            data is not None and len(data) == 16
        ), "Data must be an array of 16 bytes!"
        # Build parameters for InDataExchange command to do MiFare classic write.
        params = bytearray(19)
        params[0] = 0x01  # Max card numbers
        params[1] = MIFARE_CMD_WRITE
        params[2] = block_number & 0xFF
        params[3:] = data
        # Send InDataExchange request.
        response = self.call_function(
            _COMMAND_INDATAEXCHANGE, params=params, response_length=1
        )
        return response[0] == 0x0

    def ntag2xx_write_block(self, block_number, data):
        """Write a block of data to the card.  Block number should be the block
        to write and data should be a byte array of length 4 with the data to
        write.  If the data is successfully written then True is returned,
        otherwise False is returned.
        """
        assert data is not None and len(data) == 4, "Data must be an array of 4 bytes!"
        # Build parameters for InDataExchange command to do NTAG203 classic write.
        params = bytearray(3 + len(data))
        params[0] = 0x01  # Max card numbers
        params[1] = MIFARE_ULTRALIGHT_CMD_WRITE
        params[2] = block_number & 0xFF
        params[3:] = data
        # Send InDataExchange request.
        response = self.call_function(
            _COMMAND_INDATAEXCHANGE, params=params, response_length=1
        )
        return response[0] == 0x00

    def ntag2xx_read_block(self, block_number):
        """Read a block of data from the card.  Block number should be the block
        to read.  If the block is successfully read the first 4 bytes (after the
        leading 0x00 byte) will be returned.
        If the block is not read then None will be returned.
        """
        ntag2xx_block = self.mifare_classic_read_block(block_number)
        if ntag2xx_block is not None:
            return ntag2xx_block[0:4]  # only 4 bytes per page
        return None
