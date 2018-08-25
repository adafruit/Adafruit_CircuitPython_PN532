from Adafruit_Circuitpython_PN532 import adafruit_pn532
from digitalio import DigitalInOut, Direction, Pull
import board
import time
import busio

reset_pin = DigitalInOut(board.D3)


# I2C connection:
#i2c = busio.I2C(board.SCL, board.SDA)
#pn532 = adafruit_pn532.PN532_I2C(i2c, debug=False, reset=reset_pin)

# SPI connection:
#spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
#cs_pin = DigitalInOut(board.D2)
#pn532 = adafruit_pn532.PN532_SPI(spi, cs_pin, debug=False, reset=reset_pin)

# UART connection
#uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=100)
#pn532 = adafruit_pn532.PN532_UART(uart, debug=False, reset=reset_pin)

ic, ver, rev, support = pn532.get_firmware_version()
print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))

# Configure PN532 to communicate with MiFare cards 
pn532.SAM_configuration()

print('Waiting for MiFare card...')
while True:
    # Check if a card is available to read
    uid = pn532.read_passive_target(timeout=0.25)
    print('.', end="", flush=True)
    # Try again if no card is available.
    if uid is None:
        continue
    print('Found card with UID:', [hex(i) for i in uid])
