from Adafruit_Circuitpython_PN532 import adafruit_pn532
from digitalio import DigitalInOut, Direction, Pull
import board
import time
import busio
# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
pn532 = adafruit_pn532.PN532_I2C(i2c)

ic, ver, rev, support = pn532.get_firmware_version()
print('Found PN532 with firmware version: {0}.{1}'.format(ver, rev))

# Configure PN532 to communicate with MiFare cards.
pn532.SAM_configuration()

print('Waiting for MiFare card...')
while True:
    # Check if a card is available to read
    uid = pn532.read_passive_target(timeout=0.25)
    # Try again if no card is available.
    if uid is None:
        continue
    print('Found card with UID:', [hex(i) for i in uid])