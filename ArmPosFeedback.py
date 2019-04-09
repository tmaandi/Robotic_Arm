import os
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

from ArmConstants import *

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D22)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channels on pins 0-3
chan0 = AnalogIn(mcp, MCP.P0)
chan1 = AnalogIn(mcp, MCP.P1)
chan2 = AnalogIn(mcp, MCP.P2)
chan3 = AnalogIn(mcp, MCP.P3)

feedback_channels = [chan0, chan1, chan2, chan3]

if __name__ == "__main__":
    while(1):
        print(('Raw ADC Value: ', [chan0.value,chan1.value,chan2.value,chan3.value] ))
        print(('ADC Voltage: ' + str([chan0.voltage,chan1.voltage,chan2.voltage,chan3.voltage]) + 'V'))