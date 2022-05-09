#!/usr/bin/env python
import serial
import serial.tools.list_ports

def find_arduino(vendor_id):
	for pinfo in serial.tools.list_ports.comports():
		if pinfo.vendor_id == vendor_id:
			return serial.Serial(pinfo.device)
	raise IOError("Could not find an arduino device - is it plugged in?")

print(find_arduino('1A86'))	
