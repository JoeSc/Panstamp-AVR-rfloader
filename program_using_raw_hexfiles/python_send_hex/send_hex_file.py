import serial
import sys

f = open(sys.argv[1]).readlines()

ser = serial.Serial("/dev/tty.usbserial-A900fKM4", 57600)


def read_byte():
	dat = ser.read()
	sys.stdout.write(dat)
	sys.stdout.flush()
	return dat

while 1:
	if ser.inWaiting():
		ch = read_byte()
		if ch == 'r':
			break

for x in f:
	print "Sending " + x.strip()
	ser.write(x.strip())
	while 1:
		if ser.inWaiting():
			ch = read_byte()
			if ch == 'r':
				break
