#! /usr/bin/env python
"""
	bluetooth buddy program
	
	note that debian testing's current repository of bluez is broken
	install from source
"""

import bluetooth
import time

SLEEP_SECONDS = 1
SQUIGGLE_SECONDS = 3

RDY_MSG = "{" # we send this to the watch over and over until connected 
END_MSG = "}"

baddr = "00:06:66:09:85:1A"
port = 1
saidHello = False
while not saidHello:
	try: 
		sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
		sock.connect((baddr, port))
		sock.send(RDY_MSG + "\n")
		saidHello = True
	except bluetooth.BluetoothError as e:
		print "no connection yet! %s" % str(e) 
		sock.close()
		time.sleep(SLEEP_SECONDS)
curRecv = ""
response = ""
run = True
try:
	while run:
		# NOTE: blocking receive
		curRecv = sock.recv(1024)
		response += curRecv 
		# now that it is a single character, i can do this!
		if END_MSG in curRecv: 
			run = False
except KeyboardInterrupt:
	print '\nclosing socket...'
	sock.close()
print 'string ended, closing socket...'
sock.close()
print "response: %s" % response
outFile = open("out.txt", "w")
outFile.write(response)
outFile.close
