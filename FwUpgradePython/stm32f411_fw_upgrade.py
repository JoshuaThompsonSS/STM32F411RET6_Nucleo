"""
Desc: Upgrade firmware of ST Nucleo F411RE STM32F4 board
      Talks to Nucleo via USART1_RX (PA10) and USART1_TX (PA9)
	  See AN3155 (application note for stm32f4 bootloader)	  
"""

import serial

WriteCmd = [0x31, 0xCE]
ReadCmd  = [0x11, 0xEE]
BootCmd  = [0x7f]
DataSize = 4
uart = serial.Serial()

def initSerial(port):
	uart.port = port
	uart.baudrate = 9600
	uart.timeout = 0.5
	uart.parity = "E"
	uart.open()
	

def getCheckSum(data):
	if type(data) != list:
		print "get checksum data requires list"
		return
	checksum = 0
	for d in data:
		checksum ^= d
		
	return checksum
		
 
def startBoot():
	uart.write(BootCmd)
	r = uart.read()
	if r != 'y':
		print "failed to start boot cmd"
		return
 
def read(index_num, start_address):
	# read memory command
	# index_num = num bytes to read - 1
	if type(start_address) != list:
		print "read cmd buffer must be an array"
		return
		
	# send read cmd
	uart.write(ReadCmd)
	r = uart.read() # verify accepted by receiving an ACK byte
	if r != 'y':
		print "failed to send read cmd"
		return
	
	# Send start address bytes (4 bytes) along with checksum of start address bytes
	checksum = getCheckSum(start_address)
	address_data = start_address + [checksum]
	uart.write(address_data)
	r = uart.read()
	if r != 'y':
		print "failed to send read cmd address data"
		return
		
	# Send num bytes - 1 to be read along with checksum
	checksum = getCheckSum([0xff, index_num]) #use 0xff when only one byte to checksum
	uart.write([index_num, checksum])
	r = uart.read()
	if r != 'y':
		print "failed to send num bytes index + checksum during read cmd"
		return
	resp = []
	for i in range(index_num + 1):
		resp.append(uart.read())
	
	return resp
		
	
	
def write(buffer, start_address):
	# write memory command 
	if type(buffer) != list or type(start_address) != list:
		print "write cmd buffer must be an array"
		return
	# Send write cmd id to bootloader device
	uart.write(WriteCmd)
	r = uart.read() # verify accepted by receiving an ACK byte
	if r != 'y':
		print "failed to send write cmd"
		return
	
	# Send start address bytes (4 bytes) along with checksum of start address bytes
	checksum = getCheckSum(start_address)
	address_data = start_address + [checksum]
	uart.write(address_data)
	r = uart.read()
	if r != 'y':
		print "failed to send write cmd address data"
		return
	# Send Index num, Data Bytes, Checksum(Index Num, Data Bytes)
	num_bytes = len(buffer)
	if num_bytes%DataSize != 0:
		print "Num bytes in data buffer not a multiple of %d" % DataSize
		return
	index_num = num_bytes - 1
	checksum = getCheckSum([index_num] + buffer)
	
	uart.write([index_num] + buffer + [checksum])
	r = uart.read()
	if r != 'y':
		print "failed to write buffer data"
		return
	