"""
  dxpose by marcino239
  
  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software Foundation,
  Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

import serial
import time
import sys
from ax12 import *

import binascii

""" represents dynamixel driver """
class Driver:
	
	""" represents dynamixel packet """
	class DXPacket:
		def __init__( self ):
			self.ID = 0
			self.length = 0
			self.cmd = 0
			self.params = bytearray( '' )
			self.csum = 0
		
		def checkCSum( self ):
			tsum = self.ID + self.length + self.cmd 
			for b in self.params:
				tsum += b
			
			tsum = (0xff - tsum) & 0xff
#			print( 'tsum: ' + str( tsum ) + ', csum: ' + str( self.csum ) )
			if tsum == self.csum:
				return True
			else:
				return False

		def debug( self ):
			print( 'ID: ' + str( self.ID ) + ', len: ' + str( self.length ) + ', cmd: ' + str( self.cmd ) + ', par: ' + binascii.hexlify( self.params ) );
			

	""" Class to open a serial port and control AX-12 servos"""
	def __init__(self, port="/dev/ttyACM0",baud=115200, timeour = 1 ):
		self.ser = serial.Serial()
		self.ser.baudrate = baud
		self.ser.port = port
		self.ser.timeout = 1
		self.ser.open()

	""" returns binary packet """
	def createPacket( self, id, cmd, params ):
		
		length = 2 + len( params )
		packet = bytearray( [chr( 0xff ), chr( 0xff ), chr( id ), chr( length ), chr( cmd )] )

		for p in params:
			packet.append( chr( p & 0xff ) )
		
		csum = 0
		for b in packet:
			csum += b
		
		csum = (255 - (csum - 0xff - 0xff)) & 0xff
		packet.append( csum )

#		print( 'params len: ' + str( len( params ) ) + ', packet: ' + binascii.hexlify( packet ) )

		return packet

	""" listens for a packet """
	def getPacket( self ):

		DXS_NONE = 0
		DXS_H1 = 1
		DXS_ID = 2
		DXS_CMD = 3
		DXS_LEN = 4
		DXS_PARAMS = 5
		DXS_CSUM = 6

		dxstate = DXS_NONE
		packet = self.DXPacket()
		countParams = 0

		while True:
			b = self.ser.read()
			if b == '':
				raise Exception( "Serial Timeout" )

#			print( 'dxstate: ' + str( dxstate ) )
#			print( 'byte: ' + binascii.hexlify( b ) )

			if dxstate == DXS_NONE:
				if b != chr( 0xff ):
					raise Exception( "Wrong header" )
				dxstate = DXS_H1

			elif dxstate == DXS_H1:
				if b != chr( 0xff ):
					raise Exception( "Wrong header" )
				dxstate = DXS_ID

			elif dxstate == DXS_ID:
				packet.ID = ord( b )
				dxstate = DXS_LEN

			elif dxstate == DXS_LEN:
				packet.length = ord( b )
				dxstate = DXS_CMD

			elif dxstate == DXS_CMD:
				packet.cmd = ord( b )
				if packet.length == 2:
					dxstate = DXS_CSUM
				else:
					dxstate = DXS_PARAMS

			elif dxstate == DXS_PARAMS:
				if countParams < packet.length - 2:
					packet.params.append( ord( b ) )
					countParams += 1
				if countParams == packet.length - 2:
					dxstate = DXS_CSUM

			elif dxstate == DXS_CSUM:
				packet.csum = ord( b )
				if not packet.checkCSum():
					raise Exception( "Invalid checksum" )
				else:
					return packet
			else:
				raise Exception( "Invalid state" )

	def writeReg( self, ID, regstart, values ):
		self.ser.flushOutput()
		self.ser.write( self.createPacket( ID, AX_WRITE_DATA, [regstart] + values ) )

	def readReg( self, ID, regstart, rlength ):

		self.ser.write( self.createPacket( ID, AX_READ_DATA, [regstart, rlength] ) )
		packet = self.getPacket()

		l = len( packet.params )
		if l == 0:
			raise Exception( "Read Failed: Servo ID: " + str( ID ) )
		if l == 1:
			return packet.params[ 0 ]
		if l == 2:
			return packet.params[ 0] | packet.params[ 1 ] << 8

		return packet.params

	def ping( self, ID ):	
		self.ser.write( self.createPacket( ID, AX_PING, 1, [] ) )
		packet = self.getPacket()

		if packet.cmd == 0:
			return True
		else:
			return False


	""" synchronised write. vals has the following composition: 

	[ [ ID1, [ P1, P2 ] ], [ID2, [P1,P2], ... ]

	parameters are assumed to be uint16

	"""
	def syncWrite(self, addr, length, vals ):
		v = bytearray( '' )

		# add register address and length
		v.append( chr( addr ) )
		v.append( chr( length ) )

		for el in vals:
			v.append( el[ 0 ] )		# id
			params = el[ 1 ]		# parameters

		for pel in params:
			v.append( chr( pel & 0xff ) )
			v.append( chr( ( pel >> 8 ) & 0xff ) )

		self.ser.write( self.createPacket( AX_ID_BROADCAST, AX_SYNC_WRITE, v ) )

	""" returns position information for servos
		
		[ timestamp, Pos1, Pos2, ... ]
		
		timestamp is local controller time in milliseconds
		
	"""
	def syncRead( self, servoIDs ):
			
		self.ser.write( self.createPacket( AX_ID_CONTROLLER, AX_SYNC_READ, servoIDs ) )
		packet = self.getPacket()
		
		if packet.cmd != 0:
			raise Exception( "Invalid responose" )		

#		packet.debug()
		p = packet.params
		
		res = []
		res.append( int( (p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24) ) )

#		print( 'res: ' + str( res ) )

#		print( 'len(p): ' + str( len( p ) ) )
		length = len( p ) - 4
		if length % 2  != 0:
			raise Exception( "Invalid packet length in sync read" )

		length = length / 2
		for i in range( length ):
			res.append( int( p[ 4 + i*2 ] | (p[ 4 + i*2 + 1 ] << 8) ) )

#		print( 'res: ' + str( res ) )
		return res

	def torqueOff( self, ID ):
		self.writeReg( ID, P_TORQUE_ENABLE, [0] )
		packet = self.getPacket()
#		packet.debug()

		if packet.cmd == 0:
			return True
		else:
			return False

	def torqueOn( self, ID ):
		self.writeReg( ID, P_TORQUE_ENABLE, [1] )
		packet = self.getPacket()
#		packet.debug()

		if packet.cmd == 0:
			return True
		else:
			return False

	def words2bytes( self, vals ):
		res = []
		for v in vals:
			res.append( v & 0xff )
			res.append( (v >> 8) & 0xff )
		return res

	def setPos( self, ID, position ):
		self.writeReg( ID, P_GOAL_POSITION_L, self.words2bytes( [position] ) )
		packet = self.getPacket()
#		packet.debug()

		if packet.cmd == 0:
			return True
		else:
			return False

	def readPos( self, ID ):
		return self.readReg( ID, P_PRESENT_POSITION_L, 2 )

		
