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
from enum import Enum, unique

from ax12 import *


""" represents dynamixel driver """
class Driver:
	
	""" represents dynamixel packet """
	class DXPacket:
		def __init__():
			self.ID = 0
			self.cmd = 0
			self.length = 0
			self.params = bytearray( '' )
			self.csum = 0
		
		def checkCSum():
			tsum = self.ID + self.cmd + self.length
			for b in self.params
				tsum += b
			
			if (0xff - tsum) & 0xff == self.csum:
				return True
			else
				return False

    """ Class to open a serial port and control AX-12 servos"""
    def __init__(self, port="/dev/ttyACM0",baud=115200, timeour = 1 ):
        self.ser = serial.Serial()
        self.ser.baudrate = baud
        self.ser.port = port
        self.ser.timeout = 1
        self.ser.open()

	""" returns binary packet """
	def createPacket( id, cmd, params ):
		
		length = 2 + len( params )
		packet = bytearray( [ 0xff, 0xff, chr( id ), chr( length ), chr( cmd ) ] )
		for p in params:
			packet.append( chr( p ) )
		
		csum = 0
		for b in packet
			csum += b
		
		csum = 255 - (csum - 0xff - 0xff)
		return packet.append( csum )

	""" listens for a packet """
    def getPacket( self ):

		@unique
		class DXState( Enum ):
			DXS_NONE = 0
			DXS_H1 = 1
			DXS_ID = 2
			DXS_CMD = 3
			DXS_LEN = 4
			DXS_PARAMS = 5
			DXS_CSUM = 6
		
		dxstate = DXS_NONE
		packet = DXPacket()
		countParams = 0
		
		while True:
			b = self.ser.read()
			if b =='':
				raise Exception( "Serial Timeout" )
			
			if dxstate == DXS_NONE:
				if b != chr( 0xff ):
					raise Exception( "Wrong header" )
				dxstate = DXS_H1
			elif dxstate == DXS_H1:
				if b != chr( 0xff ):
					raise Exception( "Wrong header" )
				dxstate = DXS_ID
			elif dxstate == DXS_ID:
				packet.ID = b
				dxstate = DXS_LEN
			elif dxstate == DXS_LEN:
				packet.length = b
				dxstate = DXS_PARAMS
			elif dxstate == DXS_PARAMS:
				if countParams < packet.length - 2:
					packet.params.append( b )
					countParams += 1
				if countParams == packet.length - 2:
					dxstate = DXS_CSUM
			elif dxstate == DXS_CSUM:
				packet.csum = b
				
				if not packet.checkCSum():
					raise Exception( "Invalid checksum" )
				else
					return packet
			else
				raise Exception( "Invalid state" )


    def setReg(self, index, regstart, values):
		self.ser.flashOutput
        self.ser.write( createPacket( ID, AX_WRITE_DATA, [regstart] + values ) )

    def getReg(self, index, regstart, rlength):

		self.ser.write( createPacket( ID, AX_READ_DATA, [regstart, rlength] )

		packet = getPacket()
		
		l = len( packet.params )
		if l == 0:
			raise Exception( "Read Failed: Servo ID: " + str( ID ) )
		if l == 1:
			return packet.params[ 0 ]
		if l == 2:
			return packet.params[ 0] | packet.params[ 1 ] << 8

		return packet.params

	""" synchronised write. vals has the following composition: 
	
		[ [ ID1, [ P1, P2 ] ], [ID2, [P1,P2], ... ]

		parameters are assumed to be uint16

	"""
    def syncWrite(self, addr, length, vals ):
		v = bytearray( '' )

		# add register address and length
		v.append( chr( addr ) )
		v.append( chr( length ) )
		
		for el in vals
			v.append( el[ 0 ] )		# id
			params = el[ 1 ]		# parameters
			for pel in params
				v.append( chr( pel & 0xff ) )
				v.append( chr( ( pel >> 8 ) & 0xff )

		self.ser.write( createPacket( AX_ID_BROADCAST, AX_SYNC_WRITE, v )

	""" returns position information for servos
		
		[ timestamp, [ID1, Pos1], [ID2, Pos2], ... ]
		
		timestamp is local controller time in milliseconds
		
	"""
	def syncRead( self, servoIDs ):
		
		ser.write( createPacket( AX_ID_CONTROLLER, AX_SYNC_READ, servoIDs ) )
		packet = getPacket()
		
		res = []
		p = packet.params
		
		res.append( int( (p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24) )
		
		length = len( p ) - 4
		if length % 3  != 0:
			raise Exception( "Invalid packet length in sync read"

		length /= 3
		for i in range( length )
			r = []
			r.append( int( p[ 4 + i*3 ] ) )
			r.append( int( p[ 4 + i*3 + 1 ] | p[ 4 + i*3 + 2 ] << 8 )

			res.append( r )
		
		return res
