#!/usr/bin/python

"""
  dxpose is program to record sequences of poses of chain of Dynamixel servos
  uses opencm 9.04 to implement sync_read command - please see respective files
  
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

import sys
import getopt
import time
import kbhit
import driver
import project

def usage():
	print( 'dxpose --max-servo [max servo id] --serial [port] --play [filename] | --rec [file name]' )

def main( argv ):

	max_servo_id = 0
	port = ''
	playFName = ''
	recFname = ''

	try:
		opts, args = getopt.getopt( argv, "hm:s:p:r:", [ "help", "max-servo", "serial", "play", "rec" ] )
	except getopt.GetoptError:
		usage()
		sys.exit(2)

	for opt, arg in opts:
		if opt in ("-h", "--help"):
			usage()
			sys.exit()
		elif opt in ( "-m", "--max-servo" ):
			try:
				print( 'max servo: ' + arg )
				max_servo_id = int( arg )
			except ValueError:
				usage()
				sys.exit(2)
		elif opt in ( "-s", "--serial" ):
			print( 'serial: ' + arg )
			port = arg
		elif opt in ( "-p", "--play" ):
			playFName = opt
		elif opt in ( "-r", "--rec" ):
			recFName = opt

	if max_servo_id <= 0:
		print( 'max servo id needs to > 0' )
		sys.exit( 2 )

	# open serial port
	try:
		drv = driver.Driver( port, 115200 )
	except:
		e = sys.exc_info()[0]
		print( 'Error: ' + str( e ) + '. unable to open port: ' + port )
		sys.exit( 2 )

	# ping servo
	print( 'reading servo id' )
	res = drv.ping( 1 )

	print( 'got servo id: ' + str( res ) )
	sys.exit( 0 )

	if recFName != '':
		actionRecord( drv, max_servo_id, recFName )


	if playFName != '':
		actionPlay( drv, playFName ) 

	sys.exit( 0 )
	

""" record action """
def actionRecord( drv, max_servo_id, recFName):

	pr = Project()
	
	servoIDs = range( max_servo_id )
	pr.servoIDs = servoIDs
	
	kb = KBHit()
	while True:
		if kb.kbhit():
			print( 'Saving data into: ' + recFname )
			pr.save( recFName )
			return
			
		res = drv.syncRead( servoIDs )
	
		action = Project.Action()
		action.timestamp = res[ 0 ]
		a = []
		
		for i in range( 1, len( res ) - 1 ):
			a.append( res[ i ] )
		
		action.pose = a
		
		pr.addAction( action )
 

""" play action """
def actionPlay( drv, playFName ):

	pr = project.Project( playFName )
	
	prev_timestamp = 0
	
	for el in pr.sequence:
		if prev_timestamp != 0:
			time.sleep( (el.timestamp - prev_timestamp) / 1000.0 )

		prev_timestamp = el.timestamp
		
		drv.syncWrite( P_GOAL_POSITION_L, 2, [] )

			
if __name__ == "__main__":
    main(sys.argv[1:])
