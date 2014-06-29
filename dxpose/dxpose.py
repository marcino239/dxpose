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

# controls refresh rate [seconds]
DX_POSE_DELAY = 0.1

def usage():
	print( 'dxpose --max-servo [max servo id] --serial [port] --play [filename] | --rec [file name]' )

def main( argv ):

	max_servo_id = 0
	port = ''
	playFName = ''
	recFName = ''

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
			playFName = arg
		elif opt in ( "-r", "--rec" ):
			recFName = arg

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

	if recFName != '':
		actionRecord( drv, max_servo_id, recFName )

	if playFName != '':
		actionPlay( drv, playFName ) 

	sys.exit( 0 )
	

""" record action """
def actionRecord( drv, max_servo_id, recFName):

#	print( 'moving' )
#	drv.setPos( 1, 400 )
#	time.sleep( 1 )
#
#	for i in range( 400, 700, 10 ):
#		drv.setPos( 1, i )
#		time.sleep( 0.1 )
#

	servoIDs = range( 1, max_servo_id + 1 )
	print( 'servoIDs: ' + str( servoIDs ) )

	print( 'setting torque off' )
	for ID in servoIDs:
		print( '  ID: ' + str( ID ) )
		drv.torqueOff( ID )
	
	pr = project.Project()
	pr.servoIDs = servoIDs

#	print( 'sync read' )
#	res = drv.syncRead( servoIDs )
#	print( 'sync read res: ' + str( res ) )
#	print( 'single read' )
#	res = drv.readPos( 2 )
#	print( 'single read res: ' + str( res ) )

	print( 'recording action' )

	kb = kbhit.KBHit()
	while True:
		if kb.kbhit():
			print( 'Saving data into: ' + recFName )
			pr.save( recFName )
			return
		

#		res = drv.syncRead( servoIDs )
		res = []
		for i in servoIDs:
			res.append( drv.readPos( i ) )

		action = []
		action.append( int(round(time.time() * 1000)) )
		
		for i in range( 0, len( res ) ):
			action.append( res[ i ] )

		pr.addAction( action )
		for a in action:
			sys.stdout.write( str( a ) + ' ' )
		print( " ." )


		time.sleep( DX_POSE_DELAY )
 

""" play action """
def actionPlay( drv, playFName ):

	pr = project.Project( playFName )

	servoIDs = pr.servoIDs
	print( 'servoIDs: ' + str( servoIDs ) )

	print( 'setting torque on' )
	for ID in servoIDs:
		print( '  ID: ' + str( ID ) )
		drv.torqueOn( ID )
	
	for action in pr.sequence:
		print( 'action: ' + str( action ) )
		for p in range( 1, len( action ) ):
			drv.setPos( servoIDs[ p - 1 ], action[ p ] )

		time.sleep( DX_POSE_DELAY )

			
if __name__ == "__main__":
    main(sys.argv[1:])
