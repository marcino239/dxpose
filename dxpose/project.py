"""
  dxpose - by marcino239
  
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

import pickle

""" this class implements sequence storage
    each sequence is set of actions
    each action is a tuple (time, position1, position2, ... )
    servo id are in seroIDs variable
"""
class Project:
	def __init__( self, fName = '' ):
		self.servoIDs = []
		self.sequence = []  # list of actions
		
		if fName != '':
			self.load( fName )

	def addAction( self, action ):
		self.sequence.append( action )
		
	def save( self, fName ):
		f = open( fName, 'wb' )
		pickle.dump( self.__dict__, f )
		f.close()          

	def load( self, fName ):
		f = open( fName, 'rb' )
		tmp_dict = pickle.load( f )
		f.close()       
		self.__dict__.update( tmp_dict )

