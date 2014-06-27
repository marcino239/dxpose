// OpenCM 9.04 sketch to init servo and test it
// uses libmaple with Dynamixel library
// please see github.com/marcino239/libmaple
//
// LICENSE: GPL2
//

#include <wirish/wirish.h>


#define USB_PACKET_TIMEOUT	100			// ms
#define DXL_BAUDRATE		1000000

// registers
#define DXL_ADDR_PRESENT_POS  0x24

// dynamixel commands
#define DXL_CMD_READ_DATA	2

// controller commands
#define CMD_SYNC_READ		8

// dynamixel response
#define DXL_RES_INS_ERROR	(uint8)0x40
#define DXL_RES_NO_ERROR	(uint8)0x00

// controller address
#define ID_CONTROLLER	253

// serial port for dynamixels
#define DXL_SERIAL		Serial1
#define DXL_DIRECTION	28

// max number of dynamixels
#define MAX_DXLS		10

// libmaple constructor
__attribute__((constructor)) void premain() {
    init();
}

enum DxStates {
	DXS_WAITING = 0,
	DXS_H1,
	DXS_H2,
	DXS_ID,
	DXS_LEN,
	DXS_CMD,
	DXS_PARAMS,
	DXS_CSUM
};

// global variables
enum DxStates dxstate = DXS_WAITING;
uint8 gotID;
uint8 gotLen;
uint8 gotCmd;
uint8 paramCount;
uint8 gotCsum;
uint8 tempCsum;
int packetTimeStart;
bool gotPacket;

uint32 dxl_timeout;

// packet receive buffer
#define MAX_BUFF	256
uint8 buff[ MAX_BUFF ];

// panic function
void panic(void) {
  while( 1 ) {
    toggleLED();
    delay( 50 );
  }
}

// response functions
void respondInvalidCommand() {

	SerialUSB.write( 0xff );
	SerialUSB.write( 0xff );
	SerialUSB.write( ID_CONTROLLER );
	SerialUSB.write( 0x02 );
	SerialUSB.write( DXL_RES_INS_ERROR );	
	SerialUSB.write( (uint8)~(ID_CONTROLLER + 0x02 + DXL_RES_INS_ERROR) );

}

void respondOkCommand() {

	SerialUSB.write( 0xff );
	SerialUSB.write( 0xff );
	SerialUSB.write( ID_CONTROLLER );
	SerialUSB.write( 0x02 );
	SerialUSB.write( DXL_RES_NO_ERROR );	
	SerialUSB.write( (uint8)~(ID_CONTROLLER + 0x02 + DXL_RES_NO_ERROR) );

}

void static inline DxlSetTX() {
    digitalWrite(DXL_DIRECTION, HIGH); // TX
    asm("nop");
}

void static inline DxlSetRX() {
    asm("nop");
    digitalWrite(DXL_DIRECTION, LOW); // RX
}

// returns false if timeout or csum incorrect
//
int DxlReadPacketWord( uint8 len, uint16 *ptrWord ) {

  uint8 local_buff[ MAX_BUFF ];

  uint32 startTime = millis();

  int tLen = 0;
  for( ; tLen < len; tLen = DXL_SERIAL.available() )
    if( millis() - startTime > dxl_timeout )
      return false;
      
  // read into buffer
  int i;
  uint8 csum = 0;
  for( i=0; i<tLen; i++ ) {
    uint8 b = DXL_SERIAL.read();
    local_buff[ i ] = b;
    csum += b;
  }
    
  // check checksum
  for( i=2; i<tLen; i++ )
    csum += local_buff[ i ];
    
  if( csum != 0 )
    return false;
  else {
    *ptrWord = local_buff[ 5 ] | local_buff[ 5+1 ] << 8;
    return true;
  }
}
		

int DxlReadWord( uint8 id, uint8 addr, uint16 *ptrWord ) {

	DxlSetTX();
	DXL_SERIAL.write( 0xff );
	DXL_SERIAL.write( 0xff );
	DXL_SERIAL.write( id );
	DXL_SERIAL.write( 0x04 );
	DXL_SERIAL.write( DXL_CMD_READ_DATA );
	DXL_SERIAL.write( addr );
	DXL_SERIAL.write( 2 );
	DXL_SERIAL.write( (uint8)~(id + 0x04 + DXL_CMD_READ_DATA + addr + 2) );
	DxlSetRX();

	return DxlReadPacketWord( 2 + 6, ptrWord );
}


// this function assumes there is already received packet in buff. each element of buffer is
// id of servo to query for
// this function returns Dynamixel format packet in the form:
// ff ff _id_ _len_ _cmd_ _time_stamp_32_ _pos0_ _pos1_ ... _csum_
//
int processSyncRead() {

	if( paramCount > MAX_DXLS ) { // check if number of parameters does not exceed max servos
		respondInvalidCommand();
		return false;
	}
	
	uint8 local_buff[ MAX_BUFF ];
	
	local_buff[ 0 ] = 0xff;
	local_buff[ 1 ] = 0xff;
	local_buff[ 2 ] = ID_CONTROLLER;
	local_buff[ 3 ] = 2 + 2 * paramCount + 4;
	local_buff[ 4 ] = DXL_RES_NO_ERROR;

	uint32 timeStamp = millis();
	local_buff[ 5 ] = (timeStamp >> 24) & 0xff;
	local_buff[ 6 ] = (timeStamp >> 16) & 0xff;
	local_buff[ 7 ] = (timeStamp >> 8) & 0xff;
	local_buff[ 8 ] = (timeStamp >> 0) & 0xff;
	
	int i;
	uint8 csum = 0;
	for( i=0; i<paramCount; i++ ) {
		uint16 pos;
		
		if( !DxlReadWord( buff[ i ], DXL_ADDR_PRESENT_POS, &pos ) ) {
			respondInvalidCommand();
			return false;
		}
		
		local_buff[ 8 + 2*i     ] = (pos >> 8) & 0xff;
		local_buff[ 8 + 2*i + 1 ] = (pos >> 0) & 0xff;
		
		csum += (pos >> 8) & 0xff;
		csum += (pos >> 0) & 0xff;
	}
	
	// compute csum
	for( i=2; i<8; i++ )
		csum += local_buff[ i ];
	
	uint8 totResponseLen = 9 + 2*paramCount; 
	local_buff[ totResponseLen ] = ~csum;
	SerialUSB.write( local_buff, totResponseLen + 1 );
  return true;
}

// set up
void setup() {
	// Set up the LED to blink
	pinMode(BOARD_LED_PIN, OUTPUT);

	// init usb
	SerialUSB.begin();

	// init dynamixel
	dxl_timeout = 10000000 / DXL_BAUDRATE;
	afio_remap( AFIO_REMAP_USART1 );

	// Initializing pins
	gpio_set_mode( GPIOB, 6, GPIO_AF_OUTPUT_PP );
	gpio_set_mode( GPIOB, 7, GPIO_INPUT_FLOATING );

	// Direction pins
	pinMode( DXL_DIRECTION, OUTPUT );
	digitalWrite( DXL_DIRECTION, LOW) ;

	DXL_SERIAL.begin( DXL_BAUDRATE );

	// init variables
	dxstate = DXS_WAITING;
	gotPacket = false;
}

/*
* packet: ff ff id length ins params checksum
* same as ax-12 table, except, we define new instructions
*
* ID = 253 for these special commands
*   cmd = 8: Sync read: Returns: <32 bit mills time since init> <2B: servo 1 id pose> ... 
* 
* */

void loop(){
	uint8 b;
	
	// process messages from USB
	while( SerialUSB.available() > 0 ) {
		// We need to 0xFF at start of packet
		b = SerialUSB.read();
		
		switch( dxstate ) {
			case DXS_WAITING:
				if( b == 0xff ) {
					dxstate = DXS_H2;
					packetTimeStart = millis();
					toggleLED();
				} else
					dxstate = DXS_WAITING;
					
				break;
				
			case DXS_H2:
				if( b == 0xff )
					dxstate = DXS_ID;
				else
					dxstate = DXS_WAITING;

				break;

			case DXS_ID:
				gotID = b;
				dxstate = DXS_LEN;
				tempCsum = b;
				break;
				
			case DXS_LEN:
				gotLen = b;
				dxstate = DXS_CMD;
				tempCsum += b;
				break;
				
			case DXS_CMD:
				gotCmd = b;
				dxstate = DXS_PARAMS;
				paramCount = 0;
				tempCsum += b;
				break;

			case DXS_PARAMS:
				if( paramCount < gotLen - 3 ) {
					buff[ paramCount++ ] = b;
					tempCsum += b;
				} else
					dxstate = DXS_WAITING;

				if( paramCount == gotLen )
					dxstate = DXS_CSUM;
					
				break;
				
			case DXS_CSUM:
				gotCsum = b;
				dxstate = DXS_WAITING;
				gotPacket = true;
				tempCsum = ~tempCsum;
				break;
				
			default:
				dxstate = DXS_WAITING;
				break;
		}
		
		// process packet
		if( gotPacket ) {
			if( gotID == ID_CONTROLLER ) {
				if( tempCsum != gotCsum ) {
					respondInvalidCommand();
				} else if( gotCmd == CMD_SYNC_READ ) {
					processSyncRead();
				} else
					// invalid command
					respondInvalidCommand();

				gotPacket = false;

			} else {
				// send packet to the dynamixel network
				DxlSetTX();
				
				DXL_SERIAL.write( 0xff );
				DXL_SERIAL.write( 0xff );
				DXL_SERIAL.write( gotID );
				DXL_SERIAL.write( gotLen );
				DXL_SERIAL.write( gotCmd );
				DXL_SERIAL.write( buff, gotLen );
				DXL_SERIAL.write( gotCsum );

				DxlSetRX();
			}
		}
	}

	// check for USB packet timeout
	if( !gotPacket && dxstate != DXS_WAITING && (millis() - packetTimeStart > USB_PACKET_TIMEOUT) ) {
    // got timeout
    dxstate = DXS_WAITING;
  }


	// process dynamixel received data
	while( DXL_SERIAL.available() > 0 )
		SerialUSB.write( DXL_SERIAL.read() );
}

int main()
{
  setup();
	
  while( 1 )
	loop();

  return 0;
}
