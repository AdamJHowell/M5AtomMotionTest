#include <Wire.h>
#include <M5Atom.h>
#include <M5_DLight.h>
#include "AtomMotion.h"
#include <Arduino.h>
#include <array>


#define PORT_B  23
#define PORT_C  22
#define RED     0xFF0000
#define ORANGE  0xFF8000
#define YELLOW  0xFFFF00
#define GREEN   0x00FF00
#define BLUE    0x0000FF
#define INDIGO  0x4B0082
#define VIOLET  0xEE82EE
#define BLACK   0x000000
#define MAGENTA 0xFF00FF
#define CYAN    0x00FFFF
#define WHITE   0xFFFFFF


AtomMotion atomMotion;                                                    // An object to manage the ATOM Motion.
xSemaphoreHandle CtlSemaphore;                                            // A semaphore the ATOM Motion uses to control motors and/or servos.
unsigned long lastLoop = 0;                                               // Tracks the last time the main loop executed.
const unsigned int NUM_SENSORS = 4;                                       // The number of sensors.
const unsigned long loopDelay = 10;                                       // The maximum value of 4,294,967,295 allows for a delay of about 49.7 days.
const byte sdaGPIO = 26;                                                  // Use this to set the SDA GPIO if your board uses a non-standard GPIOs for the I2C bus.
const byte sclGPIO = 32;                                                  // Use this to set the SCL GPIO if your board uses a non-standard GPIOs for the I2C bus.
const int PCA_ADDRESS = 0x70;                                             // The I2C address of the Pa.HUB.
std::array<M5_DLight, NUM_SENSORS> sensorArray = {};                      // An array of DLIGHT sensor objects.
std::array<uint16_t, NUM_SENSORS> sensorAddresses = { 0, 1, 4, 5 };       // An array of the Pa.HUB ports with DLIGHT sensors attached.
std::array<uint16_t, NUM_SENSORS> luxValues = { 2112, 2112, 2112, 2112 }; // An array of light readings, one per sensor.
unsigned long                      lastPrintLoop   = 0;                          // Tracks the last time the print loop executed.
const unsigned long                printLoopDelay  = 1000;                       // The minimum time between serial printing of the lux values.
const unsigned int                 SERVO_MIN       = 500;                        // The minimum pulse width for the servos.
const unsigned int                 SERVO_MAX       = 2500;                       // The maximum pulse width for the servos.
const unsigned int                 DEAD_BAND       = 20;                         // The minimum delta before a servo will move.
uint16_t                           pulseWidth      = 1500;                       // Min: 500, Max: 2500, Neutral: 1500


void pcaSelect( uint8_t i )
{
	if( i > 7 )
		return;
	Wire.beginTransmission( PCA_ADDRESS );
	Wire.write( 1 << i );
	Wire.endTransmission();
} // End of pcaSelect()


/*
 * pulseWidth is a global that is updated in loop().
 * pvParameters is not used.
 */
void TaskMotion( void *pvParameters )
{
	while( 1 )
	{
    // Set each servo to the computed speed.
		for( int ch = 1; ch < 5; ch++ )
			atomMotion.SetServoPulse( ch, pulseWidth );
    // Delay for five ticks to allow other threads to regain control.
		vTaskDelay( 5 / portTICK_RATE_MS );
	}
} // End of TaskMotion()


void setup()
{
  // SerialEnable, I2CEnable, DisplayEnable
	M5.begin( true, false, true );
	// Wire.begin() must happen before atomMotion.Init().
	Wire.begin( sdaGPIO, sclGPIO );
	// Wire.begin() must happen before atomMotion.Init().
	atomMotion.Init();
	vSemaphoreCreateBinary( CtlSemaphore );
	xTaskCreatePinnedToCore(
		TaskMotion,	  // Pointer to the task entry function.
		"TaskMotion", // A descriptive name for the task.
		4096,			    // The size of the task stack specified as the number of bytes.
		NULL,			    // Pointer that will be used as the parameter for the task being created.
		2,				    // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
		NULL,			    // Used to pass back a handle by which the created task can be referenced.
		0 );				  // Values 0 or 1 indicate the index number of the CPU which the task should be pinned to.
	pinMode( PORT_B, INPUT_PULLUP );
	pinMode( PORT_C, INPUT_PULLUP );
	M5.dis.drawpix( 0, WHITE );

	for( uint8_t i = 0; i < NUM_SENSORS; i++ )
	{
		pcaSelect( sensorAddresses[i] );
		sensorArray[i].begin();
		sensorArray[i].setMode( CONTINUOUSLY_H_RESOLUTION_MODE );
	}
	Serial.println( "\nI2C scanner and lux sensor are ready!" );
} // End of setup()


void loop()
{
  M5.update();
  // Read all sensors before acting on the values.
  for( uint8_t i = 0; i < NUM_SENSORS; i++ )
  {
    pcaSelect( sensorAddresses[i] );
    luxValues[i] = sensorArray[i].getLUX();
  }
  // Sum the top sensors.
  long topRow = luxValues[0] + luxValues[1];
  // Sum the bottom sensors.
  long bottomRow = luxValues[2] + luxValues[3];
  // Sum the left sensors.
  long leftSide = luxValues[0] + luxValues[2];
  // Sum the right sensors.
  long rightSide = luxValues[1] + luxValues[3];

  if( abs( topRow - bottomRow ) > DEAD_BAND )
  {
    // I constrain to this range because once the delta is 3k, we should be moving at full speed to correct that delta.
    long rowValue = constrain( topRow - bottomRow, -3000, 3000 );
    // map( value, fromLow, fromHigh, toLow, toHigh );
    pulseWidth = map( rowValue, -3000, 3000, SERVO_MIN, SERVO_MAX );
  }

  if( M5.Btn.wasPressed() )
  {
    M5.dis.drawpix( 0, RED );
    Serial.printf( "Button was pressed.\n" );
  }

  // If the up stop is tripped, prevent the servo from moving upward.
  if( !digitalRead( PORT_B ) )
  {
    pulseWidth = constrain( pulseWidth, 1500, 2500 );
    Serial.printf( "Hit limit B!\n" );
    M5.dis.drawpix( 0, GREEN );
  }

  // If the down stop is tripped, prevent the servo from moving downward.
  if( !digitalRead( PORT_C ) )
  {
    pulseWidth = constrain( pulseWidth, 500, 1500 );
    Serial.printf( "Hit limit C!\n" );
    M5.dis.drawpix( 0, BLUE );
  }

  // Print values in a format the Arduino Serial Plotter can use.
  if( ( lastPrintLoop == 0 ) || ( millis() - lastPrintLoop ) > printLoopDelay )
  {
    Serial.printf( "L0:%d L1:%d L4:%d L5:%d delta:%d pulseWidth:%d\n", luxValues[0], luxValues[1], luxValues[2], luxValues[3], topRow - bottomRow, pulseWidth );
    lastPrintLoop = millis();
  }
} // End of loop()
