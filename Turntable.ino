// DCC Stepper Motor Controller ( A4988 ) Example
// See: https://www.dccinterface.com/how-to/assemblyguide/
// 
// Author: Alex Shepherd 2017-12-04
// 
// This example requires two Arduino Libraries:
//
// 1) The AccelStepper library from: http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
//
// 2) The NmraDcc Library from: http://mrrwa.org/download/
//
// Both libraries can be installed via the Arduino IDE Library Manager 
//

#include <AccelStepper.h>
#include <NmraDcc.h>

// Uncomment to enable Powering-Off the Stepper if its not running 
#define STEPPER_ENABLE_PIN 6

// Home Position Sensor Input
#define HOME_SENSOR_PIN 3

typedef struct
{
	int address;
	int stationFront;
	int stationBack;
}
DCCAccessoryAddress;
DCCAccessoryAddress gAddresses[13];

// for a 1.8 deg stepper, there are 200 full steps
#define FULL_STEPS_PER_REVOLUTION 400

// Uncomment the lime below for the Driver Board Settings
//#define FULL_TURN_STEPS (FULL_STEPS_PER_REVOLUTION)     // full steps
//#define FULL_TURN_STEPS (FULL_STEPS_PER_REVOLUTION * 2) // 1/2 steps
//#define FULL_TURN_STEPS (FULL_STEPS_PER_REVOLUTION * 4) // 1/4 steps
//#define FULL_TURN_STEPS (FULL_STEPS_PER_REVOLUTION * 8) // 1/8 steps
#define FULL_TURN_STEPS (FULL_STEPS_PER_REVOLUTION * 16) // 1/16 steps


#define entryStation1 ((FULL_TURN_STEPS * 2) / 36) + 42
#define entryStation2  ((FULL_TURN_STEPS * 1) / 36) + 5
#define entryStation3 16       // home location
#define houseStation1 ((FULL_TURN_STEPS / 35) * -7) - 60
#define houseStation2 (houseStation1 - (FULL_TURN_STEPS / 35) + 2 ) + 4
#define houseStation3 (houseStation2 - (FULL_TURN_STEPS / 35) + 2 ) + 4
#define houseStation4 (houseStation3 - (FULL_TURN_STEPS / 35) + 2 ) + 4
#define houseStation5 (houseStation4 - (FULL_TURN_STEPS / 35) + 2 ) + 4
#define houseStation6 (houseStation5 - (FULL_TURN_STEPS / 35) + 2 ) + 4
#define houseStation7 (houseStation6 - (FULL_TURN_STEPS / 35) + 2 ) + 4
#define houseStation8 (houseStation7 - (FULL_TURN_STEPS / 35) + 2 ) + 4
#define houseStation9 (houseStation8 - (FULL_TURN_STEPS / 35) + 2 ) + 4
#define houseStation10 (houseStation9 - (FULL_TURN_STEPS / 35) + 2 ) + 4

volatile bool bInterruptDetected = false;
bool bHomePositionFound = false;

// Now we'll wrap the stepper in an AccelStepper object
AccelStepper stepper1(1, 4, 5);

NmraDcc  Dcc;
uint16_t lastAddr = 0xFFFF;
uint8_t lastDirection = 0xFF;
//
// Decoder Init
//
void ConfigureStations()
{
	// this is home
	gAddresses[0].address = 500;
	gAddresses[0].stationFront = entryStation1;
	gAddresses[0].stationBack = entryStation1 + (FULL_TURN_STEPS / 2);

	gAddresses[1].address = 501;
	gAddresses[1].stationFront = entryStation2;
	gAddresses[1].stationBack = entryStation2 + (FULL_TURN_STEPS / 2);

	gAddresses[2].address = 502;
	gAddresses[2].stationFront = entryStation3;
	gAddresses[2].stationBack = entryStation3 + (FULL_TURN_STEPS / 2);

	gAddresses[3].address = 503;
	gAddresses[3].stationFront = houseStation1;
	gAddresses[3].stationBack = houseStation1 + (FULL_TURN_STEPS / 2);

	gAddresses[4].address = 504;
	gAddresses[4].stationFront = houseStation2;
	gAddresses[4].stationBack = houseStation2 + (FULL_TURN_STEPS / 2);

	gAddresses[5].address = 505;
	gAddresses[5].stationFront = houseStation3;
	gAddresses[5].stationBack = houseStation3 + (FULL_TURN_STEPS / 2);

	gAddresses[6].address = 506;
	gAddresses[6].stationFront = houseStation4;
	gAddresses[6].stationBack = houseStation4 + (FULL_TURN_STEPS / 2);

	gAddresses[7].address = 507;
	gAddresses[7].stationFront = houseStation5;
	gAddresses[7].stationBack = houseStation5 + (FULL_TURN_STEPS / 2);

	gAddresses[8].address = 508;
	gAddresses[8].stationFront = houseStation6;
	gAddresses[8].stationBack = houseStation6 + (FULL_TURN_STEPS / 2);

	gAddresses[9].address = 509;
	gAddresses[9].stationFront = houseStation7;
	gAddresses[9].stationBack = houseStation7 + (FULL_TURN_STEPS / 2);

	gAddresses[10].address = 510;
	gAddresses[10].stationFront = houseStation8;
	gAddresses[10].stationBack = houseStation8 + (FULL_TURN_STEPS / 2);

	gAddresses[11].address = 511;
	gAddresses[11].stationFront = houseStation9;
	gAddresses[11].stationBack = houseStation9 + (FULL_TURN_STEPS / 2);

	gAddresses[12].address = 512;
	gAddresses[12].stationFront = houseStation10;
	gAddresses[12].stationBack = houseStation10 + (FULL_TURN_STEPS / 2);

}

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower)
{
	Serial.print("notifyDccAccTurnoutOutput: ");
	Serial.print(Addr, DEC);
	Serial.print(',');
	Serial.print(Direction, DEC);
	Serial.print(',');
	Serial.println(OutputPower, HEX);

	for (int i = 0; i < (sizeof(gAddresses) / sizeof(DCCAccessoryAddress)); i++)
	{
		if ((Addr == gAddresses[i].address) && ((Addr != lastAddr) || (Direction != lastDirection)) && OutputPower)
		{
			lastAddr = Addr;
			lastDirection = Direction;

			Serial.print(F("Moving to Station : "));
			Serial.println(i, DEC);

#ifdef STEPPER_ENABLE_PIN
			stepper1.enableOutputs();
#endif
			if (Direction)
			{
				Serial.print(F("Moving to Front Position : "));
				Serial.println(gAddresses[i].stationFront, DEC);
				stepper1.moveTo(gAddresses[i].stationFront);
				break;
			}
			else
			{
				Serial.print(F("Moving to Back Position : "));
				Serial.println(gAddresses[i].stationBack, DEC);
				stepper1.moveTo(gAddresses[i].stationBack);
				break;
			}
		}
	}
};

bool lastIsRunningState;

void setupStepperDriver()
{
#ifdef STEPPER_ENABLE_PIN
	stepper1.setPinsInverted(false, false, true); // Its important that these commands are in this order
	stepper1.setEnablePin(STEPPER_ENABLE_PIN);    // otherwise the Outputs are NOT enabled initially
#endif

	stepper1.setMaxSpeed(100.0);
	stepper1.setAcceleration(10);
	stepper1.setSpeed(100);

	lastIsRunningState = stepper1.isRunning();
}

void moveToHomePosition()
{
	pinMode(HOME_SENSOR_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(HOME_SENSOR_PIN), interruptEvent, RISING);

	bInterruptDetected = false;

	Serial.println(F("Performing 2 complete turns to find home."));
	stepper1.move(FULL_TURN_STEPS * 2);
}

void interruptEvent()
{
	detachInterrupt(digitalPinToInterrupt(HOME_SENSOR_PIN));
	bInterruptDetected = true;
}

void setupDCCDecoder()
{
	Serial.println(F("Setting up DCC Decorder..."));

	// Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
	Dcc.pin(0, 2, 1);

	// Call the main DCC Init function to enable the DCC Receiver
	Dcc.init(MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0);
}

void setup()
{
	Serial.begin(115200);
	while (!Serial);   // Wait for the USB Device to Enumerate

	Serial.println(F("Example Turntable Program - www.dccinterface.com"));

	ConfigureStations();

	setupStepperDriver();

	Serial.println(F("Finding home...."));
	moveToHomePosition();
}

void loop()
{
	if (bInterruptDetected)
	{
		bInterruptDetected = false;
		bHomePositionFound = true;

		Serial.println(F("Found Home - Setting Initial Position"));

		stepper1.setCurrentPosition(58); // set the position of the sensor

		Serial.print("Moving to position ");
		Serial.println(entryStation3, DEC); // starting position

#ifdef STEPPER_ENABLE_PIN
		stepper1.enableOutputs();
#endif
		stepper1.moveTo(entryStation3); // go to the starting position

		setupDCCDecoder();
	}

	if (bHomePositionFound)
	{
		// You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
		Dcc.process();
	}

	// Process the Stepper Library
	stepper1.run();

#ifdef STEPPER_ENABLE_PIN
	if (stepper1.isRunning() != lastIsRunningState)
	{
		lastIsRunningState = stepper1.isRunning();
		if (!lastIsRunningState)
		{
			stepper1.disableOutputs();
			Serial.println(F("Disable Stepper Outputs"));
		}
	}
#endif  
}
