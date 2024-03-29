//SIMPLIFIED CAR CONTROLLER   |JACK JOYNSON 09/07/2016|  |Email: support@jackjoynson.co.uk| 

//If current is below current limit then increase the PWMduty by the PWMStep value. If current is above the current limit then reduce the PWMduty by the PWMstep value.
    //If the PWMduty is above 100 then set to 100 as otherwise would be invalid. If PWMDuty is lower than the PWMLower limit then set it to the PWMLower limit.
    
//Arduino constantly reads the sensor values. If the go switch reads as off then the Arduino waits untill it counts the 'DebounceThreshold' value untill it turns the car off. 
    //This stops the car turning off from misread signals or bouncing within the switch.
    
//Arduino polls for serial recieved - the GPHUD android app requests the data with an 'RA' command every 100ms

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//*****************CHANGE LOG**********************
//Name          Date        Comments
//Jack Joynson  03/08/16    Calibrated to Discharge rig
//Tom Lawson    11/07/17    added air temp sensor

////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>

const int PWMLowerLimit = 100; //Minimum PWM Duty (max is 255)
const double PWMStep = 0.5; //Amount by which the PWM Duty is increased or decreased by

double currentLimit = 65.0; //Maximum current target

const double CurrentCalibration_m = (0.1536/0.8777)*0.909091;
const double CurrentCalibration_c = +0.1682;
const double VoltageCalibration_m = 1.004*(0.03117207/0.9611)/1.043076923;
const double VoltageCalibration_c = 0;
const double CurrentNoiseIgnoreLimit = 11.0;
const double PWMDutyCalibration = 0.392156;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Pin Assignments

const int ThrottlePin = A5;
const int CurrentPin = A1;
const int VoltagePin = A0;
const int MotorPWMPin = 5;
const int MotorInterrupt = 2;
const int otherInterrupt = 3;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor Readings

double current;
double voltage;
double EnergyUsed = 0.0;
double MotorRPM = 0.0;
double halfRevolutions = 0.0;

///////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration Settings

bool ContinuousTransmit = false;  //RECOMMENDED AS OFF AS GPHUD ANDROID APP (JJ) REQUESTS THE DATA FROM THE ARDUINO
int intervalDuration = 100;   //RECOMMENDED AT 100 (ms)
double MircoToHours = 1.0 / (1000.0 * 1000.0 * 60.0 * 60.0);  //1.0 / (1000.0 * 1000.0 * 60.0 * 60.0)
int debounceThreshold = 20;  //RECOMMENDED AT 20 - TOO HIGH WILL CAUSE THROTTLE DELAY, TOO LOW WILL MAKE USELESS AS BOUNCE MAY HAPPEN


///////////////////////////////////////////////////////////////////////////////////////////////////
// Runtime Variables

unsigned long tickTime = 0;
unsigned long EnergyUsagePrevTime;
double PWMDuty = 0;
bool go = false;
bool Running = false;
int debouceCount = 0;


unsigned long MotorEncoderPrevTime;
unsigned long MotorEncoderCurrentTime;
unsigned long MotorTimeElapsed;

/////////////////////////////////////////////////////////////////////////////////////
//stuff for AD7415 temp sensor, uses i2C - base address is 1001 010x
int8_t AD7415_ADDR_W = 0x4a;//0x94; //write address of chip
int8_t AD7415_ADDR_R = 0x4a;//0x95; //read address of chip
int8_t AD7415_ADDR_PTR = 0x00; //pointer to address register (or with a select to pick target reg)
int8_t AD7415_SELECT_TEMP = 0x00; //lsbs selects temp register
int8_t AD7415_SELECT_CONFIG = 0x01; //lsb select config register
int8_t AD7415_SELECT_THIGH = 0x02; //lsb selects Thigh
int8_t AD7415_SELECT_TLOW = 0x03; //lsb selects Tlow
int8_t AD7415_DATA_SIZE = 2; //number of bytes to read

double Temperature =  0;


void setup()
{
	Serial.begin(115200); //SETUP SERIAL AT 115200 BAUD RATE TO MATCH THE BLUETOOTH MODULES

	TCCR0B = TCCR0B & 0b11111000 | 0x03;  // Set the PWM frequency to 976Hz
  pinMode(0, INPUT_PULLUP);
	pinMode(ThrottlePin, INPUT);
	pinMode(VoltagePin, INPUT);
	pinMode(CurrentPin, INPUT);
	pinMode(MotorPWMPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(MotorInterrupt), MotorEncoder, RISING);
}


void loop()
{
	SerialReceive();
	ReadSensors();
  RPMCalculation();

	///////////////////////////////////////////////////////////////////////////////////////////////
	// If the GO button is pressed or the go command has been received on the serial port, then 
	// check to see if the controller is already running. If it is already running
	// then continue.
	// If the GO button is not pressed and the go command hasn't been received then TURN OFF, set the PWM output low and mark the controller as not running.

	if (goButtonCheck() || go)
	{
                //IF GO BUTTON ACTIVE OR SERIAL ACTIVATED THEN SET RUNNING
		if (!Running)
		{
			Running = true;
		}
	}
	else if (Running)
	{
                //NOT GOING FROM SERIAL OR FROM THROTTLE THEN TURN RUNNING OFF.
		digitalWrite(MotorPWMPin, LOW);
		PWMDuty = 0;
		Running = false;
	}

	CurrentControl();

	///////////////////////////////////////////////////////////////////////////////////////////////
	// If the tick interval has elapsed perform a serial transmit.

	if (TickElasped())
	{
		if (ContinuousTransmit) SerialTransmit();
	}
}

bool goButtonCheck()
{
	if (digitalRead(ThrottlePin) == HIGH) return true;
	else if (digitalRead(ThrottlePin) == LOW && debouceCount < debounceThreshold)
	{
		debouceCount++;
		if (Running) return true;
		else return false;
	}
	else
	{
		return false;
	}
}


void CurrentControl()
{
	if (Running)
	{
                if (current < currentLimit)
                {
                  PWMDuty+= PWMStep;
                }
                else{
                  PWMDuty-= PWMStep;
                }
                
                if (PWMDuty > 255){
                  PWMDuty = 255;
                } 
                if (PWMDuty < PWMLowerLimit){
                  PWMDuty = PWMLowerLimit;
                }
                
		analogWrite(MotorPWMPin, PWMDuty);
	}
}


void ReadSensors()
{
        //CALCULATE CURRENT USING NOISE IGNORE CHECK
	double tempCurrent = analogRead(CurrentPin);
	current = (tempCurrent <= CurrentNoiseIgnoreLimit) ? 0.0 : ((tempCurrent - CurrentNoiseIgnoreLimit) * CurrentCalibration_m) + CurrentCalibration_c;

	unsigned long currTime = micros();
        
        //AMPHOURS IS CURRENT * CHANGE IN TIME.
        
        double EnergyAddition = current*((currTime - EnergyUsagePrevTime)*MircoToHours);
        if (EnergyAddition > 0) //LOGIC ADDED TO CHECK THAT THE AMPHOURS ONLY ADDS AND DOESNT GET RESET. IF ARDUINO RESETS AND CURRTIME = 0 THEN THE LAST VALUE WOULD HAVE RESET THE TOTAL.
        {
         	EnergyUsed += EnergyAddition;   
        }
	EnergyUsagePrevTime = currTime;

	voltage = (analogRead(VoltagePin) * VoltageCalibration_m) + VoltageCalibration_c;
   Temperature = AD7415();
}

double AD7415(void)
{
double temperature = 0;
int32_t temp = 0; //temporary register 
double fraction = 0; //fractional part
  Wire.beginTransmission(AD7415_ADDR_W); //write to device address
  Wire.write(byte(AD7415_SELECT_TEMP | AD7415_SELECT_CONFIG));    //write to config register
  Wire.write(byte(0x04));     //write to "one shot" register to start a measurement
  Wire.endTransmission();
  delay(5);           //wait 1ms for reading to happen (is about 1us)
  //demand reading
  Wire.beginTransmission(AD7415_ADDR_R); //write to device address
  Wire.write(byte(AD7415_ADDR_PTR | AD7415_SELECT_TEMP));    //write to temp register
  Wire.endTransmission();

  Wire.requestFrom(AD7415_ADDR_R, AD7415_DATA_SIZE); //ger reading
  
  if (2 <= Wire.available())
  { // if two bytes were received
  temp = Wire.read();  // receive high byte (overwrites previous reading)
  temp = temp << 8;    // shift high byte to be high 8 bits
  temp |= Wire.read(); // receive low byte as lower 8 bits
  }
  temp = temp >> 6; //shift it right 6 to remove the duff numbers (NA) from the lsb register)
  temperature = temp/4;
  return temperature; //value is only accurate to +/-2degreesC
}

void SetTick()
{
	tickTime = millis() + intervalDuration;
}


bool TickElasped() //SO INSTEAD OF USING DELAYS WHICH DISABLE ASYNC CAPABILITIES WE JUST CHECK THAT THE TIME IS WITHIN THE INTERAVAL
{
	if (tickTime < millis())
	{
		SetTick();
		return true;
	}
	else
	{
		return false;
	}
}


void SerialReceive()
{
	if (Serial.available() > 0)
	{
		switch (Serial.read())
		{
		case 'r':
		case 'R':
			DataRequest();
			break;
		case 'c':
		case 'C':
			currentLimit = Serial.parseInt();
			break;
		case 'g':
		case 'G':
			go = true;
			break;
		case 'o':
		case 'O':
			go = false;
			break;
		default:
			Serial.flush();
		}
	}
}


void DataRequest()
{
	if (Serial.available() > 0)
	{
		switch (Serial.read())
		{
		case 'a':
		case 'A':
			SerialTransmit();
			break;
		default:
			Serial.flush();
			break;
		}
	}
}


void SerialTransmit()
{
	//Serial.println("1");
	Serial.print(current);
	Serial.print(",");
	Serial.print(voltage);
	Serial.print(",");
	Serial.print(EnergyUsed);
	Serial.print(",");
	Serial.print(MotorRPM);  //MOTORRPM
	Serial.print(",");
	Serial.print(1.00);  //MOTORTEMP
	Serial.print(",");
	Serial.print(currentLimit);
	Serial.print(",");
	Serial.print(PWMDuty*PWMDutyCalibration);
	Serial.print(",");
	Serial.print(Temperature);  //BATTTEMP1
	Serial.print(",");
	Serial.print(1.00);  //BATTTEMP2
	Serial.print(",");
	Serial.println(1.00);  //WHEELRPM
}

void MotorEncoder()
{
  halfRevolutions++;
}

void RPMCalculation()
{
if (halfRevolutions >= 1) { 
     MotorRPM = 60*1000000/(micros() - MotorTimeElapsed)*halfRevolutions;
     MotorTimeElapsed = micros();
     halfRevolutions = 0;
   }
}


