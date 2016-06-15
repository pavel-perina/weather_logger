///////////////////////////////////////////////////////////////////////
/// \file	weather_logger.ino
/// \brief  Temperature, humidity and pressure Arduino-based monitor.
/// \author Pavel Perina
/// \date	June 2016
///////////////////////////////////////////////////////////////////////

#include <avr/sleep.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h> // link

const uint8_t SD_CHIP_SELECT  = 10;
const uint8_t RTC_INT_PIN     = 3;  // must be 2 or 3
// TODO: set other pin, this one powers up SD card (and lightbulb :-) )
const uint8_t LED_AWAKE_PIN   = 4;  // pin13 is SPI CLK
const uint8_t SD_AWAKE_PIN    = 4;  
const uint8_t VBAT_PIN        = A3; // analog pin for vBat 

///////////////////////////////////////////////////////////////////////
// Utility functions

static uint8_t bcd2bin (uint8_t val) 
{ 
  return val - 6 * (val >> 4); 
}


static uint8_t bin2bcd (uint8_t val)
{ 
  return val + 6 * (val / 10); 
}


///////////////////////////////////////////////////////////////////////
// HTU21D temperature / humidity
// from sparkfun library

#define HTDU21D_ADDRESS 0x40  //Unshifted 7-bit I2C address for the sensor

#define TRIGGER_TEMP_MEASURE_HOLD  0xE3
#define TRIGGER_HUMD_MEASURE_HOLD  0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5
#define WRITE_USER_REG  0xE6
#define READ_USER_REG  0xE7
#define SOFT_RESET  0xFE

void readHTU(float *temp, float *hum)
{
  *temp = -100.0; 
  *hum  = -100.0;
  
  byte msb, lsb, checksum;
  int counter;
  //Request a humidity reading
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(TRIGGER_HUMD_MEASURE_NOHOLD); //Measure humidity with no bus holding
  Wire.endTransmission();

  //Hang out while measurement is taken. 50mS max, page 4 of datasheet.
  delay(55);

  //Comes back in three bytes, data(MSB) / data(LSB) / Checksum
  Wire.requestFrom(HTDU21D_ADDRESS, 3);

  //Wait for data to become available
  counter = 0;
  while(Wire.available() < 3)
  {
    counter++;
    delay(1);
    if(counter > 100) 
      return; //Error out
  }

  msb = Wire.read();
  lsb = Wire.read();
  checksum = Wire.read();

  unsigned int rawHumidity = ((unsigned int) msb << 8) | (unsigned int) lsb;  
  //sensorStatus = rawHumidity & 0x0003; //Grab only the right two bits
  rawHumidity &= 0xFFFC; //Zero out the status bits but keep them in place
  
  //Given the raw humidity data, calculate the actual relative humidity
  float tempRH = rawHumidity / (float)65536; //2^16 = 65536
  *hum = -6 + (125 * tempRH); //From page 14   


  //Request the temperature
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(TRIGGER_TEMP_MEASURE_NOHOLD);
  Wire.endTransmission(); 

  //Hang out while measurement is taken. 50mS max, page 4 of datasheet.
  delay(55);

  //Comes back in three bytes, data(MSB) / data(LSB) / Checksum
  Wire.requestFrom(HTDU21D_ADDRESS, 3);

  //Wait for data to become available
  counter = 0;
  while(Wire.available() < 3)
  {
    counter++;
    delay(1);
    if(counter > 100)
       return; //Error out
  }

  msb = Wire.read();
  lsb = Wire.read();
  checksum = Wire.read();

  unsigned int rawTemperature = ((unsigned int) msb << 8) | (unsigned int) lsb;

  rawTemperature &= 0xFFFC; //Zero out the status bits but keep them in place

  //Given the raw temperature data, calculate the actual temperature
  float tempTemperature = rawTemperature / (float)65536; //2^16 = 65536
  *temp = (float)(-46.85 + (175.72 * tempTemperature)); //From page 14
}


///////////////////////////////////////////////////////////////////////
// BMP180 temperature / pressure sensor
// from sparkfun library
#define BMP180_ADDR 0x77 // 7-bit address

#define BMP180_REG_CONTROL 0xF4
#define BMP180_REG_RESULT 0xF6

#define BMP180_COMMAND_TEMPERATURE 0x2E
#define BMP180_COMMAND_PRESSURE0 0x34
#define BMP180_COMMAND_PRESSURE1 0x74
#define BMP180_COMMAND_PRESSURE2 0xB4
#define BMP180_COMMAND_PRESSURE3 0xF4

struct tBmp180CalibrationData {
    int16_t AC1,AC2,AC3,VB1,VB2,MB,MC,MD;
    uint16_t AC4,AC5,AC6; 
    double c5,c6,mc,md,x0,x1,x2,y0,y1,y2,p0,p1,p2;
} bmpCal;


///////////////////////////////////////////////////////////////////////
// RTC code for DS323X chips (own)
// Note: #include <Wire.h> before this

#define DS3231_I2C_ADDRESS 0x68

/// \brief  Get datetime of DS3231 RTC, assumes UTC (same date format as GPX file)
/// \return Pointer to static buffer with Time
char * rtcDateTimeStr()
{
  //                        0123456789012345678
  static char datetime[] = "20YY-MM-DDThh:mm:ssZ\0";
  byte bcd;
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);                              // set DS3231 register pointer to 00h
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);    // request seven bytes of data from DS3231 starting from register 00h
  // second, minute, hour
  bcd = Wire.read() & 0x7f;
  datetime[17] = '0' + (bcd>>4);
  datetime[18] = '0' + (bcd&0x0f);
  bcd = Wire.read();
  datetime[14] = '0' + (bcd>>4);
  datetime[15] = '0' + (bcd&0x0f);
  bcd = Wire.read() & 0x3f;
  datetime[11] = '0' + (bcd>>4);
  datetime[12] = '0' + (bcd&0x0f);
  // day of week (ignored), day, month, year
  bcd = Wire.read();
  bcd = Wire.read();
  datetime[ 8] = '0' + (bcd>>4);
  datetime[ 9] = '0' + (bcd&0x0f);
  bcd = Wire.read();
  datetime[ 5] = '0' + (bcd>>4);
  datetime[ 6] = '0' + (bcd&0x0f);
  bcd = Wire.read();
  datetime[ 2] = '0' + (bcd>>4);
  datetime[ 3] = '0' + (bcd&0x0f);
  return datetime;
}


/// \brief  Set DS32XX alarm in X minutes from now
void rtcAlarmInMinutes(uint8_t inMinutes)
{
  Wire.begin(DS3231_I2C_ADDRESS);
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write((byte) 1);   // set address to 01 (minutes)
  Wire.endTransmission();

  Wire.requestFrom(DS3231_I2C_ADDRESS, 1);
  uint8_t mins = bcd2bin(Wire.read());  // 01 minutes
  mins += inMinutes;
  if (mins > 59) 
    mins -= 60; 
  
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  // A1M4=1 A1M3=1 A1M2=0 A1M2=0 -> alarm when minutes and seconds match
  Wire.write((byte) 7);
  Wire.write(0b00000000);     // 07 A1M1, seconds
  Wire.write(bin2bcd(mins));  // 08 A1M2, minutes
  Wire.write(0b10000000);     // 09 A1M3, hours
  Wire.write(0b10000000);     // 0A A1M4, day
  Wire.write(0b00000000);     // 0B A2M2, minutes
  Wire.write(0b00000000);     // 0C A2M3, hours
  Wire.write(0b00000000);     // 0D A2M4, day
  Wire.write(0b00000101);     // 0E INTCN-int pin mode, A1IE (alarm1 int enable)
  Wire.write(0b00000000);     // 0F Reset current alarm(s)
  Wire.endTransmission();
}

/// \brief Disable RTC alarms and clear interrupt (RTC_INT_PIN pin to HIGH)
static void rtcDisableAlarm()
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write((byte) 0x0E);
  Wire.write(0b00000100);     // 0E INTCN pin in alarm mode, no alarms set
  Wire.write(0b00000000);     // 0F Reset current alarm(s)
  Wire.endTransmission();
}

///////////////////////////////////////////////////////////////////////
// Sleep and interrupt

volatile bool intTriggered = false;


void rtcIntHandler()
{
  intTriggered = true;
}

/// \brief 
/// Setup interrupt on falling edge (LOW means RTC interrupt occured)
///  
/// \note
/// According to some sources LOW is only valid option for SLEEP_MODE_PWR_DOWN
/// Some sources e.g. datasheet are wrong.
///
/// We can setup interrupt on FALLING edge and do not bother with disabling
/// and detaching interrupt.
///
/// It was discovered accidentally and confirmed by <CITATION_NEEDED>
/// \todo Cite source(s)
void rtcIntSetup()
{
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  attachInterrupt (RTC_INT_PIN - 2, rtcIntHandler, FALLING);
}


void sleepNow()
{
  // Wait for serial writes to complete
  Serial.flush();
  delay(100);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
}



///////////////////////////////////////////////////////////////////////
// SD Card code


//////////////////////////////////////////////////////////////////////////////////////////////////
// VCC MONITOR

// https://code.google.com/archive/p/tinkerit/wikis/SecretVoltmeter.wiki
long vccVoltage()
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC))
    ;
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}


/// \todo Implement battery voltage
long batVoltage()
{
  return 0;
}

class LedPin
{
private:

  int8_t pin;

public:

  LedPin()
    : pin(-1)
  {
  }
  
  void init(int8_t _pin) 
  {
    pin = _pin;
    if (pin > 0)
      pinMode(pin, OUTPUT);  
  }

  void on() 
  {
    if (pin>0)
      digitalWrite(pin, HIGH);      
  }

  void off()
  {
    if (pin>0)
      digitalWrite(pin, LOW);
  }

};

///////////////////////////////////////////////////////////////////////
// Main program


LedPin ledAwake;

void setup()
{
  ledAwake.init(LED_AWAKE_PIN);
  ledAwake.on();
     
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("\n\nBoot up ...");       

  Wire.begin(); 
  rtcDisableAlarm();
  Serial.print("RTC time is: ");
  Serial.println(rtcDateTimeStr());
    
  //ReadSensors();
  //Serial.println(logLine);
  
    
  Serial.println("Setting up alarm clock and going to sleep. See you in minute or two ...");
  
  rtcIntSetup();
  delay(500);
  rtcAlarmInMinutes(2); // at least two minutes, alarm can happen within second here
  /// \todo fix alarm code, add safeGuardSeconds parameter
  ledAwake.off();
  sleepNow();  
  
}

void loop()
{
  sleep_disable();
  ledAwake.on();

  // Read sensors
  float htu_temp, htu_hum;
  readHTU(&htu_temp, &htu_hum);

  Serial.print("\"");
  Serial.print(rtcDateTimeStr());
  Serial.print("\",");
  Serial.print(vccVoltage());
  Serial.print(",");
  Serial.print(batVoltage());
  Serial.print(",");
  Serial.print(htu_temp);
  Serial.print(",");
  Serial.print(htu_hum);
  Serial.println("");
  
  rtcAlarmInMinutes(1);
  ledAwake.off();
  sleepNow();

}
