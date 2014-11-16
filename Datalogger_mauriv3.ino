/*
  SD card datalogger

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 modified Oct 19, 2014 
 by Mauri Niininen
 This example code is in the public domain.

 */

#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <Wire.h>


#include <avr/sleep.h> //Needed for sleep_mode
#include <avr/power.h> //Needed for powering down perihperals such as the ADC/TWI and Timers
#include <avr/wdt.h> //Needed for the reset command - uses watch dog timer to reset IC

#define SLEEPTIME 5000

long timeSleep = 0;  // total time due to sleep
float calibv = 0.93; // ratio of real clock with WDT clock
volatile byte isrcalled = 0;  // WDT vector flag

// Internal function: Start watchdog timer
// byte psVal - Prescale mask
void WDT_On (byte psVal)
{
  // prepare timed sequence first
  byte ps = (psVal | (1<<WDIE)) & ~(1<<WDE);
  cli();
  wdt_reset();
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = ps;
  sei();
}

// Internal function.  Stop watchdog timer
void WDT_Off() {
  cli();
  wdt_reset();
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1<<WDRF);
  /* Write logical one to WDCE and WDE */
  /* Keep old prescaler setting to prevent unintentional time-out */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* Turn off WDT */
  WDTCSR = 0x00;
  sei();
}

// Calibrate watchdog timer with millis() timer(timer0)
void calibrate() {
   
  // timer0 continues to run in idle sleep mode
  set_sleep_mode(SLEEP_MODE_IDLE); 
  long tt1=millis();
  doSleep(256);
  long tt2=millis();
  calibv = 256.0/(tt2-tt1);
}

// Estimated millis is real clock + calibrated sleep time
long estMillis() {
  return millis()+timeSleep;
}

// Delay function
void sleepCPU_delay(long sleepTime) {
  ADCSRA &= ~(1<<ADEN);  // adc off
  PRR = 0xEF; // modules off

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  int trem = doSleep(sleepTime*calibv);
  timeSleep += (sleepTime-trem);

  PRR = 0x00; //modules on
    /* Re-enable the peripherals. */
  ADCSRA |= (1<<ADEN);  // adc on
}

// internal function.  
int doSleep(long timeRem) {
  byte WDTps = 9;  // WDT Prescaler value, 9 = 8192ms

  isrcalled = 0;
  sleep_enable();
  while(timeRem > 0) {
    //work out next prescale unit to use
    while ((0x10<<WDTps) > timeRem && WDTps > 0) {
      WDTps--;
    }
    // send prescaler mask to WDT_On
    WDT_On((WDTps & 0x08 ? (1<<WDP3) : 0x00) | (WDTps & 0x07));
    isrcalled=0;
    while (isrcalled==0) {
      // turn bod (brown out detector) off
      MCUCR |= (1<<BODS) | (1<<BODSE);
      MCUCR &= ~(1<<BODSE);  // must be done right before sleep
      sleep_cpu();  // sleep here
    }
    // calculate remaining time
    timeRem -= (0x10<<WDTps);
  }
  sleep_disable();
  return timeRem;
}




// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = 10;
//uint32_t lastSyncTime;
long day = 86400000; // 86400000 milliseconds in a day
long hour = 3600000; // 3600000 milliseconds in an hour
long minute = 60000; // 60000 milliseconds in a minute
long second =  1000; // 1000 milliseconds in a second

long lastSyncTime;
int seconds;
int minutes;
int hours;

volatile int f_wdt=1;

//STAT1 is a general LED and indicates serial traffic
#define STAT1  5 //On PORTD
#define STAT1_PORT  PORTD
#define STAT2  5 //On PORTB
#define STAT2_PORT  PORTB
const byte blueled = 5;  //This is the normal status LED
const byte greenled = 13; //This is the SPI LED, indicating SD traffic
#define TMP102_I2C_ADDRESS 72 // This is the I2C address for our chip 0x48 

#define LED_PIN (13)


//  Description: Watchdog Interrupt Service. This
//               is executed when watchdog timed out.
ISR(WDT_vect)
{
  WDT_Off();
  isrcalled=1;

  if(f_wdt == 0) {
    f_wdt=1;
  }
  //else
  //  Serial.println("WDT Overrun!!!");
}




/***************************************************
 *  Name:        enterSleep
 *
 *  Returns:     Nothing.
 *
 *  Parameters:  None.
 *
 *  Description: Enters the arduino into sleep mode.
 *
 ***************************************************/
void enterSleep(void)
{
   /* Don't forget to clear the flag. */
   f_wdt = 0;

  // SLEEP_MODE_IDLE         -the least power savings
  // SLEEP_MODE_ADC
  // SLEEP_MODE_PWR_SAVE
  // SLEEP_MODE_STANDBY
  // SLEEP_MODE_PWR_DOWN     -the most power savings

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
  
  /* Re-enable the peripherals. */
  power_all_enable();
}


void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  
  pinMode(blueled, OUTPUT);
  pinMode(greenled, OUTPUT);
  
  //lastSyncTime = millis(); //Keeps track of the last time the file was synced
 
  
  /*** Setup the WDT ***/
   /* set new watchdog timeout prescaler value */
  // 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  // 1<<WDP1 | 1<<WDP2; /* 1.0 seconds */
  // 1<<WDP0 | 1<<WDP2; /* 0.5 seconds */
  //WDT_On (0);
  
  /* Clear the reset flag. */
  //MCUSR &= ~(1<<WDRF);
  
  /* In order to change WDE or the prescaler, we need to
   * set WDCE (This will allow updates for 4 clock cycles).
   */
  //WDTCSR |= (1<<WDCE) | (1<<WDE);

  /* set new watchdog timeout prescaler value */
  //WDTCSR = 1<<WDP0 | 1<<WDP3; /* 8.0 seconds */
  //WDTCSR = 1<<WDP1 | 1<<WDP2; /* 1.0 seconds */
  //  WDTCSR = 1<<WDP0 | 1<<WDP2; /* 0.5 seconds */
    
  /* Enable the WD interrupt (note no reset). */
  //WDTCSR |= _BV(WDIE);
  
  Serial.println("Initialisation complete.");
  delay(100); //Allow for serial print to complete.

//  read_system_settings(); //Load all system settings from EEPROM
  //lastSyncTime = millis(); //Reset the last sync time to now  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }


  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("starting measurements.");
  delay(100); //Allow for serial print to complete.
  
  lastSyncTime = 0;
  seconds = 1;
  minutes = 58;
  hours = 0;
  
} // end of setup()



float getTemp102() {
  byte firstbyte, secondbyte; //these are the bytes we read from the TMP102 temperature registers
  int val; /* an int is capable of storing two bytes, this is where we "chuck" the two bytes together. */
  float convertedtemp; /* We then need to multiply our two bytes by a scaling factor, mentioned in the datasheet. */
 

  /* Reset the register pointer (by default it is ready to read temperatures)
You can alter it to a writeable register and alter some of the configuration -
the sensor is capable of alerting you if the temperature is above or below a specified threshold. */
  digitalWrite(blueled, HIGH);
  Wire.beginTransmission(TMP102_I2C_ADDRESS); //Say hi to the sensor.
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(TMP102_I2C_ADDRESS, 2);
  Wire.endTransmission();
 
 
  firstbyte      = (Wire.read());
/*read the TMP102 datasheet - here we read one byte from
 each of the temperature registers on the TMP102*/
  secondbyte     = (Wire.read());
/*The first byte contains the most significant bits, and
 the second the less significant */
    val = firstbyte;
    if ((firstbyte & 0x80) > 0) {
      val |= 0x0F00;
    } 
    val <<= 4;
    
 /* MSB */
    val |= (secondbyte >> 4);    
/* LSB is ORed into the second 4 bits of our byte.
Bitwise maths is a bit funky, but there's a good tutorial on the playground*/
    convertedtemp = val*0.0625;

   digitalWrite(blueled, LOW);
   return(convertedtemp);

}

String time(long lastSyncTime) {

String timeStr;

 
int days = lastSyncTime / day ;                                //number of days
int hours = (lastSyncTime % day) / hour;                       //the remainder from days division (in milliseconds) divided by hours, this gives the full hours
int minutes = ((lastSyncTime % day) % hour) / minute ;         //and so on...
int seconds = (((lastSyncTime % day) % hour) % minute) / second;

 // digital clock display of current time
 timeStr = String(days);
 timeStr += printDigits(hours);  
 timeStr += printDigits(minutes);
 timeStr += printDigits(seconds);
 return timeStr;  
 
}

String printDigits(byte digits){
 // utility function for digital clock display: prints colon and leading 0
 String value; 
 value = ":";
 if(digits < 10)
   value +='0';
 value += String(digits);
 return value;
}

void loop()
{
 
  if(f_wdt == 1)
  {
    
     
    // make a string for assembling the data to log:
    String dataString = "";
    digitalWrite(blueled, HIGH); //Turn OFF  blue LED to save power
    digitalWrite(greenled, HIGH); //Turn OFF stat LED to save power
    
    dataString = String(getTemp102());
    dataString += ",";
    dataString += String(lastSyncTime);
    dataString += ",";
    dataString += time(lastSyncTime);
    
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
  
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
      delay(50);

    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }

    lastSyncTime = estMillis();

    // Calibrate clock every 5 minutes 
    if (((minutes+1) % 5) == 0)
       calibrate();

   /* Don't forget to clear the flag. */
   f_wdt = 0;
   
    /* Re-enter sleep mode. */
    //enterSleep();
    sleepCPU_delay(SLEEPTIME);
  }
  else
  {

  }
  

}









