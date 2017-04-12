///
/// @mainpage	Logger-Arduino-AM2315
///
/// @details	Weather conditions logger
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		4/2/17 8:10 PM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2017
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Logger_Arduino_AM2315.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](http://embedXcode.weebly.com)
///
/// @author		Charles McClelland
/// @author		Charles McClelland
/// @date		4/2/17 8:10 PM
/// @version	<#version#>
///
/// @copyright	(c) Charles McClelland, 2017
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///

/*
                            ***********  Temperature and Humidity Logger  *************
    In this sketch, we will use an hourly alarm to wake the Arduino and get it to take a Temerature and Humidity measurement
    We will store the hourly measurements for 169 days.  We will also keep 28 days of highs and lows.  The Simblee portion of this 
    application focused on making the data available on  mobile application.
*/


// Core library for code-sense - IDE-based
#if defined(WIRING) // Wiring specific
#include "Wiring.h"
#elif defined(MAPLE_IDE) // Maple specific
#include "WProgram.h"
#elif defined(ROBOTIS) // Robotis specific
#include "libpandora_types.h"
#include "pandora.h"
#elif defined(MPIDE) // chipKIT specific
#include "WProgram.h"
#elif defined(DIGISPARK) // Digispark specific
#include "Arduino.h"
#elif defined(ENERGIA) // LaunchPad specific
#include "Energia.h"
#elif defined(LITTLEROBOTFRIENDS) // LittleRobotFriends specific
#include "LRF.h"
#elif defined(MICRODUINO) // Microduino specific
#include "Arduino.h"
#elif defined(TEENSYDUINO) // Teensy specific
#include "Arduino.h"
#elif defined(REDBEARLAB) // RedBearLab specific
#include "Arduino.h"
#elif defined(RFDUINO) // RFduino specific
#include "Arduino.h"
#elif defined(SPARK) || defined(PARTICLE) // Particle / Spark specific
#include "application.h"
#elif defined(ESP8266) // ESP8266 specific
#include "Arduino.h"
#elif defined(ARDUINO) // Arduino 1.0 and 1.5 specific
#include "Arduino.h"
#else // error
#error Platform not defined
#endif // end IDE


// Set parameters
// There are some new pin assignments when using the new v10 board - default for the PIR sensor since we can turn it off
#define V10BOARD 1
#if V10BOARD                        // These are the pin assignments for the v9 board
#define ALARMPIN 3                  // This one will be used for the RTC Alarm in v9
#define INT2PIN 2                   // This is the interrupt pin that registers taps
#define INTNUMBER 0                 // so I don't have to use the lookup function
#define PIRPIN 5                    // This is a pin which connects to the i2c header - future use
#define I2CPWR 8                    // Turns the i2c port on and off
#define RESETPIN 16                 // This a modification using a bodge wire
#define TALKPIN 14                  // This is the open-drain line for signaling i2c mastery (A0 on the Uno is 14)
#define THE32KPIN 15                // This is a 32k squarewave from the DS3231 (A1 on the Uno is 15)
#else                               // These are the pin assignments for the v8b board
#define SENSORPIN 2                 // Not used now but wired for future use
#define PIRPIN 3                    // This is the interrupt pin for the PIR Sensor, Active High, Push-Pull
#define ALARMPIN 5                  // This is the pin with the RTC Alarm clock - not used on Arduino side
#define I2CPWR 8                    // Turns the i2c port on and off
#define RESETPIN 16                 // This a modification using a bodge wire
#define TALKPIN 14                  // This is the open-drain line for signaling i2c mastery (A0 on the Uno is 14)
#define THE32KPIN 15                // This is a 32k squarewave from the DS3231 (A1 on the Uno is 15)
#endif

//Time Period Definitions - used for debugging
#define HOURLYPERIOD hour(t)        // Normally hour(t) but can use minute(t) for debugging
#define DAILYPERIOD day(t)          // Normally day(t) but can use minute(t) or hour(t) for debugging

//These defines let me change the memory map and configuration without hunting through the whole program
#define VERSIONNUMBER 9             // Increment this number each time the memory map is changed
#define WORDSIZE 8                  // For the Word size
#define PAGESIZE 4096               // Memory size in bytes / word size - 256kb FRAM
// First Word - 8 bytes for setting global values
#define DAILYOFFSET 3               // First word of daily counts
#define HOURLYOFFSET 31             // First word of hourly counts (remember we start counts at 1)
#define DAILYCOUNTNUMBER 28         // used in modulo calculations - sets the # of days stored
#define HOURLYCOUNTNUMBER 4065      // used in modulo calculations - sets the # of hours stored - 256k (4096-28-3)
#define VERSIONADDR 0x0             // Memory Locations By Name not Number
#define PARKOPENSADDR 0x1           // When does the park open
#define PARKCLOSESADDR 0x2          // when does the park close
#define MONTHLYREBOOTCOUNT 0x3      // This is where we store the reboots - indication of system health
#define DAILYPOINTERADDR 0x4        // One byte for daily pointer
#define HOURLYPOINTERADDR 0x5       // Two bytes for hourly pointer
#define CONTROLREGISTER 0x7         // This is the control register acted on by both Simblee and Arduino
//Second and Third Words - 16 bytes for storing current counts
#define CURRENTTEMPADDR 0x8         // Last Temp Reading
#define CURRENTHUMIDADDR 0x9        // Last Humid Reading
#define CURRENTLOWTEMPADDR 0xA      // Current Daily Low Temp
#define CURRENTHIGHTEMPADDR 0xB     // Current Daily High Temp
#define CURRENTLOWHUMIDADDR 0xC     // Current Daily Low Humdity
#define CURRENTHIGHHUMIDADDR 0xD    // Current Daily High Humdity
#define CURRENTCOUNTSTIME 0xE       // Time of last count
//These are the hourly and daily offsets that make up the respective words
#define DAILYDATEOFFSET 1           //Offsets for the value in the daily words
#define DAILYLOWTEMPOFFSET 2        // Daily Low Temp
#define DAILYHIGHTEMPOFFSET 3       // Daily High Temp
#define DAILYLOWHUMIDOFFSET 4       // Daily Low Humid
#define DAILYHIGHHUMIDOFFSET 5      // Daily High Humid
#define DAILYBATTOFFSET 6           // Where the battery charge is stored
#define HOURLYTEMPOFFSET 4          // Hourly Temp Reading
#define HOURLYHUMIDOFFSET 5         // Hourly Humid Setting
#define HOURLYBATTOFFSET 6          // Hourly Batt Setting
// LED Pin Value Variables
#define REDLED 6                    // led connected to digital pin 4
#define YELLOWLED 4                 // The yellow LED
#define LEDPWR 7                    // This pin turns on and off the LEDs
// Finally, here are the variables I want to change often and pull them all together here
#define SOFTWARERELEASENUMBER "0.5.0"

// Include application, user and local libraries
#include <avr/sleep.h>              // For Sleep Code
#include <avr/power.h>              // Power management
#include <avr/wdt.h>                // Watchdog Timer
#include <EEPROM.h>                 // Library for accessing the Arduino's EEPROM
#include "MAX17043.h"               // Drives the LiPo Fuel Gauge
#include <Wire.h>                   //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <TimeLib.h>                //http://www.arduino.cc/playground/Code/Time
#include "DS3232RTC.h"              //http://github.com/JChristensen/DS3232RTC
#include "Adafruit_FRAM_I2C.h"      // Library for FRAM functions
#include "Adafruit_AM2315.h"        // Adafruit library for the AM2315 outdoor temp and humidity sensor
#include "FRAMcommon.h"             // Where I put all the common FRAM read and write extensions


// Prototypes
MAX17043 batteryMonitor;            // Initialize the Fuel Gauge
Adafruit_AM2315 am2315;             // Initialize the Tem / Humidity Sensor

// Prototypes for General Functions
void StartStopTest(boolean startTest);  // Since the test can be started from the serial menu or the Simblee - created a function
boolean initializeAM2315();             // Turn on the power and fire up the sensor for a reading
void BlinkForever();                    // Ends execution for fatal flaws at startup
void enable32Khz(uint8_t enable);       // Need to turn on the 32k square wave for bus moderation
boolean CheckTempHumid();               // This is where we test, validate and store the temp and humidity data
void LogHourlyEvent();                  // Log Hourly Event()
void LogDailyEvent();                   // Log Daily Event()
void pinChangeISR();                    // Sensor Interrupt Handler
void sleepNow();                        // Puts the Arduino to Sleep
void NonBlockingDelay(int millisDelay); // Used for a non-blocking delay
int freeRam ();                         // Debugging code, to check usage of RAM
int8_t RoundFloatToSignedByte (float floater);   // Will round floater and not allow for overflow or lost data
int8_t Celsius2Fahrenheit(int8_t celsius);       // Note we are only using a byte so numbers must stay in range -127 to 127


// Prototypes for Date and Time Functions
void SetTimeDate();                     // Sets the RTC date and time
void PrintTimeDate(time_t t);           // Prints to the console
void toArduinoTime(time_t unixT);       //Converts to Arduino Time for use with the RTC and program


// FRAM and Unix time variables
time_t t;                               // Will use t throughout program to store time
byte currentHourlyPeriod;               // This is where we will know if the period changed
byte currentDailyPeriod;                // We will keep daily counts as well as period counts
int countTemp = 0;                      // Will use this to see if we should display a day or hours counts

// Variables for the control byte
// Control Register  (8 - 7 Reserved, 6 - Simblee Reset, 5-Clear Counts, 4-Simblee Sleep, 3-Start / Stop Test, 2-1 Reserved)
byte controlRegisterValue;                  // Holds the current control register value
byte oldControlRegisterValue = B00000000;   // Makes sure we can detect a change in the register value
unsigned long lastCheckedControlRegister;   // When did we last check the control register
int controlRegisterDelay = 1000;            // How often will we check the control register
byte toggleStartStop = B00000100;           // Flag to start and stop recording data
byte toggleSimbleeSleep = B00001000;        // Tells the Arduino to reset the Simblee after a disconnect
byte signalClearCounts = B00010000;         // Clears current counts
byte clearClearCounts = B11101111;          // Signals that clear counts is complete
byte signalSimbleeReset = B00100000;        // Simblee will set this flag on disconnect
byte clearSimbleeReset = B11011111;         // The only thing the Arduino will do is clear the Simblee Reset Bit

// Temperature and Humidity Sensor Variables
unsigned long warmUpTime = 1000;            // Give the sensor some time to warm up
unsigned long lastAwakening = 0;            // Make sure we get a reading, then delay sleep
unsigned int stayAwakeTime = 10000;         // How often will stay awake (ititial value here - subequent in sleepNow();)
const unsigned int stayAwakeAfterFirstSleep = 3000;   // Shorter time awake as less to do after it is set up and running
unsigned int lastReading = 0;               // Rate limit the readings while awake
const unsigned int timeBetweenReadings = 2000;  // How often to try to get a new reading
int8_t currentTemp = 0;                     // Last temperature reading
uint8_t currentHumid = 0;                   // Last humidity reading
int8_t dailyLowTemp = 0;                    // Daily low temp so far
int8_t dailyHighTemp = 0;                   // Daily high temp so far
uint8_t dailyLowHumid  = 0;                 // Daily low humidity so far
uint8_t dailyHighHumid = 0;                 // Daily high humidity so far

// Battery monitor
float stateOfCharge = 0;                    // stores battery charge level value

//Menu and Program Variables
bool ledState = false;                      // variable used to store the last LED status, to toggle the light
bool refreshMenu = true;                    //  Tells whether to write the menu
bool inTest = false;                        // Are we in a test or not
bool LEDSon = true;                         // Are the LEDs on or off
int menuChoice=0;                           // Menu Selection
int numberHourlyDataPoints;                 // How many hourly counts are there
int numberDailyDataPoints;                  // How many daily counts are there
const char* releaseNumber = SOFTWARERELEASENUMBER;  // Displays the release on the menu
byte bootcount = 0;                         // Counts reboots
int bootCountAddr = 0;                      // Address for Boot Count Number


// Add setup code
void setup()
{
    wdt_reset();                            // Since we are using watchdog timer for i2c, we need to
    MCUSR=0;                                // Ensure that the watchdog is reset at bootup
    WDTCSR|=_BV(WDCE) | _BV(WDE);           // This string works better than using wdt_disable();
    WDTCSR=0;                               // End of wdt commands
    Serial.begin(9600);                     // Initialize communications with the terminal
    delay(200);                             // wait for Serial to initialize
    Serial.println("");                     // Header information
    Serial.print(F("Connected Sensor Temp / Humidity - release ")); // I do this so I can see what release is fielded
    Serial.println(releaseNumber);          // when I am doing field checks
    Wire.begin();                           // Use standard Wire
    pinMode(REDLED, OUTPUT);                // declare the Red LED Pin as an output
    pinMode(YELLOWLED, OUTPUT);             // declare the Yellow LED Pin as as OUTPUT
    pinMode(LEDPWR, OUTPUT);                // declare the Power LED pin as as OUTPUT
    digitalWrite(LEDPWR, LOW);              // Turn on the power to the LEDs at startup for as long as is set in LEDsonTime
    pinMode(I2CPWR, OUTPUT);                // This is for V10 boards which can turn off power to the external i2c header
    pinMode(RESETPIN,INPUT);                // Just to make sure - if set to output, you can program the SIMBLEE
    pinMode(PIRPIN, INPUT);                 // Set up the interrupt pins, they're set as active low with an external pull-up
    pinMode(THE32KPIN,INPUT);               // These are the pins tha are used to negotiate for the i2c bus
    pinMode(TALKPIN,INPUT);                 // These are the pins tha are used to negotiate for the i2c bus

    enable32Khz(1);                         // turns on the 32k squarewave - to moderate access to the i2c bus TakeTheBus required from now on
    
    TakeTheBus();                           // Need the i2c bus for initializations
        if (fram.begin()) {                 // you can stick the new i2c addr in here, e.g. begin(0x51);
            Serial.println(F("Found I2C FRAM"));
        } else {
            Serial.println(F("No I2C FRAM found ... check your connections"));
            BlinkForever();
        }
    GiveUpTheBus();                         // Done with i2c initializations Arduino gives up the bus here.
    digitalWrite(YELLOWLED, HIGH);          // Step 1 - First time we have taken the i2c bus
    
    if (FRAMread8(VERSIONADDR) != VERSIONNUMBER) {  // Check to see if the memory map in the sketch matches the data on the chip
        Serial.print(F("FRAM Version Number: "));
        Serial.println(FRAMread8(VERSIONADDR));     // FRAM version is stored in 1st memory position and in the sketch
        Serial.read();
        Serial.println(F("Memory/Sketch mismatch! Erase FRAM? (Y/N)"));     // Mismatch requires memory wipe
        while (!Serial.available());
        switch (Serial.read()) {            // Give option to erase and reset memory
            case 'Y':
                ResetFRAM();
                break;
            case 'y':
                ResetFRAM();
                break;
            default:
                Serial.println(F("Cannot proceed"));
                BlinkForever();
        }
    }
    digitalWrite(REDLED, HIGH);                 // Step 2  - FRAM initialized
    
    TakeTheBus();                               // Initialize the rest of the i2c devices
        batteryMonitor.reset();                 // Initialize the battery monitor
        batteryMonitor.quickStart();
        setSyncProvider(RTC.get);               // Set up the clock as we will control it and the alarms here
        Serial.println(F("RTC Sync"));
        if (timeStatus() != timeSet) {
            Serial.println(F(" time sync fail!"));
            BlinkForever();                     // What if the clock fails - then we have to abort
        }
        t = RTC.get();                          // We need the time for the EEPROM offset - might as well get it here.
        // We need to set an Alarm or Two in order to ensure that the Simblee is put to sleep at night
        RTC.squareWave(SQWAVE_NONE);                    //Disable the default square wave of the SQW pin.
        RTC.alarm(ALARM_1);                             // This will clear the Alarm flags
        RTC.alarm(ALARM_2);                             // This will clear the Alarm flags
        RTC.setAlarm(ALM1_MATCH_MINUTES, 00, 00, 00, 00); // Alarm every hour
        //RTC.setAlarm(ALM1_MATCH_SECONDS, 00, 00, 00, 00); // Alarm every minute for testing
        RTC.alarmInterrupt(ALARM_2, false);             // Connect the Interrupt to the Alarms (or not)
        RTC.alarmInterrupt(ALARM_1, true);              // We will use ALARM 1 for this sketch
    GiveUpTheBus();
    digitalWrite(YELLOWLED, LOW);                       // Step 3 - Clock and Alarms set
    
    Serial.print(F("Monthly reboot count is "));        // We will keep track of the number of reboots - in service these come from WDT
    bootCountAddr = EEPROM.read(0);                     // Will use a monthly offset stored in 0 byte - reduce memory burn-out
    bootcount = EEPROM.read(bootCountAddr);             // Use the offset to get to this month's boot count
    bootcount++;                                        // Increment the boot count
    Serial.print(bootcount);                            // Print out bootcounts for Serial montitor
    EEPROM.write(bootCountAddr, bootcount);             // Write it back into the correct spot
    FRAMwrite8(MONTHLYREBOOTCOUNT, bootcount);          // Store in FRAM for access by Simblee in user interface
    Serial.print(F(" with a monthly offset of: "));
    bootCountAddr = month(t);                           // Boot counts are offset by month to reduce burn - in risk
    EEPROM.update(0, bootCountAddr);                    // Will update the month if it has changed but only at reboot
    Serial.println(EEPROM.read(0));                     // Print so we can see if code is working
    digitalWrite(REDLED, LOW);                          // Step 4 - EEPROM and first time check done
    
    Serial.print(F("Free memory: "));                   // Check to make sure we have enough working memory
    Serial.println(freeRam());
    
    if (!initializeAM2315()) {
        Serial.println("Sensor not found, check wiring & pullups!");
        BlinkForever();                                 // Temp Probe failed - need to halt here
    }
    TakeTheBus();
        currentHumid = RoundFloatToSignedByte(am2315.readHumidity());
        currentTemp =  RoundFloatToSignedByte(am2315.readTemperature());
    GiveUpTheBus();
    if (currentHumid == 0) {
        BlinkForever();                                 // Temp probe is not giving a valid return - need to halt here
    }
    FRAMwrite8(CURRENTTEMPADDR, currentTemp);           // Seed the current temp and humidity values
    FRAMwrite8(CURRENTHUMIDADDR, currentHumid);
    
    digitalWrite(I2CPWR, LOW);                          // Got the data, now we can turn off the i2c port
    digitalWrite(LEDPWR, HIGH);                         // Setup complete - you can turn off the lights.
    FRAMwrite8(CONTROLREGISTER, toggleStartStop);       // Reset the control register and start the test
}

void loop()
{
    if (refreshMenu) {
        refreshMenu = 0;
        Serial.println(F("Temp / Humid Sensor Menu"));
        Serial.println(F("0 - Display Menu"));
        Serial.println(F("1 - Display status"));
        Serial.println(F("2 - Set the clock"));
        Serial.println(F("3 - Reset the memory"));
        Serial.println(F("4 - Start / stop measuring"));
        Serial.println(F("5 - Dump hourly readings"));
        Serial.println(F("6 - Last 28 days readings"));
        NonBlockingDelay(100);
    }
    if (Serial.available() >> 0) {      // Only enter if there is serial data in the buffer
        switch (Serial.read()) {          // Read the buffer
            case '0':
                refreshMenu = 1;
                break;
            case '1':   // Display Current Status Information
                Serial.print(F("Current Time:"));
                TakeTheBus();
                    t = RTC.get();
                GiveUpTheBus();
                PrintTimeDate(t);  // Give and take the bus are in this function as it gets the current time
                TakeTheBus();
                    stateOfCharge = batteryMonitor.getSoC();
                GiveUpTheBus();
                Serial.print(F("Daily High / Low Temps (F): "));
                Serial.print(Celsius2Fahrenheit(dailyHighTemp));
                Serial.print(F(" / "));
                Serial.println(Celsius2Fahrenheit(dailyLowTemp));
                Serial.print(F("Daily High / Low Humidity:  "));
                Serial.print(dailyHighHumid);
                Serial.print(F("% / "));
                Serial.print(dailyLowHumid);
                Serial.println(F("%"));
                Serial.print(F("State of charge: "));
                Serial.print(stateOfCharge);
                Serial.println(F("%"));
                Serial.print("Temp: "); Serial.println(currentTemp);
                Serial.print("Humidity: "); Serial.print(currentHumid);
                Serial.println(F("%"));
                Serial.print(F("Free memory: "));
                Serial.println(freeRam());
                Serial.print(F("Reboots: "));
                Serial.println(FRAMread8(MONTHLYREBOOTCOUNT));
                break;
            case '2':     // Set the clock
                SetTimeDate();
                PrintTimeDate(t);
                Serial.println(F("Date and Time Set"));
                break;
            case '3': // Reset FRAM Memory
                ResetFRAM();
                break;
            case '4':  // Start or stop the test
                if (inTest == 0) {
                    FRAMwrite8(CONTROLREGISTER, toggleStartStop | controlRegisterValue);    // Toggle the start stop bit high
                    StartStopTest(1);
                }
                else {
                    FRAMwrite8(CONTROLREGISTER, toggleStartStop ^ controlRegisterValue);    // Toggle the start stop bit low
                    StartStopTest(0);
                    refreshMenu = 1;
                }
                break;
            case '5':   // Dump the hourly data to the monitor
                numberHourlyDataPoints = FRAMread16(HOURLYPOINTERADDR); // Put this here to reduce FRAM reads
                Serial.print("Retrieving ");
                Serial.print(HOURLYCOUNTNUMBER);
                Serial.println(" hourly counts");
                Serial.println(F("Hour Ending -   Temp  - Humidity"));
                for (int i=0; i < HOURLYCOUNTNUMBER; i++) { // Will walk through the hourly count memory spots - remember pointer is already incremented
                    unsigned int address = (HOURLYOFFSET + (numberHourlyDataPoints + i) % HOURLYCOUNTNUMBER)*WORDSIZE;
                    byte hourlyTemp = FRAMread8(address+HOURLYTEMPOFFSET);
                    delay(1);
                    if (hourlyTemp > 0) {
                        time_t unixTime = FRAMread32(address);
                        toArduinoTime(unixTime);
                        Serial.print(F(" - "));
                        Serial.print(hourlyTemp);
                        Serial.print(F("  -  "));
                        Serial.print(FRAMread8(address+HOURLYHUMIDOFFSET));
                        Serial.println(F("%"));
                    }
                    delay(1);
                }
                Serial.println(F("Done"));
                break;
            case '6':  // Download all the daily counts
                numberDailyDataPoints = FRAMread8(DAILYPOINTERADDR);        // Put this here to reduce FRAM reads
                Serial.println(F("Date - Low Temp - High Temp - Low Humid  High Humid - Battery %"));
                for (int i=0; i < DAILYCOUNTNUMBER; i++) {                  // Will walk through the 30 daily count memory spots - remember pointer is already incremented
                    int address = (DAILYOFFSET + (numberDailyDataPoints + i) % DAILYCOUNTNUMBER)*WORDSIZE;      // Here to improve readabiliy - with Wrapping
                    if (FRAMread8(address+DAILYHIGHTEMPOFFSET)) {           // Since we will step through all 30 - don't print empty results
                        Serial.print(FRAMread8(address));
                        Serial.print(F("/"));
                        Serial.print(FRAMread8(address+DAILYDATEOFFSET));
                        Serial.print(F("   -    "));
                        Serial.print(FRAMread8(address+DAILYLOWTEMPOFFSET));
                        Serial.print(F("   -   "));
                        Serial.print(FRAMread8(address+DAILYHIGHTEMPOFFSET));
                        Serial.print(F("     -     "));
                        Serial.print(FRAMread8(address+DAILYLOWHUMIDOFFSET));
                        Serial.print(F("%    -    "));
                        Serial.print(FRAMread8(address+DAILYHIGHHUMIDOFFSET));
                        Serial.print(F("%    -     "));
                        Serial.print(FRAMread8(address+DAILYBATTOFFSET));
                        Serial.println(F("%"));
                        delay(2);
                    }
                }
                Serial.println(F("Done"));
                break;
            default:
                Serial.println(F("Invalid choice - try again"));
        }
        Serial.read();  // Clear the serial buffer
    }
    if (millis() >= lastCheckedControlRegister + controlRegisterDelay) {
        controlRegisterValue = FRAMread8(CONTROLREGISTER);
        lastCheckedControlRegister = millis();
        if (controlRegisterValue ^ oldControlRegisterValue) // XOR - will give a positive value if there has been a change
        {
            oldControlRegisterValue = controlRegisterValue;  //   By resetting here - every clause below needs to leave the flag correctly set
            if ((controlRegisterValue & toggleStartStop) >> 2 && !inTest)
            {
                StartStopTest(1);  // If the control says start but we are stopped
            }
            else if (!((controlRegisterValue & toggleStartStop) >> 2) && inTest)
            {
                StartStopTest(0); // If the control bit says stop but we have started
            }
            else if (controlRegisterValue & signalClearCounts)
            {
                TakeTheBus();
                t = RTC.get();
                GiveUpTheBus();
                currentTemp = 0;
                currentHumid = 0;
                FRAMwrite8(CURRENTLOWTEMPADDR, currentTemp);  // Clear current values in FRAM
                FRAMwrite8(CURRENTHIGHTEMPADDR, currentTemp);
                FRAMwrite8(CURRENTLOWHUMIDADDR, currentHumid);
                FRAMwrite8(CURRENTHIGHHUMIDADDR, currentHumid);
                FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
                Serial.println(F("Current Counts Cleared"));
                controlRegisterValue &= clearClearCounts;
                FRAMwrite8(CONTROLREGISTER, controlRegisterValue);
            }
            else if (controlRegisterValue & signalSimbleeReset)  // If the reset flag is set
            {
                Serial.println("Resetting the Simblee");
                if (!(controlRegisterValue & toggleSimbleeSleep)) // Only reset if the Simblee is awake
                {
                    pinMode(RESETPIN, OUTPUT);
                    digitalWrite(RESETPIN, LOW);
                    NonBlockingDelay(100);
                    digitalWrite(RESETPIN, HIGH);
                    pinMode(RESETPIN, INPUT);
                }
                FRAMwrite8(CONTROLREGISTER, controlRegisterValue & clearSimbleeReset);  // Reset the Simblee Sleep flag
            }
        }
    }
    if (inTest == 1)
    {
        boolean result = false;
        if (millis() >= lastReading + timeBetweenReadings)
        {
            result = CheckTempHumid();
        }
        if (millis() >= lastAwakening + stayAwakeTime && result)
        {
            Serial.println(F("Got reading now going back to sleep"));
            sleepNow();
        }
    }
}

boolean CheckTempHumid() // This is where we test, validate and store the temp and humidity data
{
    TakeTheBus();
        t = RTC.get();
    GiveUpTheBus();
    if (t == 0) return 0;     // This means there was an error in reading the real time clock - will simply throw out this count
    lastReading = millis();
    if (HOURLYPERIOD != currentHourlyPeriod)
    {
        initializeAM2315();  // Turn on the sensor
        TakeTheBus();
            currentHumid = RoundFloatToSignedByte(am2315.readHumidity());
            currentTemp =  RoundFloatToSignedByte(am2315.readTemperature());
        GiveUpTheBus();
        FRAMwrite8(CURRENTTEMPADDR, currentTemp);
        FRAMwrite8(CURRENTHUMIDADDR, currentHumid);
        FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
        if (currentTemp > dailyHighTemp)
        {
            dailyHighTemp = currentTemp;
            FRAMwrite8(CURRENTHIGHTEMPADDR, currentTemp);
            Serial.print(F("New Daily High Temp: "));
            Serial.println(Celsius2Fahrenheit(currentTemp));
        }
        if ((currentTemp < dailyLowTemp) || dailyLowTemp == 0)
        {
            dailyLowTemp = currentTemp;
            FRAMwrite8(CURRENTLOWTEMPADDR, currentTemp);
            Serial.print(F("New Daily Low Temp: "));
            Serial.println(Celsius2Fahrenheit(currentTemp));
        }
        if (currentHumid > dailyHighHumid)
        {
            dailyHighHumid = currentHumid;
            FRAMwrite8(CURRENTHIGHHUMIDADDR, currentHumid);
            Serial.print(F("New Daily High Humidity: "));
            Serial.print(currentHumid);
            Serial.println(F("%"));
        }
        if ((currentHumid < dailyLowHumid) || dailyLowHumid == 0)
        {
            dailyLowHumid = currentHumid;
            FRAMwrite8(CURRENTLOWHUMIDADDR, currentHumid);
            Serial.print(F("New Daily Low Humidity: "));
            Serial.print(currentHumid);
            Serial.println(F("%"));
        }
        LogHourlyEvent();
        Serial.print(F("Temp: ")); Serial.print(Celsius2Fahrenheit(currentTemp));
        Serial.print(F("C  Humidity: ")); Serial.print(currentHumid);
        Serial.print(F("%  "));
        Serial.print(F(" Reboots: "));
        Serial.print(bootcount);
        Serial.print(F("  Time: "));
        PrintTimeDate(t);
    }
    if (DAILYPERIOD != currentDailyPeriod) {
        LogDailyEvent();
        Serial.println(F("Logged Daily reading"));
    }
    digitalWrite(I2CPWR, LOW);         // Turns off the i2c port
    return 1;
}

void StartStopTest(boolean startTest)  // Since the test can be started from the serial menu or the Simblee - created a function
{
    tmElements_t tm;
    TakeTheBus();
        t = RTC.get();                    // Gets the current time
    GiveUpTheBus();
    if (startTest) {
        inTest = true;
        //currentHourlyPeriod = HOURLYPERIOD;   // Sets the hour period for when the count starts (see #defines)
        //currentDailyPeriod = DAILYPERIOD;     // And the day  (see #defines)
        // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
        time_t unixTime = FRAMread32(CURRENTCOUNTSTIME);
        breakTime(unixTime, tm);
        currentHourlyPeriod = tm.Hour;
        currentDailyPeriod = tm.Day;
        currentTemp = FRAMread8(CURRENTTEMPADDR);
        currentHumid = FRAMread8(CURRENTHUMIDADDR);
        dailyLowHumid = FRAMread8(CURRENTLOWHUMIDADDR);
        dailyHighHumid = FRAMread8(CURRENTHIGHHUMIDADDR);
        dailyLowTemp = FRAMread8(CURRENTLOWTEMPADDR);
        dailyHighTemp = FRAMread8(CURRENTHIGHTEMPADDR);
        Serial.println(F("Test Started"));
    }
    else {
        inTest = false;                 // Set the flag that indicates we are not in a test
        FRAMwrite32(CURRENTCOUNTSTIME, t);   // Write to FRAM - this is so we know when the last counts were saved
        Serial.println(F("Test Stopped"));
    }
}

void LogHourlyEvent() // Log Hourly Event()
{
    tmElements_t timeElement;       // We will need to break down the current time
    time_t LogTime = FRAMread32(CURRENTCOUNTSTIME);     // This is the last event recorded - this sets the hourly period
    //breakTime(LogTime, timeElement);                    // Break the time into its pieces
    unsigned int pointer = (HOURLYOFFSET + FRAMread16(HOURLYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
    //LogTime -= (60*timeElement.Minute + timeElement.Second); // Subtract the minutes and seconds needed to take to the top of the hour
    FRAMwrite32(pointer, LogTime);   // Write to FRAM - this is the end of the period
    FRAMwrite8(pointer+HOURLYTEMPOFFSET,currentTemp);
    FRAMwrite8(pointer+HOURLYHUMIDOFFSET,currentHumid);
    TakeTheBus();
        stateOfCharge = batteryMonitor.getSoC();
    GiveUpTheBus();
    FRAMwrite8(pointer+HOURLYBATTOFFSET,stateOfCharge);
    unsigned int newHourlyPointerAddr = (FRAMread16(HOURLYPOINTERADDR)+1) % HOURLYCOUNTNUMBER;  // This is where we "wrap" the count
    FRAMwrite16(HOURLYPOINTERADDR,newHourlyPointerAddr);
    currentHourlyPeriod = HOURLYPERIOD;  // Change the time period
    Serial.println(F("Hourly Event Logged"));
}

void LogDailyEvent() // Log Daily Event()
{
    tmElements_t timeElement;
    time_t LogTime = FRAMread32(CURRENTCOUNTSTIME);// This is the last event recorded - this sets the daily period
    breakTime(LogTime, timeElement);
    int pointer = (DAILYOFFSET + FRAMread8(DAILYPOINTERADDR))*WORDSIZE;  // get the pointer from memory and add the offset
    FRAMwrite8(pointer,timeElement.Month); // The month of the last count
    FRAMwrite8(pointer+DAILYDATEOFFSET,timeElement.Day);  // Write to FRAM - this is the end of the period  - should be the day
    // This code will ensure the pointer in EEPROM for reboots us updated to the right month if needed
    bootCountAddr = timeElement.Month;                   // Boot counts are offset by month to reduce burn - in risk
    EEPROM.update(0, bootCountAddr);            // Will update the month if it has changed
    // End EEPROM section - back to Logging Daily Event
    FRAMwrite8(pointer+DAILYLOWTEMPOFFSET,dailyLowTemp);
    FRAMwrite8(pointer+DAILYHIGHTEMPOFFSET, dailyHighTemp);
    FRAMwrite8(pointer+DAILYLOWHUMIDOFFSET ,dailyLowHumid);
    FRAMwrite8(pointer+DAILYHIGHHUMIDOFFSET, dailyHighHumid);
    TakeTheBus();
        stateOfCharge = batteryMonitor.getSoC();
    GiveUpTheBus();
    FRAMwrite8(pointer+DAILYBATTOFFSET,stateOfCharge);
    byte newDailyPointerAddr = (FRAMread8(DAILYPOINTERADDR)+1) % DAILYCOUNTNUMBER;  // "wrap" the count to stay in our memory space
    FRAMwrite8(DAILYPOINTERADDR,newDailyPointerAddr);
    currentDailyPeriod = DAILYPERIOD;  // Change the time period
    dailyLowHumid = dailyLowTemp = 100;  // Reset the daily values
    dailyHighHumid = dailyHighTemp = 0;  // Reset the daily values
    Serial.println(F("Logged a Daily Event"));
}

void SetTimeDate()  // Function to set the date and time from the terminal window
{
    tmElements_t tm;
    Serial.println(F("Enter Seconds (0-59): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Second = Serial.parseInt();
    Serial.println(F("Enter Minutes (0-59): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Minute = Serial.parseInt();
    Serial.println(F("Enter Hours (0-23): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Hour= Serial.parseInt();
    Serial.println(F("Enter Day of the Month (1-31): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Day = Serial.parseInt();
    Serial.println(F("Enter the Month (1-12): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Month = Serial.parseInt();
    Serial.println(F("Enter the Year (e.g. 2017): "));
    while (Serial.available() == 0) {  // Look for char in serial queue and process if found
        continue;
    }
    tm.Year = CalendarYrToTm(Serial.parseInt());
    t= makeTime(tm);
    TakeTheBus();
    RTC.set(t);             //use the time_t value to ensure correct weekday is set
    setTime(t);
    GiveUpTheBus();
}

void PrintTimeDate(time_t t)  // Prints time and date to the console
{
    Serial.print(year(t), DEC);
    Serial.print('/');
    Serial.print(month(t), DEC);
    Serial.print('/');
    Serial.print(day(t), DEC);
    Serial.print(F(" "));
    Serial.print(hour(t), DEC);
    Serial.print(':');
    if (minute(t) < 10) Serial.print(F("0"));
    Serial.print(minute(t), DEC);
    Serial.print(':');
    if (second(t) < 10) Serial.print(F("0"));
    Serial.print(second(t), DEC);
    Serial.println();
}


void pinChangeISR()        // Sensor Interrupt Handler
{
    // execute code here after wake-up before returning to the loop() function
    // timers and code using timers (serial.print and more...) will not work here.
    // we don't really need to execute any special functions here, since we
    // just want the thing to wake up
    sleep_disable();         // first thing after waking from sleep is to disable sleep...
    detachInterrupt(digitalPinToInterrupt(ALARMPIN));      // disables interrupt
}
void sleepNow()
{
    // Here is a great tutorial on interrupts and sleep: http://www.gammon.com.au/interrupts
    stayAwakeTime = stayAwakeAfterFirstSleep;
    Serial.print(F("Entering Sleep mode..."));
    Serial.flush ();            // wait for Serial to finish outputting
    RTC.alarm(ALARM_1);                     // This will clear the Alarm flags
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
    noInterrupts ();            // make sure we don't get interrupted before we sleep
    sleep_enable ();            // enables the sleep bit in the mcucr register
    attachInterrupt(digitalPinToInterrupt(ALARMPIN),pinChangeISR, LOW); // use interrupt and run function
    interrupts ();              // interrupts allowed now, next instruction WILL be executed
    sleep_cpu ();               // here the device is put to sleep
    // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
    delay(10);                  // This small delay gives the i2c bus time to reinitialize
    lastAwakening = millis();   // So we can set a delay before it goes back to sleep
    Serial.println(F("Waking up"));
}

void toArduinoTime(time_t unixT) // Puts time in format for reporting
{
    tmElements_t timeElement;
    breakTime(unixT, timeElement);
    Serial.print(timeElement.Month);
    Serial.print(F("/"));
    Serial.print(timeElement.Day);
    Serial.print(F("/"));
    Serial.print(1970+timeElement.Year);
    Serial.print(F(" "));
    Serial.print(timeElement.Hour);
    Serial.print(F(":"));
    if(timeElement.Minute < 10) Serial.print(F("0"));
    Serial.print(timeElement.Minute);
    Serial.print(F(":"));
    if(timeElement.Second < 10) Serial.print(F("0"));
    Serial.print(timeElement.Second);
}

boolean initializeAM2315()          // Turn on the power and fire up the sensor for a reading
{
    digitalWrite(I2CPWR, HIGH);         // Turns on the i2c port
    Serial.println(F("Turned on I2C power"));
    NonBlockingDelay(5000);
    TakeTheBus();
        boolean beginResult = am2315.begin();
    GiveUpTheBus();
    return beginResult;
}

void BlinkForever() // When something goes badly wrong...
{
    Serial.println(F("Error - Reboot"));
    while(1) {
        digitalWrite(REDLED,HIGH);
        delay(200);
        digitalWrite(REDLED,LOW);
        delay(200);
    }
}

void enable32Khz(uint8_t enable)  // Need to turn on the 32k square wave for bus moderation - could set the Talk line here
{
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.endTransmission();
    
    // status register
    Wire.requestFrom(0x68, 1);
    
    uint8_t sreg = Wire.read();
    
    sreg &= ~0b00001000; // Set to 0
    if (enable == true)
        sreg |=  0b00001000; // Enable if required.
    
    Wire.beginTransmission(0x68);
    Wire.write(0x0F);
    Wire.write(sreg);
    Wire.endTransmission();
}

int freeRam ()  // Debugging code, to check usage of RAM
{
    // Example Call: Serial.println(freeRam());
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

int8_t RoundFloatToSignedByte (float floater)       // Will round floater and not allow for overflow or lost data
{
    if (floater >= 128) return (int8_t)128;
    if (floater <= -128) return (int8_t)-128;
    if (floater >= 0) return (int8_t)(floater+0.5);
    if (floater < 0) return (int8_t)(floater - 0.5);
}

int8_t Celsius2Fahrenheit(int8_t celsius)   // Note we are only using a byte so numbers must stay in range -127 to 127
{
    if (celsius < 0) return (celsius * 18 - 5)/10 + 32; else return (celsius * 18 + 5)/10 + 32;
    
}
