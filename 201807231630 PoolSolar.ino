/*
 * Project Pool Solar Controller
 * Description:
 * Measures 3 data points from 8 swimming pool solar heating panels (direct 
 * sunlight on black pipe array).
 * 1) Water temperature at each inflow point of coil or array
 * 2) Water temperature at each outplow point of coil or array
 * 3) Water flow rate througheach coil or array
 * A manual valve at each array or coil allows the water flow to be throttled, 
 * thereby assuring that all coils receive optimal water flow. 
 * Two additional temperature sensors measure air temp in the sun and in the shade.
 *
 * TEMPERATURE SENSORS
 * ===================
 * The temperature sensors are waterproof DS18B20 with the following wiring
 *      BLK        = Ground
 *      RED        = Power (either 5V or 3.3V ok)
 *      WHT or YEL = Data
 * The wire topology is Linear topology because Star topology proved to be unreliable, see
 * http://www.vigesdal.net/weather/my_station/pdf_files/guidelines_for_reliable_1_wire_networks.pdf
 * The Data line needs an external pull-up resistor. I chose 2.2 K Ohm.
 * Both the Photon and the DS18B20 can then pull the voltage down to zero from time to time during data transmission.
 * Because of the 5V logic used, only 5V tolerant GPIO pins must be used on the Photon 
 * (all except A3 and A6/DAC are 5V tolerant)
 *
 *  --o----------o-----------------------o-----------------------o--------------- Vcc (+5V)
 *    |          |                       |                       |
 *   PULL        |                       |                       |
 *   UP          |                       |                       |
 *   RES         |                       |                       |
 *   2.2K        | RED                   |                       |
 *    |          |                       |                       |
 *    |     +---------+ YEL or WHT  +---------+             +---------+
 *    |     | DS18B20 |------+      | DS18B20 |------+      | DS18B20 |------+                         
 *    |     +---------+      |      +---------+      |      +---------+      |               
 *    |          |           |           |           |           |           |
 *    |          | BLK       |           |           |           |           |       To Particle Photon 
 *    +--------  |  ---------o---------  |  ---------o---------  |  ---------o-----> 5V tolerant input pin  
 *               |                       |                       |                   
 *               |                       |                       |                   
 *               |                       |                       |             
 *               |                       |                       |
 *  -------------o-----------------------o-----------------------o-------------- Gnd 
 *  
 * Multiple (typically 2) temperature sensors are bussed together on each "onewire" bus.
 * There are 9 OneWire buses in this project to support 16 water temp sensors plus 2 air temp sensors.
 * Each sensor has a globally unique serial number built in at the factory.
 * The low level software protocol (in the onewire library) allows the bus master (this device) 
 * to probe for the presence of sensors and returns in an array of integers the serial numbers 
 * of all sensors found.
 * Format of the serial number (6 Bytes). The first Byte is always 0x28
 *     +--------+--------+--------+--------+--------+--------+--------+--------+
 *     | int[0] | int[1] | int[2] | int[3] | int[4] | int[5] | int[6] | int[7] |
 *     +--------+--------+--------+--------+--------+--------+--------+--------+
 *     |  0x28  | Ser#[0]| Ser#[1]| Ser#[2]| Ser#[3]| Ser#[4]| Ser#[5]|  CRC   | 
 *     +--------+--------+--------+--------+--------+--------+--------+--------+
 * Since the order in which sensors are found is unknown, this sketch uses hardcoded
 * sensor addresses. This way we know which sensor address (serial numFber) belongs to which 
 * physical sensor. A helper routine identifySensor() reads and prints the factory serial number
 * of one sensor.
 *
 * 
 * WATER FLOW SENSORS
 * ==================
 * The water flow rate sensor model HS43TB consists of a turbine inside a short segment of 3/4 "
 * brass or stainless steel water pipe which triggers a Hall efect sensor as it rotates.
 * revolution. The water flow rate is a function of the turbine speed:
 *      Pulse frequency [Hz] = 8.1 * Flow Rate [liters per minute]   (+/- 10%)
 *      or: 1 liter = approx. 486 pulses
 * The water flow sensors use the following wiring
 *      BLK = Ground
 *      RED = 5.0V   
 *      YEL = Data
 * The Data line needs an external pull-up resistor. I chose 33 K Ohm.
 * A second resistor of twice the resistance (66 K Ohm) ensures that the voltage fed to the 
 * Particle Photon digital input pin is never higher that 3.3 V (2/3 of 5 V).
 * The HS43TB can then pull the voltage down to zero from time to time during an active pulse cycle.
 *
 *                  ------------o--------o------------- Vcc (+5V)
 *                              |        | 
 *                              |        |
 *                              |       PULL
 *                              |       UP 
 *          +-----------+  RED  |       RES
 *          |           |-------+       33K
 *          |           |                |
 *          |           |  YEL           |
 *          |  HS43TB   |----------------o---------> To Particle Photon Digital input pin
 *          |           |                |           (+3.3 V max,  0 V min)
 *          |           |  BLK           |        
 *          |           |-------+        |          
 *          +-----------+       |       RES        
 *                              |       66K 
 *                              |        |        
 *                              |        |        
 *                              |        |       
 *                              |        |        
 *                  ------------o--------o--------------- Gnd 
 *  
 * 
 * The 3.3 V signal is fed to a digital input pin on the Particle Photon.
 * Processing the Hall effect pulses is accomplished via interrupts. Not all input pins are interrupt capable
 * and of those that are, some share the same interrupt line and only one of them must be used.
 * In the end, the following 11 input lines *can* be used:
 *    A0           (or A3 or D2) -  but only one of them
 *    A1           (or D4)
 *    A2
 *    A4           (or D1)
 *    A6 aka DAC   (or D3)
 *    A7 aka WKP   
 *    D5
 *    D6
 *    D7
 *    Rx
 *    Tx
 * Note that A5 and D0 must not be used at all for this purpose
 *
 * In this project we use the following 8 input lines
 *    A0           
 *    A1           
 *    A2
 *    A4           
 *    A6   
 *    A7    
 *    Rx
 *    Tx
 *

 *
 *
 *
 * 
 * Author: Thomas Hildebrand
 * 04/01/2018   v.01 
 * 04/20/2018   v.02 Single pin bus works with 12 temp sensors
 * 04/29/2018   v.03 Added support for Hall effect water flow sensors
 * 05/14/2018   v.04 Added enumeration and naming of temperature sensors
 * 05/18/2018   v.05 Store sensor addresses, busnumbers and names into cloud variables
 * 05/18/2018   v.06 Clean up code
 * 06/02/2018   v.07 Restructured oneWire star topology to linear topology and used 5V logic instead of 3V3
 * 06/11/2018   v.08 Added logging function cloudMessage() and improved temp sensor search algorithm
 * 07/15/2018   v.09 Added Particle.publish() of values for ThingSpeak (once every 60 seconds)
 *
 **********************************************************************************************************/

#define VERSION "v.08"    //This string is used in log and debug messages
#include <DS18B20.h>      // The library for the temperature sensor, it also pulls in the Onewire library


// Some tuning parameters (at compile time)
#define USBCONSOLE             TRUE    // Set to TRUE if using the serial USB console
#define nTempSensorsOwned      23      // All the sensors I own (installed or not)
#define nTempSensorsexpected    16     // Installed sensors. We'll retry locating all until this many are found.
#define nOneWireBuses           9      // 8 buses for water temp plus 1 bus for air temp
#define sensOnBus {2,2,2,2,2,2,2,2,2}  // Number of sensors on Bus1, Bus2 etc

#define nFlowSensors            8
#define MAXRETRYLOCATEBUS       15   // Retries to locate all sensors on one bus
#define MAXRETRYLOCATEALL       7    // Retries to locate all sensors on all buses
#define MAXRETRYTEMP            5    // Retries to read temperature
#define MSSAMPLETIME           60000 // Sample every 60 sec    Less jitter if same as MSPUBLISHTIME
#define MSPUBLISHTIME          60000 // Publish every 60 sec
#define TIMEZONEOFFSET         -7    // Pacific Standard Time

// Temperature sensors are found in random order. Sort them here. 
// The order in which they are found and assigned to their respective array elements mySensorAddresses[], etc.
// can be found in the boot log. Use that to populate the list of solar coils here, and give them meaningful names.
// Note that the flow sensors are hardwired: Coil 0 owns flowCount1, coil 1 owns flowcount2, etc.
#define coil01                 { 0, 1, 0, "Poolside east"    }   // Sensor# temp In,  Sensor# temp Out,  FlowMeter#, Name 
#define coil02                 { 2, 3, 1, "Poolside middle"  }
#define coil03                 { 4, 5, 2, "Poolside west"    }
#define coil04                 { 6, 7, 3, "Chimney east"     }
#define coil05                 { 8, 9, 4, "Chimney west"     }
#define coil06                 {10,11, 5, "Villanueva south" }
#define coil07                 {12,13, 6, "Villanueva north" }
#define coil08                 {14,15, 7, "Single northwest" }
#define airtemp                {16,17,99, "AirTemp"          }   // Air temp sunny, Air temp shade, (no flowsensor), name

// Some configuration parameters
#define OneWirePin1 D0
#define OneWirePin2 D1
#define OneWirePin3 D2
#define OneWirePin4 D3
#define OneWirePin5 D4
#define OneWirePin6 D5
#define OneWirePin7 D6
#define OneWirePin8 D7
#define OneWirePin9 A5

// Misc compile time constants
#define LEDON      HIGH
#define LEDOFF     LOW
#define APPENDMESS TRUE
#define NEWMESS    FALSE

#define name1 "Poolside east (in)"
#define addr1 {0x28,0x5F,0x04,0x62,0x09,0x00,0x00,0x12}

#define name2 "Poolside east (out)"
#define addr2 {0x28,0xFF,0x76,0x18,0x20,0x17,0x03,0x69}

#define name3 "Poolside middle (in)"
#define addr3 {0x28,0x20,0x6E,0x61,0x09,0x00,0x00,0x8A}

#define name4 "Poolside middle (out)"
#define addr4 {0x28,0xFF,0x67,0x9F,0x24,0x17,0x03,0x19}

#define name5 "Poolside west (in)"
#define addr5 {0x28,0x64,0xDB,0x60,0x09,0x00,0x00,0x31}

#define name6 "Poolside west (out)"
#define addr6 {0x28,0xFF,0xC0,0x85,0x24,0x17,0x03,0x58}

#define name7 "Chimney east (in)"
#define addr7 {0x28,0xFB,0xC4,0x60,0x09,0x00,0x00,0xB7}

#define name8 "Chimney east (out)"
#define addr8 {0x28,0xFF,0x11,0x48,0x81,0x14,0x02,0xA7}

#define name9 "Chimney west (in)"
#define addr9 {0x28,0x70,0xCF,0x63,0x09,0x00,0x00,0x5D}

#define name10 "Chimney west (out)"
#define addr10 {0x28,0xFF,0xDF,0xE2,0x80,0x14,0x02,0x3E}

#define name11 "Villanueva south (in)"
#define addr11 {0x28,0x2C,0x65,0x61,0x09,0x00,0x00,0x87}

#define name12 "Villanueva south (out)"
#define addr12 {0x28,0xFF,0x27,0xCA,0x20,0x17,0x04,0xB4}

#define name13 "Villanueva north (in)"
#define addr13 {0x28,0x75,0xCF,0x63,0x09,0x00,0x00,0xB6}

#define name14 "Villanueva north (out)"
#define addr14 {0x28,0xFF,0x07,0x7B,0x24,0x17,0x03,0x4F}

#define name15 "Single northwest (in)"
#define addr15 {0x28,0x83,0xC1,0x62,0x09,0x00,0x00,0x5B}

#define name16 "Single northwest (out)"
#define addr16 {0x28,0xFF,0x2D,0xCC,0x20,0x17,0x04,0x00}

#define name17 "unused1"
#define addr17 {0x28,0xFF,0x8A,0x3C,0x60,0x17,0x03,0xA1}

#define name18 "unused2"
#define addr18 {0x28,0xFF,0x75,0x11,0x20,0x17,0x03,0xB4}

#define name19 "unused3"
#define addr19 {0x28,0xFF,0x51,0xBE,0x60,0x17,0x05,0x8C}

#define name20 "unused4"
#define addr20 {0x28,0xFF,0x3C,0x7A,0x60,0x17,0x03,0x9B}

#define name21 "unused5"
#define addr21 {0x28,0xFF,0xF2,0x48,0x60,0x17,0x03,0x54}

#define name22 "unused6"
#define addr22 {0x28,0xFF,0x3A,0x0C,0x60,0x17,0x03,0xA6}

#define name23 "unused7"
#define addr23 {0x28,0xFF,0x2C,0x8C,0x60,0x17,0x03,0x9F}

#define nameUnknown "Name_unknown"



//========================== CODE STARTS HERE ===========================================
// Select antenna: 
// ANT_EXTERNAL = external
// ANT_INTERNAL = internal
// ANT_AUTO     = continually switch between the two
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));  

void interruptFlow1(void);        //Function definition of the Interrupt Service Routine
void interruptFlow2(void);        
void interruptFlow3(void);      
void interruptFlow4(void);        
void interruptFlow5(void);        
void interruptFlow6(void);        
void interruptFlow7(void);         
void interruptFlow8(void);        

int mySensorAddresses [][8] = 
   {
   addr1,addr2,addr3,addr4,addr5,addr6,addr7,addr8,addr9,addr10,
   addr11,addr12,addr13,addr14,addr15,addr16,addr17,addr18,addr19,addr20,
   addr21,addr22,addr23
   };
   
int sensorsOnBus [] = sensOnBus;  
   
int mySensorBus[nTempSensorsOwned];   //Holds the number of the OneWire bus that the sensor is attached to
   
const char *mySensorNames [] =       //This array contains pointers to the DEFINEd names of all sensors
   {
   name1,name2,name3,name4,name5,name6,name7,name8,name9,name10,
   name11,name12,name13,name14,name15,name16,name17,name18,name19,name20,
   name21,name22,name23
   };

char *connectedSensorNames [nTempSensorsOwned] ;  //This array contains pointers to the given names of all connected sensors

volatile uint32_t flowCount1;         //volatile because the value can change w/o the compiler knowing
volatile uint32_t flowCount2;         // 64bit to avoid overflow, (we receive about 50 interrupts per sec)      
volatile uint32_t flowCount3;         
volatile uint32_t flowCount4;        
volatile uint32_t flowCount5;         
volatile uint32_t flowCount6;        
volatile uint32_t flowCount7;         
volatile uint32_t flowCount8; 

retained uint8_t sensorBusAddresses[23][8];  // "retained" places it in Particle backup RAM (optional)

uint32_t msSampleTime  = MSSAMPLETIME  ; //assign value as chosen in #define 
uint32_t msPublishTime = MSPUBLISHTIME ;                                                    

int flowPin[nFlowSensors] = {A0, A1, A2, A4, A6, A7, RX, TX};
int nTempSensorsFound;
float WattofCoil[9];   // For ThingSpeak test
float AirSun ;
float AirShade;

float celsius[23];
uint32_t lastMillis=0;   // Needed for reporting and publishing
uint32_t lastFlow[9]={0,0,0,0,0,0,0,0,0};


struct solarCoilStruct
   { 
   int tempIn; 
   int tempOut; 
   int flowMeter; 
   char *coilName[25];
   };
   
struct solarCoilStruct SolarCoil[9] = { coil01 , coil02 , coil03 , coil04 , coil05 , coil06 , coil07 , coil08, airtemp };

char LogText[622];          // Particle cloud variable. Can be up to 622 (640 - 2 - 16) Bytes long
char ValueText[622];        // Particle cloud variable
char PublishText[622];      // Particle cloud variable
char DebugText[622];        // Particle cloud variable
char MessageBuffer[5000];   // Temporary buffer, can probably be smaller
char CloudVariableBuff[1000];
char CloudPublishBuff[1000];
char CloudDebugBuff[1000];


//Instantiate 9 OneWire bus objects for the temperature sensors
DS18B20 ds18b20Bus1(OneWirePin1);
DS18B20 ds18b20Bus2(OneWirePin2);
DS18B20 ds18b20Bus3(OneWirePin3);
DS18B20 ds18b20Bus4(OneWirePin4);
DS18B20 ds18b20Bus5(OneWirePin5);
DS18B20 ds18b20Bus6(OneWirePin6);
DS18B20 ds18b20Bus7(OneWirePin7);
DS18B20 ds18b20Bus8(OneWirePin8);
DS18B20 ds18b20Bus9(OneWirePin9);





//===================================== SETUP =========================================
void setup() 

  {
  int index;
  int attempt;
  char dateAndTime[100];

  //Register cloud-accessible variables before anything else
  Particle.variable("BootLog", LogText); 
  Particle.variable("Values",  ValueText);  
  Particle.variable("Debug",   DebugText);
  
  sprintf(LogText ,    "");  // Initialize with empty string
  sprintf(ValueText ,  "");  // Initialize with empty string
  sprintf(DebugText ,  "");  // Initialize with empty string
  
  Time.zone(-7);    // Accurate during PDT, one hour off during PST. That's ok.
  waitFor(Time.isValid, 60000);   // Wait until we have a valid local time
  strcpy(dateAndTime, Time.timeStr() );

  snprintf(MessageBuffer, sizeof(MessageBuffer),"SolarCtl Ver %s \r\n", VERSION);
  cloudMessage(LogText, MessageBuffer, APPENDMESS);
  snprintf(MessageBuffer, sizeof(MessageBuffer),"%s  t=%d ms |\r\n", dateAndTime , millis() );
  cloudMessage(LogText, MessageBuffer, APPENDMESS);
  
  if (USBCONSOLE==TRUE) 
    {
    Serial.begin(9600);             // For the serial USB debug console    
    delay(5000);                    // Milliseconds. Give putty a chance to connect.        
    }

  attachInterrupt(A0, interruptFlow1, RISING);
  attachInterrupt(A1, interruptFlow2, RISING);
  attachInterrupt(A2, interruptFlow3, RISING);
  attachInterrupt(A4, interruptFlow4, RISING);
  attachInterrupt(A6, interruptFlow5, RISING);
  attachInterrupt(A7, interruptFlow6, RISING);
  attachInterrupt(RX, interruptFlow7, RISING);
  attachInterrupt(TX, interruptFlow8, RISING);

  for (index=0; index<nFlowSensors; index++)      // Set up water flow sensor pins
    {
    pinMode(flowPin[index], INPUT);               // Use as input without pullup (pullup is external)
    }

    flowCount1=0;   //Resetting it here because pinMode leaves arbitrary numbers behind
    flowCount2=0;
    flowCount3=0;
    flowCount4=0;
    flowCount5=0;
    flowCount6=0;
    flowCount7=0;
    flowCount8=0;

    if (USBCONSOLE==TRUE) {listMySensors();}

  attempt=1;   
  do
   {
   nTempSensorsFound=0;   //We haven't found any yet
   locateAllSensors();   // Locate all sensors on all buses and store their addresses, bus number and name (by index)
   attempt++;
   }   while (attempt<=MAXRETRYLOCATEALL && nTempSensorsFound<nTempSensorsexpected);

  snprintf(MessageBuffer, sizeof(MessageBuffer),"%d sensors found in %d attempt(s)|\n",nTempSensorsFound,attempt-1);
  cloudMessage(LogText, MessageBuffer, APPENDMESS);
  for (index=0;index<nTempSensorsFound;index++)
     {
     snprintf(MessageBuffer, sizeof(MessageBuffer),"Bus%d %s|\n" ,mySensorBus[index],connectedSensorNames[index]);
     cloudMessage(LogText, MessageBuffer, APPENDMESS);
     }

  snprintf(MessageBuffer, sizeof(MessageBuffer),"Setup complete t=%d ...exiting\n",millis());
  cloudMessage(LogText, MessageBuffer, APPENDMESS);
  if (USBCONSOLE==TRUE) Serial.printf("%s",LogText);  

  
  } //end setup()


//===================================== LOOP =========================================
void loop()
  {
  static uint32_t msSample  = 0;   //Initially 0, retains value across invocations due to "static"
  static uint32_t msPublish = 0;   //Initially 0, retains value across invocations due to "static"

  if (millis() - msSample >= msSampleTime) 
    {
    msSample = millis();
    assembleValueData();
    if (USBCONSOLE==TRUE) Serial.printf("ASSEMBLING: %s",ValueText);
    }   //End data assemmbly

  if (millis() - msPublish >= msPublishTime) 
    {
    msPublish = millis();
    publishMinuteData();
    if (USBCONSOLE==TRUE) Serial.printf("PUBLISHING: %s",PublishText);
    }   //End publishing
    
  } //end loop()
  


//=========================== INTERUPT SERVICE ROUTINES ===============================
void interruptFlow1()
   { flowCount1++; }  

void interruptFlow2()
   { flowCount2++; }  

void interruptFlow3()
   { flowCount3++; }  

void interruptFlow4()
   { flowCount4++; }  

void interruptFlow5()
   { flowCount5++; }  

void interruptFlow6()
   { flowCount6++; }  

void interruptFlow7()
   { flowCount7++; }  

void interruptFlow8()
   { flowCount8++; }  




// =============================== OTHER FUNCTIONS =======================================
// =======================================================================================


//====================== Locate all connected Sensors=========================================
// Locate all sensors connected, i.e.found by probing the bus(es)
void locateAllSensors()
   {
   int  OneWireBus;
   bool busSuccess;
   int  startSensor;
   int  busAttempt;

   startSensor=0;   // The first one in the linear list of all temp sensors
   for(OneWireBus=1; OneWireBus<= nOneWireBuses; OneWireBus++)
    {
    busAttempt=0;
    do   // Attempt this bus until it succeeds or times out
      {
      busAttempt++;
      switch(OneWireBus)
        {
        case 1: ds18b20Bus1.resetsearch(); break; 
        case 2: ds18b20Bus2.resetsearch(); break;
        case 3: ds18b20Bus3.resetsearch(); break;
        case 4: ds18b20Bus4.resetsearch(); break;
        case 5: ds18b20Bus5.resetsearch(); break;
        case 6: ds18b20Bus6.resetsearch(); break;
        case 7: ds18b20Bus7.resetsearch(); break;
        case 8: ds18b20Bus8.resetsearch(); break;
        case 9: ds18b20Bus9.resetsearch(); break;
        }  
      busSuccess=locateSensorsOnBus(OneWireBus, startSensor);    
      }   while (busSuccess!=TRUE && busAttempt<MAXRETRYLOCATEBUS);   // Keep trying until the entire bus is good

    if (busSuccess==TRUE)
      {
      nTempSensorsFound += sensorsOnBus[OneWireBus-1];   // This bus was good. Tally up how many we found this time.
      startSensor       += sensorsOnBus[OneWireBus-1];   // This bus was good. Sensors for the next bus start here. 
      }

    }   // End one bus
  
   }   // End function locateAllSensors()





//********************* Locate all sensors on this bus ************************************
// Tries to locate as many sensors on this bus as we 
// expext (per applicable entry in the global array sensorsOnBus[])
// Returns TRUE if we successfully found as many as we expected
// Returns FALSE if the search failed on at least one sensor
bool locateSensorsOnBus(int busNumber, int firstSensorOnBus)
   {
   int sensorSuccess;
   int sensorNumberOnBus;
   int sensor;
   int  sensorNameIdx;
   char *sensorName;

   sensor=firstSensorOnBus;
   for (sensorNumberOnBus=0; sensorNumberOnBus<sensorsOnBus[busNumber-1]  ;sensorNumberOnBus++)  // Try another one sensor on this bus
     {
     sensorSuccess=locateOneSensor(busNumber,sensor);
     if (sensorSuccess==FALSE)   // Abort!
       {
       return(FALSE);
       }
     sensorNameIdx=sensorNameIndex(sensorBusAddresses[sensor]);
     if(sensorNameIdx>=0)   // A valid name was found
       {
       sensorName = strdup(mySensorNames[sensorNameIdx]);  // needed for conversion from const char to char*
       connectedSensorNames[sensor]=sensorName;   
       }
     else
       {
       sensorName = strdup("***Invalid sensor name***");  // needed for conversion from const char to char*
       connectedSensorNames[sensor]=sensorName;      
       }
     mySensorBus[sensor]=busNumber;   // Mark the sensor as being on this Bus
     sensor++;
     }   // End trying next sensor
   // All searches were successful
   return(TRUE);
   }   // End function locateSensorsOnBus()





//********************* Locate one sensor on this bus ************************************
// Locate the next sensor on this bus
bool locateOneSensor(int busNumber, int sensor)
   {
   int searchSuccess;

   switch(busNumber)
     {
     case 1:  searchSuccess=ds18b20Bus1.search(sensorBusAddresses[sensor]); break; 
     case 2:  searchSuccess=ds18b20Bus2.search(sensorBusAddresses[sensor]); break;      
     case 3:  searchSuccess=ds18b20Bus3.search(sensorBusAddresses[sensor]); break;      
     case 4:  searchSuccess=ds18b20Bus4.search(sensorBusAddresses[sensor]); break;      
     case 5:  searchSuccess=ds18b20Bus5.search(sensorBusAddresses[sensor]); break;      
     case 6:  searchSuccess=ds18b20Bus6.search(sensorBusAddresses[sensor]); break;      
     case 7:  searchSuccess=ds18b20Bus7.search(sensorBusAddresses[sensor]); break;      
     case 8:  searchSuccess=ds18b20Bus8.search(sensorBusAddresses[sensor]); break;      
     case 9:  searchSuccess=ds18b20Bus9.search(sensorBusAddresses[sensor]); break;      
     }

   return (searchSuccess);
   }   //End function locateOneSensor()






//============================= Get Sensor Temperature ====================================
// Retrieves the temp reading from one sensor on one bus, performs CRC check,
// returns temperature in a double-lenght float variable if successful
// returns "Not a Number" (NAN) if failure

double sensorTemp(int busNum,uint8_t addr[8])  
  {
  double mytemp;
  int   numRetries = 0;

  switch (busNum)
    { 
    case 1: do { mytemp = ds18b20Bus1.getTemperature(addr); } while (!ds18b20Bus1.crcCheck() && MAXRETRYTEMP > numRetries++); break;
    case 2: do { mytemp = ds18b20Bus2.getTemperature(addr); } while (!ds18b20Bus2.crcCheck() && MAXRETRYTEMP > numRetries++); break;
    case 3: do { mytemp = ds18b20Bus3.getTemperature(addr); } while (!ds18b20Bus3.crcCheck() && MAXRETRYTEMP > numRetries++); break;
    case 4: do { mytemp = ds18b20Bus4.getTemperature(addr); } while (!ds18b20Bus4.crcCheck() && MAXRETRYTEMP > numRetries++); break;
    case 5: do { mytemp = ds18b20Bus5.getTemperature(addr); } while (!ds18b20Bus5.crcCheck() && MAXRETRYTEMP > numRetries++); break;
    case 6: do { mytemp = ds18b20Bus6.getTemperature(addr); } while (!ds18b20Bus6.crcCheck() && MAXRETRYTEMP > numRetries++); break;
    case 7: do { mytemp = ds18b20Bus7.getTemperature(addr); } while (!ds18b20Bus7.crcCheck() && MAXRETRYTEMP > numRetries++); break;
    case 8: do { mytemp = ds18b20Bus8.getTemperature(addr); } while (!ds18b20Bus8.crcCheck() && MAXRETRYTEMP > numRetries++); break;
    case 9: do { mytemp = ds18b20Bus9.getTemperature(addr); } while (!ds18b20Bus9.crcCheck() && MAXRETRYTEMP > numRetries++); break;
    } // end switch
  if (numRetries == MAXRETRYTEMP) {mytemp = NAN;}   //we timed out without a usable reading
  return mytemp;
  }  //end function sensorTemp()


//============================= Get Sensor Name ====================================
// Looks up the name of the sensor given in this project (see #define section), 
// based on its factory-set address 8-Byte address. Returns the index of the name in 
// the array of names if found, and -1 if the name is not found.

int sensorNameIndex(uint8_t addr[8])  
  {
  bool weFoundIt;
  int  NameCandidate;
  int  ByteIndex;
  bool soFarSoGood;

  weFoundIt=FALSE;
  NameCandidate=0;
  do   //try one sensor
    {
    ByteIndex=0;
    soFarSoGood=TRUE;
    do    
       {   //try one address byte
       if (addr[ByteIndex] != mySensorAddresses[NameCandidate][ByteIndex])
          {
          soFarSoGood=FALSE;
          }
          ByteIndex++;    
       }  while (ByteIndex < 8 && soFarSoGood == TRUE);  // end try one address byte
       if (soFarSoGood == TRUE) { weFoundIt=TRUE; }
    NameCandidate++;
    }  while (NameCandidate<nTempSensorsOwned && weFoundIt==FALSE);   // end try one sensor
  if (weFoundIt==TRUE) 
    return NameCandidate-1;  // -1 because we overshot at the end of the loop
  else
    return -1;
  }  //end function sensorNameIndex()

  
  
  
//================ Assemble Flow and Temperature Value Data (used in Particle cloud variable and publishing ======================  
void assembleValueData()
  {
  int sensor;
  int busNumber;
  int myCoil;
  int sensorIn;
  int sensorOut;
  int flowSensor;
  int deltaMillis;
  int deltaFlow;
  int tempDate;
  int publishDate;
  int publishTime;
  uint32_t flowCount;
  uint32_t thisMillis;

  char *coilName;
  float temperatureIn;
  float temperatureOut;  
  float temperatureDelta;
  float deltaLiters;
  float Joule;
  float Watt;
  float LiterPerSecond;
  
//debug
int x01;
int x02;
int x04;
int x05;
// End debug

  
  
  // Clear the strings for the cloud variable and for the publish string
  snprintf(CloudVariableBuff, sizeof(CloudVariableBuff),"" );
  snprintf(CloudPublishBuff, sizeof(CloudPublishBuff),"" );
  
  // Compute time since last report
  thisMillis=millis();
x04=thisMillis;
x05=lastMillis;
  deltaMillis=thisMillis-lastMillis;
  lastMillis=thisMillis;

    // Get the CURRENT DATE AND TIME
    // Arrange human readable date:time as decimal numbers like 20170704:1526 for 2017/07/04 at 3:26 p.m.
    publishDate = 10000*Time.year(Time.now()) + 100*Time.month(Time.now()) + 1*Time.day(Time.now()); 
    publishTime = 100*Time.hour(Time.now()) + 1*Time.minute(Time.now()); 
    snprintf(CloudVariableBuff+strlen(CloudVariableBuff), sizeof(CloudVariableBuff), "%08d:%04d",publishDate,publishTime); 
    snprintf(CloudPublishBuff+strlen(CloudPublishBuff), sizeof(CloudPublishBuff), "%08d:%04d",publishDate,publishTime); 
    
  for (myCoil=0; myCoil<9; myCoil++)
    {
    sensorIn  = SolarCoil[myCoil].tempIn;   
    sensorOut = SolarCoil[myCoil].tempOut;     
    flowSensor= SolarCoil[myCoil].flowMeter; 
    coilName  = *SolarCoil[myCoil].coilName;
    
    // Get the NAME of the coil
    snprintf(CloudVariableBuff+strlen(CloudVariableBuff), sizeof(CloudVariableBuff),",%s", coilName );    
//  snprintf(CloudPublishBuff+strlen(CloudPublishBuff), sizeof(CloudPublishBuff),",%s", coilName );    
    
    // Get the WATER FLOW
   switch(myCoil)
        {
        case 0: flowCount=flowCount1; break;
        case 1: flowCount=flowCount2; break;
        case 2: flowCount=flowCount3; break;
        case 3: flowCount=flowCount4; break;
        case 4: flowCount=flowCount5; break;
        case 5: flowCount=flowCount6; break;
        case 6: flowCount=flowCount7; break;
        case 7: flowCount=flowCount8; break;
        case 8: flowCount=99999;      break;
        }
//    snprintf(CloudVariableBuff+strlen(CloudVariableBuff), sizeof(CloudVariableBuff),"F=%d ", flowCount );
//    snprintf(CloudPublishBuff+strlen(CloudPublishBuff), sizeof(CloudPublishBuff),"F=%d ", flowCount );

    // Get the TEMPERATURES
    busNumber = mySensorBus[sensorIn];
    temperatureIn = sensorTemp(busNumber, sensorBusAddresses[sensorIn]);
    busNumber = mySensorBus[sensorOut];
    temperatureOut = sensorTemp(busNumber, sensorBusAddresses[sensorOut]);
    temperatureDelta=temperatureOut-temperatureIn; 
    if(myCoil==8)   // Air temp
      {
      AirSun   = temperatureOut;
      AirShade = temperatureIn;
      }
    
    snprintf(CloudVariableBuff+strlen(CloudVariableBuff), sizeof(CloudVariableBuff),",%.1f+%.1f=%.1f", temperatureIn, temperatureDelta, temperatureOut );
    snprintf(CloudPublishBuff+strlen(CloudPublishBuff),   sizeof(CloudPublishBuff), ",%.1f,%.1f,%.1f", temperatureIn ,temperatureDelta, temperatureOut );
    
if ( myCoil<8 )  // Only the 8 real coils, not the air temp
    {
    // Compute metrics since last report
x01=flowCount;
x02=lastFlow[myCoil];
    deltaFlow=flowCount-lastFlow[myCoil];
    lastFlow[myCoil]=flowCount;
    
    deltaLiters=(float)deltaFlow/(float)486;     // 1 liter = approx. 486 pulses

    LiterPerSecond=deltaLiters * ( 1000 / float(deltaMillis) );
    snprintf(CloudVariableBuff+strlen(CloudVariableBuff), sizeof(CloudVariableBuff),",%.3fLPS", LiterPerSecond );
    snprintf(CloudPublishBuff+strlen(CloudPublishBuff),   sizeof(CloudPublishBuff), ",%.3f", LiterPerSecond );
     
    Joule = 4186*deltaLiters*temperatureDelta;   // Energy required to heat 1 Kg (=1 Liter) of water by 1 degree Celsius is 4186 Joules (=Ws)

    Watt = Joule* 1000 / float(deltaMillis);
    snprintf(CloudVariableBuff+strlen(CloudVariableBuff), sizeof(CloudVariableBuff),",%.0fW", Watt );
    snprintf(CloudPublishBuff+strlen(CloudPublishBuff), sizeof(CloudPublishBuff),",%.0f", Watt );
WattofCoil[myCoil]=Watt;  // For ThingSpeak test
    }



// Debug the jitters
if (myCoil==7)  // let's only look at one coil
{
  sprintf(CloudDebugBuff,"");  // Initialize again.
  snprintf(CloudDebugBuff+strlen(CloudDebugBuff), sizeof(CloudDebugBuff),"X01=%d X02=%d X03=%d X04=%d X05=%d X06=%d X07=%f X08=%f X09=%f X10=%f X11=%f",x01,x02,deltaFlow,x04,x05,deltaMillis,deltaLiters,LiterPerSecond,temperatureDelta,Joule,Watt);
  cloudMessage(DebugText, CloudDebugBuff, NEWMESS);
}
// End debug the jitters

    
    }   //End one coil
    cloudMessage(ValueText, CloudVariableBuff, NEWMESS);
    cloudMessage(PublishText, CloudPublishBuff, NEWMESS);


    
  }   //end assemble Value Data
    
    

    
    
//========================== Publish to the Particle Cloud once every minute ===============
void publishMinuteData() 
  {
  Particle.publish("ByMinute", CloudPublishBuff, PRIVATE);
  if(USBCONSOLE==TRUE) {Serial.printf("Now publishing: %s\r\n\n\n",CloudPublishBuff);}
  
//ThingSpeak test
delay(1100); // Max 1 Particle.publish() per second allowed
String ThingSpeak01 = String(WattofCoil[0]);
Particle.publish("Watt01", ThingSpeak01, PRIVATE);

delay(1100); // Max 1 Particle.publish() per second allowed
String ThingSpeak02 = String(WattofCoil[1]);
Particle.publish("Watt02", ThingSpeak02, PRIVATE);

delay(1100); // Max 1 Particle.publish() per second allowed
String ThingSpeak03 = String(WattofCoil[2]);
Particle.publish("Watt03", ThingSpeak03, PRIVATE);

delay(1100); // Max 1 Particle.publish() per second allowed
String ThingSpeak04 = String(WattofCoil[3]);
Particle.publish("Watt04", ThingSpeak04, PRIVATE);

delay(1100); // Max 1 Particle.publish() per second allowed
String ThingSpeak05 = String(WattofCoil[4]);
Particle.publish("Watt05", ThingSpeak05, PRIVATE);

delay(1100); // Max 1 Particle.publish() per second allowed
String ThingSpeak06 = String(WattofCoil[5]);
Particle.publish("Watt06", ThingSpeak06, PRIVATE);

delay(1100); // Max 1 Particle.publish() per second allowed
String ThingSpeak07= String(WattofCoil[6]);
Particle.publish("Watt07", ThingSpeak07, PRIVATE);

delay(1100); // Max 1 Particle.publish() per second allowed
String ThingSpeak08 = String(WattofCoil[7]);
Particle.publish("Watt08", ThingSpeak08, PRIVATE);

delay(1100); // Max 1 Particle.publish() per second allowed
String ThingSpeak09 = String(AirShade);
Particle.publish("AirShade", ThingSpeak09, PRIVATE);

delay(1100); // Max 1 Particle.publish() per second allowed
String ThingSpeak10 = String(AirSun);
Particle.publish("AirSun", ThingSpeak10, PRIVATE);
// End ThingSpeak test


  } //end publishMinuteData()





//========================== List all my Sensors ============================================
// Print a list of all sensors I own (addresses are hard coded in the #define section)
// This is for debugging purposes only

void listMySensors()
  {
  int sensor;
  int addressField;     
  Serial.printf("These are all the sensors I own (connected or not):\r\n");
  for (sensor=0; sensor<nTempSensorsOwned; sensor++)
     {
     Serial.printf("%s: ",mySensorNames[sensor]);
     for (addressField=0; addressField<8; addressField++)
        {
        Serial.printf(" %2.2X",mySensorAddresses[sensor][addressField]);
        }
     Serial.println();  
     }
  Serial.println();   
  }




//================= Append message string to Cloud variable =================================
// This function takes a message string and appends it to a Particle cloud variable, which must be properly set up
bool cloudMessage(char cloudVariable[], char newMessage[], bool append)
  {
  int existingLength;
  int additionalLength;
  int maximumLength;
  int indey;
  
  maximumLength = 622;  // cloud variable string can be up to 622 char long
  if (append==TRUE)
     {
     existingLength = strlen(cloudVariable);
     }
  else
     {
     existingLength = 0;
     }
  additionalLength = strlen(newMessage);
  if (existingLength + additionalLength <= maximumLength)   
    {
    strcpy(cloudVariable+existingLength, newMessage) ;
    return TRUE;  // success
    }
  else
    {
    for (indey=0; indey < (maximumLength-existingLength); indey++)
      {
      cloudVariable[existingLength+indey] = '$';  // pad with overflow signs ($$) to the end
      }
    cloudVariable[maximumLength-1] = 0;  // Terminate string
    return FALSE;
    }
  }



bool unique()
// This function checks for duplicate sensors found.
// For some reason, the library function (method??) ds18b20Bus1.search() returns with 2 sensors 
// found (which is correct), but they're both the same. This function unique() compares all 
// sensors found (by address) to see if there are any duplicates. The function retrurns TRUE if 
//  all sensor found are unique, and FALSE if one or more are duplicates of each other.
  {
  int uniqueIndex;
  int uniqueIndey;  
  int uniqueIndez;
  int uniqueIndew;
  bool sameAddress;
  
  for (uniqueIndex=0; uniqueIndex < nTempSensorsFound; uniqueIndex++)                // source sensor #
    {
    for (uniqueIndey=0; uniqueIndey<8; uniqueIndey++)                           // source sensor address byte
      {
      for (uniqueIndez=uniqueIndex; uniqueIndez<nTempSensorsFound; uniqueIndez++)   // destination sensor #
        {
        sameAddress=TRUE;   // for now
        for (uniqueIndew=0; uniqueIndew<8; uniqueIndew++)                       // destination sensor address byte
          {
          if (sensorBusAddresses[uniqueIndex][uniqueIndey] != sensorBusAddresses[uniqueIndez][uniqueIndew])
            {
                sameAddress=FALSE;  // this address byte is different. Good.
            }   // end if
          }   // end w
          if (sameAddress==TRUE)   // none of the 8 address bytes were different, i.e. we have a duplicate sensor.
            {return FALSE;}    // at least one sensor has all 8 address bytes identical to another sensor
        }   // end z
      }   // end y
    }   // end x   
  return TRUE;  // since we didn't find a duplicate earlier, all sensors must be unique.
  }  // end function unique()     
  
  
  
  
  

    
    
    
    
    