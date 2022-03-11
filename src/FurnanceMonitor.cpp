/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "/Users/robertvinci/VSC_Development/Furnace_Monitor2/src/FurnanceMonitor.ino"
//*************************
//    FURNACE MONITOR2
//  Author: R. Vinci
//  Original Date:   8/7/18
//  Version 2.0 : 2/3/22
//*************************
//#include "ThingSpeak.h"
#include "FurnaceMonitor.h"
#include "DS18B20.h"
// #include "MPU6050.h"
#include "Ubidots.h"

//HISTORY
// 02/10/22 B2.1.0.0 - More accurace readings, UBIDOTS for Boiler and DHW OA(sensor not implemented), and WiFI RSSI
// 02/03/22 B2.0.0.0 - Imported into VSC, disabled MPU6050 because of conflicts with UBIDOTS
//    Redefine function of Furnace Monitor2
//    1. Read temperatures from the controller temperature, rather than from additional sensors. It's redundant and cleaner
//    2. Will need to correlate the temperature "raw" values by reading the voltage across the input terminals of the Boiler, Demand Hot Water and Outside Air
//    3. Will also need to detect id there is a logic level to indicate when the oil pump is started.
//    4. Will record the Temperatures for the 3 probes on  UBIDOTS as well as the pump On-time hours.
//    5. Also get the Outside air temperature from the OWM web service and plot on UBIDOTS
//    6. Additional feature to add would be measuring the flu temperature and the CO2 content when the burner is firing to get the burner effiency.
//    7. Work on cleaning up and modularizing the code. i.e putting code in classes and adding include files.
// 2/10/22 - Fixed bufg where LUT was not returning the best answer.

// 08/07/18 B1.0.0.0 - Debug of Fuel Used; no gallon units roll over. Added WireScan.cpp
// 08/08/18 B1.0.1.0 - Added retained vars for restarts, pump time, and start date
// 08/08/18 B1.0.1.1 - Misc bug fixes,
// 08/09/18 B1.0.2.1 - Changed I2C reset() to ResetDS18b20() for resetting one-wire address search
// 08/09/18 B1.0.3.1 - Refactored for consistency
// 08/10/18 B1.0.4.1 - Added RGB LED for status indication (green "ok", blue "pump ON", red "error")
// 08/17/18 B1.0.5.1 - Add LED test in setup
// 08/20/18 B1.0.6.1 - Added gallonsInTankRef and setGallonsRemain()
// 09/10/18 B1.0.7.1 - Make gallonsInTankRef non-volatile retained variable
// 09/11/18 B1.0.8.1 - Added totalFurnaceHours
// 09/12/18 B1.1.8.1 - Added UBIDOTS support
// 09/12/18 B1.1.9.1 - Added  deviceId suffix for UBIDOTS
// 09/14/18 B1.2.9.1 - Threshold controlled data logging for UBIDOTS to cut back on logging load
// 09/14/18 B1.3.0.1 - Changed WIFI_LOG_THRSH to 5db from 3db to cut down on unnecessary logging
// 09/14/18 B1.3.1.1 - Change pumpState logging to record only when pump is ON.
// 09/14/18 B1.4.1.1 - Fixed pumpState logging to record only 0->1 and 1->0 transitions
// 09/15/18 B1.5.1.1 - Added Debug DEFINE for release version.
// 09/16/18 B1.5.3.1 - Renabled Publish events for IFTTT and JavaScript to Google Sheets connection
// 09/19/18 B1.6.3.1 - Support for Google Sheets and multiple Particle device logging

void setup();
void loop();
int resetSysOn_Date(String extra);
int resetDS18b20_Search(String extra);
int diagTestFcn(String extra);
int setGallonsRemain(String extra);
int setFurnaceHours(String extra);
void statusLED(int led, bool ledState, int dly);
int getTempFromVolts(double rdgVolts);
void publishTemperatureData(int sen);
int checkTempFromVolts(String rdgVoltsStr);
int setPublishTimeInMins(String minsToPublish);
#line 46 "/Users/robertvinci/VSC_Development/Furnace_Monitor2/src/FurnanceMonitor.ino"
#undef  DEBUG       //Comment out to debug
String photonOSverStr = System.version().c_str();
String appVerStr = "B2.1.0.0";

//UBIDOTS
#ifndef TOKEN
#define TOKEN  (char*)"BBFF-XVzIhb6cvOjc8zPdRwX1ficgAyURDq"  // Put here your Ubidots TOKEN
#endif
Ubidots ubidots(TOKEN, UBI_TCP);

//******** Change THIS when Flashing Device
#define DEVICE 7          //Photon OP_P7_FURNACE
#if  (DEVICE == 1)
  const char deviceIdName[] = {"OP_P1"};
  const int deviceIdNum = 1;
#elif (DEVICE == 7)
  const char deviceIdName[] = {"OP_P7_FURNACE"};
  const int deviceIdNum = 2;
#elif  (DEVICE == 9)
  const char deviceIdName[] = {"OP_P9"};
  const int deviceIdNum = 3;
#endif

//P1 = 1, P7 = 2, P9 = 3

// REMEMBER TO CHANGE SELECTED DEVICE BEFORE PROGRAMMING!!!

//Important Note:
//Cloud 'Particle.variable' names and 'Particle.Publish' names MUST NOT be identical
//There is no warning, they will just not work right!

//System Features
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

//I2C Wire Bus
const int clockSpeed = CLOCK_SPEED_100KHZ;    //Default is 100kHz

// MPU6050 variables:
// MPU6050 accelgyro;
// int16_t ax, ay, az;
// int16_t gx, gy, gz;
// int axMx, axMn, ayMx, ayMn, azMx, azMn;
// int gxMx, gxMn, gyMx, gyMn, gzMx, gzMn;
// int32_t axSum, aySum, azSum;
// int axAv, ayAv, azAv;
// int gxAv, gyAv, gzAv;
// int axAbs, ayAbs, azAbs;             //Absolute value = (ax - axOffset) includes correction
// int axOffset, ayOffset, azOffset;
// int axThrsh = 750;  //detection threshold acceleration X
// int ayThrsh = 750;  //detection threshold acceleration Y
// int azThrsh = 750;  //detection threshold acceleration Z
int32_t axAlrmCnt = 0;
int32_t ayAlrmCnt = 0;
int32_t azAlrmCnt = 0;

// int axThrsh = 1500;     //For debug use
// int ayThrsh = 1500;     //For debug use
// int azThrsh = 1500;     //For debug use
bool xThreshAlarm = false;
bool yThreshAlarm = false;
bool zThreshAlarm = false;

const int FA_OFFSET_NUM = 10;
int FA_Offset_cntr =  FA_OFFSET_NUM; //Samples to calculate "First Average" offset after system reset
int MA_NUM = 5;                     //Moving Average variable count changeable from GUI
int MA_cntr = MA_NUM;               //Sample counter to calculate "Moving Average"
bool firstAveDone =  false;         //First average flag used to calculate First Average offset
bool MA_done = false;

//Furnace Statistics
bool psTransition = false;            //Used to get UBIDOTS to plot transition state changes
bool pumpState = false;            // Is true only after valid detection (min dwell time)
bool lastPumpState = false;         // This will track pump state transitions
                                   //Note: 'true' will log inital 0 pump state condition
retained int PumpOnTime_Ctr;       //Accumulate seconds of ON time (fuel pump is ON)
int pumpOnTime_Hours;              //Pump ON time in nearest hours
int pumpOnTime_Mins;               //Pump ON time in nearest minutes
int pumpOnTime_Secs;               //Pump ON time in nearest seconds
float pumpOnTimeInDecimal = 0.0;   //For plotting time the pump has been on since tge last reset
retained int totalFurnaceHours = 0; //Total lifetime hours of the furnace
String totalFurnaceHoursStr = "";
String pumpOnHMSstr;               //ON time for fuel pump in hours and minutes
retained time_t sysStartDate;     //Day and time pump was started
retained int systemRestarts;       //How many times has the system be restarted
time_t pumpStopDate;               //Day and time pump was stopped
String sysOnDateStr = "";
int32_t minDwellOn = 6000;         //Minimum dwell period mS, before pumpState is set true
int32_t minDwellOff = 1000;        //Minimum dwell period mS, before pumpState is set false
const int UPDATE_RATE = 1000;      //Period in mS to check pump state
float flowRatePerMin = 0.0166667;  // gallons-per-minute (assume 1 gal/hr for current furnace nozzle)
float gallonsUsed = 0;             // consumption in gallons
float lastGallonsUsed = 0;
String gallonsUsedStr = "";
retained float correctionFactor = 1.0000;   // adjustable correction factor based on actual usage
retained float gallonsInTankRef = 0.0;      // Retained value that is initially set (or measured)
retained float gallonsRemaining = 0.0;
String gallonsRemainingStr = "";

//Logging thresholds
const float GALLONS_USED_LOG_THRSH = 0.1;    //Oil consumption (gallons) difference logging threshold
const int WIFI_LOG_THRSH = 5;                //WiFi signal difference (dB) logging threshold
const float BOILER_TEMP_LOG_THRSH = 5.0;     //Boiler temperature (degF) difference logging theshold
const float DHW_TEMP_LOG_THRSH = 5.0;        //Demand Hot Water temperature (degF) difference logging theshold
const float OA_TEMP_LOG_THRSH = 2.0;         //Outside Air temperature (degF) difference logging theshold
//AAA

//WiFi Signal Strength
int wifiRSSI = 0;
int lastWifiRSSI = 0;

//Note: Potential Usefule features
//Detect short interval pump operation (fault may be ignition problem or no fuel)
//Detect long interval pump operation  (could be caused by extremly cold weather)
//Detect no pump operation             (power failure, pump failure)
//Detect loss of power                 (AC failure)
//Monitor Furnace TEMPSCALE            (Furnace jacket, Outside Air, Dometic HW, Interior temps)
//Analysis of Recorded Data Features
//Capacity of Heating System Suitability (analyzes the duration and freq of power on cycles)
//Predictive failure                    (Detects characteristic failure signature of pump motor)
//CO and CO2 stack gasses               (realtime efficiency indicator)

//Temperature Sensor ds18B20
uint8_t numOfSensors = 3;
uint8_t snPtr = 0;
int  led = D7;
char szInfo[64];
float pubTemp;

double boilerTemp = 0.0;
double dhwTemp = 0.0;
double oaTemp = 0.0;

double lastBoilerTemp = 0.0;
double lastDhwTemp = 0.0;
double lastOaTemp = 0.0;
String boilerTempStr = "";
String dhwTempStr = "";
String oaTempStr = "";

const int MAXRETRY = 3;
const int pinOneWire  = D2;
const int LED_RED_PIN = D3;
const int LED_GREEN_PIN  = D4;
const int LED_BLUE_PIN   = D5;
const int pinLED = D7;
const int nSENSORS = 3;
const int STATUS_RED = 1;
const int STATUS_GREEN = 2;
const int STATUS_BLUE = 3;
const int LED_OFF = 0;
const int LED_ON = 1;
const int LED_BLINK = 2;

//Need to determine ID order of Sensors based on addresses, may need to be out of order!
const int BOILER_SENSOR_ID = 0;
const int DHW_SENSOR_ID = 1;
const int OA_SENSORID  = 2;
String sensorNames[nSENSORS] = {"Boiler", "Demand HW", "Outside Air"};
uint8_t sensorAddresses[nSENSORS][8];

DS18B20 ds18b20(pinOneWire);

uint8_t DEG_F = 0;
uint8_t DEG_C = 1;
#define TEMPSCALE DEG_F    //temperature scale DEF_F or DEG_C
float celsius[nSENSORS] = {NAN, NAN};

// Sensor Type
#define SENTYPE DS18B20    	//DS18B20 single wire temperature probe
// Pin 1: Red: ()+3.3V -  5V)
// Pin 2: Yellow (Signal Output)
// Pin 3: Black (Ground)

enum ReturnStatus{
  RET_OK = 0,
  RET_ERR_RANGE = -1,
  RET_UNDEFINED = -2,
};

enum SenRetStates{
  SEN_RET_OK = 0,
  SEN_RET_ERR = -1,
  SEN_RET_UNDERRANGE = -2,
  SEN_RET_OVERRANGE = -3,
};

const int SEN_VAL_UNKNOWN = -99;
String SenStateDesc[] = {"Undefined", "OK", "Not Defined", "Below Minimum", "Above Maximum", "UnderRange", "OverRange"};

enum SenState{
    SEN_UNDEFINED  = 0,
    SEN_OK,
    SEN_ERR_ND,
    SEN_ERR_BELOW_MIN,
    SEN_ERR_ABOVE_MAX,
    SEN_ERR_UNDERRANGE,
    SEN_ERR_OVERRANGE
};

//
//Logamatic 2107 - Furnace controller manufactured by Bosch for the Buderus G117 95KBTU output furnace
//
const int LOGAMATIC_SENS = 3;
struct Logamatic
{
  String name = "undef";
  int  adcPin = 0;
  int tempDegF = 0;
  int tempDegC = 0;
  int minTemp =  0;
  int maxTemp =  0;
  String statusDesc = " ";
  int status = SEN_UNDEFINED;
};

//Temperature Sensor Inputs to Photon ADC pins
int analogPin_OA     = A0;    //Outside Air input to Logamatic. (10K ohm sensor) Read with div-by-2 voltage divider for ADC limit 3.3V
int analogPin_BLR    = A1;    //Boiler input to Logamatic 2107. Read with div-by-2 voltage divider for ADC limit 3.3V
int analogPin_DHW    = A2;    //Demand Hot Water (DHW) input to Logamatic 2107. Read with div-by-2 voltage divider for ADC limit 3.3V

enum LOGAMATIC{
    L_OA,          //Logamatic Outside Air sensor
    L_BLR,         //Logamatic Boiler sensor
    L_DHW          //Logamatic Demand Hot Water sensor
};

Logamatic  LM[LOGAMATIC_SENS] = {
    "Outside Air",      analogPin_OA,  -99, -99, -40, 120, "Undefined", false,
    "Boiler",           analogPin_BLR, -99, -99, 55,  225, "Undefined", false,
    "Demand Hot Water", analogPin_DHW, -99, -99, 55,  145, "Undefined", false
};

//Sampling Rate
//uint32_t Temp_Sample_Period = 60 * 1000;          //60 secs (60,000 milliseconds) between sensor samples
uint32_t Temp_Sample_Period = 10 * 1000;            //10 secs (10,000 milliseconds) between sensor samples
uint32_t TempSampleTime;

const int MS_PER_MIN = 60000;
const int MIN_PUB_MINUTES  =  1;
const int MAX_PUB_MINUTES  =  60;


//Publishing Rate
//unsigned int Temps_Publish_Period = 60000;          //60 seconds
unsigned int Temps_Publish_Period = 60000 * 5;        //5 minutes
unsigned int Temps_NextPublishTime;

//unsigned int Stats_Publish_Period = 60000;          //60 seconds
unsigned int Stats_Publish_Period = 60000 * 5;        //5 minutes
unsigned int Stats_NextPublishTime;
int pubStatsInMins = Stats_Publish_Period/MS_PER_MIN;

bool firstPublish = false;

//Function Declarations
void publishData(int);
void movingAve(int16_t, int16_t, int16_t, int16_t, int16_t, int16_t);
void chkAccGyrMxMn(int16_t, int16_t, int16_t);
void publishFurnaceStats();
void publishTemperatureData(int);

bool Stats_Publish();             //Checks if it is time to publish statistic
bool Temps_Publish();             //Checks if it is time to publish temperatures
bool Temps_Sample();              //Checks if it is time to sample temperature sensors

//Utility Functions
void chkAccAlarms(int, int, int);
void detectPumpState();
void resetX();
void resetY();
void resetZ();
void updateOnTime();
void statusLED(int, bool, int);
int  setPublishTimeInMins(String);

int readLogamaticTemperatures();
int getTempFromVolts(double);
int checkTempFromVolts(String);

//External functions
void wireScan();

//Timers
Timer timer1(minDwellOn, detectPumpState, false); //determine pump state
Timer timerX(minDwellOff, resetX, true);          // resettable one-shot pulse for X
Timer timerY(minDwellOff, resetY, true);          // resettable one-shot pulse for Y
Timer timerZ(minDwellOff, resetZ, true);          // resettable one-shot pulse for Z
Timer timer2(UPDATE_RATE, updateOnTime, false);   // If pump is ON add time to counter

int wifi_RSSI = 0;    //WiFi Signal Strength

//********************************
//**********  SETUP  *************
//********************************
void setup() {
  #if defined(DEBUG_BUILD)
  BLE.off();
  #endif
  systemRestarts++;                     //indicates number of system restarts (retained)
  //LED Status annunciators
  pinMode(LED_RED_PIN, OUTPUT);         //error status
  pinMode(LED_GREEN_PIN, OUTPUT);       //normal operating condition
  pinMode(LED_BLUE_PIN, OUTPUT);        //pump on

  digitalWrite(LED_RED_PIN, 1);        //Test Red LED
  delay(1000);
  digitalWrite(LED_RED_PIN, 0);
  digitalWrite(LED_GREEN_PIN, 1);      //Test Green LED
  delay(1000);
  digitalWrite(LED_GREEN_PIN, 0);
  digitalWrite(LED_BLUE_PIN, 1);       //Test Blue LED
  delay(1000);
  digitalWrite(LED_BLUE_PIN, 0);

  Serial.begin(115200);
  Serial.print("Furnace Monitor\n");
  Serial.printf("OS Firmware Version: %s\n", photonOSverStr.c_str());   //Serial.println(photonOSverStr);
  Serial.printf("App Version: %s\n", appVerStr.c_str());               //Serial.println(appVerStr);

  // Set time zone to Eastern USA daylight saving time
  Time.zone(-4);
  #ifdef  DEBUG
  Serial.print("The time is: ");
  Serial.println(Time.timeStr(Time.now()));
  #endif
  //ACCELROMETER - MPU6050 Initialization
  // pinMode(ledPin, OUTPUT);

  //Disabled Wirescan of I2C
  // Wire.begin();
  // wireScan();   //Search for I2C devices

  // Certify MPU6050 connection
  // accelgyro.initialize();
  // #ifdef  DEBUG
  // Serial.println("Testing MPU6050 connections...");
  // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful!" : "MPU6050 connection failed");
  // #endif

  // //initialize acc offset and sums
  // axOffset = 0;
  // ayOffset = 0;
  // azOffset = 0;
  // axSum = 0;
  // aySum = 0;
  // azSum = 0;

  //Define Particle functions and variables
  Particle.function("RST_Date", resetSysOn_Date);
  // Particle.function("RST_AccMxMn", resetAccMaxMin);
  // Particle.function("RST_AccCalib", resetAccCalibration);
  Particle.function("RST_DS18b20", resetDS18b20_Search);
  Particle.function("Diag_Test", diagTestFcn);
  Particle.function("Set_Gals", setGallonsRemain);
  Particle.function("Set_TotHrs", setFurnaceHours);
  Particle.function("Check_VoltsToTemp", checkTempFromVolts);
  Particle.function("Set_Stats_Pub_Mins", setPublishTimeInMins);


  Particle.variable("SysRestarts", systemRestarts);
  Particle.variable("SysStartDate", sysOnDateStr);
  Particle.variable("Pump_State", pumpState);
  Particle.variable("Pump_Hours", pumpOnHMSstr);
  Particle.variable("Fuel_Used", gallonsUsedStr);
  // Particle.variable("Acc_X_Min", axMn);
  // Particle.variable("Acc_X_Max", axMx);
  // Particle.variable("Acc_Y_Min", ayMn);
  // Particle.variable("Acc_Y_Max", ayMx);
  // Particle.variable("Acc_Z_Min", azMn);
  // Particle.variable("Acc_Z_Max", azMx);
  Particle.variable("Alarm_X_Cnt", axAlrmCnt);
  Particle.variable("Alarm_Y_Cnt", ayAlrmCnt);
  Particle.variable("Alarm_Z_Cnt", azAlrmCnt);
  Particle.variable("WiFi_Signal", wifiRSSI);
  Particle.variable("OS_Version", photonOSverStr);
  Particle.variable("App_Version", appVerStr);
  Particle.variable("Gals_Remain", gallonsRemainingStr);
  Particle.variable("OA_Temp",  oaTempStr);
  Particle.variable("BLR_Temp", boilerTempStr);
  Particle.variable("DHW_Temp", dhwTempStr);
  Particle.variable("Stats_Publish_Mins", pubStatsInMins);

  //1820B Temperature Sensors
  // pinMode(D2, INPUT); //Boiler (B)
  // pinMode(D3, INPUT); //Domestic Hot Water (DHW)
  // pinMode(D4, INPUT); //Outside Air (OA)

//TEMPERATURE - DS18b20

  // ds18b20.resetsearch();
  // for (int i = 0; i < nSENSORS; i++) {   // try to read the sensor addresses
  //   ds18b20.search(sensorAddresses[i]);  // and if available store
  //   #ifdef  DEBUG
  //     Serial.print("Found sensor "); Serial.print(i); Serial.print(": ");
  //   #endif
  //   for (int j = 0; j < 8; j++) {
  //     Serial.print(String(sensorAddresses[i][j]));
  //   }
  //   Serial.println();
  // }

  #ifdef  DEBUG
    Serial.print("Pump State is: "); Serial.println(pumpState);
  #endif
  timer1.start(); //Callback detectPumpState determines pump state
  timer2.start(); //Callback updateOnTime() accumulates time while pump state is ON
  sysStartDate = Time.now();
  sysOnDateStr = Time.timeStr(sysStartDate);
  #ifdef  DEBUG
    Serial.print("Pump Start Date: "); Serial.println(Time.timeStr(sysStartDate));
  #endif

  digitalWrite(LED_BLUE_PIN, 0);      //Turn off blue
  digitalWrite(LED_GREEN_PIN, 1);     //Setup complete - normal operation

  //Initialize data publish periods
  Stats_NextPublishTime = millis() + Stats_Publish_Period;
  Temps_NextPublishTime = millis() + Temps_Publish_Period;
  TempSampleTime        = millis() + Temp_Sample_Period;
} //setup


//*******************************
//**********  LOOP  *************
//*******************************
void loop() {
 // static uint32_t msSample = 0;
  //WiFi signal strength
  wifi_RSSI = WiFi.RSSI();

  // Read furnace, DHW and Outside Air temperatures and store
  if (Temps_Sample() == true){
 //   msSample = millis();

      time_t time = Time.now();
      String strFormat = "%H:%M:%S"; //"%Y-%m-%d-%H-%M-%S";
      Time.setFormat(strFormat);
      time = Time.now();

      Serial.printf("[%s] LOOP: Temps_Sample \n", Time.format(time).c_str());

      oaTemp = LM[L_OA].tempDegF;
      boilerTemp = LM[L_BLR].tempDegF;
      dhwTemp = LM[L_DHW].tempDegF;

      oaTempStr = String(LM[L_OA].tempDegF);
      boilerTempStr = String(LM[L_BLR].tempDegF).c_str();
      dhwTempStr = String(LM[L_DHW].tempDegF).c_str();

      // /* Serial.print("Sensor: "); Serial.print(i); Serial.print(" - " ); Serial.print(sensorNames[i] + " "); */
      if (readLogamaticTemperatures() == SEN_RET_OK){
        for (int i = 0; i < nSENSORS; i++) {
          sprintf(szInfo, "%d", LM[i].tempDegF);
          switch (i){
            case 0: oaTempStr = szInfo; break;
            case 1: boilerTempStr = szInfo; break;
            case 2: dhwTempStr = szInfo; break;
          }
        }
      }
      else {
        //ERROR - Add logging code here
      }


  }

  // static uint32_t msPublish = 0;
  //   Read raw accel/gyro measurements from device
  //   accelgyro.getAcceleration(&ax, &ay, &az);
  //   accelgyro.getRotation(&gx, &gy, &gz);
  //   movingAve(ax, ay, az, gx, gy, gz);

  // For debug
  // if (MA_done && firstAveDone){
  //   Serial.print("ax: "); Serial.print(ax); Serial.print("  ax-axAv: "); Serial.println(ax-axAv);
  //   Serial.print("ay: "); Serial.print(ay); Serial.print("  ay-ayAv: "); Serial.println(ay-ayAv);
  //   Serial.print("az: "); Serial.print(az); Serial.print("  az-azAv: "); Serial.println(az-azAv);
  // }

  // if (firstAveDone == true){
  //   chkAccAlarms(ax, ay, az);
  //   chkAccGyrMxMn(axAbs, ayAbs, azAbs);
  //   /* statusLED(STATUS_RED, LED_BLINK, 100); */
  // }else{
  //   #ifdef  DEBUG
  //     Serial.print("Calculating first average offset: "); Serial.println(10 - FA_Offset_cntr);
  //   #endif
  // }


  if (Stats_Publish() == true){
    #ifdef  DEBUG
      Serial.println("Publishing Furnace data...");
    #endif
    publishFurnaceStats();
  }

  if (Temps_Publish() == true){
    #ifdef DEBUG
      Serial.println("Publishing Temperature Data...");
    #endif
    // publishTemperatureData(snPtr);
  }

  // System.sleep(20);
} //Loop

// //***********************
// //Particle Functions
// //***********************
// //Clear MPU6050 accelerometer max/min values
// int resetAccMaxMin(String extra){
//   if (extra.compareTo("1") == 0){
//     Serial.println("Rec'd '1' in resetAccMaxMin()");
//     axMx = 0, axMn = 0, ayMx = 0, ayMn = 0, azMx = 0, azMn = 0;
//     return 1;
//   }
//   return -1;
// }


// //Reset Acc calibration readings for offset
// int resetAccCalibration(String extra){
//   if (extra.compareTo("1") == 0){
//     Serial.println("Rec'd '1' in resetAccCalibration()");
//     firstAveDone = false;
//     FA_Offset_cntr =  FA_OFFSET_NUM;
//   }
//   if (extra.compareTo("2") == 0){
//     resetAccMaxMin("1");
//   }
//   return 1;
// }


// Temps_Publish - Check if it is time to publish temperatures
// Resets time counter Temps_NextPublishTime
// Returns - true if time to publish and false otherwise
bool Temps_Publish(){
    if (millis() > Temps_NextPublishTime){
      Temps_NextPublishTime = millis() + Temps_Publish_Period;
      return true;   //time to publish temperatures
    }
  return false;
}

// Stats_Publish - Check if it is time to publish statistics
// Resets time counter Stats_NextPublishTime
// Returns - true if time to publish and false otherwise
bool Stats_Publish(){
    if (millis() > Stats_NextPublishTime){
      Stats_NextPublishTime = millis() +  Stats_Publish_Period;
      return true;   //time to publish Stats
    }
  return false;
}

// Temps_Sample - Check if it is time to sample temp sensors
// Resets time counter TempSampleTime
// Returns - true if time to sample and false otherwise
bool Temps_Sample(){
    if (millis() > TempSampleTime){
      TempSampleTime = millis() +  Temp_Sample_Period;
      return true;   //time to sample Temperatures
    }
  return false;
}


// resetSysOn_Date - Reset the System Turn ON Date
// returns 1: OK, -1: error
int resetSysOn_Date(String extra){
  if (extra.compareTo("1") == 0){
    Serial.println("Rec'd '1' in resetSysOn_Date()");
    //Clear counters
    PumpOnTime_Ctr   = 0;
    pumpOnTime_Hours = 0;
    pumpOnTime_Mins  = 0;
    pumpOnTime_Secs  = 0;
    pumpOnHMSstr = "";
    gallonsUsed = 0; gallonsUsedStr = "";   //Retained variable
    axAlrmCnt = 0; ayAlrmCnt = 0; azAlrmCnt = 0;

    //Reset system start date
    sysStartDate = Time.now();             //Retained variable
    sysOnDateStr = Time.timeStr(sysStartDate);
    systemRestarts = 1;                      //Retained variable
    Serial.print("sysStartDate: "); Serial.println(Time.timeStr(sysStartDate));
    return 1;
  }
  return -1;
}

//Reset the I2C bus
int resetDS18b20_Search(String extra){
  if (extra.compareTo("1") == 0){
    Serial.println("Rec'd '1' in resetDS18b20_Search()");
    ds18b20.resetsearch();
    for (int i = 0; i < nSENSORS; i++) {   // try to read the sensor addresses
      ds18b20.search(sensorAddresses[i]);  // and if available store
      Serial.print("Found sensor "); Serial.print(i); Serial.print(": ");
      for (int j = 0; j < 8; j++) {
        Serial.printf("%2.2X:", sensorAddresses[i][j]);
      }
      Serial.println();
    }
  }
  return 1;
}

//Increment the Pump Ctr for testing purposes
int diagTestFcn(String extra){
  if (extra.compareTo("1") == 0){
    Serial.println("Rec'd '1' in diagTestFcn()");
    //Increment pump time counter
    ++PumpOnTime_Ctr;
  }
  if (extra.compareTo("2") == 0){
    Serial.println("Rec'd '2' in diagTestFcn()");
    System.reset();
  }
  return 1;
}

int setGallonsRemain(String extra){
  if (extra.toFloat() != 0){
    Serial.println("Reset Gallons in setGallonsRemain()");
    gallonsInTankRef = extra.toFloat();
    sprintf(szInfo, "%2.2f", gallonsInTankRef); gallonsRemainingStr = szInfo;
  } else {
    Serial.println("Err Code 100: Illegal value for gallonsInTankRef");
  }
  return 1;
}

int setFurnaceHours(String extra){
  if (extra.toInt() != 0){
    Serial.println("Set furnace hours in setFurnaceHours()");
    totalFurnaceHours = extra.toInt();
  } else {
    Serial.println("Err Code 200: Illegal value for totalFurnaceHours");
  }
  return 1;
}


// //MPU 6050 Display LED
// int ledPin = D7;
// bool ledState = false;
// void toggleD7Led() {
//     ledState = !ledState;
//     digitalWrite(ledPin, ledState);
// }


void statusLED(int led, bool ledState, int dly){

  if (ledState == LED_ON || ledState == LED_OFF) {
    digitalWrite(led, ledState);
    delay(dly);
    digitalWrite(led, !ledState);
  }

  // if (ledState == LED_BLINK) {
  //   ledState = !ledState;
  //   digitalWrite(ledPin, ledState);
  // }
}//statusLED


// readTempsLogamatic() - Reads temperatures directly from Logamatic 2107 thermocouple inputs.
// There are currently 3 sensors, 2 are BOSCH OEM sensors for the B oiler and Demand Hot Water DHW.
// The 3rd sensor is a 10K ohm Outside Air sensor with a different temperature profile curve. See project documention
// for Logamatic 2107 sensor Volts-to-Temperature profile in documentation folder.
// The sensors voltages are read from the Logamatic 2107 through Photon 12-bit adc inputs, and converted to degrees Fahrenheit.
// Logamatic data structure LM is used to define parameters and store values.
int readLogamaticTemperatures() {
int32_t adcRdg       = 0;
double sensorVoltRdg = 0.0;
int sensorDegF    = 0;         //Only need to read temperature values with integer precision

time_t time = Time.now();
time = Time.now();
Serial.printf("[%s] readLogamaticTemperatures: \n", Time.format(time).c_str());

//Need to read ADC voltage from the 3 sensors. Voltage may get as high as 5V+, so need divider network
//Will use 2 - 10K resistors connected from + terminal to 1 terminal to divide voltage by 2
//So max voltage should be less than the max Particle ADC voltage of 3.3V. Verify there is no effect on the Logamatic reading

//Convert raw adc reading to a voltage. Since the adc reading is half the actual value (voltage divider)
//we must multiply by 2, and find the closest voltage in the lookup table Volts_to_DegF[] to get the
//corresponding temperature.

// Example:
// Sensor signal from the boiler is 4.0 Volts. What temperature does this correspond to?

// We divide this signal by two with resistor divider network so actual voltage to ADC input is 2.0 Volts
// because we don't want to blow up the ADC of the Photon which has internal reference of 3.3V.

// This results in an ADC reading: adcRdg = (2.0/3.3) * 4096 =  2482
// To get the actual voltage reading of the sensor we must:
// sensorValInVolts = adcRdg * ADC_REF_V * 2
//                  = 2482/4096 * 3.3 * 2 = 3.999164 (CLose enough)
//Convert this voltage to an actual temperature

  //Put these in the FurnaceMonitor.H file
  const int32_t ADC_12_BITS   = 4096;
  const double  ADC_REF_V     = 3.3;
  const uint    VDIVIDER_MULT = 2;

  for (int senIdx = L_BLR; senIdx <= L_DHW; senIdx++){
    // Read the adc for the Logamatic Boiler and DHW temperature sensors
    // Note: The Outside Air sensor uses a different sensor probe which has its own conversion function.
    // May have a selectable sensor LUT in the future (Feature Enhancement)

    adcRdg = analogRead(LM[senIdx].adcPin);
    sensorVoltRdg = double((double)adcRdg/(double)ADC_12_BITS * ADC_REF_V * (double)VDIVIDER_MULT);

#ifdef DEBUG
 static int tvIdx = 0;
  //Read from TestVolts table to verify operation of getTempFromVolts();
    sensorVoltRdg = TestVolts[tvIdx];
    Serial.printf("[%s] rdLogTemps:(TestVolts) tvIdx: %d  sensorVoltRdg: %0.4f\n", Time.format(time).c_str(), (int)tvIdx, sensorVoltRdg);
    tvIdx++;
    if (tvIdx >= int(sizeof(TestVolts)/sizeof(TestVolts[0])-1)){
      tvIdx = 0;
    }
#endif


    Serial.printf("[%s] rdLogTemps: adcRdg: %d  sensorVoltRdg: %0.4f\n", Time.format(time).c_str(), (int)adcRdg, sensorVoltRdg);

    sensorDegF = getTempFromVolts(sensorVoltRdg);       //Find the closest voltage and returm temperature in the LUT in LM

    //Test return for errors and limits
    if (sensorDegF == SEN_RET_UNDERRANGE) {             //Below LUT Minimum Voltage
      LM[senIdx].status = SEN_ERR_UNDERRANGE; LM[senIdx].statusDesc = SenStateDesc[SEN_ERR_UNDERRANGE];
      LM[senIdx].tempDegF = SEN_VAL_UNKNOWN;
    }
    else if(sensorDegF == SEN_RET_OVERRANGE) {          //Above LUT Maximum Voltage
      LM[senIdx].status = SEN_ERR_OVERRANGE; LM[senIdx].statusDesc = SenStateDesc[SEN_ERR_OVERRANGE];
      LM[senIdx].tempDegF = SEN_VAL_UNKNOWN;
    }
    else if(sensorDegF == SEN_RET_ERR) {                //Return Error - Unspecified
      LM[senIdx].status = SEN_ERR_ND; LM[senIdx].statusDesc = SenStateDesc[SEN_ERR_ND];
      LM[senIdx].tempDegF = SEN_VAL_UNKNOWN;
    }
    else if (sensorDegF < LM[senIdx].minTemp) {     //Below Minimum Temp
      LM[senIdx].status = SEN_ERR_BELOW_MIN; LM[senIdx].statusDesc = SenStateDesc[SEN_ERR_BELOW_MIN];
      LM[senIdx].tempDegF = sensorDegF;
    }
    else if(sensorDegF > LM[senIdx].maxTemp) {      //Above Maximum Temp
      LM[senIdx].status = SEN_ERR_ABOVE_MAX; LM[senIdx].statusDesc = SenStateDesc[SEN_ERR_ABOVE_MAX];
      LM[senIdx].tempDegF = sensorDegF;
    }
    else {                                          //OK
        LM[senIdx].status = SEN_OK; LM[senIdx].statusDesc = SenStateDesc[SEN_OK];
        LM[senIdx].tempDegF = sensorDegF;
    }
    Serial.printf("[%s] rdLogTemps: SenName: %s  Status: %s  tempDegF: %d  minTemp: %d  maxTemp: %d \n",
                 Time.format(time).c_str(), LM[senIdx].name.c_str(), LM[senIdx].statusDesc.c_str(), LM[senIdx].tempDegF, LM[senIdx].minTemp, LM[senIdx].maxTemp);
    Serial.println();
  }
  return SEN_RET_OK;
}//readLogamaticTemperatures()

// readTemperature Old function that read ds18b20 sensors
// double readTemperature(uint8_t addr[8]) {
//   double _temp;
//   int   i = 0;

//   do {
//     _temp = ds18b20.getTemperature(addr);
//   } while (!ds18b20.crcCheck() && MAXRETRY > i++);

//   if (i < MAXRETRY) {
//     _temp = ds18b20.convertToFahrenheit(_temp);
//     // Serial.println(_temp);
//   }
//   else {
//     _temp = NAN;
//     #ifdef  DEBUG
//       Serial.println("Invalid reading");
//     #endif
//   }
//   return _temp;
// }

//Algorithm for detecting xyz thresholded detection for Oil Pump ON state
//Rules:
// X or Y or Z sets independent on-shot timers ON--->OFF
// If X, Y, or Z detected again, on-shot is reset
// After first detection of alarm... set detectTest = true; set waitTimer
// Wait "minimum on time before valid "detect";


// getTempFromVolts() - Search the TempVolts array to find closest entry rdgVolts to voltage in table
// Passes sensor reading rdg in volts
// Returns a temperature integer DegF. Integer precision to 1 degree is sufficient for this application
// If error, returns Error Codes
int getTempFromVolts(double rdgVolts) {
  int rdgDegF = 0;
  int rangeErr = 0;
  time_t time = Time.now();
  time = Time.now();
  Serial.printf("[%s] getTempFromVolts: rdgVolts: %0.4f\n", Time.format(time).c_str(), rdgVolts);

  //We have the voltage reading and want the closest value and return the corresponding temperature
  // from the Volt_to_DegF LUT array.

  //Strategy 1
  //Use a binary search on an array of SizeArray(Volts_to_DegF)
  // In this case the array has 194 entries of elements VoltDegF(volts, degF)

  //Get the middle of the array and compare the reading with the LUT value for volts.
  //Middle of array equals Size of Array(Volts_to_DegF) / 2 = 72
  // size
  //Assume a sensor voltage rdg of 2.7942 Volts
  //The value of array Volts_to_DegF[72].volts = 1.8383
  //Since the rdg is > the array value, we must search lower in the LUT, so divide
  //If the reading is > than LUT, then try lower. Read the middle of the

  uint dimSz = sizeof Volts_to_DegF/sizeof Volts_to_DegF[0];
  //Maximum length of a binary search should be Log2(n) i.e, if n = 256, then Log2(256) = 8.
  //In our case the size of the LUT is 144 entries so Log2(144) = 7.17 (round down to 7?) Check and see

  //Test: Assume the actual dimension size of the LUT was 256, in our example.
  //      Also assume the rdgVolts = The following comments are useful.
  //      So the initial lutIdx would be 128 (1st comparison)

//Assume LUT size = 144, then.....
//        Initial lutIdx = 144/2 = 72
//        rdgVolts = 1.2850 Volts (which would be between entries [95] 1.3048V, 172F and [96] 1.2840V, 173F
//  const double RDG_RESL_ERR = 0.03;  //0.03;    //Difference between lastLutHighV and lastLutLowV less than
  //const double RDG_RESL_ERR = 0.0266;    //Difference between lastLutHighV and lastLutLowV less than resolution necessary to resolve between voltage entries in the volts-to-temperature table
  //2/8/22 - Choose LUT entry 120 degF, 2.540717 Volts (delta 0.0266V) because don't care about precision of the lower boler temperatures. Need to verify

//Check Out of Range conditions (voltage reading below or above min/max LookUp Table
  if (rdgVolts < Volts_to_DegF[dimSz-1].volts){
    rangeErr = SEN_RET_UNDERRANGE;    //rdg voltage was lower than last entry LUT - under volt (range)
  }
  else if (rdgVolts > Volts_to_DegF[0].volts){
    rangeErr = SEN_RET_OVERRANGE;     //rdg voltage was higher than first entry LUT - over volt (range)
  }
  if (rangeErr < SEN_RET_OK){
    Serial.printf("[%s] getTempFromVolts: RETURN rangeErr: %d\n", Time.format(time).c_str(), rangeErr);
    return rangeErr;
  }

  int maxCnt = (int)log2(dimSz);
  int lutIdx = dimSz/2;       //Look Up Table Index should start search midway (dimSz/2)
  double lastLutHighV = 0.0;
  double lastLutLowV = 0.0;
  int deltaIdx = dimSz/4;
  int upperIdx = 0;
  int lowerIdx = 0;
  int dirSign = 0;

  for (int cnt = 0; cnt < maxCnt; cnt++){
    double lutVolts = Volts_to_DegF[lutIdx].volts;

    if (rdgVolts > lutVolts){                           //increase idx (decrease LUT voltage)
      upperIdx = lutIdx;
      dirSign = -1;                                     //search direction -1 : decreasing index, (increasing LUT voltage))
    }
    else  {                                             //idecrease idx (increase LUT voltage)
      lastLutLowV = lutVolts;
      lowerIdx = lutIdx;
      dirSign = +1;                                     //search direction +1 : increasing index, (decreasing LUT voltage))
    }

    Serial.printf("[%s] getTmpFrmV: cnt: %d   upperIdx: %d   lowerIdx: %d   deltaIdx: %d   lutIdx: %d   tmpV: %0.4f   degF: %d\n", Time.format(time).c_str(), cnt, upperIdx, lowerIdx, deltaIdx, lutIdx, lutVolts, Volts_to_DegF[lutIdx].degF);

    Serial.printf("[%s] getTmpFrmV: lastLutHighV: %0.4f  lastLutLowV: %0.4f\n", Time.format(time).c_str(), lastLutHighV, lastLutLowV);
    if (deltaIdx == 0){      //Select the closest reading
        double diffV_P1 = 0.0;
        double diffV_M1 = 0.0;
        double diffV_Lidx;

        diffV_P1 = abs(abs(Volts_to_DegF[lutIdx].volts - rdgVolts) - abs(Volts_to_DegF[lutIdx+1].volts - rdgVolts));
        diffV_M1 = abs(abs(Volts_to_DegF[lutIdx].volts - rdgVolts) - abs(Volts_to_DegF[lutIdx-1].volts - rdgVolts));

        diffV_Lidx = abs(Volts_to_DegF[lutIdx].volts - rdgVolts);
        diffV_P1 = abs(Volts_to_DegF[lutIdx+1].volts - rdgVolts);
        diffV_M1 = abs(Volts_to_DegF[lutIdx-1].volts - rdgVolts);
        // minVal =  std::min(diffV_P1, diffV_M1, diffV_Lidx);
        Serial.printf("[%s] getTmpFrmV:  difLidx: %4f  diffV_P1 %4f  diffV_M1 %4f \n", Time.format(time).c_str(), diffV_Lidx, diffV_P1, diffV_M1);

        if (dirSign == 1){
          if (abs(Volts_to_DegF[lutIdx].volts - rdgVolts) < abs(Volts_to_DegF[lutIdx+1].volts - rdgVolts) ){
            Serial.printf("[%s] getTmpFrmV: dirSign: %d  lutIdx: %d   Ret Idx%d\n", Time.format(time).c_str(), dirSign, lutIdx, lutIdx);
            return Volts_to_DegF[lutIdx].degF;                                                                //lutIdx diff < lutIdx+1 diff : ret lutIdx rdg
          }
          else  {
            Serial.printf("[%s] getTmpFrmV: dirSign: %d  lutIdx: %d   Ret Idx%d\n", Time.format(time).c_str(), dirSign, lutIdx, lutIdx + 1);
            return Volts_to_DegF[lutIdx+1].degF;                                                              //lutIdx diff > lutIdx+1 diff : ret lutIdx+1 rdg
          }
        }
        else  { //dirSign == -1
          if (abs(Volts_to_DegF[lutIdx].volts - rdgVolts) < abs(Volts_to_DegF[lutIdx-1].volts - rdgVolts)){
              Serial.printf("[%s] getTmpFrmV: dirSign: %d  lutIdx: %d   Ret Idx%d\n", Time.format(time).c_str(), dirSign, lutIdx, lutIdx);
              return Volts_to_DegF[lutIdx].degF;                                                                //lutIdx diff < lutIdx-1 diff : ret lutIdx rdg
            }
            else  {                                                                                             //lutIdx diff > lutIdx+1 diff : ret lutIdx+1 rdg
              Serial.printf("[%s] getTmpFrmV: dirSign: %d  lutIdx: %d   Ret Idx%d\n", Time.format(time).c_str(), dirSign, lutIdx, lutIdx - 1);
              return Volts_to_DegF[lutIdx+1].degF;
            }
          }
     }//deltaIdx


    lutIdx = lutIdx + (deltaIdx * dirSign);
    deltaIdx = deltaIdx/2;
  } //for cnt

  Serial.printf("[%s] getTmpFrmV: UNDEFINED ERROR rdgDegF: %d\n", Time.format(time).c_str(), rdgDegF);
  return SEN_RET_ERR;       //Return error
} //getTempFromVolts()


void updateOnTime(){
  int remainder = 0;

  if (pumpState == true){
    PumpOnTime_Ctr += 1;
  }
  //Convert to hours:minutes:seconds
  pumpOnTime_Hours = PumpOnTime_Ctr/3600;
  remainder = PumpOnTime_Ctr % 3600;
  pumpOnTime_Mins = remainder/60;
  pumpOnTime_Secs = remainder % 60;
  pumpOnHMSstr = String("HMS ") +
              String(pumpOnTime_Hours)  + String(":") +
              String(pumpOnTime_Mins) + String(":") +
              String(pumpOnTime_Secs);
//YYY
  gallonsUsed = flowRatePerMin * ((pumpOnTime_Hours * 60) + pumpOnTime_Mins);  //Note: round to nearest minute
  gallonsRemaining = gallonsInTankRef - gallonsUsed;
  sprintf(szInfo, "%2.4f", gallonsUsed); gallonsUsedStr = szInfo;
  sprintf(szInfo, "%2.4f", gallonsRemaining); gallonsRemainingStr =  szInfo;
  sprintf(szInfo, "%u", totalFurnaceHours + pumpOnTime_Hours); totalFurnaceHoursStr = szInfo;

  wifiRSSI = WiFi.RSSI();
} //updateOnTime

void detectPumpState(){
  //Check if any XYZ threshold is still true
  // Serial.println("detectPumpState() callback");

  /* if ((xThreshCnt > MIN_DETECTION_CNT) ||
      (yThreshCnt > MIN_DETECTION_CNT) ||
      (yThreshCnt > MIN_DETECTION_CNT)){
        pumpState = true;
        Serial.println("pumpState = true;");
        xThreshAlarm = yThreshAlarm = zThreshAlarm = false;
  } else
      //Check if ALL XYZ thresholds are false
      if ((xThreshAlarm && yThreshAlarm && zThreshAlarm) == false){
        //Pump state is now ON after minimum dwell period
        pumpState = false;
        Serial.println("pumpState = false;");
      }
  } */

  if (xThreshAlarm || yThreshAlarm || zThreshAlarm){
    //Pump state is now ON after minimum dwell period
    pumpState = true;
    #ifdef  DEBUG
      Serial.println("pumpState = true;");
    #endif
    digitalWrite(LED_GREEN_PIN, 0);
    digitalWrite(LED_BLUE_PIN, 1);
    xThreshAlarm = yThreshAlarm = zThreshAlarm = false;
  } else {
    //Check if ALL XYZ thresholds are false
    if ((xThreshAlarm && yThreshAlarm && zThreshAlarm) == false){
      //Pump state is now ON after minimum dwell period
      pumpState = false;
      #ifdef DEBUG
        Serial.println("pumpState = false;");
      #endif
      digitalWrite(LED_GREEN_PIN, 1);
      digitalWrite(LED_BLUE_PIN, 0);
    }
  }
} //detectPumpState()

//Timer callbacks to reset XYZ
void resetX(){
    xThreshAlarm = false;
}
void resetY(){
    yThreshAlarm = false;
}
void resetZ(){
    zThreshAlarm = false;
} //resetX()

// //Check XYZ Acc Thresholds to detect pump motion vibration (i.e. pump ON)
// void  chkAccAlarms(int x, int y, int z){
//   axAbs =  abs(x) - abs(axOffset);
//   ayAbs =  abs(y) - abs(ayOffset);
//   azAbs =  abs(z) - abs(azOffset);

//   if (axAbs > axThrsh){
//     // Serial.print("Alarm for ax: "); Serial.println(axAbs);
//     // Serial.print("ax - axAv: "); Serial.println(x - axAv);
//     xThreshAlarm = true;
//     axAlrmCnt++;
//     timerX.start();
//   }

//   if (ayAbs > ayThrsh){
//     // Serial.print("Alarm for ay: "); Serial.println(ayAbs);
//     // Serial.print("ay - ayAv: "); Serial.println(x - ayAv);
//     yThreshAlarm = true;
//     ayAlrmCnt++;
//     timerY.start();
//   }

//   if (azAbs > azThrsh){
//     // Serial.print("Alarm for az: "); Serial.println(azAbs);
//     // Serial.print("az - azAv: "); Serial.println(x - azAv);
//     zThreshAlarm = true;
//     azAlrmCnt++;
//     timerZ.start();
//   }

//   if (timer1.isActive() == false){
//     if (xThreshAlarm || yThreshAlarm || zThreshAlarm){
//       #ifdef DEBUG
//         Serial.println("Timer1 Started");
//       #endif
//       timer1.start();
//     }
//   }
// } //chkAccAlarms

// void movingAve(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz){
//   if (firstAveDone == false){
//       axSum += ax;
//       aySum += ay;
//       azSum += az;
//       FA_Offset_cntr--;
//       if (FA_Offset_cntr <= 0){
//         axOffset = axSum/FA_OFFSET_NUM;
//         ayOffset = aySum/FA_OFFSET_NUM;
//         azOffset = azSum/FA_OFFSET_NUM;
//         firstAveDone = true;
//         #ifdef DEBUG
//           Serial.print("firstAveDone = true  "); Serial.println(FA_Offset_cntr);
//           Serial.printlnf("Offset Sums: ax: %d\t   ay: %d\t   az: %d", axSum, aySum, azSum);
//           Serial.printlnf("Offset Averages: ax: %d\t   ay: %d\t   az: %d", axOffset, ayOffset, azOffset);
//         #endif
//           axSum = aySum = azSum = 0;
//       }
//   } else {
//       axSum += ax;
//       aySum += ay;
//       azSum += az;
//       MA_cntr--;
//       if (MA_cntr <= 0){
//         MA_done = true;
//         axAv = axSum/MA_NUM;
//         ayAv = aySum/MA_NUM;
//         azAv = azSum/MA_NUM;
//         axSum = aySum = azSum = 0;
//         MA_cntr = MA_NUM;
//       }
//   }
// } //movingAve

// void chkAccGyrMxMn(int16_t absx, int16_t absy, int16_t absz){
//   //Check for new minimums
//   if (absx <= axMn) axMn = absx;
//   if (absy <= ayMn) ayMn = absy;
//   if (absz <= azMn) azMn = absz;
//   // if (gx <= gxMn) gxMn = gx;
//   // if (gy <= gyMn) gyMn = gy;
//   // if (gz <= gzMn) gzMn = gz;

//   //Check for new maximums
//   if (absx >= axMx) axMx = absx;
//   if (absy >= ayMx) ayMx = absy;
//   if (absz >= azMx) azMx = absz;
//   // if (gx >= gxMx) gxMx = gx;
//   // if (gy >= gyMx) gyMx = gy;
//   // if (gz >= gzMx) gzMx = gz;

//   // Serial.println("In chkAccGyrMxMn");
//   // Serial.println("acc       accMax      accMin:");
//   // Serial.print(ax); Serial.print("\t"); Serial.print(axMn); Serial.print("\t"); Serial.println(axMx);
//   // Serial.print(ay); Serial.print("\t"); Serial.print(ayMn); Serial.print("\t"); Serial.println(ayMx);
//   // Serial.print(az); Serial.print("\t"); Serial.print(azMn); Serial.print("\t"); Serial.println(azMx);
// } //chkAccGyrMxMn

void publishFurnaceStats(){
  /* Particle.publish("Pump_State", (pumpState ? "ON" : "OFF"), PRIVATE);
  Particle.publish("Pump_Hours", pumpOnHMSstr, PRIVATE);
  Particle.publish("Fuel_Used", gallonsUsedStr, PRIVATE); */
  // Particle.publish("Fuel_Used", gallonsUsedStr, PRIVATE);
  // Particle.publish("WiFi_RSSI", String(wifiRSSI), PRIVATE);

  // Particle.publish("OA_Temp_Logamatic",  oaTempStr);
  // Particle.publish("BLR_Temp_Logamatic", boilerTempStr);
  // Particle.publish("DHW_Temp_Logamatic", dhwTempStr);

  ubidots.add((char*)"Outside_Air_Temp_Logamatic",   oaTemp);
  ubidots.add((char*)"Boiler_Temp_Logamatic",   boilerTemp);
  ubidots.add((char*)"DHW_Temp_Logamatic",  dhwTemp);
  //ubidots.add((char*)"WiFi_Signal", wifi_RSSI);
  // ubidots.add((char*)"Outside_Temp_OWM", currentOA_Temp);

  if (ubidots.send()){
      Serial.println("Data sent to UBIDOTS");
  }


// // GOOGLE SPREADSHEETS
//   char payload[128] = "";
// float tFHrs = totalFurnaceHours + pumpOnTimeInDecimal;
//   sprintf(payload,"{\"Dev_Id\":%.1d,\"Pump\":%.1d,\"Fuel_Used\":%.2f, \"Fuel_Remain\":%.2f, \"Total_Hours\":%.1f, \"WiFi_RSSI\":%.1d}",
//                   deviceIdNum, pumpState, gallonsUsed, gallonsRemaining, tFHrs, wifiRSSI);
//   Particle.publish("postGoogle", payload, PRIVATE);
// // GOOGLE SPREADSHEETS

  // Stats_NextPublishTime = millis() + Stats_Publish_Period;

  // pumpOnTimeInDecimal = double(pumpOnTime_Hours) + double(pumpOnTime_Mins/60.0) + double(pumpOnTime_Secs/60.0);
  // tFHrs = totalFurnaceHours + pumpOnTimeInDecimal;

  // // char buf1[32];
  // char buf2[32]; char buf3[32]; char buf4[32]; char buf5[32]; char buf6[32];
  String caption = "";

  // //This saves a lot of data, as pump is on only for 10 - 15 minutes and will plot correctly in UBIDOTS
  // caption = "Pump State "; caption.concat(deviceIdName); caption.toCharArray(buf1, sizeof(buf1));
  // if (firstPublish == false){
  //   ubidots.add(buf1, pumpState);
  //   firstPublish = true;
  //   lastPumpState = pumpState;
  // }
  // if (lastPumpState == 0 && pumpState == 1){
  //   if (psTransition == false){
  //     ubidots.add(buf1, 0);
  //     psTransition = true;
  //   }else{
  //     ubidots.add(buf1, 1);
  //     psTransition = false;
  //     lastPumpState = pumpState;
  //   }
  // }else if (lastPumpState == 1 && pumpState == 0){
  //     if (psTransition == false){
  //       ubidots.add(buf1, 1);
  //       psTransition = true;
  //     }else{
  //       ubidots.add(buf1, 0);
  //       psTransition = false;
  //       lastPumpState = pumpState;
  //     }
  //   }

  // //Log data ONLY if gallonsUsed changes by > 0.1 gallons.
  // if ((gallonsUsed - lastGallonsUsed) >= GALLONS_USED_LOG_THRSH){
  //   caption = "Gallons Used "; caption.concat(deviceIdName); caption.toCharArray(buf2, sizeof(buf2));
  //   ubidots.add(buf2, gallonsUsed);
  //   lastGallonsUsed = gallonsUsed;

  //   caption = "Pump Hours "; caption.concat(deviceIdName); caption.toCharArray(buf4, sizeof(buf4));
  //   ubidots.add(buf4, pumpOnTimeInDecimal);
  //   caption = "Total Furnace_Hours "; caption.concat(deviceIdName); caption.toCharArray(buf5, sizeof(buf5));
  //   ubidots.add(buf5, tFHrs);

  //   caption = "Gallons Remaining "; caption.concat(deviceIdName); caption.toCharArray(buf3, sizeof(buf3));
  //   ubidots.add(buf3, gallonsRemaining);
  // }

  // //Log data ONLY if WiFi Signal changes by > 2.0 dB.
  // if (abs((abs(wifiRSSI) - abs(lastWifiRSSI))) >= WIFI_LOG_THRSH){
  //   caption = "WiFi Signal "; caption.concat(deviceIdName); caption.toCharArray(buf6, sizeof(buf6));
  //   ubidots.add(buf6, wifiRSSI);
  //   lastWifiRSSI = wifiRSSI;
  // }

  // P07_FURNACE can have upto 3 additional external 1-wire temperature probes
  // if (strcmp(deviceIdName, "P7") == 0){
  //   char buf7[32]; char buf8[32]; char buf9[32];

  //   //Log data ONLY if Boiler Temp changes by > 1.0 degF.
  //   if (abs(abs(boilerTempStr.toFloat()) - abs(lastBoilerTemp)) >= BOILER_TEMP_LOG_THRSH){
  //     caption = "Boiler Temp "; caption.concat(deviceIdName); caption.toCharArray(buf7, sizeof(buf7));
  //     ubidots.add(buf7, boilerTempStr.toFloat());
  //     lastBoilerTemp = boilerTempStr.toFloat();
  //   }

  //   if (abs(abs(oaTempStr.toFloat()) - abs(lastOaTemp)) >= OA_TEMP_LOG_THRSH){
  //     caption = "Outside Air Temp "; caption.concat(deviceIdName); caption.toCharArray(buf8, sizeof(buf8));
  //     ubidots.add(buf8, oaTempStr.toFloat());
  //     lastOaTemp = oaTempStr.toFloat();
  //   }

  //   if (abs(abs(dhwTempStr.toFloat()) - abs(lastDhwTemp)) >= DHW_TEMP_LOG_THRSH){
  //     caption = "DHW Temp "; caption.concat(deviceIdName); caption.toCharArray(buf9, sizeof(buf9));
  //     ubidots.add(buf9, dhwTempStr.toFloat());
  //     lastDhwTemp = dhwTempStr.toFloat();
  //   }
  // }
    //Chart the following variables from the furnace
    // Outside Air temperature (Logamatic sensor)
    // Boiler temperature (Logamatic sensor)
    // Demand Hot Water temperature (Logamatic sensor)
    // Outside Temperature from OpenWeatherMap API (see https://home.openweathermap.org/)
}

void publishTemperatureData(int sen){
  // Serial.print("Publish Temp Sensor "); Serial.println(sen);
  // Give time for the message to reach ThingSpeak
  //delay(2500);
  //   switch (sen)
  // {
  //   case 0:
  //     // if(!ds18b20.crcCheck())
  //     //   return;
      // sprintf(szInfo, "%2.2f", boilerTemp);
      Particle.publish("Boiler", boilerTempStr, PRIVATE);
      #ifdef DEBUG
        Serial.print("Publish Sensor "); //Serial.print(sen);
        Serial.print(" Boiler: "); Serial.print(boilerTempStr); Serial.println(" degF");
      #endif
    //   break;
    // case 1:
    //   // if(!ds18b20_DHW.crcCheck())
    //   //   return;
      // sprintf(szInfo, "%2.2f", dhwTemp);
      Particle.publish("Hot Water", String(dhwTempStr), PRIVATE);
      #ifdef DEBUG
        Serial.print("Publish Sensor "); //Serial.print(sen);
        Serial.print(" Hot Water: "); Serial.print(dhwTempStr); Serial.println(" degF");
      #endif
    //   break;
    // case 2:
    //   // if(!ds18b20_OA.crcCheck())
    //   //   return;
      // sprintf(szInfo, "%2.2f", oaTemp);
      Particle.publish("Outside Air", String(oaTempStr), PRIVATE);
      #ifdef DEBUG
        Serial.print("Publish Sensor "); //Serial.print(sen);
        Serial.print(" Outside Air: "); Serial.print(oaTempStr); Serial.println(" degF");
      #endif
  //     break;
  // }
  //Particle.publish("dsTmp", szInfo, PRIVATE);
  Temps_NextPublishTime = millis() + Temps_Publish_Period;
}


//**************************************************
//**************************************************
//
//               UTILITY FUNCTIONS
//
//**************************************************
//**************************************************

// checkTempFromVolts() - 

int checkTempFromVolts(String rdgVoltsStr){
      //Check if we are in range of the LUT
      double rdgVolts = rdgVoltsStr.toFloat();         //Array Volts_to_DegF[] stores high to low values; arry[0] --> to arry[dim_size]
      uint dimSz = sizeof Volts_to_DegF/sizeof Volts_to_DegF[0];
      // int retVal = 0;
      if (rdgVolts > Volts_to_DegF[0].volts){          //Overrange 'volts', i.e. data is before first value in LUT
        return SEN_RET_OVERRANGE;
      }
      else if (rdgVolts < Volts_to_DegF[dimSz].volts){ //Underrange 'volts', i.e. data is after last value in LUT
        return SEN_RET_UNDERRANGE;
      }
      //Get closest value in LUT
      return getTempFromVolts(rdgVolts);
}


// setPublishTimeInMins() Pass a String value to set the next stats publishing time for Particle and UBIDOTS
// Returns - OK: 0, RET_ERR_RANGE: -1
int setPublishTimeInMins(String minsToPublish){
  //Check if we are in range
  double mins = minsToPublish.toInt();

  if (mins < MIN_PUB_MINUTES || mins > MAX_PUB_MINUTES){   //Value out of range
    return RET_ERR_RANGE;
  }
  pubStatsInMins = mins;
  Stats_Publish_Period = mins * MS_PER_MIN;
  Stats_NextPublishTime = millis() + Temps_Publish_Period;
  return RET_OK;
}