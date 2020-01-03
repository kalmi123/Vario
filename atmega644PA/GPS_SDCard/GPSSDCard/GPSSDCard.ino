#include <MS5611.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <SD.h>

#define REDLED 20
#define GREENLED 19
#define BUZZER 18
#define BT_RX 10
#define BT_TX 11
#define GPS_RX 0
#define GPS_TX 1

MS5611 SENSOR;

TinyGPSPlus GPS;
static const uint32_t GPSBaud = 9600;
SoftwareSerial GPS_SoftwareSerial(GPS_RX, GPS_TX);

SSD1306AsciiAvrI2c lcdDisplay;

static const uint32_t BTBaud = 9600;
SoftwareSerial BT_SoftwareSerial(BT_RX, BT_TX);

static const uint32_t SerialBaud = 115200;
bool debug = true;

const int SDChipSelect = 4;                        //SD card chip select pin

float new_pressure = 0.0;
float old_pressure = 0.0;
int pressure_count = 0;

float delta_altitude = 0.0;

unsigned long old_millisec = 0;
unsigned long delta_millisec = 0;

unsigned long elevationTestStartedMillisec = 0;
bool elevationTest = false;

float elevation = 0.0;
float old_elevation = 0.0;
float air_temperature = 0.0;

unsigned long currentMillis = 0;
bool new_elevation = false;
bool buzzerActive = false;
int buzz_freq = 0;
int cycle_ms = 0;
unsigned long buzzStartedTime = 0;
unsigned long buzzPausedTime = 0;

float last_estimate = 0.0;
float err_estimate = 0.0;
float err_measure = 0.0;
float current_estimate = 0.0;
float last_current = 0.0;
float kalman_gain = 0.0;
float q = 0.0;

String receivedConfigString = "";

int c = 0;

char receivedChar;

int checksum = 0;
byte messageLength;
String message = "";
String BTmessageToSend = "";

String logString;
File dataFile;

unsigned long gps_LastPrintedSentenceTime = 0;
unsigned long display_UpdatedTime = 0;

int eeAddress = 0;

struct Settings {
  byte mode = 1;
  int volume = 50;
  float climbThreshold = 0.10;
  float sinkThreshold = -0.10;
  int climbFreq[3] = {400, 800, 1200};
  int sinkFreq[2] = {300, 150};
  int climbCycle[3] = {600, 450, 150};
  bool sdLogging = false;
  bool lcdDisplay = true;
  bool gpsAvailable = true;
};
Settings settings;


void setup() 
{
  //saveSettings();
  loadSettings();                                   //loading settings from memory  
  initializeDevices();                              //initialize Serial, BTSerial, GPSSerial,  MS5611 sensor, SD, LCD
  initializeAndCheckPins();                         //initialize LCD and Buzzer pins and checks
  initializeInitialVariables();                     //initializes Kalman-filter variables and initial pressure values
  printDebug("Setup finished");
  BT_SoftwareSerial.println("Hello from Vario");    //send welcome message
}


//initializing devices
bool initializeDevices(){
  //initialize Serial monitor for debugging  
  if(debug){
    printDebug("Serial initializing...");
    Serial.begin(SerialBaud);                       
    printDebug("Serial initialized");
  }
  
  //initialize bluetooth serial communication
  printDebug("Bluetooth serial initializing...");
  BT_SoftwareSerial.begin(BTBaud);                  
  printDebug("Bluetooth serial initialized");

  //initialize GPS serial communication  
  if(settings.gpsAvailable){
    printDebug("GPS serial initializing...");
    GPS_SoftwareSerial.begin(GPSBaud);                 
    printDebug("GPS serial initialized");
  }

  //initializing MS5611 pressure sensor
  printDebug("MS5611 pressure sensor initializing...");
  while(!SENSOR.begin(MS5611_ULTRA_HIGH_RES)){      
    printDebug("Could not find a valid MS5611 sensor!");
    BT_SoftwareSerial.println("Could not find a valid MS5611 sensor!");
    delay(1000);
  }
  printDebug("MS5611 pressure sensor initialized");

  if(settings.sdLogging){                           //initializing SD card
    printDebug("SD card reader initializing...");
    while (!SD.begin(SDChipSelect)) {
      printDebug("Card failed, or not present!");
      BT_SoftwareSerial.println("Card failed, or not present!");
      delay(1000);
    }
    printDebug("SD card reader initialized");
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    printDebug("datalog.txt opened");
  }

  //initializing LCD display
  if(settings.lcdDisplay){  
    printDebug("LCD display initializing...");
    lcdDisplay.begin(&Adafruit128x32, 0x3C);  //Adafruit128x32: type of lcd   //0x3C: address of lcd display
    lcdDisplay.setFont(font5x7);              //font
    lcdDisplay.clear();                       //clear display
    printDebug("LCD display initialized");
  }

  return true;
}


//initializing LED and Buzzer pins
bool initializeAndCheckPins(){  
  pinMode(GREENLED, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  printDebug("Pins initialized");

  //Switch on and off leds and buzzer
  tone(BUZZER, 700);
  digitalWrite(GREENLED, HIGH);
  digitalWrite(REDLED, HIGH);
  delay(1000);
  noTone(BUZZER);
  digitalWrite(GREENLED, LOW);
  digitalWrite(REDLED, LOW);
  printDebug("LED&BUZZER test finished");
}


//initializing initial variables
bool initializeInitialVariables(){
  //initializing kalman-filter variables  
  err_measure = 1.0;
  err_estimate = 1.0;
  q = 0.01;  
  
  old_pressure = SENSOR.readPressure(true);
  last_estimate = old_pressure;
  old_millisec = millis();
  printDebug("Variables initialized");
}

//debug printing function
void printDebug(String str){
  if(debug){
    Serial.println(str);
  }
}

//calculating pressure with kalman-filter
float calcPressure(float mea)
{  
  kalman_gain = err_estimate / (err_estimate + err_measure);
  current_estimate = last_estimate + kalman_gain * (mea - last_estimate);
  last_current = last_estimate - current_estimate;
  if(last_current < 0.0)
    last_current = last_current * -1.0;
  err_estimate =  (1.0 - kalman_gain) * err_estimate + last_current*q;
  //err_estimate =  (1.0 - _kalman_gain)*err_estimate + fabs(last_estimate-current_estimate)*q;
  last_estimate=current_estimate;
  return current_estimate;
}




void calculateElevation(){  
  if(pressure_count == 8){
    new_pressure = new_pressure / pressure_count;                           //calculate pressure average
    air_temperature = int(SENSOR.readTemperature(true) * 10) / 10.0;        //read temperature
    delta_millisec = (millis() - old_millisec);                             //calculate elapsed time from last average pressure
	  old_millisec = millis();
    delta_altitude = SENSOR.getAltitude(new_pressure) - SENSOR.getAltitude(old_pressure);		//Calculate altitude difference
    old_pressure = new_pressure;
    pressure_count = 0;
    new_pressure = 0;   

    if(!elevationTest){                                                   
      elevation = delta_altitude / (delta_millisec / 1000.0);             //calculating elevation delta_altitude/delta_time
      elevation = (int)(elevation * 10) / 10.0;                           //converting elevation to one decimal
    }
    
    if(old_elevation != elevation && elevationTest == false){
      new_elevation = true;                                               //new_elevation is true when the actual calculated elevation is different from before
      old_elevation = elevation;
    }
    printDebug("Elevation: " + String(elevation));

    sendBT();                                                             //send via bluetooth
  }

  new_pressure = new_pressure + calcPressure(SENSOR.readPressure(true));  //measure pressure and give it to kalman-filter, and add this to sum
  pressure_count = pressure_count + 1;                                      //increase count
}



//calculate LK8EX1 message checksum
int check_sum(char s[], int j) {
    for(int i = 0; i < j; i = i + 1)
        c ^= s[i];
    return c;
}



//send via bluetooth
void sendBT(){
  if(settings.mode = 1){
    message = "$LK8EX1," + String(old_pressure, 0) + ",99999," + String(elevation*100, 0) + "," + String(air_temperature) + ",999,*";   //setting up the protocol message (payload)
    messageLength = message.length();                                                                                                   //calcukate message length
    char messagearray[messageLength];                                 //creating char array from message
    message.toCharArray(messagearray, messageLength);                 //creating char array from message
    checksum = check_sum(messagearray, messageLength);                //calculating checksum
	  BTmessageToSend = message + String(checksum, HEX);// + "\r\n";    //setting up BT message
    printDebug("BT message: " + BTmessageToSend);
    BT_SoftwareSerial.println(BTmessageToSend);                       //sending    
  }
  else if(settings.mode = 2){
    printDebug("BT message: " + String((unsigned long)old_pressure));
    BT_SoftwareSerial.println(String((unsigned long)old_pressure));   //sending just pressure    
  }
}



//loading settings from memory
void loadSettings(){
  EEPROM.get(eeAddress, settings);
  printDebug("Settings are loaded");
  BT_SoftwareSerial.println("Settings have been loaded!");
  BT_SoftwareSerial.println("Mode: " + String(settings.mode));
  BT_SoftwareSerial.println("Volume: " + String(settings.volume));
  BT_SoftwareSerial.println("Climb threshold: " + String(settings.climbThreshold));
  BT_SoftwareSerial.println("Sink threshold: " + String(settings.sinkThreshold));
  BT_SoftwareSerial.println("Climb frequencies: " + String(settings.climbFreq[0]) + " " + String(settings.climbFreq[1]) + " " + String(settings.climbFreq[2]));
  BT_SoftwareSerial.println("Sink frequencies: " + String(settings.sinkFreq[0]) + " " + String(settings.sinkFreq[1]));
  BT_SoftwareSerial.println("Climb cycle: " + String(settings.climbCycle[0]) + " " + String(settings.climbCycle[1]) + " " + String(settings.climbCycle[2]));
  BT_SoftwareSerial.println("SD logging: " + String(settings.sdLogging));
  BT_SoftwareSerial.println("LCD: " + String(settings.lcdDisplay));  
  BT_SoftwareSerial.println("GPS: " + String(settings.gpsAvailable));  
}




void saveSettings(){
  EEPROM.put(eeAddress, settings);
  printDebug("Settings are saved");
  BT_SoftwareSerial.println("Settings have been saved!");
}



//splitting command parameters
void splitCommandValues(String str, String cmd){
  int splitStartIndex = 0;
  int currentIndex = 0;
  if(cmd == "climbtone"){
    for(int i = 0; i < str.length(); i = i + 1){
      if(str.charAt(i) == ','){        
        settings.climbFreq[currentIndex] = (str.substring(splitStartIndex, i)).toInt();
        currentIndex = currentIndex + 1;
        splitStartIndex = i + 1;      
      }
    }
  }
  else if(cmd == "sinktone"){
    for(int i = 0; i < str.length(); i = i + 1){
      if(str.charAt(i) == ','){
        settings.sinkFreq[currentIndex] = (str.substring(splitStartIndex, i)).toInt();
        currentIndex = currentIndex + 1;
        splitStartIndex = i + 1;      
      }
    }    
  } 
  else if(cmd == "climbcycle"){
    for(int i = 0; i < str.length(); i = i + 1){
      if(str.charAt(i) == ','){        
        settings.climbCycle[currentIndex] = (str.substring(splitStartIndex, i)).toInt();
        currentIndex = currentIndex + 1;
        splitStartIndex = i + 1;      
      }
    }
  }
}


//reading and setting modified configuration
void Configure(){
  printDebug(receivedConfigString);
  if(receivedConfigString.charAt(0) == 'm'){
    if(receivedConfigString.charAt(1) == '1'){
      settings.mode = 1;
      printDebug("Mode changed to: 1 (LK8EX1)");
      BT_SoftwareSerial.println("Mode changed to: 1 (LK8EX1)");      
    }
    else if(receivedConfigString.charAt(1) == '2'){
      settings.mode = 2;
      printDebug("Mode changed to: 2 (only pressure)");
      BT_SoftwareSerial.println("Mode changed to: 2 (only pressure)");      
    }    
  }
  else if(receivedConfigString.charAt(0) == 'v'){
    int vol = receivedConfigString.substring(1).toInt();
    if(vol >= 0 && vol <= 100){
      settings.volume = vol;
      printDebug("Volume changed to: " + String(vol));
      BT_SoftwareSerial.println("Volume changed to: " + String(vol));      
    }
    else {
      printDebug("Value must be between 0 and 100!");
      BT_SoftwareSerial.println("Value must be between 0 and 100!");      
    }    
  }
  else if(receivedConfigString.substring(0, 14) == "climbthreshold"){
    settings.climbThreshold = (receivedConfigString.substring(14)).toFloat();
    printDebug("Climb threshold changed to: " + String(settings.climbThreshold));
    BT_SoftwareSerial.println("Climb threshold changed to: " + String(settings.climbThreshold));    
  }
  else if(receivedConfigString.substring(0, 13) == "sinkthreshold"){
    settings.sinkThreshold = (receivedConfigString.substring(13)).toFloat();
    printDebug("Sink threshold changed to: " + String(settings.sinkThreshold));
    BT_SoftwareSerial.println("Sink threshold changed to: " + String(settings.sinkThreshold));    
  }
  else if(receivedConfigString.substring(0, 10) == "climbtone="){
    splitCommandValues(receivedConfigString.substring(10), "climbtone");
    printDebug("Climb tones changed to: " + String(settings.climbFreq[0]) + " " + String(settings.climbFreq[1]) + " " + String(settings.climbFreq[2]));
    BT_SoftwareSerial.println("Climb tones changed to: " + String(settings.climbFreq[0]) + " " + String(settings.climbFreq[1]) + " " + String(settings.climbFreq[2]));    
  }
  else if(receivedConfigString.substring(0, 9) == "sinktone="){
    splitCommandValues(receivedConfigString.substring(9), "sinktone");
    printDebug("Sink tones changed to: " + String(settings.sinkFreq[0]) + " " + String(settings.sinkFreq[1]));
    BT_SoftwareSerial.println("Sink tones changed to: " + String(settings.sinkFreq[0]) + " " + String(settings.sinkFreq[1]));    
  }
  else if(receivedConfigString.substring(0, 11) == "climbcycle="){
    splitCommandValues(receivedConfigString.substring(11), "climbcycle");
    printDebug("Climb cycle changed to: " + String(settings.climbCycle[0]) + " " + String(settings.climbCycle[1]) + " " + String(settings.climbCycle[2]));
    BT_SoftwareSerial.println("Climb cycle changed to: " + String(settings.climbCycle[0]) + " " + String(settings.climbCycle[1]) + " " + String(settings.climbCycle[2]));    
  }
  else if(receivedConfigString == "lcdon"){
    printDebug("Switch LCD ON");
    settings.lcdDisplay = true;
  }
  else if(receivedConfigString == "lcdoff"){
    printDebug("Switch LCD OFF");
    settings.lcdDisplay = false;
  }
  else if(receivedConfigString == "sdon"){
    printDebug("Switch SD ON");
    settings.sdLogging = true;
  }
  else if(receivedConfigString == "sdoff"){
    printDebug("Switch SD OFF");
    settings.sdLogging = false;
  }
  else if(receivedConfigString == "gpson"){
    printDebug("Switch GPS ON");
    settings.gpsAvailable = true;
  }
  else if(receivedConfigString == "gpsoff"){
    printDebug("Switch GPS OFF");
    settings.gpsAvailable = false;
  }
  else if(receivedConfigString.substring(0, 5) == "test="){
    elevation = receivedConfigString.substring(5).toFloat();
    printDebug("Elevation test: " + String(elevation));
    BT_SoftwareSerial.println("Elevation test: " + String(elevation));    
    elevationTestStartedMillisec = millis();
    elevationTest = true;
    new_elevation = true;    
  }
  else if(receivedConfigString == "loadsettings"){
    printDebug("Settings is loading...");
    loadSettings();
  }
  else if(receivedConfigString == "default"){
    settings.mode = 1;
    settings.volume = 50;
    settings.climbThreshold = 0.00;
    settings.sinkThreshold = -0.10;
    settings.climbFreq[0] = 400;
    settings.climbFreq[1] = 800;
    settings.climbFreq[2] = 1200;
    settings.sinkFreq[0] = 300;
    settings.sinkFreq[1] = 150;
    settings.climbCycle[0] = 600;
    settings.climbCycle[1] = 450;
    settings.climbCycle[2] = 150;
    settings.sdLogging = false;
    settings.lcdDisplay = true;
    settings.gpsAvailable = true;
    printDebug("Default settings are loaded!");
    BT_SoftwareSerial.println("Default settings are loaded!");    
  }  
  else {
    printDebug("Not supported command!");
    BT_SoftwareSerial.println("Not supported command!");    
  }

  saveSettings();                     //saving modified settings
  receivedConfigString = "";
}


//printing GPS informations
void printGPSInfos(){
    Serial.print("Sats= "); Serial.print(GPS.satellites.value()); Serial.print("    ");
    Serial.print("LAT= "); Serial.print(GPS.location.lat(), 6); Serial.print("    ");
    Serial.print("LNG= "); Serial.print(GPS.location.lng(), 6); Serial.print("    ");
    Serial.print("Alt= "); Serial.print(GPS.altitude.meters()); Serial.print("    ");
    Serial.print("Speed= "); Serial.print(GPS.speed.kmph()); Serial.print("    ");
    Serial.print("Course= "); Serial.print(GPS.course.deg()); Serial.print("    ");
    Serial.print("Date= ");
    Serial.print(GPS.date.year()); Serial.print(":");
    Serial.print(GPS.date.month()); Serial.print(":");
    Serial.print(GPS.date.day()); Serial.print("    ");
    Serial.print("Time= ");
    Serial.print(GPS.time.hour()); Serial.print(":");
    Serial.print(GPS.time.minute()); Serial.print(":");
    Serial.print(GPS.time.second()); Serial.print("    ");
    Serial.print("FailedCHKS= ");Serial.println(GPS.failedChecksum());    
}


//printing to LCD display
void printDisplay(){
  if(settings.lcdDisplay){
    lcdDisplay.clear();
    lcdDisplay.println(elevation);
    lcdDisplay.println(GPS.location.lat());
    lcdDisplay.println(GPS.failedChecksum());
  }  
}


//logging to SD
void SD_log(String str){
  if(settings.sdLogging){
    if(dataFile){
      logString = "";
      dataFile.println(logString);
      //dataFile.close();
    }
  }
}



void loop(){    
  while (BT_SoftwareSerial.available()) {       //reading all characters from bluetooth serial
    delay(3);  
    receivedChar = BT_SoftwareSerial.read();    //read one cgaracter to receivedChar
    receivedConfigString += receivedChar;       //concatenate characters
  }

  if(receivedConfigString.length() > 0){      //if got configuring command from bluetooth
    Configure();
  }

  if(elevationTest){                                          //elevation test
    if(millis() > (elevationTestStartedMillisec + 4000)){
      elevationTest = false;
    }
  }
  
  if(settings.gpsAvailable){
    while (GPS_SoftwareSerial.available() > 0){           //encoding GPS serial
      GPS.encode(GPS_SoftwareSerial.read());
    }
  }

  if(debug && settings.gpsAvailable){
    if((millis() - gps_LastPrintedSentenceTime) > 1000){    //printing GPS informations in every second if debug and GPS is active
      gps_LastPrintedSentenceTime = millis();
      printGPSInfos();
    }
  }

    
  calculateElevation();       //calculate elevation

  if((millis() - display_UpdatedTime) > 500){       //if LCD is active update display every 500 ms
    display_UpdatedTime = millis();
    if(settings.lcdDisplay){
      printDisplay();    
    }
  }
  

  if(new_elevation){
    new_elevation = false;    
    if(elevation > 3.0 && elevation < 10.0){
      buzz_freq = settings.climbFreq[1] + (elevation - 3.0) * (settings.climbFreq[2] - settings.climbFreq[1]) / 7;
      cycle_ms = settings.climbCycle[1] - (elevation - 3.0) * (settings.climbCycle[1] - settings.climbCycle[2]) / 7;
    }
    else if(elevation > 0.0){
      buzz_freq = settings.climbFreq[0] + elevation * (settings.climbFreq[1] - settings.climbFreq[0]) / 3;
      cycle_ms = settings.climbCycle[0] - elevation * (settings.climbCycle[0] - settings.climbCycle[1]) / 3;
    }
    else if(elevation < 0.0 && elevation > -10.0){
      buzz_freq = settings.sinkFreq[0] + elevation * (settings.sinkFreq[0] - settings.sinkFreq[1]) / 10;
    } 
    printDebug("Elevation: " + String(elevation) + "   " + "Freq: " + String(buzz_freq) + "   " + "Cycle: " + String(cycle_ms));  
  }

  if(elevation < settings.sinkThreshold && elevation > -10.0){
    tone(BUZZER, buzz_freq);
    buzzerActive = false;
    digitalWrite(REDLED, HIGH);
    digitalWrite(GREENLED, LOW);
  }
  else if(elevation > settings.climbThreshold && elevation < 10.0){
    currentMillis = millis();    
    if(!buzzerActive && (currentMillis - buzzPausedTime) > cycle_ms){
      buzzStartedTime = millis();
      tone(BUZZER, buzz_freq);
      buzzerActive = true;
      digitalWrite(GREENLED, HIGH);
      digitalWrite(REDLED, LOW);
      currentMillis = millis();      
    }
    if(buzzerActive && (currentMillis - buzzStartedTime) > cycle_ms){
      buzzPausedTime = millis();
      noTone(BUZZER);
      buzzerActive = false;
      digitalWrite(GREENLED, LOW);
      currentMillis = millis();
    }
  }
  else {
    noTone(BUZZER);
    buzzerActive = false;
    digitalWrite(GREENLED, LOW);
    digitalWrite(REDLED, LOW);
  }
  
}
