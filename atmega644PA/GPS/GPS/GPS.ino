#include <MS5611.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <TinyGPS++.h>

#define REDLED 20
#define GREENLED 19
#define BUZZER 18
#define BT_RX 10
#define BT_TX 11
#define GPS_RX 0
#define GPS_TX 1

MS5611 SENSOR;

TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;
SoftwareSerial GPS_Serial(GPS_RX, GPS_TX);

SSD1306AsciiAvrI2c lcdDisplay;

SoftwareSerial BT(BT_RX, BT_TX);

float new_pressure;
float old_pressure;
int pressure_count = 0;

float delta_altitude;

unsigned long old_millisec;
unsigned long delta_millisec;

unsigned long elevationTestStartedMillisec = 0;
bool elevationTest = false;

float elevation = 0.0;
float old_elevation = 0.0;
float air_temperature;

unsigned long currentMillis = 0;
bool new_elevation = false;
bool buzzerActive = false;
int buzz_freq = 0;
int cycle_ms = 0;
unsigned long buzzStartedTime = 0;
unsigned long buzzPausedTime = 0;

float last_estimate;
float err_estimate;
float err_measure;
float current_estimate;
float last_current;
float kalman_gain;
float q;

String receivedConfigString;

int c = 0;

char receivedChar;

int checksum;
byte messageLength;
String message;
String BTmessageToSend;

unsigned long gps_LastPrintedSentenceTime;
unsigned long display_UpdatedTime;

int eeAddress = 0;

struct Settings {
  byte mode = 1;
  int volume = 50;
  float climbThreshold = 0.10;
  float sinkThreshold = -0.10;
  int climbFreq[3] = {400, 800, 1200};
  int sinkFreq[2] = {300, 150};
  int climbCycle[3] = {600, 450, 150};
};
Settings settings;


void setup() 
{
  err_measure = 1.0;
  err_estimate = 1.0;
  q = 0.01;
  
  Serial.begin(115200);

  BT.begin(9600);
  GPS_Serial.begin(GPSBaud);
      
  while(!SENSOR.begin(MS5611_ULTRA_HIGH_RES))
  {    
    BT.println("Could not find a valid MS5611 sensor!!!");
    delay(2000);
  }
  
  delay(1000);
  ///Serial.println("AAAAA"); 

  lcdDisplay.begin(&Adafruit128x32, 0x3C);
  lcdDisplay.setFont(font5x7);
  lcdDisplay.clear();

  pinMode(GREENLED, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  BT.println("Hello from Vario");

  loadSettings();
  
  old_pressure = SENSOR.readPressure(true);
  last_estimate = old_pressure;
  old_millisec = millis();
}


float updateEstimate(float mea)
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
    new_pressure = new_pressure / pressure_count;
    air_temperature = int(SENSOR.readTemperature(true) * 10) / 10.0;
    delta_millisec = (millis() - old_millisec);
	  old_millisec = millis();
    delta_altitude = SENSOR.getAltitude(new_pressure) - SENSOR.getAltitude(old_pressure);		//Calculate altitude difference
    old_pressure = new_pressure;
    pressure_count = 0;
    new_pressure = 0;   

    if(!elevationTest){
      elevation = delta_altitude / (delta_millisec / 1000.0);
      elevation = (int)(elevation * 10) / 10.0;
    }
    
    if(old_elevation != elevation && elevationTest == false){
      new_elevation = true;
      old_elevation = elevation;
    }

    sendBT();
    
  }

  new_pressure = new_pressure + updateEstimate(SENSOR.readPressure(true));  //measure pressure and give it to kalman-filter, and add this to sum
  pressure_count = pressure_count + 1;
}




int check_sum(char s[], int j) {
    for(int i = 0; i < j; i = i + 1)
        c ^= s[i];
    return c;
}




void sendBT(){
  if(settings.mode = 1){
    message = "$LK8EX1," + String(old_pressure, 0) + ",99999," + String(elevation*100, 0) + "," + String(air_temperature) + ",999,*";
    messageLength = message.length();
    char messagearray[messageLength];
    message.toCharArray(messagearray, messageLength);
    checksum = check_sum(messagearray, messageLength);   
	  BTmessageToSend = message + String(checksum, HEX);// + "\r\n";   
    BT.println(BTmessageToSend);
  }
  else if(settings.mode = 2){
    BT.println(String((unsigned long)old_pressure));
  }
}




void loadSettings(){
  EEPROM.get(eeAddress, settings);
  BT.println("Settings have been loaded!");
  BT.println("Mode: " + String(settings.mode));
  BT.println("Volume: " + String(settings.volume));
  BT.println("Climb threshold: " + String(settings.climbThreshold));
  BT.println("Sink threshold: " + String(settings.sinkThreshold));
  BT.println("Climb frequencies: " + String(settings.climbFreq[0]) + " " + String(settings.climbFreq[1]) + " " + String(settings.climbFreq[2]));
  BT.println("Sink frequencies: " + String(settings.sinkFreq[0]) + " " + String(settings.sinkFreq[1]));
  BT.println("Climb cycle: " + String(settings.climbCycle[0]) + " " + String(settings.climbCycle[1]) + " " + String(settings.climbCycle[2]));
}




void saveSettings(){
  EEPROM.put(eeAddress, settings);
  BT.println("Settings have been saved!");
}




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



void Configure(){
  Serial.println(receivedConfigString);
  if(receivedConfigString.charAt(0) == 'm'){
    if(receivedConfigString.charAt(1) == '1'){
      settings.mode = 1;
      BT.println("Mode changed to: 1 (LK8EX1)");
      Serial.println("Mode changed to: 1 (LK8EX1)");
    }
    else if(receivedConfigString.charAt(1) == '2'){
      settings.mode = 2;
      BT.println("Mode changed to: 2 (only pressure)");
      Serial.println("Mode changed to: 2 (only pressure)");
    }    
  }
  else if(receivedConfigString.charAt(0) == 'v'){
    int vol = receivedConfigString.substring(1).toInt();
    if(vol >= 0 && vol <= 100){
      settings.volume = vol;
      BT.println("Volume changed to: " + String(vol));
      Serial.println("Volume changed to: " + String(vol));
    }
    else {
      BT.println("Value must be between 0 and 100!");
      Serial.println("Value must be between 0 and 100!");
    }    
  }
  else if(receivedConfigString.substring(0, 14) == "climbthreshold"){
    settings.climbThreshold = (receivedConfigString.substring(14)).toFloat();
    BT.println("Climb threshold changed to: " + String(settings.climbThreshold));
    Serial.println("Climb threshold changed to: " + String(settings.climbThreshold));
  }
  else if(receivedConfigString.substring(0, 13) == "sinkthreshold"){
    settings.sinkThreshold = (receivedConfigString.substring(13)).toFloat();
    BT.println("Sink threshold changed to: " + String(settings.sinkThreshold));
    Serial.println("Sink threshold changed to: " + String(settings.sinkThreshold));
  }
  else if(receivedConfigString.substring(0, 10) == "climbtone="){
    splitCommandValues(receivedConfigString.substring(10), "climbtone");
    BT.println("Climb tones changed to: " + String(settings.climbFreq[0]) + " " + String(settings.climbFreq[1]) + " " + String(settings.climbFreq[2]));
    Serial.println("Climb tones changed to: " + String(settings.climbFreq[0]) + " " + String(settings.climbFreq[1]) + " " + String(settings.climbFreq[2]));
  }
  else if(receivedConfigString.substring(0, 9) == "sinktone="){
    splitCommandValues(receivedConfigString.substring(9), "sinktone");
    BT.println("Sink tones changed to: " + String(settings.sinkFreq[0]) + " " + String(settings.sinkFreq[1]));
    Serial.println("Sink tones changed to: " + String(settings.sinkFreq[0]) + " " + String(settings.sinkFreq[1]));
  }
  else if(receivedConfigString.substring(0, 11) == "climbcycle="){
    splitCommandValues(receivedConfigString.substring(11), "climbcycle");
    BT.println("Climb cycle changed to: " + String(settings.climbCycle[0]) + " " + String(settings.climbCycle[1]) + " " + String(settings.climbCycle[2]));
    Serial.println("Climb cycle changed to: " + String(settings.climbCycle[0]) + " " + String(settings.climbCycle[1]) + " " + String(settings.climbCycle[2]));
  }
  else if(receivedConfigString.substring(0, 5) == "test="){
    elevation = receivedConfigString.substring(5).toFloat();
    BT.println("Elevation test: " + String(elevation));
    Serial.println("Elevation test: " + String(elevation));
    elevationTestStartedMillisec = millis();
    elevationTest = true;
    new_elevation = true;    
  }
  else if(receivedConfigString == "getsettings"){
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
    BT.println("Default settings are loaded!");
    Serial.println("Default settings are loaded!");
  }
  else {
    BT.println("Not supported command!");
    Serial.println("Not supported command!");
  }

  saveSettings();
  receivedConfigString = "";
}



void printGPS(){
    Serial.print("Sats= "); Serial.print(gps.satellites.value()); Serial.print("    ");
    Serial.print("LAT= "); Serial.print(gps.location.lat(), 6); Serial.print("    ");
    Serial.print("LNG= "); Serial.print(gps.location.lng(), 6); Serial.print("    ");
    Serial.print("Alt= "); Serial.print(gps.altitude.meters()); Serial.print("    ");
    Serial.print("Speed= "); Serial.print(gps.speed.kmph()); Serial.print("    ");
    Serial.print("Course= "); Serial.print(gps.course.deg()); Serial.print("    ");
    Serial.print("Date= ");
    Serial.print(gps.date.year()); Serial.print(":");
    Serial.print(gps.date.month()); Serial.print(":");
    Serial.print(gps.date.day()); Serial.print("    ");
    Serial.print("Time= ");
    Serial.print(gps.time.hour()); Serial.print(":");
    Serial.print(gps.time.minute()); Serial.print(":");
    Serial.print(gps.time.second()); Serial.print("    ");
    Serial.print("FailedCHKS= ");Serial.println(gps.failedChecksum());

    
}


void printDisplay(){
  lcdDisplay.clear();
  lcdDisplay.println(elevation);
  lcdDisplay.println(gps.location.lat());
  lcdDisplay.println(gps.failedChecksum());
}



void loop(){  
  
  while (BT.available()) {
    delay(3);  
    receivedChar = BT.read();
    receivedConfigString += receivedChar; 
  }

  if(receivedConfigString.length() > 0){
    Configure();
  }

  if(elevationTest){
    if(millis() > (elevationTestStartedMillisec + 4000)){
      elevationTest = false;
    }
  }
  

  while (GPS_Serial.available() > 0){
    gps.encode(GPS_Serial.read());
  }

  if((millis() - gps_LastPrintedSentenceTime) > 1000){
    gps_LastPrintedSentenceTime = millis();
    printGPS();    
  }

    
  calculateElevation();

  if((millis() - display_UpdatedTime) > 500){
    display_UpdatedTime = millis();
    printDisplay();    
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
    //Serial.println("Freq: " + String(buzz_freq));
    //Serial.println("Cycle: " + String(cycle_ms));    
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
