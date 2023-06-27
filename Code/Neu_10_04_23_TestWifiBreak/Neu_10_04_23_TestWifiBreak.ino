#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <Wire.h>
#include <WiFiManager.h>
WiFiManager wfm;
#include <WiFi.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <HTTPClient.h>
HTTPClient http;
#include <Preferences.h>
Preferences pref;
#include <ezTime.h>
Timezone myTZ;

//Water Level Sensor
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL SerialUSB
#else
#define SERIAL Serial
#endif

//TEMP
#include <OneWire.h>
#include <DallasTemperature.h>

//Define InfluxDB
#include "Config.h" // You can change your InfluxDB Instance here
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
Point sensor("SampleData");

//TDS
#define TdsSensorPin 35
#define VREF 3.3 // analog reference voltage(Volt) of the ADC

//Relay
#define HeatPin 27
#define StirrerPin 26
#define LightPin 25

//Water Level I2C
#define I2C_SDA_PIN 13 
#define I2C_SCL_PIN 14
unsigned char low_data[8] = {0};
unsigned char high_data[12] = {0};
#define NO_TOUCH       0xFE
#define THRESHOLD      100
#define ATTINY1_HIGH_ADDR   0x78
#define ATTINY2_LOW_ADDR   0x77
#define StatusLED 33


int reconnectattempt = 0;

//TDS
int analogBuffer[0]; // store the analog value in the array, read from ADC
int analogBufferTemp[0];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0;

//TEMP
#define ONE_WIRE_BUS 32
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Trigger for Reset
#define RESET_TRIGGER 12
int buttonState = 0;    
int lastButtonState = 0; 
int startPressed = 0;  
int endPressed = 0;      
int holdTime = 0;        
int idleTime = 0;   

bool dataSent = false;  
int previousMinute = -1;   

//HalfHourly Time Sync
unsigned long previousMillisSync = 0;   
unsigned long previousMillisPrint = 0;   
const unsigned long intervalSync = 1800000;
const unsigned long intervalPrint = 60000;

// Box für Zieltemperatur
WiFiManagerParameter MaxTemp_Text_Box("MaxTemp_Text", "Maximale Temperatur", "32", 2);
// Box für Minimaltemperatur
WiFiManagerParameter MinTemp_Text_Box("MinTemp_Text", "Minimale Temperatur", "25", 2);
// Box für Abstand zwischen Rühren
WiFiManagerParameter IntervalStirrer_Text_Box("IntervalStirrer_Text", "Zeit zwischen den Rührvorgängen in Minuten", "30", 4);
// Box für Rührdauer
WiFiManagerParameter DurationStirrer_Text_Box("DurationStirrer_Text", "Dauer eines Rührvorgangs in Minuten", "1", 4);
// Box für Licht an
WiFiManagerParameter LightOn_Text_Box("LightOn_Text", "Uhrzeit (0-23) zum anschalten des Lichts", "8", 2);
// Box für Licht aus 
WiFiManagerParameter LightOff_Text_Box("LightOff_Text", "Uhrzeit (0-23) zum abschalten des Lichts", "20", 2);

void setup()
{
  Serial.begin(115200);

  pref.begin("Prefs", false);

  if (pref.getBool("namebool") == false) 
  {
    int Reaktornummer = (int)random(100000, 999999);
    String ReaktornameGenerated = String("ESP32_" + String(Reaktornummer));
    pref.putString("ReaktornameUser", ReaktornameGenerated);
    pref.putBool("namebool", true);
    Serial.print(ReaktornameGenerated);
  }


    WiFiManagerParameter Token_Text_Box("Token_Text", "Name des Reaktors", pref.getString("ReaktornameUser").c_str(), 20);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // Custom I2C Pins for WaterLevel
    pinMode(HeatPin, OUTPUT);
    pinMode(StirrerPin, OUTPUT);
    pinMode(LightPin, OUTPUT);
    pinMode(RESET_TRIGGER, INPUT_PULLUP); //Pullup for Reset Button
    pinMode(2, OUTPUT); //set LED to Output
    pinMode(StatusLED, OUTPUT); //set LED to Output
    pinMode(TdsSensorPin, INPUT); //Initialize TDS
    sensors.begin(); //initialize TEMPsensor

    wfm.addParameter(&Token_Text_Box);
    wfm.addParameter(&MaxTemp_Text_Box);
    wfm.addParameter(&MinTemp_Text_Box);
    wfm.addParameter(&IntervalStirrer_Text_Box);
    wfm.addParameter(&DurationStirrer_Text_Box);
    wfm.addParameter(&LightOn_Text_Box);
    wfm.addParameter(&LightOff_Text_Box);


  if (digitalRead(RESET_TRIGGER) == LOW)
    {
      Serial.println("HARDRESET");
      pref.putBool("setupbool", false);
      pref.putBool("namebool", false);
      pref.clear();
      wfm.resetSettings();
      blinkLEDs(1,500);
      ESP.deepSleep(10000000*10000000); //Sleep until Reset 
    }

  bool res;
    // res = wm.autoConnect("AutoConnectAP"); // anonymous ap
    wfm.setWiFiAutoReconnect(true);
    wfm.setConnectRetries(10);
    wfm.setConfigPortalTimeout(300);

    res = wfm.autoConnect(pref.getString("ReaktornameUser").c_str()); // password protected ap

      if(!res)
      {
          Serial.println("Failed to connect");
      } 
      else
      {

        Serial.println("connected...yeey :)");
        waitForSync(); 
        myTZ.setLocation(F("Europe/Berlin")); 

        if (pref.getBool("resetbool") == false)
        {

          pref.putString("MinTemp", MinTemp_Text_Box.getValue());
          pref.putString("MaxTemp", MaxTemp_Text_Box.getValue());
          pref.putString("IntervalStirrer", IntervalStirrer_Text_Box.getValue());
          pref.putString("DurationStirrer", DurationStirrer_Text_Box.getValue());
          pref.putString("LightOn", LightOn_Text_Box.getValue());
          pref.putString("LightOff", LightOff_Text_Box.getValue());

          pref.putBool("resetbool", true);

        }

        if (pref.getBool("setupbool") == false) 
        {
          pref.putString("ReaktornameUser", Token_Text_Box.getValue());
          pref.putBool("setupbool", true);
          blinkLEDs(10,20);
        }
      }

    String ReaktornameUser = pref.getString("ReaktornameUser");
    const char* Reaktorname = ReaktornameUser.c_str();
    Serial.print("Your Reactor is called: ");
    Serial.println(Reaktorname);
    sensor.addTag("Reaktorname", Reaktorname);


    //Write one initial Point for initial User Feedback
    AddSensorData();
    WriteSensorData();
      
}


void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    setStatusLEDsHigh();
    // Update Current Time
    // https://en.wikipedia.org/wiki/List_of_tz_database_time_zones

    // Check Button State and initialize Reset
    checkResetButton();
    HalfHourlySyncDeviceTime();
    PrintValues();
    // Control the Parameters
    controlWaterTemp();
    controlStirring();
    controlLight();

    //Write Points 
    int currentHour = myTZ.hour();
    int currentMinute = myTZ.minute();
      
    if (currentMinute != previousMinute)
    {
        dataSent = false;
        previousMinute = currentMinute;
    }

    delay(10);

    if (((currentHour == 0 && currentMinute == 0) ||
        (currentHour == 6 && currentMinute == 0) ||
        (currentHour == 14 && currentMinute == 15) ||
        (currentHour == 18 && currentMinute == 0)) &&
        (!dataSent))
      {
        AddSensorData();
        WriteSensorData();
        
        // Set flag to true to indicate that the data has been sent
        dataSent = true;
        previousMinute = myTZ.minute();
        delay(10);
      }
  }
  else 
  { 
    setStatusLEDsLow();
    wfm.disconnect();
    if (wfm.getWiFiIsSaved()) wfm.setEnableConfigPortal(false);
    wfm.autoConnect("AP");
    wfm.autoConnect(pref.getString("ReaktornameUser").c_str());
    Serial.println("No WiFi");
    
  }
  Serial.print(".");
  delay(100);
}


void configModeCallback (WiFiManager *myWiFiManager)
{
  Serial.println("Entered config mode");
}


float readTemperature()
    {
    //TEMP
    sensors.requestTemperatures(); // Send the command to get temperatures

    float tempC = sensors.getTempCByIndex(0);

    return tempC;
    }


float readLeitfaehigkeit()
{   //TDS
    float currentTemp = readTemperature();
    analogBuffer[0] = analogRead(TdsSensorPin); // read the analog value and store into the buffer
    averageVoltage = analogBuffer[0] * (float)VREF / 4095.0; // convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (currentTemp - 25.0); // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVoltage = averageVoltage / compensationCoefficient; // temperature compensation
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

    // Serial.print("Leitfähigkeit: ");
    // Serial.print(tdsValue,0);
    // Serial.println(" ppm");

    return tdsValue;
} 


float readKonzentration()
    {
    //Turbidity
    int sensorValue = analogRead(34);
    float voltValue = ((sensorValue * 3.3) / 4095);  
    // Serial.print("Spannung am Trübheitssensor ");
    // Serial.print(voltValue);
    // Serial.println(" V");
    // Serial.println("________________________________________________");
    
    return voltValue;
    }
    //Water Level


int readWaterlevel()
{
  int sensorvalue_min = 250;
  int sensorvalue_max = 255;
  int low_count = 0;
  int high_count = 0;
  float water_level = 0;
      
    uint32_t touch_val = 0;
    uint8_t trig_section = 0;
    low_count = 0;
    high_count = 0;
    getLow8SectionValue();
    getHigh12SectionValue();
    for (int i = 0; i < 8; i++)
    {
      if (low_data[i] >= sensorvalue_min && low_data[i] <= sensorvalue_max)
      {
        low_count++;
      }
      if (low_count == 8)
      {
        Serial.print("      ");
        Serial.print("PASS");
      }
    }

    for (int i = 0; i < 12; i++)
    {
      if (high_data[i] >= sensorvalue_min && high_data[i] <= sensorvalue_max)
      {
        high_count++;
      }
      if (high_count == 12)
      {
        Serial.print("      ");
        Serial.print("PASS");
      }
    }

    for (int i = 0 ; i < 8; i++) {
      if (low_data[i] > THRESHOLD) {
        touch_val |= 1 << i;

      }
    }
    for (int i = 0 ; i < 12; i++) {
      if (high_data[i] > THRESHOLD) {
        touch_val |= (uint32_t)1 << (8 + i);
      }
    }

    while (touch_val & 0x01)
    {
      trig_section++;
      touch_val >>= 1;
    }

    water_level =  trig_section*5;
    // Serial.print("Füllstand ");
    // Serial.println(water_level);


    return water_level;
}


void AddSensorData()
{
    sensor.clearFields();
    
    //add Sensor Readings to Influx
    float WaterLevel = readWaterlevel();
    sensor.addField("Fuellvolumen [mL]", WaterLevel);
    Serial.println("Added Waterlevel");

    float Temperature = readTemperature();
    sensor.addField("Temperatur [°C]", Temperature);
    Serial.println("Added Temperature");

    float Leitfaehigkeit = readLeitfaehigkeit();
    sensor.addField("TDS [ppm]", Leitfaehigkeit); 
    Serial.println("Added Leitfaehigkeit");

    float Konzentration = readKonzentration();
    sensor.addField("Konzentration [g/L]", Konzentration);
    Serial.println("Added Konzentration");

    //add Custom Parameter to Influx
    sensor.addField("LichtAnZeit", pref.getString("LightOn").toFloat());
    sensor.addField("LichtAusZeit", pref.getString("LightOff").toFloat());
    sensor.addField("MinimalTemperatur", pref.getString("MinTemp").toFloat());
    sensor.addField("MaximalTemperatur", pref.getString("MaxTemp").toFloat());
    sensor.addField("RuehrInterval", pref.getString("IntervalStirrer").toFloat());
    sensor.addField("RuehrDauer", pref.getString("DurationStirrer").toFloat());

}

    
void WriteSensorData()
{
    //Add Data Point
  client.setHTTPOptions(HTTPOptions().httpReadTimeout(10000));
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  
  
  bool isWritten = client.writePoint(sensor);


  if (!isWritten)
  {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());

  }
  else {
    Serial.print("InfluxDB write SUCCES!");
    blinkLEDs(7,80);
  }
}

void getHigh12SectionValue(void)
{
  memset(high_data, 0, sizeof(high_data));
  Wire.requestFrom(ATTINY1_HIGH_ADDR, 12);
  while (12 != Wire.available());

  for (int i = 0; i < 12; i++) {
    high_data[i] = Wire.read();
  }
  delay(10);
}


void getLow8SectionValue(void)
{
  memset(low_data, 0, sizeof(low_data));
  Wire.requestFrom(ATTINY2_LOW_ADDR, 8);
  while (8 != Wire.available());

  for (int i = 0; i < 8 ; i++) {
    low_data[i] = Wire.read(); // receive a byte as character
  }
  delay(10);
}

void checkResetButton()
{
  if (digitalRead(RESET_TRIGGER) == HIGH)
  {
      startPressed = millis();
      idleTime = startPressed - endPressed;
  } 
  else 
  {
      endPressed = millis();
      holdTime = endPressed - startPressed;

      if (holdTime >= 5000)
      {
          wfm.resetSettings();
          pref.putBool("resetbool", false);
          setStatusLEDsHigh();
          delay(1000);
          setStatusLEDsLow();
          ESP.restart();          
      }
  }
}

void controlWaterTemp()
{
  
  float CurrentTemp = readTemperature();
  float MinimumTemp = atof(pref.getString("MinTemp").c_str());
  float MaximumTemp = atof(pref.getString("MaxTemp").c_str());
  bool sufficientWater = readWaterlevel() > 50;
  bool ValidTemp = CurrentTemp > 0;
  // Serial.print("Sufficient Water: ");
  // Serial.println(sufficientWater);
  // Serial.println("CurrentTemp(ControlHeater): " + String(CurrentTemp));
  // Serial.println("MinTemp: " + String(MinimumTemp));
  // Serial.println("MaxTemp: " + String(MaximumTemp));

    if (sufficientWater && ValidTemp)
    {
      if (CurrentTemp < (MinimumTemp))
      {
        digitalWrite(HeatPin, HIGH);
      }
      else if (CurrentTemp > (MaximumTemp))
      {
        digitalWrite(HeatPin, LOW);
      }
      // otherwise, do nothing 
    }
    else
    {
      digitalWrite(HeatPin, LOW);
    }
}

void controlStirring() {
  float intervalMinutes = atof(pref.getString("IntervalStirrer").c_str());
  float durationMinutes = atof(pref.getString("DurationStirrer").c_str());
  // Serial.print("Zeit zwischen Rührvorgängen ");
  // Serial.println(intervalMinutes);
  // Serial.print("Zeit eines Rührvorgängs ");
  // Serial.println(durationMinutes);
  
  static bool stirring = false;
  static unsigned long startTime = 0;
  static unsigned long lastStirTime = 0;

  unsigned long currentTime = millis();

  // Check if it's time to start stirring
  if (!stirring && currentTime - lastStirTime >= intervalMinutes * 60 * 1000) {
    stirring = true;
    startTime = currentTime;
    digitalWrite(StirrerPin, HIGH);
  }

  // Check if it's time to stop stirring
  if (stirring && currentTime - startTime >= durationMinutes * 60 * 1000) {
    stirring = false;
    lastStirTime = currentTime;
    digitalWrite(StirrerPin, LOW);
  }

  // Check if it's time to start the next stirring cycle
  if (!stirring && currentTime - lastStirTime >= (intervalMinutes - durationMinutes) * 60 * 1000) {
    stirring = true;
    startTime = currentTime;
    digitalWrite(StirrerPin, HIGH);
  }
}

void controlLight()
{
  int lightOnTime = atoi(pref.getString("LightOn").c_str());
  int lightOffTime = atoi(pref.getString("LightOff").c_str());
  int currentHour = myTZ.hour();

  if (currentHour >= lightOnTime && currentHour < lightOffTime) {
    digitalWrite(LightPin, HIGH); // turn on the light
  } else {
    digitalWrite(LightPin, LOW); // turn off the light
  }
}

void HalfHourlySyncDeviceTime()
{

  unsigned long currentMillisSync = millis();  // Get the current millis value

  if (currentMillisSync - previousMillisSync >= intervalSync)
  {
  previousMillisSync = currentMillisSync;  // Update the previousMillis variable
  waitForSync(); 
  }
}

void PrintValues()
{
  unsigned long currentMillisPrint = millis();  // Get the current millis value

  if ((currentMillisPrint - previousMillisPrint >= intervalPrint) || (previousMillisPrint == 0))
  {
    previousMillisPrint = currentMillisPrint;  // Update the previousMillis variable

    int PrintlightOnTime = atof(pref.getString("LightOn").c_str());
    int PrintlightOffTime = atof(pref.getString("LightOff").c_str());
    float PrintCurrentTemp = readTemperature();
    float PrintWaterlevel = readWaterlevel();
    float PrintKonzentration = readKonzentration();
    float PrintLeitfaehigkeit = readLeitfaehigkeit();
    float PrintMinimumTemp = atof(pref.getString("MinTemp").c_str());
    float PrintMaximumTemp = atof(pref.getString("MaxTemp").c_str());
    float PrintintervalMinutes = atof(pref.getString("IntervalStirrer").c_str());
    float PrintdurationMinutes = atof(pref.getString("DurationStirrer").c_str());
    String ReaktornameUser = pref.getString("ReaktornameUser");

    Serial.println("Your Reactor is called: " + ReaktornameUser);
    Serial.print(F("Europe: "));
    Serial.println(myTZ.dateTime());
    Serial.println("___________________________________");
    Serial.println("Konzentration: " + String(PrintKonzentration));
    Serial.println("Leitfaehigkeit: " + String(PrintLeitfaehigkeit));
    Serial.println("Fuellvolumen: " + String(PrintWaterlevel));
    Serial.println("Temperatur: " + String(PrintCurrentTemp));
    Serial.println("___________________________________");
    Serial.println("Zeit zwischen Rührvorgängen " + String(PrintintervalMinutes));
    Serial.println("Zeit eines Rührvorgangs " + String(PrintdurationMinutes));
    Serial.println("MinTemp: " + String(PrintMinimumTemp));
    Serial.println("MaxTemp: " + String(PrintMaximumTemp));
    Serial.println("Licht an um " + String(PrintlightOnTime));
    Serial.println("Licht aus um " + String(PrintlightOffTime));
  }
}

void setStatusLEDsHigh()
{
  digitalWrite(2, HIGH);
  digitalWrite(StatusLED, HIGH);
}

void setStatusLEDsLow()
{
  digitalWrite(2, LOW);
  digitalWrite(StatusLED, LOW);
}

void blinkLEDs(int numBlinks, int delayTime) {
  // Check the current state of the LED
  bool initialState = digitalRead(StatusLED);

  for (int i = 0; i < numBlinks; i++) {
    digitalWrite(2, !initialState);
    digitalWrite(StatusLED, !initialState);
    delay(delayTime);                       
    digitalWrite(2, initialState);
    digitalWrite(StatusLED, initialState);  
    delay(delayTime);                       
  }
}
