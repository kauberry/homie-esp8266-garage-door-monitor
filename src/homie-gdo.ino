#include <Homie.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include "spi_flash.h"

#define TEMP_PIN            2
#define UP_IND_PIN          15
#define DN_IND_PIN          14

#define UP_SENSE_PIN        5
#define DN_SENSE_PIN        4
#define DOOR_TRIG_PIN       13
#define LIGHT_TRIG_PIN      12
#define DOOR_TRIG_DELAY_MS  500

#define UP_STATE_VAL        -1
#define DN_STATE_VAL        1

#define HOSTNAME            "garage-main"
#define OSWATCH_RESET_TIME  30

#define FW_NAME             "node-gdo-wifi"
#define FW_VERSION          "1.0.0"


OneWire ds(TEMP_PIN);
DallasTemperature DS18B20(&ds);

HomieNode DoorNode("door_state", "trigger");
// HomieNode LightNode("light_trigger", "trigger");

HomieNode InfoNode("info", "status");
HomieNode IdentifyNode("identify", "identify");

HomieNode TemperatureNode("temperature", "temperature");

float case_temperature, ambient_temperature;
int door_status;

String door_status_description;

long lastStatusReadTime, lastTemperatureReportingTime;

static float temp_delta_trigger = 0.5;
static long door_sensor_read_dwell_time = 100;
static long temperature_read_dwell_time = 2000;
static long temperature_reporting_dwell_time = 30000;

int up_value, dn_value;
int test_up, test_dn, is_up;
int entry_state;

long last_forced_report = 0;
long last_msg = 0;
long last_temp_msg = 0;

enum door_state_list {
  open = 0,
  closed = 1,
  opening = 2,
  closing = 3,
  stopped = 4
};

float previousAmbientTempC, previousCaseTempC;
float currentAmbientTempC, currentCaseTempC;

bool signalStartup(int delayTime){
  for(int i=0;i<=5;i++){
    digitalWrite(UP_IND_PIN,HIGH);
    delay(delayTime);
    digitalWrite(UP_IND_PIN, LOW);
    digitalWrite(DN_IND_PIN,HIGH);
    delay(delayTime);
    digitalWrite(DN_IND_PIN, LOW);
  }
  return true;
}

float getTemperature(uint8_t sensorIndex) {
  float temp;
  do {
    DS18B20.requestTemperatures();
    temp = getTempCByIndex(sensorIndex);
    delay(100);
  } while (temp > 80.0 || temp < (-100.0));
  return temp;
}

float temperature_request(uint8_t sensorIndex) {
  float myTempC = getTemperature(sensorIndex);
  myTempC = round(myTempC * 10.0) / 10.0;
  return myTempC;
}

bool door_trigger(){
  if(door_status != 0){
    digitalWrite(DOOR_TRIG_PIN,HIGH);
    if(door_status == 1){
      digitalWrite(UP_IND_PIN, LOW);
      digitalWrite(DN_IND_PIN, HIGH);
    }else if(door_status == -1){
      digitalWrite(DN_IND_PIN, LOW);
      digitalWrite(UP_IND_PIN, HIGH);
    }
    delay(DOOR_TRIG_DELAY_MS);
    digitalWrite(DOOR_TRIG_PIN, LOW);
    digitalWrite(DN_IND_PIN,LOW);
    digitalWrite(UP_IND_PIN,LOW);
  }
  return true;
}

bool triggerHandler(const HomieRange& range, const String& value) {
  return door_trigger();
}

bool identifyHandler(const HomieRange& range, const String& value) {
  return signalStartup(500);
}

bool statusHandler(const HomieRange& range, const String& value) {
  return sendStatus(value);
}

void setupSPIFFS(){
  SPIFFS.begin();
  if(!SPIFFS.exists("/formatComplete.txt")) {
    Serial1.println("Please wait 30 secs for SPIFFS to be formatted");
    SPIFFS.format();
    Serial1.println("SPIFFS formatted");

    File f = SPIFFS.open("/formatComplete.txt", "w");
    if(!f){
      Serial1.println("File opening failed");
    }else{
      f.println("Format Complete");
    }
  }else{
    Serial1.println("SPIFFS is properly formatted.");
  }
}

int getCurrentDoorSensorValues() {
  up_value = digitalRead(UP_SENSE_PIN) * (-1);
  dn_value = digitalRead(DN_SENSE_PIN);
  digitalWrite(UP_IND_PIN, up_value * -1);
  digitalWrite(DN_IND_PIN, dn_value * 1);
  return up_value + dn_value;
}

String enumerateCurrentState(int current_state, int previous_state){
  //we get to this if the new current state is different from the entry state
  String myState;

  if(previous_state == 1 && current_state == 0){
    //we're in motion towards open
    // stateDescription = "opening";
    myState = "2";
  }else if(previous_state == -1 && current_state == 0){
    //we're in motion towards closed
    // stateDescription = "closing";
    myState = "3";
  }else if(current_state == -1 && previous_state == 0){
    // stateDescription = "open";
    myState = "0";
  }else if(current_state == 1 && previous_state == 0){
    // stateDescription = "closed";
    myState = "1";
  }else{
    // stateDescription = "unknown";
    myState = "4";
  }
  return myState;
}


bool sendStatus(String value) {
  if(value == "door_state" || value == "") {
    door_status = enumerateCurrentState(getCurrentDoorSensorValues(), previous_state);
    DoorNode.setProperty("door_state").send(String(door_status))
  }
  if(value == "ambient_temperature" || value == "") {
    TemperatureNode.setProperty("ambient").send(String(temperature_request(0)));
  }
  if(value == "case_temperature" || value == "") {
    TemperatureNode.setProperty("ambient").send(String(temperature_request(1)));
  }
}

void setupHandler() {

}



void setup() {
  Serial.begin(115200);
  // Homie.setLoggingPrinter(&Serial);
  // Homie.setSetupFunction(setupHandler);
  setupSPIFFS();

  Homie_setFirmware(FW_NAME, FW_VERSION);

  DoorNode.advertise("trigger").settable(triggerHandler);
  IdentifyNode.advertise("node").settable(identifyHandler);

  lastStatusReadTime = 0;
  lastTemperatureReportingTime = 0;

  previous_state = 0;
  previousAmbientTempC = 4000.0;
  previousCaseTempC = 4000.0;
  Homie.setup();
}

long last_loop;

void loop() {
  Homie.loop();
  last_loop = millis();

  if(abs(millis() - lastStatusReadTime) >= door_sensor_read_dwell_time) {
    door_status = enumerateCurrentState(getCurrentDoorSensorValues(), previous_state);
    lastStatusReadTime = millis();
    if(door_status.toInt() <= 1) {
      sendStatus("door_state")
    }
  }

  if(abs(millis() - lastTemperatureReadTime) >= temperature_read_dwell_time) {
    lastTemperatureReadTime = millis();
    currentAmbientTempC = temperature_request(0);
    currentCaseTempC = temperature_request(1);
  }

  if(abs(millis() - lastTemperatureReportingTime) >= temperature_reporting_dwell_time || abs(currentAmbientTempC - previousAmbientTempC) >= temp_delta_trigger) {
    lastTemperatureReportingTime =  millis();
    sendStatus("ambient_temperature");
    sendStatus("case_temperature");
  }



}
