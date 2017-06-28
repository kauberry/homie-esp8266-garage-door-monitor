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

enum DoorStates {
  DOOR_OPEN = -2,
  DOOR_CLOSING = -1,
  DOOR_INTERMED = 0,
  DOOR_OPENING = 1,
  DOOR_CLOSED = 2
}

float case_temperature, ambient_temperature;
int door_status;

static float temp_delta_trigger = 0.5;
static long forced_reporting_interval = 300000;

int up_value, dn_value;
int test_up, test_dn, is_up;
int entry_state;

long last_forced_report = 0;
long last_msg = 0;
long last_temp_msg = 0;

float currentAmbientTempC, currentCaseTempC;

int door_trigger();

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

String describeCurrentState(int current_state, int previous_state){
  //we get to this if the new current state is different from the entry state
  String stateDescription;
  if(previous_state == 1 && current_state == 0){
    //we're in motion towards open
    stateDescription = "opening";
  }else if(previous_state == -1 && current_state == 0){
    //we're in motion towards closed
    stateDescription = "closing";
  }else if(current_state == -1 && previous_state == 0){
    stateDescription = "open";
  }else if(current_state == 1 && previous_state == 0){
    stateDescription = "closed";
  }else{
    stateDescription = "unknown";
  }
  return stateDescription;

}


bool sendStatus(String value) {

}

void setupHandler() {

}



void setup() {
  Serial.begin(115200);
  // Homie.setLoggingPrinter(&Serial);
  // Homie.setSetupFunction(setupHandler);
  setupSPIFFS();

  Homie_setFirmware(FW_NAME, FW_VERSION);
  Homie.setup();
}

void loop() {
  Homie.loop();
  door_status = getCurrentDoorSensorValues();



}
