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

int getCurrentDoorStatus() {
  up_value = digitalRead(UP_SENSE_PIN);
  dn_value = digitalRead(DN_SENSE_PIN);

  //current status == 1 if door is open, -1 if door is closed
  int current_status;
  current_status = up_value - dn_value;
  if(current_status == 1 && current_status > door_status) {
    //should be going from opening to open

  }else if(current_status == 0 && current_status < door_status) {
    //should be going from open to closing

  }else if(current_status == 0 && current_status > door_status) {
    //should be going from closed to opening

  }else if(current_status == -1 && current_status < door_status) {
    //should be going from closing to closed

  }else{
    //current status is unchanged from existing status

  }

  digitalWrite(UP_IND_PIN, up_value);
  digitalWrite(DN_IND_PIN, dn_value);
  if(current_status != door_status){

  }
}

bool sendStatus(String value) {
  if(value == "door_state" || value == "") {
    // String doorState = is_up ? ""
    // DoorNode.setProperty("state").send
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
  Homie.setup();
}

void loop() {
  Homie.loop();
}
