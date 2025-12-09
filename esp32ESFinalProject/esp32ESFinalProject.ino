// WiFi ---------------------------------------
#include <WiFi.h>
WiFiClient espClient;

const char* ssid     = "sedijeyms";
const char* password = "cediejamessss";

// PubSub ---------------------------------------
#include <PubSubClient.h>
PubSubClient client(espClient);

// MQTT ---------------------------------------
const char* mqtt_server = "broker.hivemq.com";
const int   mqtt_port   = 1883;
const char* readingsTopic    = "47524F555038/es/finals/readings";
const char* lightTopic    = "47524F555038/es/finals/lightControl";

// JSON ---------------------------------------
#include <ArduinoJson.h>

// FAN ---------------------------------------
#include "EEPROM.h"

#define lightPin 14
int lightState;

// LED ---------------------------------------
#define greenLEDPin 18
#define redLEDPin 19
#define blueLEDPin 21

bool greenLEDOn = false;
bool redLEDOn = false;
bool blueLEDOn = false;

// Arduino Serial ---------------------------------------
#define RX2 16
#define TX2 17

HardwareSerial arduinoSerial(1); // Use UART1 (we can assign pins)

String readings;

void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  msg.trim();

  if(msg != "toggle light") return;

  // toggle fan
  digitalWrite(lightPin, lightState ? LOW : HIGH);

  lightState = !lightState;

  Serial.println(lightState);
  EEPROM.write(0, lightState);
  EEPROM.commit();

  int currentlightState = EEPROM.read(0);
  Serial.println(currentlightState);

  client.publish(lightTopic, "light toggled");
}

bool reconnect() {
  turnOnRedLED();
  Serial.print("Attempting MQTT connection... ");
  String clientId = "ESP32Client-";
  clientId += String(random(0xffff), HEX);

  if (client.connect(clientId.c_str())) {

    turnOnGreenLED();

    Serial.println("connected");
    client.subscribe(lightTopic);

    lightState = EEPROM.read(0);
    Serial.println(lightState);
    
    // initialize fan
    digitalWrite(lightPin, lightState ? HIGH : LOW);
    return true;
  } 
  else {
    turnOnRedLED();

    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" retrying...");
    return false;
  }
}

void turnOnGreenLED() {
  if(greenLEDOn) return;

  digitalWrite(greenLEDPin, HIGH); 
  greenLEDOn = true;

  digitalWrite(redLEDPin, LOW); 
  redLEDOn = false;

  digitalWrite(blueLEDPin, LOW); 
  blueLEDOn = false;
}

void turnOnRedLED() {
  if(redLEDOn) return;

  digitalWrite(redLEDPin, HIGH);
  redLEDOn = true;

  digitalWrite(greenLEDPin, LOW);
  greenLEDOn = false;

  digitalWrite(blueLEDPin, LOW); 
  blueLEDOn = false;
}

void turnOnBlueLED() {
  if(blueLEDOn) return;

  digitalWrite(blueLEDPin, HIGH);
  blueLEDOn = true;

  digitalWrite(greenLEDPin, LOW);
  greenLEDOn = false;

  digitalWrite(redLEDPin, LOW); 
  redLEDOn = false;
}


void ConnectToWifi(void *parameter) {

  while (1) {

    if(WiFi.status() != WL_CONNECTED) {

      turnOnRedLED();

      vTaskDelay(10 / portTICK_PERIOD_MS);
      Serial.println();
      Serial.print("Connecting to ");
      Serial.println(ssid);

      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);

      while (WiFi.status() != WL_CONNECTED) {
        turnOnRedLED();
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
      }

      Serial.println();
      Serial.print("WiFi connected, IP: ");
      Serial.println(WiFi.localIP());
    }

    // check wifi status every one second
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void ReconnectToMQTT(void *parameter) {

  while (1) {

    if(!client.connected() && WiFi.status() == WL_CONNECTED) {
      turnOnRedLED();
      reconnect();
    }

    // check mqtt status every five seconds
    vTaskDelay(pdMS_TO_TICKS(5000));    
  }
}

void ClientLoop(void *parameter) {

  while(1) {

    if(client.connected()) {
      client.loop();
    }

    // give time to other tasks
    vTaskDelay(pdMS_TO_TICKS(10));    
  }
}

void ReceiveReadings(void *parameter) {

  while(1) {

    if(arduinoSerial.available()) {
      readings = arduinoSerial.readStringUntil('\n');
      readings.trim();
    }

    // give time to other tasks
    vTaskDelay(pdMS_TO_TICKS(500));    
  }
}

void PublishToDHT11(void *parameter) {
  while(1) {

    if(client.connected() && WiFi.status() == WL_CONNECTED && readings != "") {

      // catch failed reads
      if(readings == "Reading Failed.") {
        Serial.println("Reading Failed.");
        turnOnBlueLED();
        vTaskDelay(pdMS_TO_TICKS(500));    
        turnOnRedLED();
        continue;
      }

      int p1 = readings.indexOf('|');
      int p2 = readings.indexOf('|', p1 + 1);
      int p3 = readings.indexOf('|', p2 + 1);
      int p4 = readings.indexOf('|', p3 + 1);
      int p5 = readings.indexOf('|', p4 + 1);
      int p6 = readings.indexOf('|', p5 + 1);

      String windDirection = readings.substring(0, p1);
      String windSpeed = readings.substring(p1 + 1, p2);
      String pressure = readings.substring(p2 + 1, p3);
      String temperature = readings.substring(p3 + 1, p4);
      String humidity = readings.substring(p4 + 1, p5);
      String rain = readings.substring(p5 + 1, p6);
      String ldr = readings.substring(p6 + 1);

      StaticJsonDocument<512> postDoc;
      postDoc["windDirection"] = windDirection;
      postDoc["windSpeed"] = windSpeed;
      postDoc["pressure"] = pressure;
      postDoc["temperature"] = temperature;
      postDoc["humidity"] = humidity;
      postDoc["rainValue"] = rain;
      postDoc["ldrValue"] = ldr;

      char buffer[512];
      size_t size = serializeJson(postDoc, buffer);

      client.publish(readingsTopic, buffer, size);
      
      if(temperature.toFloat() < 24.0 || temperature.toFloat() > 30.0 || humidity.toFloat() < 40.0 || humidity.toFloat() > 80.0 || rain.toInt() < 700) {
        turnOnBlueLED();
      }
      else {
        turnOnGreenLED();
      }     
      
    }

    // publish every one second
    vTaskDelay(pdMS_TO_TICKS(1000));    
  }
}

void setup() {
  Serial.begin(115200);
  arduinoSerial.begin(115200, SERIAL_8N1, RX2, TX2);

  EEPROM.begin(64);

  // light
  pinMode(lightPin, OUTPUT);
  digitalWrite(lightPin, LOW);

  // LED
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);

  // create task for connecting to wifi
  xTaskCreate( 
    ConnectToWifi,      // Function to be called
    "Connect to Wifi",  // Name of task
    4096,               // Stack size
    NULL,               // argument to pass to function
    5,                  // Task Priority 
    NULL                // Task handle
  );

  // MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // create task for reconnecting to mqtt broker
  xTaskCreate( 
    ReconnectToMQTT,              // Function to be called
    "Reconnect to MQTT Broker",   // Name of task
    4096,                         // Stack size
    NULL,                         // argument to pass to function
    4,                            // Task Priority 
    NULL                          // Task handle
  );

  // create task for mqtt client loop
  xTaskCreate( 
    ClientLoop,         // Function to be called
    "MQTT Client Loop", // Name of task
    4096,               // Stack size
    NULL,               // argument to pass to function
    3,                  // Task Priority 
    NULL                // Task handle
  );

  // create task for mqtt client loop
  xTaskCreate( 
    ReceiveReadings,         // Function to be called
    "Receive Arduino Readings", // Name of task
    4096,               // Stack size
    NULL,               // argument to pass to function
    2,                  // Task Priority 
    NULL                // Task handle
  );

  // create task for publishing dht11 data
  xTaskCreate( 
    PublishToDHT11,           // Function to be called
    "Publish To DHT11 Topic", // Name of task
    4096,                     // Stack size
    NULL,                     // argument to pass to function
    1,                        // Task Priority 
    NULL                      // Task handle
  );
}

void loop() {}
