#include <ArduinoJson.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi
const char *ssid = "IniWiFi"; // Enter your WiFi name
const char *password = "password112233";  // Enter WiFi password
unsigned long previousMillis = 0;
unsigned long interval = 3000;

void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Successfully connected to Access Point");
}

void Get_IPAddress(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("WIFI is connected!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WIFI access point");
  Serial.println("Reconnecting...");
  WiFi.begin(ssid, password);
}

// MQTT Broker
const char *mqtt_broker = "192.168.17.41";
const char *topic_relay = "dev/esp32_1/relay/+";
const char *topic_sensors = "dev/esp32_1/sensors/response";
const char *topic_control = "dev/esp32_1/control";
const char *mqtt_username = "dev-be";
const char *mqtt_password = "dev-be";
const int mqtt_port = 1883;

// Sensor
char bufferSensor[100];
// Ultrasonic
const int ultrasonicTrig = 5;
const int ultrasonicEcho = 18;
long ultrasonicDuration;
float distanceCm;
const double SOUND_SPEED  = 0.034;
const int containerHeight = 38;
// Relay
const int ledPin = 13; //Lamp
const int sprayerPin = 33 //Pump;
// pH
// TDS
// DHT
const int dhtPin = 25;
#define DHTTYPE DHT11
DHT dht(dhtPin, DHTTYPE);
// Sensor Reading Params
long lastMillisSensor = 0;
int readingInterval = 2000; // in ms

// Relay
long lastMillisPump = 0;

// JSON
bool lamp_status;
float ph_min;
float ph_max;
int tds_min;
int tds_max;
int spray_interval = 5;
int spray_duration = 5;

// Initial Params from BE
bool isInitial=0;

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  // Set software serial baud to 115200;
  Serial.begin(115200);
  // connecting to a WiFi network
  Serial.print("WIFI status = ");
  Serial.println(WiFi.getMode());
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  delay(1000);
  Serial.print("WIFI status = ");
  Serial.println(WiFi.getMode());
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  WiFi.onEvent(Wifi_connected,SYSTEM_EVENT_STA_CONNECTED);
  WiFi.onEvent(Get_IPAddress, SYSTEM_EVENT_STA_GOT_IP);
  WiFi.onEvent(Wifi_disconnected, SYSTEM_EVENT_STA_DISCONNECTED); 
 
  //connecting to a mqtt broker
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  connectMQTT();

  // publish and subscribe
  client.subscribe(topic_relay);
  client.subscribe(topic_control);
  client.subscribe(topic_sensors);

  // Pin Setup
  pinMode(ledPin, OUTPUT);
  pinMode(sprayerPin, OUTPUT);
  pinMode(ultrasonicTrig, OUTPUT);
  pinMode(ultrasonicEcho, INPUT);
  digitalWrite(ledPin, LOW);
  digitalWrite(sprayerPin, HIGH);

  // Sensors Setup
  dht.begin(); 
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
      messageTemp += (char)payload[i];
  }
  Serial.println();
  Serial.println("-----------------------");
  if (String(topic) == "dev/esp32_1/relay/1") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("Relay 1 on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("Relay 1 off");
      digitalWrite(ledPin, LOW);
    }
  }
  if (String(topic) == "dev/esp32_1/control"){
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    ph_min = doc["ph_min"];
    ph_max = doc["ph_max"];
    tds_min = doc["tds_min"];
    tds_max = doc["tds_max"];
    spray_interval = doc["spray_interval"];
    spray_duration = doc["spray_duration"];

    Serial.println(spray_interval);
    Serial.println(spray_duration);
  }
}

void connectMQTT(){
  while (!client.connected()) {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s trying to connects to the %s broker\n", client_id.c_str(), mqtt_broker);
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
        Serial.printf("The %s mqtt broker connected", mqtt_broker);
    } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 2 seconds");
        // Wait 2 seconds before retrying
        delay(2000);
    }
  }  
}

void loop() {
  if (!client.connected()) {
    Serial.print("Connection with MQTT Broker Lost. Trying to reconnect");
    connectMQTT();
  }
  
  client.loop();

  long now=millis();
  if (now - lastMillisSensor > readingInterval) {
    lastMillisSensor = now;
  
    //DHT
    char tempString[8];
    char humString[8];
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    dtostrf(temperature, 1, 2, tempString);
    dtostrf(humidity, 1, 2, humString);
    strcpy(bufferSensor,tempString);
    strcat(bufferSensor,",");
    strcat(bufferSensor,humString);
  
    Serial.println(bufferSensor);
  
    //client.publish("dev/esp32_1/sensors", bufferSensor);
  
    //Ultrasonic
    digitalWrite(ultrasonicTrig, LOW);
    delayMicroseconds(2);
    digitalWrite(ultrasonicTrig, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasonicTrig, LOW);
    
    ultrasonicDuration = pulseIn(ultrasonicEcho, HIGH);
  
    distanceCm = ultrasonicDuration * SOUND_SPEED/2;
    Serial.println(100 - (distanceCm/(containerHeight-4)*100));
  }

  if (now - lastMillisPump > spray_interval * 1000) {
    digitalWrite(sprayerPin, LOW);
  }
  if (now - lastMillisPump > (spray_duration + spray_interval) * 1000 ) {
    lastMillisPump = now;
    digitalWrite(sprayerPin, HIGH);
  }
}
