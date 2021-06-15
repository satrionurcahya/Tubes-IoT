#include <WiFi.h>
#include <PubSubClient.h>
 
const char* ssid = "Galaxy A30sCC47";
const char* password = "12ai23vr34ar";
const char* mqttServer = "192.168.43.212";
const int mqttPort = 1883;

#define MQTT_PUB_TEMP "esp32/dht/temperature"
#define MQTT_PUB_HUM  "esp32/dht/humidity"
#define MQTT_PUB_HEAR "esp32/heart/heartpulse"

unsigned long previousMillis = 0;   
const long interval = 10000;        

#include "DHT.h"
#define DHTPIN 4     
#define DHTTYPE DHT11   
DHT dht(DHTPIN, DHTTYPE);
float t = dht.readTemperature();
float h = dht.readHumidity();

int const PULSE_SENSOR_PIN = 32;   // 'S' Signal pin connected to A0
int Signal;                // Store incoming ADC data. Value can range from 0-1024
int Threshold = 3500;       // Determine which Signal to "count as a beat" and which to ignore.
 
WiFiClient espClient;
PubSubClient client(espClient);

int Buzzer = 12;

void WiFiMQTT(){
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}
 
void setup() { 
  Serial.begin(115200);
  WiFiMQTT();
  Serial.println("Mulai Deteksi");
  dht.begin();
  pinMode (Buzzer, OUTPUT);
 
}
 
void loop() {
  delay(1000);
  unsigned long currentMillis = millis();
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (isnan(h) || isnan(t)) {
      Serial.println("Semsor tidak terbaca!");
      return;
    } else {
      Signal = analogRead(PULSE_SENSOR_PIN); // Read the sensor value

      if(Signal >= Threshold){                // If the signal is above threshold, turn on the LED
        if(t >= 32.61 || h >= 63.42){
          heart();
          Serial.println();
          dht11();
          Serial.println();
          buzzerOn();
          
        }
      } else {
        client.publish("esp32/esp32test", "Hello from ESP32learning");
        // Publish an MQTT message on topic esp32/dht/temperature
        uint16_t packetIdPub1 = client.publish(MQTT_PUB_TEMP, String(t).c_str());                            
        Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_TEMP, packetIdPub1);
        Serial.printf("Message: %.2f \n", t);

        // Publish an MQTT message on topic esp32/dht/humidity
        uint16_t packetIdPub2 = client.publish(MQTT_PUB_HUM, String(h).c_str());                            
        Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
        Serial.printf("Message: %.2f \n", h);

        //Publish an MQTT message on topic esp32/heart/heartpulse
        
        uint16_t packetIdPub3 = client.publish(MQTT_PUB_HEAR, String(Signal).c_str());
        Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HEAR, packetIdPub3);
        Serial.printf("Message: %.2f \n", Signal);
    
        buzzerOff();
        heart();
        Serial.println();
        dht11();
      }
    }
  }
}

void dht11(){
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.println();
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" *C ");
}

void heart(){
  Serial.print("Heart Rate: ");
  Serial.print(Signal);                // Send the signal value to serial plotter
}

void buzzerOn(){
  digitalWrite (Buzzer, HIGH); //turn buzzer on
  delay(5000);
}

void buzzerOff(){
  digitalWrite (Buzzer, LOW);  //turn buzzer off
  delay(1000);
}
