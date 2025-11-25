#include <Arduino.h>
#include <pubsubclient.h>
#include <WiFi.h>
#include "DFRobot_BloodOxygen_S.h"

#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x57
  DFRobot_BloodOxygen_S_I2C MAX30102(&Wire ,I2C_ADDRESS);
#else
/* ---------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |
 * ---------------------------------------------------------------------------------------------------------------*/
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
SoftwareSerial mySerial(4, 5);
DFRobot_BloodOxygen_S_SoftWareUart MAX30102(&mySerial, 9600);
#else
DFRobot_BloodOxygen_S_HardWareUart MAX30102(&Serial1, 9600); 
#endif
#endif







// ======================MQTT setting ========================

const char *mqtt_ecg = "homeTrainerCastres/Group1-B/ECG"; // BPM

// ====================== MQTT globals =======================
static unsigned long lastDebugPublishTime = 0;

// ====================== ECG settings =======================

const int ecgPin = A0;          // sortie AD8232 → A0
int threshold = 380;            // seuil de détection de battement (à ajuster)
unsigned long lastBeat = 0;     // temps du dernier battement (ms)

// Échantillonnage ECG pour MQTT (tous les 20 ms ≈ 50 Hz)
unsigned long lastECGPublishTime = 0;
const unsigned long ecgPublishInterval = 2000; // ms

const int greenLedPin = 4;
const int yellowLedPin = 2;
const int redLedPin = 3;
const int boutonPin = 7;



// WiFi settings
// TODO : Replace with your WiFi credentials here
const char *ssid = "Allumettes ";
const char *password = "salutc'estmoichoupi";

// MQTT Broker settings
// TODO : Update with your MQTT broker settings here if needed
const char *mqtt_broker = "broker.emqx.io";     // EMQX broker endpoint
const char *mqtt_topic1 = "homeTrainerCastres/Group1-B/MAC"; // MQTT topic
const char *mqtt_heartbeat = "homeTrainerCastres/Group1-B/Heartbeat"; // MQTT topic
const char *mqtt_spo2 = "homeTrainerCastres/Group1-B/SPO2"; // MQTT topic
const char *mqtt_temperature = "homeTrainerCastres/Group1-B/Temperature"; // MQTT topic
const char *mqtt_reset = "homeTrainerCastres/Group1-B/reset";
const int mqtt_port = 1883;                     // MQTT port (TCP)
String client_id = "ArduinoClient-";
String MAC_address = "";

// Other global variables
static unsigned long lastPublishTime = 0;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

void connectToWiFi();
void connectToMQTTBroker();
void mqttCallback(char *topic, byte *payload, unsigned int length);

void setup()
{
  Serial.begin(9600);
  connectToWiFi();
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(mqttCallback);
  connectToMQTTBroker();

  while (false == MAX30102.begin())
  {
    Serial.println("init fail!");
    delay(1000);
  }
  Serial.println("init success!");
  Serial.println("start measuring...");
  MAX30102.sensorStartCollect();

  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(boutonPin, INPUT_PULLUP);

  digitalWrite(greenLedPin, LOW);
  digitalWrite(yellowLedPin, LOW);
  digitalWrite(redLedPin, LOW);
  mqtt_client.setCallback(mqttCallback);
}

void printMacAddress()
{
  byte mac[6];
  Serial.print("MAC Address: ");
  WiFi.macAddress(mac);
  for (int i = 0; i < 6; i++)
  {
    MAC_address += String(mac[i], HEX);
    if (i < 5)
      MAC_address += ":";
    if (mac[i] < 16)
    {
      client_id += "0";
    }
    client_id += String(mac[i], HEX);
  }
  Serial.println(MAC_address);
}

void connectToWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  delay(3000);
  printMacAddress();
  Serial.println("Connected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());
}

void connectToMQTTBroker()
{
  while (!mqtt_client.connected())
  {
    Serial.print("Connecting to MQTT Broker as ");
    Serial.print(client_id.c_str());
    Serial.println(".....");
    if (mqtt_client.connect(client_id.c_str()))
    {
      Serial.println("Connected to MQTT broker");
      mqtt_client.subscribe(mqtt_topic1);
      mqtt_client.subscribe(mqtt_heartbeat);
      mqtt_client.subscribe(mqtt_spo2);
      mqtt_client.subscribe(mqtt_temperature);
      mqtt_client.subscribe(mqtt_reset);
      // mqtt_client.subscribe(mqtt_led);
      // Publish message upon successful connection
      String message = "Hello EMQX I'm " + client_id;
      mqtt_client.publish(mqtt_topic1, message.c_str());
    }
    else
    {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  String messageTemp;
  for (int i = 0; i < length; i++)
  {
    // Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println(messageTemp);
  Serial.println("-----------------------");
  
  // if(MAX30102._sHeartbeatSPO2.Heartbeat < 85)
    // {
    //   digitalWrite(greenLedPin, HIGH);
    //   digitalWrite(yellowLedPin, LOW);
    //   digitalWrite(redLedPin, LOW);
    // }
    // else if(MAX30102._sHeartbeatSPO2.Heartbeat < 120)
    // {
    //   digitalWrite(yellowLedPin, HIGH);
    //   digitalWrite(greenLedPin, LOW);
    //   digitalWrite(redLedPin, LOW);
    // }
    // else if(MAX30102._sHeartbeatSPO2.Heartbeat > 120)
    // {
    //   digitalWrite(redLedPin, HIGH);
    //   digitalWrite(greenLedPin, LOW);
    //   digitalWrite(yellowLedPin, LOW);
    // }


}

void loop()
{
  if (!mqtt_client.connected())
  {
    connectToMQTTBroker();
  }
  mqtt_client.loop();
  // TODO: Add your main code here, to run repeatedly (e.g., sensor readings, publishing messages, etc. )
  // Example below : Publish a message every 10 seconds
  unsigned long currentTime = millis();
  if (currentTime - lastPublishTime >= 10000) // 10 seconds
  {
    // String message = "Joey et Florestan :))) MAC = " + MAC_address;
    MAX30102.getHeartbeatSPO2();

    Serial.print("SPO2 is : ");
    Serial.print(MAX30102._sHeartbeatSPO2.SPO2);
    Serial.println("%");
    String message = String(MAX30102._sHeartbeatSPO2.SPO2);
    mqtt_client.publish(mqtt_spo2, message.c_str());

    Serial.print("heart rate is : ");
    Serial.print(MAX30102._sHeartbeatSPO2.Heartbeat);
    Serial.println("Times/min");
    message = String(MAX30102._sHeartbeatSPO2.Heartbeat);
    mqtt_client.publish(mqtt_heartbeat, message.c_str());

    // if(MAX30102._sHeartbeatSPO2.Heartbeat < 85)
    // {
    //   digitalWrite(greenLedPin, HIGH);
    //   digitalWrite(yellowLedPin, LOW);
    //   digitalWrite(redLedPin, LOW);
    // }
    // else if(MAX30102._sHeartbeatSPO2.Heartbeat < 120)
    // {
    //   digitalWrite(yellowLedPin, HIGH);
    //   digitalWrite(greenLedPin, LOW);
    //   digitalWrite(redLedPin, LOW);
    // }
    // else if(MAX30102._sHeartbeatSPO2.Heartbeat > 120)
    // {
    //   digitalWrite(redLedPin, HIGH);
    //   digitalWrite(greenLedPin, LOW);
    //   digitalWrite(yellowLedPin, LOW);
    // }


    Serial.print("Temperature value of the board is : ");
    Serial.print(MAX30102.getTemperature_C());
    Serial.println(" ℃");
    message = String(MAX30102.getTemperature_C());
    mqtt_client.publish(mqtt_temperature, message.c_str());
    lastPublishTime = currentTime;
  }









  // --------- 1) Lecture & publication ECG brut ---------
  if (currentTime - lastECGPublishTime >= ecgPublishInterval)
  {
    lastECGPublishTime = currentTime;

    int ecg = analogRead(ecgPin);        // valeur ECG brute
    String ecgMessage = String(ecg);
    mqtt_client.publish(mqtt_topic1, ecgMessage.c_str()); // ECG -> topic1

    // aussi sur le port série pour debug
    Serial.print("ECG: ");
    Serial.println(ecg);

    // --------- 2) Détection du battement & BPM ---------
    if (ecg > threshold && (currentTime - lastBeat) > 300) // 300 ms => max ~200 BPM
    {
      int bpm = 0;
      if (lastBeat > 0)
      {
        bpm = 60000 / (currentTime - lastBeat); // calcul BPM
      }
      lastBeat = currentTime;

      // Publier BPM sur ECG
      String bpmMessage = String(bpm);
      mqtt_client.publish(mqtt_ecg, bpmMessage.c_str());

      Serial.print("BPM: ");
      Serial.println(bpm);
    }
  }

  // Exemple : message de debug toutes les 10 s (optionnel)
  if (currentTime - lastDebugPublishTime >= 10000)
  {
    String debugMsg = "Still alive at " + String(currentTime / 1000) + "s";
    mqtt_client.publish(mqtt_topic1, debugMsg.c_str());
    lastDebugPublishTime = currentTime;
  }
  String reset = "Reset";
  digitalRead(boutonPin); //rep soit 1(peut etre pas appuiller) soit 0
  if (digitalRead(boutonPin)==0){
    mqtt_client.publish(mqtt_reset, reset.c_str());
  }

}