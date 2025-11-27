#include <Arduino.h>
#include <pubsubclient.h>
#include <WiFi.h>
#include "DFRobot_BloodOxygen_S.h"

#define I2C_COMMUNICATION

#ifdef I2C_COMMUNICATION
#define I2C_ADDRESS 0x57
DFRobot_BloodOxygen_S_I2C MAX30102(&Wire, I2C_ADDRESS);
#else
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

// ====================== ECG settings AMÉLIORÉS =======================
const int ecgPin = A0;

// Timing pour échantillonnage rapide
unsigned long lastECGSampleTime = 0;
const unsigned long ecgSampleInterval = 10; // 10ms = 100Hz échantillonnage

// Filtrage du signal
const int BUFFER_SIZE = 10;
int ecgBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int ecgFiltered = 0;
int ecgPrevious = 0;

// Détection de battements
unsigned long lastBeat = 0;
const unsigned long MIN_BEAT_INTERVAL = 300;  // 300ms = max 200 BPM
const unsigned long MAX_BEAT_INTERVAL = 2000; // 2000ms = min 30 BPM
bool beatDetected = false;
int peakValue = 0;

// Seuil adaptatif
int signalMax = 0;
int signalMin = 1023;
int threshold = 512;
unsigned long lastThresholdUpdate = 0;

// Calcul BPM moyenné
const int BPM_BUFFER_SIZE = 5;
int bpmBuffer[BPM_BUFFER_SIZE];
int bpmBufferIndex = 0;
int bpmSum = 0;
int bpmCount = 0;
int currentBPM = 0;

// Publication MQTT ECG (toutes les 2 secondes comme avant)
unsigned long lastECGPublishTime = 0;
const unsigned long ecgPublishInterval = 2000;

// Pins
const int greenLedPin = 4;
const int yellowLedPin = 2;
const int redLedPin = 3;
const int boutonPin = 7;

// WiFi settings
const char *ssid = "Allumettes ";
const char *password = "salutc'estmoichoupi";

// MQTT Broker settings
const char *mqtt_broker = "broker.emqx.io";
const char *mqtt_topic1 = "homeTrainerCastres/Group1-B/MAC";
const char *mqtt_heartbeat = "homeTrainerCastres/Group1-B/Heartbeat";
const char *mqtt_spo2 = "homeTrainerCastres/Group1-B/SPO2";
const char *mqtt_temperature = "homeTrainerCastres/Group1-B/Temperature";
const char *mqtt_reset = "homeTrainerCastres/Group1-B/reset";
const char *mqtt_led = "homeTrainerCastres/Group1-B/led";
const int mqtt_port = 1883;
String client_id = "ArduinoClient-";
String MAC_address = "";

// Other global variables
static unsigned long lastPublishTime = 0;
WiFiClient espClient;
PubSubClient mqtt_client(espClient);

void connectToWiFi();
void connectToMQTTBroker();
void mqttCallback(char *topic, byte *payload, unsigned int length);

// ========== FONCTIONS ECG AMÉLIORÉES ==========

// Filtrage du signal (moyenne mobile)
int filterECGSignal(int rawValue)
{
  ecgBuffer[bufferIndex] = rawValue;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;

  long sum = 0;
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    sum += ecgBuffer[i];
  }
  return sum / BUFFER_SIZE;
}

// Mise à jour du seuil adaptatif
void updateThreshold(int value)
{
  if (value > signalMax)
    signalMax = value;
  if (value < signalMin)
    signalMin = value;

  // Recalculer le seuil toutes les 2 secondes
  if (millis() - lastThresholdUpdate > 2000)
  {
    int range = signalMax - signalMin;
    threshold = signalMin + (range * 6) / 10; // 60% entre min et max

    // Reset progressif pour adaptation continue
    signalMax = (signalMax * 9 + value) / 10;
    signalMin = (signalMin * 9 + value) / 10;

    lastThresholdUpdate = millis();

    Serial.print("Seuil: ");
    Serial.print(threshold);
    Serial.print(" | Range: [");
    Serial.print(signalMin);
    Serial.print(" - ");
    Serial.print(signalMax);
    Serial.println("]");
  }
}

// Calcul BPM moyenné
int calculateAverageBPM(int newBPM)
{
  // Ignorer les valeurs aberrantes
  if (newBPM < 30 || newBPM > 200)
  {
    return currentBPM; // Garder la valeur précédente
  }

  // Ajouter au buffer
  if (bpmCount < BPM_BUFFER_SIZE)
  {
    bpmSum += newBPM;
    bpmBuffer[bpmBufferIndex] = newBPM;
    bpmCount++;
  }
  else
  {
    bpmSum = bpmSum - bpmBuffer[bpmBufferIndex] + newBPM;
    bpmBuffer[bpmBufferIndex] = newBPM;
  }

  bpmBufferIndex = (bpmBufferIndex + 1) % BPM_BUFFER_SIZE;
  return bpmSum / bpmCount;
}

// Traitement ECG principal
void processECG()
{
  unsigned long currentTime = millis();

  // Échantillonnage rapide (100Hz)
  if (currentTime - lastECGSampleTime >= ecgSampleInterval)
  {
    lastECGSampleTime = currentTime;

    // Lecture et filtrage
    int ecgRaw = analogRead(ecgPin);
    ecgFiltered = filterECGSignal(ecgRaw);

    // Mise à jour du seuil adaptatif
    updateThreshold(ecgFiltered);

    // Détection de battement
    unsigned long timeSinceLastBeat = currentTime - lastBeat;

    // Détection de montée (crossing threshold)
    if (!beatDetected && ecgFiltered > threshold && ecgPrevious <= threshold)
    {
      if (timeSinceLastBeat > MIN_BEAT_INTERVAL)
      {
        beatDetected = true;
        peakValue = ecgFiltered;
      }
    }

    // Suivre le pic
    if (beatDetected)
    {
      if (ecgFiltered > peakValue)
      {
        peakValue = ecgFiltered;
      }
      // Détection de descente (fin du pic)
      else if (ecgFiltered < threshold)
      {
        // Battement confirmé!
        if (lastBeat > 0 && timeSinceLastBeat < MAX_BEAT_INTERVAL)
        {
          int instantBPM = 60000 / timeSinceLastBeat;
          currentBPM = calculateAverageBPM(instantBPM);

          Serial.print(">>> BATTEMENT! BPM instant: ");
          Serial.print(instantBPM);
          Serial.print(" | BPM moyen: ");
          Serial.println(currentBPM);
        }

        lastBeat = currentTime;
        beatDetected = false;
      }
    }

    ecgPrevious = ecgFiltered;
  }

  // Publication MQTT toutes les 2 secondes
  if (currentTime - lastECGPublishTime >= ecgPublishInterval)
  {
    lastECGPublishTime = currentTime;

    if (currentBPM > 0)
    {
      String bpmMessage = String(currentBPM);
      mqtt_client.publish(mqtt_ecg, bpmMessage.c_str());

      Serial.print("ECG BPM publié: ");
      Serial.println(currentBPM);
    }
    else
    {
      Serial.println("ECG: En attente de battements...");
      Serial.println(0 + " bpm/min");
    }
  }
}

// ========== SETUP ==========
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

  // Initialiser les buffers ECG
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    ecgBuffer[i] = 512;
  }
  for (int i = 0; i < BPM_BUFFER_SIZE; i++)
  {
    bpmBuffer[i] = 0;
  }

  Serial.println("=== Système ECG initialisé ===");
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
      mqtt_client.subscribe(mqtt_led);
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
    messageTemp += (char)payload[i];
  }
  Serial.println(messageTemp);
  Serial.println("-----------------------");

  if (String(topic) == String(mqtt_led))
  {
    if (messageTemp == "green")
    {
      digitalWrite(greenLedPin, HIGH);
      digitalWrite(yellowLedPin, LOW);
      digitalWrite(redLedPin, LOW);
    }
    else if (messageTemp == "yellow")
    {
      digitalWrite(yellowLedPin, HIGH);
      digitalWrite(greenLedPin, LOW);
      digitalWrite(redLedPin, LOW);
    }
    else if (messageTemp == "red")
    {
      digitalWrite(redLedPin, HIGH);
      digitalWrite(greenLedPin, LOW);
      digitalWrite(yellowLedPin, LOW);
    }
    else if (messageTemp == "none")
    {
      digitalWrite(yellowLedPin, LOW);
      digitalWrite(greenLedPin, LOW);
      digitalWrite(redLedPin, LOW);
    }
  }
}

// ========== LOOP PRINCIPAL ==========
void loop()
{
  if (!mqtt_client.connected())
  {
    connectToMQTTBroker();
  }
  mqtt_client.loop();

  unsigned long currentTime = millis();

  // MAX30102 - Publication toutes les 10 secondes
  if (currentTime - lastPublishTime >= 2000)
  {
    MAX30102.getHeartbeatSPO2();
    String message;

    // Serial.print("SPO2 is : ");
    // Serial.print(MAX30102._sHeartbeatSPO2.SPO2);
    // Serial.println("%");
    // String message = String(MAX30102._sHeartbeatSPO2.SPO2);
    // mqtt_client.publish(mqtt_spo2, message.c_str());

    Serial.print("heart rate is : ");
    Serial.print(MAX30102._sHeartbeatSPO2.Heartbeat);
    Serial.println("Times/min");
    message = String(MAX30102._sHeartbeatSPO2.Heartbeat);
    mqtt_client.publish(mqtt_heartbeat, message.c_str());

    // Serial.print("Temperature value of the board is : ");
    // Serial.print(MAX30102.getTemperature_C());
    // Serial.println(" ℃");
    // message = String(MAX30102.getTemperature_C());
    // mqtt_client.publish(mqtt_temperature, message.c_str());
    lastPublishTime = currentTime;
  }

  // ========== TRAITEMENT ECG AMÉLIORÉ ==========
  processECG();

  // Message de debug toutes les 10s
  if (currentTime - lastDebugPublishTime >= 10000)
  {
    String debugMsg = "Alive " + String(currentTime / 1000) + "s | ECG BPM: " + String(currentBPM);
    mqtt_client.publish(mqtt_topic1, debugMsg.c_str());
    lastDebugPublishTime = currentTime;
  }

  // Bouton reset
  if (digitalRead(boutonPin) == LOW)
  {
    String reset = "Reset";
    mqtt_client.publish(mqtt_reset, reset.c_str());
  }
}