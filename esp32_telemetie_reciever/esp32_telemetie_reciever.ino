#include <WiFi.h>
#include <PubSubClient.h>

// Update these with values suitable for your network.
const int16_t SendStatus = 5000;
uint32_t LastMillisTelemetrie;
uint32_t LastMillis4GOnline;

// == WiFi Settings ==
// Replace the next variables with your SSID/Password combination
char* ssid = "Zonneboot";
char* password = "Zonnepanelen1";

// == MQTT Broker settings ==
const char* mqttServer = "telemetrie.zonnebootteam.nl";
const int mqttPort = 1883;
const char* mqttUser = "Zonneboot";
const char* mqttPassword = "Zonnepanelen1";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (300)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, "esp32/4G_online") == 0) {
    LastMillis4GOnline = millis();
  }
  else if ((strcmp(topic, "esp32/status") == 0) && millis() - LastMillisTelemetrie > SendStatus) {
    LastMillisTelemetrie = millis();
    LastMillis4GOnline = millis();
    Serial.print("Vvl: ");
    Serial.print((char)payload[0]);
    Serial.print(", Avl: ");
    Serial.print((char)payload[2]);
    Serial.print(", Scherm: ");
    Serial.println((char)payload[4]);
  }
  else if (strcmp(topic, "esp32/pid") == 0) {
    LastMillisTelemetrie = millis();
    LastMillis4GOnline = millis();
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
  
  Serial.println();
}
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // ... and resubscribe
      client.subscribe("esp32/pid");
      client.subscribe("esp32/status");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(230400);
  setup_wifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (millis() - LastMillis4GOnline > SendStatus) {
    LastMillis4GOnline = millis();
    Serial.println("esp32 Offline");
  } else if (millis() - LastMillisTelemetrie > SendStatus) {
    
    static uint32_t lastMillisOnline = 0;
    if (millis() - lastMillisOnline > SendStatus) {
      lastMillisOnline = millis();
      Serial.println("esp32 Online");
    }
  }
}
