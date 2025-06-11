/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com
*********/
#include <SPI.h>
#include <mcp2515.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GPS.h>
#include <cstring>

typedef uint8_t u8;
//CAN setup
struct can_frame canMsg;
MCP2515 mcp2515(5);
int msg_count = 0;
unsigned long prev_millis = 0;
const long interval = 10000;
/* pinout
VCC -> 3.3v
GND -> GND
CS -> pin 5
SCK -> GPIO 18
MOSI -> GPIO 23
MISO -> GPIO 19
INT -> GPIO 4 (optional)

*/

// voor de accu gaan er tussen de 1670-1685 berichten over de can bus.
// als dit niet het geval is dan is er waarschijnlijk een kabel los/stuk.
#define NUM_CELLS 12
#define MS_PER_SEC 1000
#define MS_PER_MIN 60000
#define TESTING false
#define RESET_PIN 14

double GPSspeed_filter;
char speedbuf[8];

// == GPS Settings ==
//HardwareSerial Serial1(2);
#define GPSSerial Serial2
Adafruit_GPS GPS(&Serial2);
#define GPSECHO false
uint32_t timer = millis();

// == buffer ==
//char* fix2;

// == WiFi Settings ==
// Replace the next variables with your SSID/Password combination
char* ssid = "Zonneboot";
char* password = "Zonnepanelen1";


// == MQTT Broker settings ==
const char* mqtt_server = "telemetrie.zonnebootteam.nl:1880";
const int mqtt_server_port = 1883;// moet misschien aangepast worden naar 1880 
const char* mqtt_username = "Zonneboot";
const char* mqtt_password = "Zonnepanelen1";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

//== LED Settings ==
const int ledPin = 22;
const char* test = "";

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_server_port);
  client.setCallback(callback);

  Serial.println F(("Start CAN"));

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  Serial.println F(("CAN done"));

  pinMode(ledPin, OUTPUT);
  // == SETUP GPS ==
  Serial.println("GPS Serial started");
  GPS.begin(9600);
  // setting up to recieve RMC and GGA data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Setting update rate, either 1HZ, 5HZ or 10HZ
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); // 5 Hz update rate for sending data from gps to micocontroller
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ); // 5 Hz update rate for the gps it self
  //Request updates on antenna status, we have no clue what it does....
  //GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
  // Changes the output state according to the message
  if (String(topic) == "esp32/gps") {
    Serial.print("Changing output to ");
    if (messageTemp == "on") {
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if (messageTemp == "off") {
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("espClient", mqtt_username, mqtt_password)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/gps");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {
  if (!client.connected()) {
    reconnect();
  } else{ // if connected
    if ((mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)) {
      // Serial.println(canMsg.can_id);
      // handelt alle can frames in de queue
      if (canMsg.can_id == 0x302 || canMsg.can_id == 0x402 || canMsg.can_id == 0x482 || canMsg.can_id == 0x202) {
        lees_accu();
      } else if (canMsg.can_id > 0x180 && canMsg.can_id < 0x2A0) {
        //Serial.println(canMsg.can_id, HEX);
        //if (canMsg.can_id < 0x190 && canMsg.can_id > 0x183)
        lees_mppts();
      }
    }
  }

  client.loop();

  // == GPS Main Loop ==
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  long now = millis();

  static long lastSpeed;

  if (now - lastSpeed > 100) {
    lastSpeed = now;
    // == Speed ==
    //char speedbuf[8];
     // GPSspeed_filter
    double GPSspeed = GPS.speed;
    GPSspeed_filter = GPSspeed_filter * 0.80 + GPSspeed * 0.20;
    dtostrf(GPSspeed_filter, 3, 2, speedbuf);

    Serial.print("Speed: ");
    Serial.println(GPSspeed, 2);
   // client.publish("esp32/gps/speed", speedbuf);
  }

  if (now - lastMsg > 1000) {
    lastMsg = now;
    // == FIX ==
    char fixbuf[8];
    double GPSfix = GPS.fix;
    dtostrf(GPSfix, 2, 0, fixbuf);
    Serial.print("Fix: ");
    Serial.println(GPSfix);
    client.publish("esp32/gps/fix", fixbuf);

    // == Quality ==
    char qualbuf[8];
    double GPSqual = GPS.fixquality;
    dtostrf(GPSqual, 2, 0, qualbuf);
    Serial.print("Quality: ");
    Serial.println(GPSqual);
    client.publish("esp32/gps/quality", qualbuf);

    // == Latitude ==
    char latitudebuf[16];
    double GPSlatitude = GPS.latitude;
    dtostrf(GPSlatitude, 6, 4, latitudebuf); // todo: test hogere resolutie dtostrf(GPSlatitude, 8, 5, latitudebuf);
    Serial.print("Latitude: ");
    Serial.println(GPSlatitude, 5);
    client.publish("esp32/gps/latitude", latitudebuf);

    // == Longitude ==
    //    char longitudebuf[16];
    //    double GPSlongitude = GPS.longitude;
    //    dtostrf(GPSlongitude,7,5,longitudebuf);
    //    Serial.print("Longitude: ");
    //    Serial.println(GPSlongitude, 4);
    //    client.publish("esp32/gps/longitude", longitudebuf);

    // == Longitude v2 ==
    client.publish("esp32/gps/longitude", String(GPS.longitude).c_str());

    // == Poles ==
    client.publish("esp32/gps/latitude/pole", String(GPS.lat).c_str());
    client.publish("esp32/gps/longitude/pole", String(GPS.lon).c_str());

    // == Speed ==
/*  char speedbuf[8];
    double GPSspeed_filter;
    double GPSspeed = GPS.speed;
    GPSspeed_filter = GPSspeed_filter * 0.0 + GPSspeed * 1.0;
    dtostrf(GPSspeed_filter, 3, 2, speedbuf);
*/
    Serial.print("Speed_filter: ");
    Serial.println(GPSspeed_filter, 2);
    client.publish("esp32/gps/speed", speedbuf);

    // == Angle ==
    char anglebuf[8];
    double GPSangle = GPS.angle;
    dtostrf(GPSangle, 3, 2, anglebuf);
    Serial.print("Angle: ");
    Serial.println(GPSangle, 2);
    client.publish("esp32/gps/angle", anglebuf);

    // == Altitude ==
    char altbuf[8];
    double GPSalt = GPS.altitude;
    dtostrf(GPSalt, 3, 2, altbuf);
    Serial.print("Altitude: ");
    Serial.println(GPSalt, 2);
    client.publish("esp32/gps/altitude", altbuf);

    // == Satellites ==
    char satbuf[8];
    double GPSsat = GPS.satellites;
    dtostrf(GPSsat, 2, 0, satbuf);
    Serial.print("Satellites: ");
    Serial.println(GPSsat, 2);
    client.publish("esp32/gps/satellites", satbuf);

    //    humidity = bme.readHumidity();

    // Convert the value to a char array
    //    char humString[8];
    //    dtostrf(humidity, 1, 2, humString);
    //    Serial.print("Humidity: ");
    //    Serial.println(humString);
    //    client.publish("esp32/gps", humString);
  }
}

struct
{
  unsigned long accu_volts = 0;
  unsigned long current = 0;
  unsigned long current_discharge = 0;
  unsigned long current_charge = 0;
  unsigned long charge_percent = 0;
  unsigned long time_to_go = 0;
  unsigned long cell_temp_coll = 0;
  unsigned long bms_state = 0;
  unsigned long power_level = 0;
  unsigned long volt_per_cell[NUM_CELLS] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

} accu_ms;

// accu
void lees_accu() {
  // can data volgens volgend format, elk item is een byte:
  // [source node id, index byte 1, index byte 2, subindex,
  // data1, data2, data3, data4]
  // voor de accu zijn source node id en index high en low niet relevant, en worden dus niet gebruikt

  String topic = "accu/";
  if (canMsg.can_id == 0x302) {
    // data[3] is de subindex
    if (canMsg.data[3] == 1) {
      //uint16 voltage accu in mv
      if (send_per_sec(&accu_ms.accu_volts)) {
        topic += "voltage_mv";
        uint16_t val = concat_uint16(canMsg.data[4], canMsg.data[5]);
        client.publish(topic.c_str(), String(val).c_str());
      }
    } else if (canMsg.data[3] == 2) {
      // int16 current in 0.01amps
      if (send_per_sec(&accu_ms.current)) {
        topic += "current";
        int val = concat_uint16(canMsg.data[4], canMsg.data[5]);
        client.publish(topic.c_str(), String(val).c_str());
      }
    } else if (canMsg.data[3] == 3) {
      // int16 discharge current in 0.01 amps
      if (send_per_sec(&accu_ms.current_discharge)) {
        topic += "discharge_current";
        int val = concat_uint16(canMsg.data[4], canMsg.data[5]);
        client.publish(topic.c_str(), String(val).c_str());
      }
    } else if (canMsg.data[3] == 4) {
      // int16 charge current in 0.01 amps
      if (send_per_sec(&accu_ms.current_charge)) {
        topic += "charge_current";
        int val = concat_uint16(canMsg.data[4], canMsg.data[5]);
        client.publish(topic.c_str(), String(val).c_str());
      }
    } else if (canMsg.data[3] == 5) {
      // u8 charge in %
      if (send_per_sec(&accu_ms.charge_percent)) {
        topic += "charge_percent";
        client.publish(topic.c_str(), String(canMsg.data[4]).c_str());
      }
    } else if (canMsg.data[3] == 6) {
      // 6 is leeg volgens datasheet, maar hier toch afgevangen in het geval dat er iets binnenkomt
    } else if (canMsg.data[3] == 7) {
      // uint16 time to go in minuten
      if (send_per_min(&accu_ms.time_to_go)) {
        topic += "time_to_go";
        uint16_t val = concat_uint16(canMsg.data[4], canMsg.data[5]);
        client.publish(topic.c_str(), String(val).c_str());
      }
    }
  } else if (canMsg.can_id == 0x202) {
    // u8 power level in %
    // wat het precies is is voorlopig onbekend
    if (send_per_sec(&accu_ms.power_level)) {
      topic += "power_level";
      client.publish(topic.c_str(), String(canMsg.data[4]).c_str());
    }
  } else if (canMsg.can_id == 0x402) {
    if (canMsg.data[3] == 9) {
      // u8 cell temp high
      // word niet verstuurd temp collection word verstuurd
    } else if (canMsg.data[3] == 10) {
      // 10 is leeg volgens datasheet, maar hier toch afgevangen in het geval dat er iets binnenkomt
    } else if (canMsg.data[3] == 11) {
      // u8 cell temp low
      // word niet verstuurd temp collection word verstuurd
    } else if (canMsg.data[3] == 12) {
      // u8 cell voltage high
      // word niet doorgestuurd volt per cell wel
    } else if (canMsg.data[3] == 13) {
      // u8 cell voltage low
      // word niet doorgestuurd volt per cell wel
    } else if (canMsg.data[3] == 14) {
      // bms_state uint32 doel onbekend
      // staat volgens mij niet in datasheet wat het betekent
      if (send_per_min(&accu_ms.bms_state)) {
        topic += "bms_state";
        client.publish(topic.c_str(), String(concat_uint32(canMsg.data[3], canMsg.data[4],
                                                           canMsg.data[5], canMsg.data[6]))
                                        .c_str());
      }
    } else if (canMsg.data[3] == 15) {
      // cell temp collection 4x u8 low low_mid high_mid high
      if (send_per_sec(&accu_ms.cell_temp_coll)) {
        // 3 is offset
        topic += "temp_";
        String names[4] = { "low", "25th_percentile", "75th_percentile", "high" };
        for (int i = 0; i < 4; i++) {
          client.publish((topic + names[i]).c_str(), String(canMsg.data[3 + i]).c_str());
        }
      }
    }
  } else if (canMsg.can_id == 0x482) {
    // data[3] is cell nummer (u8) gevolgd door voltage in 0,001 volt (int16)
    if (send_per_sec(&accu_ms.volt_per_cell[canMsg.data[3]])) {
      topic += "voltage_cell_" + String(canMsg.data[3]);
      client.publish(topic.c_str(), String(concat_int16(canMsg.data[4], canMsg.data[5])).c_str());
    }

  } else {
    Serial.println F(("failed to resolve can_id:"));
    print_msg_raw();
    return;
  }
}

struct
{
  unsigned long volt_in[6] = { 0, 0, 0, 0, 0, 0 };
  unsigned long watt_uit[6] = { 0, 0, 0, 0, 0, 0 };
} mppts_ms;

void lees_mppts() {
  String topic = "mppt_";
  int mppt_num = 0;
  // vier data bytes, twee keer een 16 bit float?
  if (canMsg.can_id < 0x280) {
    mppt_num = canMsg.can_id - 0x183;
    if (send_per_sec(&mppts_ms.volt_in[mppt_num - 1])) {
      topic += String(mppt_num) + "/volt_in";
      client.publish(topic.c_str(), String(concat_float_from_can_data(4)).c_str());
    }
  } else {
    mppt_num = canMsg.can_id - 0x283;
    if (send_per_sec(&mppts_ms.watt_uit[mppt_num - 1])) {
      topic += String(mppt_num) + "/milliwatt_uit";
      client.publish(topic.c_str(), String(concat_float_from_can_data(4)).c_str());
    }
  }
}

// voids to make the main code more readable
bool send_per_sec(unsigned long* last_millis) {
  unsigned long _millis = millis();
  if (_millis - *last_millis > MS_PER_SEC) {
    *last_millis = _millis;
    return true;
  } else {
    return false;
  }
}
bool send_per_min(unsigned long* last_millis) {
  unsigned long _millis = millis();
  if (_millis - *last_millis > MS_PER_MIN) {
    *last_millis = _millis;
    return true;
  } else {
    return false;
  }
}

uint16_t concat_uint16(u8 b1, u8 b2) {
  // maakt van twee bytes een uint16_t
  uint16_t ret;
  ret = b1 | (uint16_t)b2 << 8;
  return ret;
}
int16_t concat_int16(u8 b1, u8 b2) {
  // maakt van twee bytes een int16_t
  int16_t ret;
  ret = b1 | (int16_t)b2 << 8;
  return ret;
}
uint32_t concat_uint32(u8 b1, u8 b2, u8 b3, u8 b4) {
  // maakt van vier bytes een uint32_t
  uint32_t ret;
  ret = b1 | (uint32_t)b2 << 8 | (uint32_t)b2 << 16 | (uint32_t)b2 << 24;
  return ret;
}
float concat_float_from_can_data(uint8_t start_idx) {
  byte byteVal[sizeof(float)];
  for (int i = 0; i < sizeof(float); i++) {
    byteVal[i] = canMsg.data[start_idx + i];
  }
  float f;
  memcpy(&f, byteVal, sizeof(float));
  return f;
}

void print_msg_raw() {
  // dumpt het canMsg varable naar de serial als hexadecimal
  Serial.print F(("id: "));
  Serial.print(canMsg.can_id, HEX);  // print ID
  Serial.print F((" dlc: "));
  Serial.print(canMsg.can_dlc, HEX);  // print DLC
  Serial.print F((" data: "));

  for (int i = 0; i < canMsg.can_dlc; i++)  // print the data
  {
    Serial.print(canMsg.data[i], HEX);
    Serial.print F((" "));
  }
  Serial.println F((""));
}