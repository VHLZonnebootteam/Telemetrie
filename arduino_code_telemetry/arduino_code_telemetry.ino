#include <SPI.h>
#include <Ethernet.h>
#include <mcp2515.h>
#include <PubSubClient.h>

typedef uint8_t u8;
// Update these with values suitable for your network.
const byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
const char* mqttServer = "telemetrie.zonnebootteam.nl";
const int mqttPort = 1883;
const char* mqttUser = "Zonneboot";
const char* mqttPassword = "Zonnepanelen1";

float pitch;
float roll;
int P_Vvl;
int I_Vvl;
int D_Vvl;
int pidVvlTotal;
int P_Avl;
int I_Avl;
int D_Avl;
int pidAvlTotal;
int8_t P_Balans;
int8_t I_Balans;
int8_t D_Balans;
int8_t pidBalansTotal;
int8_t Distance;
int8_t PID_debug;
int8_t StatusScherm;
int8_t StatusVvl;
int8_t StatusAvl;

uint32_t last_millis_sendDisplay;
const int16_t sendPidMillis = 33;
uint32_t last_millis_Status;
const int16_t sendStatusMillis = 1000;
uint32_t last_millis_readCAN;
const uint16_t readCANMillis = 1;
bool readCAN = true;

// voor de accu gaan er tussen de 1670-1685 berichten over de can bus.
// als dit niet het geval is dan is er waarschijnlijk een kabel los/stuk.
#define NUM_CELLS 12
#define MS_PER_SEC 1000
#define MS_PER_MIN 60000
#define TESTING false
#define RESET_PIN 14

/* pinout:
   spi bus 11, 12 en 13
   chip select ethernet 10
   chip select can shield 9
   chip select sd kaart 4
   indcator leds rood, geel, groen. 2,3,5
   reset pin is A0 (pin 14)
*/

struct can_frame canMsg;
MCP2515 mcp2515(9);
MCP2515 mcp2515_scherm(8);
int msg_count = 0;
unsigned long prev_millis = 0;
u8 speed = 0;
const long interval = 10000;


bool has_been_connected = false;
int indicator_leds[] = { 2, 3, 5 };  // led pin
enum indicator_color { RED,
                       YELLOW,
                       GREEN
};
int indicator_leds_len = 3;
u8 reconn_tries = 0;



//u8 speed = 0;
EthernetClient ethClient;
PubSubClient client(ethClient);


void setup() {
  Serial.begin(115200);
  for (u8 i = 0; i < indicator_leds_len; ++i) {
    pinMode(indicator_leds[i], OUTPUT);
  }
  set_indicator_leds(RED);
  //pinMode(RESET_PIN, OUTPUT);

  Serial.println F(("Start setup"));
  Serial.println F(("Start CAN"));

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  mcp2515_scherm.reset();
  mcp2515_scherm.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515_scherm.setNormalMode();
  Serial.println F(("CAN done"));

  Serial.println F(("Start mqtt"));
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);  // set onvangst callback
  Serial.println F(("Mqtt done"));

  Serial.println F(("Delay 4 seconds for router boot up"));
  delay(4000);  // delay to give the router time to boot up

  Serial.println F(("Start ethernet"));
  Ethernet.begin(mac);
  Serial.println F(("Ethernet done"));

  // Allow the hardware to sort itself out
  delay(1500);
  prev_millis = millis();
  Serial.println F(("setup done"));
}

void loop() {
  if (!client.connected()) {
    if (has_been_connected) {
      set_indicator_leds(YELLOW);
    }
    reconnect();
  } else {
    if (!has_been_connected) {
      has_been_connected = true;
      set_indicator_leds(YELLOW);
    }
    // if verbonden

    if ((mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)) {
      // Serial.println(canMsg.can_id);
      // handelt alle can frames in de queue
      if (canMsg.can_id == 0x302 || canMsg.can_id == 0x402 || canMsg.can_id == 0x482 || canMsg.can_id == 0x202) {
        lees_accu();
      } else if (canMsg.can_id > 0x180 && canMsg.can_id < 0x2A0) {
        //Serial.println(canMsg.can_id, HEX);
        //if (canMsg.can_id < 0x190 && canMsg.can_id > 0x183)
        lees_mppts();
      } else if (canMsg.can_id < 0x5) {
        lees_motor_controller();
      }
      set_indicator_leds(GREEN);
    }
    if ((mcp2515_scherm.readMessage(&canMsg) == MCP2515::ERROR_OK) && ((millis() - last_millis_readCAN) > readCANMillis)) {
      last_millis_readCAN = millis();
       Serial.println("CAN Scherm");// debug
      if (canMsg.can_id == 0x32) {
        lees_PitchAndRoll();
      } else if (canMsg.can_id == 0x33) {
        lees_Vvl();
      } else if (canMsg.can_id == 0x34) {
        lees_Avl();
      } else if (canMsg.can_id == 0x35) {
        lees_BalansDistance();
      } else if (canMsg.can_id == 0x36){
        lees_StatusControle();
      }
      set_indicator_leds(GREEN);

    }

    else {
      // todo error handeling?
    }

    if (TESTING) {
      if (millis() - prev_millis > 1000) {
        prev_millis = millis();
        client.publish("Zonneboot/Snelheid", String(speed).c_str());
        speed += 20;
        set_indicator_leds(GREEN);
      }
    }
    /*
      speed++;
      if (speed == 0)
      {
        unsigned long new_millis = micros();
        Serial.println(((new_millis - last_millis) / 256) - 20000);
        last_millis = new_millis;
      }
      client.publish("Zonneboot/Snelheid", String(speed).c_str());
      delay(20);
    */
  }
  client.loop();
  if (millis() - last_millis_sendDisplay > sendPidMillis && PID_debug) 
  {
    last_millis_sendDisplay = millis();
    Serial.print("test");
    int pitch_int = pitch * 100;
    int roll_int = roll * 100;

    char text[106];
    snprintf(text, 106, "%d,%d,%d,%d,%d,%d,%d,%d,%hhi,%hhi,%hhi,%hhi,%hhi,%d,%d", P_Vvl, I_Vvl, D_Vvl, pidVvlTotal, P_Avl, I_Avl, D_Avl, pidAvlTotal, P_Balans, I_Balans, D_Balans, pidBalansTotal, Distance, pitch_int, roll_int);
    //Serial.println(text);
    client.publish("esp32/pid", text);
  }
  if (millis() - last_millis_Status > sendStatusMillis) {
    last_millis_Status = millis();
    char text[20];
    snprintf(text, 20 ,"%hhi,%hhi,%hhi", StatusVvl, StatusAvl, StatusScherm );
    client.publish("esp32/status", text ); // to do status in notered
   StatusScherm = false; // reset statusscherm zodat deze niet op true blijft staan
   }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print F(("Attempting MQTT connection..."));
    // Attempt to connect
    if (client.connect("arduinoClient", mqttUser, mqttPassword)) {
      Serial.println F(("connected"));
      reconn_tries = 0;
    } else {
      Serial.print F(("failed, rc="));
      Serial.print(client.state());
      Serial.println F((" try again in 5 seconds"));
      if (reconn_tries < 5) {
        reconn_tries++;
      } else {
        Serial.println F(("hard_resetting"));
        delay(100);
        digitalWrite(RESET_PIN, HIGH);
      }
      // Wait 5 seconds before retrying
      delay(5000);
    }
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

struct
{
  unsigned long packets[4] = { 0, 0, 0, 0 };
} controller_ms;

struct
{
  uint16_t inkomend_vermogen;
  uint32_t motor_rpm;
  int pwm_inkomend;
  int pwm_uitgaand;
  uint16_t cntlr_warning;
  uint16_t cntlr_failure;
  uint32_t odometer;
  uint8_t cntlr_temp;
} cntlr_values;

void lees_motor_controller() {
  //client.publish("Zonneboot/Snelheid", String(20));
  String temp = "500";
  if (canMsg.can_id == 0x1) {
    // packet 1
    if (send_per_sec(&controller_ms.packets[0])) {
      // todo battery voltage
      cntlr_values.inkomend_vermogen = concat_uint16(canMsg.data[2], canMsg.data[3]) / 10;  // todo geef volle preciesie door
      cntlr_values.motor_rpm = concat_uint32(canMsg.data[4], canMsg.data[5], canMsg.data[6], 0) * 10;
      client.publish("mtr_cntlr/battery_current", String(cntlr_values.inkomend_vermogen).c_str());
      client.publish("mtr_cntlr/mtr_toeren", String(cntlr_values.motor_rpm).c_str());
    }
  }

  else if (canMsg.can_id == 0x2) {
    // packet 2
    if (send_per_sec(&controller_ms.packets[1])) {
      cntlr_values.odometer = concat_uint32(canMsg.data[0], canMsg.data[1], canMsg.data[2], canMsg.data[3]);
      cntlr_values.cntlr_temp = canMsg.data[4];
      client.publish("mtr_cntlr/temp",
                     String(cntlr_values.cntlr_temp).c_str());
      client.publish("mtr_cntlr/odometer",
                     String(cntlr_values.odometer).c_str());

      /* niet gebruiken klopt niks van
          is voor sensors die niet zijn aangesloten.
        ret_msg += "motor temp in c " + String(canMsg.data[4]);
        ret_msg += line_start;
        ret_msg += "accu temp in c " + String(canMsg.data[4]);
        ret_msg += line_start;
      */
    }

  }

  else if (canMsg.can_id == 0x3) {
    // packet 3
    if (send_per_sec(&controller_ms.packets[2])) {
      cntlr_values.pwm_inkomend = concat_int16(canMsg.data[0], canMsg.data[1]);
      cntlr_values.pwm_uitgaand = concat_int16(canMsg.data[2], canMsg.data[3]);
      cntlr_values.cntlr_warning = concat_uint16(canMsg.data[4], canMsg.data[5]);
      cntlr_values.cntlr_failure = concat_uint16(canMsg.data[6], canMsg.data[7]);

      client.publish("mtr_cntlr/pwm_inkomend",
                     String(cntlr_values.pwm_inkomend).c_str());
      client.publish("mtr_cntlr/pwm_uitgaand",
                     String(cntlr_values.pwm_uitgaand).c_str());
      client.publish("mtr_cntlr/warning",
                     String(cntlr_values.cntlr_warning).c_str());
      client.publish("mtr_cntlr/failure",
                     String(cntlr_values.cntlr_failure).c_str());
    }

  } else if (canMsg.can_id == 0x4) {
    // packet 4
    /* niet gebruiken klopt niks van
      ret_msg += line_start;
      ret_msg += "battery capacity % " + String(canMsg.data[1]);
    */
  }
}
void lees_PitchAndRoll() {

  pitch = concat_float_from_can_data(0);  // byte 0-3 is float pitch
  roll = concat_float_from_can_data(4);   // byte 4-7 is float roll
}
void lees_Vvl() {
  P_Vvl = int16_from_can(canMsg.data[0], canMsg.data[1]);
  Serial.println(P_Vvl);
  I_Vvl = int16_from_can(canMsg.data[2], canMsg.data[3]);
  D_Vvl = int16_from_can(canMsg.data[4], canMsg.data[5]);
  pidVvlTotal = int16_from_can(canMsg.data[6], canMsg.data[7]);
}
void lees_Avl() {
  P_Avl = int16_from_can(canMsg.data[0], canMsg.data[1]);
  I_Avl = int16_from_can(canMsg.data[2], canMsg.data[3]);
  D_Avl = int16_from_can(canMsg.data[4], canMsg.data[5]);
  pidAvlTotal = int16_from_can(canMsg.data[6], canMsg.data[7]);
}
void lees_BalansDistance() {
  P_Balans = canMsg.data[0];
  I_Balans = canMsg.data[1];
  D_Balans = canMsg.data[2];
  pidBalansTotal = canMsg.data[3];
  Distance = canMsg.data[4];
  PID_debug = canMsg.data[5];  // alleen wanneer de pid op het display actief is word de data doorgestuurd om data te besparen.
}
void lees_StatusControle(){
  StatusVvl = canMsg.data[0];
  StatusAvl = canMsg.data[1];
  StatusScherm = true;
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
int16_t int16_from_can(uint8_t b1, uint8_t b2) {
  // maakt van twee bytes een int16_t
  int16_t ret;
  ret = b1 | (int16_t)b2 << 8;
  return ret;
}

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

void callback(char* topic, byte* payload, unsigned int length) {
  // draait waneer een bericht word ontvangen.
  // voorlopig niet in gebruikt print bericht naar serial
  Serial.print F(("Message arrived ["));
  Serial.print(topic);
  Serial.print F(("] "));
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
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

void set_indicator_leds(u8 state)
//state is an enumerable list, where the color corresponds to a number in a list, red corresponds to 0, yellow to 1, green to 2.
{
  if (state < indicator_leds_len)  //state corresponds to a color state is either 0,1,2 (red,yellow,green)
  {
    for (u8 i = 0; i < indicator_leds_len; ++i)
    //for every indicator led we check the state and set the led on/off
    {
      digitalWrite(indicator_leds[i], i == state);  //if the led to be checked ([i]) is equal to the state we turn on that led.
    }
  }
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

String log_empty() {
  String ret_msg;
  ret_msg += "found unknown data at canid ";
  ret_msg += canMsg.can_id;
  ret_msg += " subindex ";
  ret_msg += String(canMsg.data[3]);
  ret_msg += " data: ";
  for (int i = 4; i < canMsg.can_dlc; i++) {
    ret_msg += String(canMsg.data[i], HEX);
    ret_msg += " ";
  }

  return ret_msg;
}
