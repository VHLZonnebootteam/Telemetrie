import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import time

topics = [
    "mtr_cntlr/inkomend_vermogen",
    "mtr_cntlr/mtr_toeren",
    "mtr_cntlr/temp",
    "mtr_cntlr/odometer",
    "mtr_cntlr/pwm_inkomend",
    "mtr_cntlr/pwm_uitgaand",
    "mtr_cntlr/warning",
    "mtr_cntlr/failure",
    "accu/voltage_mv",
    "accu/current",
    "accu/discharge_current",
    "accu/charge",
    "accu/charge_percent",
    "accu/time_to_go",
    "accu/bms_state",
    "accu/temp_low",
    "accu/temp_25th_percentile",
    "accu/temp_75th_percentile",
    "accu/temp_high",
    "accu/voltage_cell_1",
    "accu/voltage_cell_2",
    "accu/voltage_cell_3",
    "accu/voltage_cell_4",
    "accu/voltage_cell_5",
    "accu/voltage_cell_6",
    "accu/voltage_cell_7",
    "accu/voltage_cell_8",
    "accu/voltage_cell_9",
    "accu/voltage_cell_10",
    "accu/voltage_cell_11",
    "accu/voltage_cell_12",
    "mppt_1/volt_in",
    "mppt_2/volt_in",
    "mppt_3/volt_in",
    "mppt_4/volt_in",
    "mppt_5/volt_in",
    "mppt_6/volt_in",
    "mppt_1/milliwatt_uit",
    "mppt_2/milliwatt_uit",
    "mppt_3/milliwatt_uit",
    "mppt_4/milliwatt_uit",
    "mppt_5/milliwatt_uit",
    "mppt_6/milliwatt_uit",
]

# The callback for when the client receives a CONNACK response from the server.
def on_mqtt_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))



# The callback for when a PUBLISH message is received from the server.
def on_mqtt_message(client, userdata, msg):
    try:
        #update_db_file_name()
        #db_cursor.execute("INSERT INTO " + topic.replace("/", "_") + "(time, value) VALUES(?,?)", [ get_time(), 100 ] )
        print("bla")
        print(str(get_time()) + msg.topic+" "+  str(msg.payload) )
    except Exception as e:
        print(e)

if __name__== "__main__":
    
    client = mqtt.Client()
    client.on_connect = on_mqtt_connect
    client.on_message = on_mqtt_message
    client.connect("127.0.0.1")
    val = 0
    while True:
        for topic in topics:
            publish.single(topic, str(val))
            #print("published to " + topic)
            time.sleep(0.02)
        val += 1
