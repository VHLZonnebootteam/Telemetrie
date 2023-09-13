import time
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import sqlite3
from my_utils.my_logging import log_message as log
from my_utils.platform_vars import dir_sep, ROOTDIR
import datetime


database_dir = ROOTDIR + dir_sep + "dbs" + dir_sep
get_time = lambda: datetime.datetime.now().replace(microsecond=0)
get_db_path = lambda: database_dir + time.strftime("%Y_%m_%d.db")
current_db_file_path = get_db_path()
uncommited_msg_count = 0
max_uncommited_msg = 100

db_conn = None
db_cursor = None

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


def update_db_file_name():
    global current_db_file_path
    global get_db_path
    if get_db_path().find(current_db_file_path) < 0:
        db_conn.commit()
        db_conn.close()
        current_db_file_path = get_db_path()
        create_db_connection()
        log("changed to new db at " + current_db_file_path)
        

def create_db_connection():
    """ create a database connection to a SQLite database """
    try:
        global db_conn
        global db_cursor        
        db_conn = sqlite3.connect(current_db_file_path)
        db_cursor = db_conn.cursor()
        log(sqlite3.version)
        log("connected to " + current_db_file_path)
        log("creating tables")
        for topic in topics:
            # je zou hier een prepared statement moeten gebruiken maar dat kan hier blijkbaar niet 
            table = topic.replace("/", "_")
            if topic.find("mppt") > 0:
                db_cursor.execute("CREATE TABLE IF NOT EXISTS " + table + "(time DATETIME, value REAL)")
            else:
                db_cursor.execute("CREATE TABLE IF NOT EXISTS " + table + "(time DATETIME, value INTEGER)")
        db_conn.commit()
        log("done")
    except sqlite3.Error as e:
        log(e)

# The callback for when the client receives a CONNACK response from the server.
def on_mqtt_connect(client, userdata, flags, rc):
    log("Connected with result code "+str(rc))
    for topic in topics:
        client.subscribe(topic)
    log("subbed to all topics")


# The callback for when a PUBLISH message is received from the server.
def on_mqtt_message(client, userdata, msg):
    try:
        
        global db_cursor
        global db_conn
        global uncommited_msg_count
        update_db_file_name()
        #log(msg.topic+ " "+  msg.payload.decode('utf-8') )
        if msg.topic.find("mppt") > 0:
            print()
            db_cursor.execute("INSERT INTO " + msg.topic.replace("/", "_") + "(time, value) VALUES(?,?)", [ str(get_time()), msg.payload.decode('utf-8')] )
        else:
            db_cursor.execute("INSERT INTO " + msg.topic.replace("/", "_") + "(time, value) VALUES(?,?)", [ str(get_time()), msg.payload.decode('utf-8') ] )
        #log(str(get_time()) + msg.topic+" "+  str(msg.payload) )
        
        uncommited_msg_count += 1
        if uncommited_msg_count > max_uncommited_msg:
            db_conn.commit()
            uncommited_msg_count = 0
            log(str(get_time()) + " committed")
    except Exception as e:
        log(e)


if __name__== "__main__":
    create_db_connection()
    
    client = mqtt.Client()
    client.on_connect = on_mqtt_connect
    client.on_message = on_mqtt_message
    client.connect("127.0.0.1")
    val = 0
    #publish.single(topics[4], str(val))
    client.loop_forever()
