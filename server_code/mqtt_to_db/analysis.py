import sqlite3
from my_utils.my_logging import log_message as log
from my_utils.platform_vars import dir_sep, ROOTDIR
import datetime


database_dir = ROOTDIR
current_db_file_path = database_dir + dir_sep + "dbs" + dir_sep + "2019_09_07.db"
db_conn = sqlite3.connect(current_db_file_path)
db_cursor = db_conn.cursor()

max = "select max(value) from {} where value > 0"
min = "select min(value) from {} where value > 0"
cell_table_list = []


if __name__== "__main__":
	for i in range(12):
		cell_table_list.append("accu_voltage_cell_{}".format(i + 1))
	
	for name in cell_table_list:
		db_cursor.execute(min.format(name))
		min_val = db_cursor.fetchone()[0]
		db_cursor.execute(max.format(name))
		max_val = db_cursor.fetchone()[0]
		print("{} min:{} max:{}".format(name, min_val, max_val))
		
		

	
