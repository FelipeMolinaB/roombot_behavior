import pandas as pd

def gen_db(columns,db_name):
	db = pd.DataFrame(columns=columns)
	db.to_csv(db_name,index = False)

def main():
	try:
		poses = pd.read_csv("poses.csv",index_col = False)
		while True:
			r = raw_input("There is an existing poses data base with {0} items. Do you want to overwrite it?(y/n)".format(poses.shape[0]))
			if r.lower() == 'y':
				columns = ["Nombre","Px","Py","Pz","Ox","Oy","Oz","Ow","Descripci\xa2n"]
				gen_db(columns,"poses.csv")
				break
			elif r.lower() == 'n':
				break
			else:
				print("Invalid command")
	except:
		columns = ["Nombre","Px","Py","Pz","Ox","Oy","Oz","Ow","Descripci\xa2n"]
		gen_db(columns,"poses.csv")

	try:
		buttons = pd.read_csv("buttons.csv",index_col = False)
		while True:
			r = raw_input("There is an existing buttons data base with {0} items. Do you want to overwrite it?(y/n)".format(buttons.shape[0]))
			if r.lower() == 'y':
				columns = ["Nombre","Boton"]
				gen_db(columns,"buttons.csv")
				break
			elif r.lower() == 'n':
				break
			else:
				print("Invalid command")
	except:
		columns = ["Nombre","Boton"]
		gen_db(columns,"buttons.csv")
	
	print("Process completed")

main()
