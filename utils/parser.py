#!/usr/bin/python

import urllib
import urllib2
import datetime
import sys


if len (sys.argv) != 3 :
    print "Usage: python parser.py <id_arduino> <file>"
    sys.exit (1)

inputfile = sys.argv[2]

# Id to upload data
id_arduino = sys.argv[1]

def upload(Valor):
	data = {}

	data['idarduino'] = id_arduino
	data['data'] = Valor.data
	data['temp1'] = Valor.sensor1
	data['temp2'] = Valor.sensor2
	data['temp3'] = Valor.sensor3
	# Temperatura ambiental
	data['ta1'] = Valor.ambient1_temp
	data['ta2'] = Valor.ambient2_temp
	# Humitat ambiental
	data['ha1'] = Valor.ambient1_humetat
	data['ha2'] = Valor.ambient2_humetat
	# LUX
	data['ldr1'] = Valor.lux
	# PH
	data['ph1'] = Valor.ph

	
	url_values = urllib.urlencode(data)
	print url_values

	url = 'http://sensors.openspirulina.com/afegir.php'
	full_url = url + '?' + url_values


	try: response = urllib2.urlopen(full_url)
	except urllib2.URLError as e:
		print e.reason

	print response.read()

class Valor:
	def __init__(self):
		self.data = None
		# Sensors temperatura aigua
		self.sensor1 = None
		self.sensor2 = None
		self.sensor3 = None
		# Temperatura ambient
		self.ambient1_temp = None
		self.ambient2_temp = None
		# Humetat ambient
		self.ambient1_humetat = None
		self.ambient2_humetat = None
		# Lux
		self.lux = None
		# PH
		self.ph = None

with open(inputfile, 'r') as f:
	next(f)		
	for line in f:
		try:
			# Eliminar \n cada linea
			line = line.replace('\n','')
			# Afegir a estructura dades
			temporal = line.split('#')
			valor_tmp = Valor()
			# Data
			mala_fecha = temporal[0].replace('/','-')
			fecha = datetime.datetime.strptime(mala_fecha, '%d-%m-%Y %H:%M:%S')
			valor_tmp.data = fecha.strftime('%Y-%m-%d %H:%M:%S')
			# Temp 1
			valor_tmp.sensor1 = temporal[1]
			# Temp 2
			valor_tmp.sensor2 = temporal[2]
			# Temp 3
			valor_tmp.sensor3 = temporal[3]
			# Temp Ambient 1
			valor_tmp.ambient1_temp = temporal[4]
			# Temp Ambient 2
			valor_tmp.ambient2_temp = temporal[5]
			# Humetat Ambient 1
			valor_tmp.ambient1_humetat = temporal[6]
			# Humetat Ambient 2
			valor_tmp.ambient2_humetat = temporal[7]
			# LUX
			valor_tmp.lux = temporal[8]
			# PH
			valor_tmp.ph = temporal[9]

			# Send to uploader
			upload (valor_tmp)
		except IOError:
			print "Could not read line:", line
		except Exception:			
			pass

	