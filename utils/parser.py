#!/usr/bin/python

import urllib
import urllib2


inputfile = 'dump_1.txt'

# Id to upload data
id_arduino = 2

def upload(Valor):
	print Valor.data

data = {}
data['name'] = 'Somebody Here'
data['location'] = 'Northampton'
data['language'] = 'Python'

url_values = urllib.urlencode(data)
#print url_values

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
			# Data
			valor_tmp = Valor()
			valor_tmp.data = temporal[0]
			# Temp 1
			valor_tmp.sensor1 = temporal[1]
			# Temp 2
			valor_tmp.sensor2 = temporal[2]
			# Temp 3
			valor_tmp.sensor3 = temporal[3]

			
			upload (valor_tmp)
		except IOError:
			print "Could not read line:", line

	