#!/usr/bin/python

import urllib
import urllib2


inputfile = 'dump_1.txt'
elements = []

data = {}
data['name'] = 'Somebody Here'
data['location'] = 'Northampton'
data['language'] = 'Python'

url_values = urllib.urlencode(data)
print url_values

url = 'http://www.example.com/example.cgi'
full_url = url + '?' + url_values
data = urllib2.urlopen(full_url)

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

with open(inputfile) as f:
	next(f)		
	for line in f:
		# Eliminar \n cada linea
		line = line.replace('\n','')
		# Afegir a estructura dades
		temporal = line.split('#')
		valor_tmp = Valor()
		valor_tmp.data = temporal[0]
		
		print valor_tmp.data
	