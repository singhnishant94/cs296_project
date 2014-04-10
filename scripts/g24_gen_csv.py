#! /usr/bin/env python3.3

from subprocess import call

#a =str(call(["./mybins/cs296_24_exe", "150"]))
import subprocess

from itertools import islice

import re
import random
import numpy as np

import csv

with open('../data/ex1.csv' , 'w') as csvfile:
	spamwriter= csv.writer(csvfile,delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
	for i in range(8000):
		for j in range(5):
			output = subprocess.Popen(["../mybins/cs296_24_exe","auto",str(i+1)], stderr=subprocess.STDOUT, stdout=subprocess.PIPE).communicate()[0]
			b = re.findall(r'\d+.\d+', output)
			for p in range(len(b)):
				b[p] = float(b[p])
				b1=re.findall(r'\d+', output) 
                                if len(b1) == 10:
				    b.append(b1[9])
				
				b = re.findall(r'[-+]?\d*\.\d+|\d+', output)
					
			b= [i+1] + [j+1] + b[0:len(b)+1]
			spamwriter.writerow(b)
			
			
			
"""

with open('../data/ex1.csv','r') as csvfile:
	spamreader=csv.reader(csvfile,delimiter=',', quotechar='|')
	with open('../data/ex2.csv' , 'w') as csvfile1:
		spamwriter= csv.writer(csvfile1,delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
		data = np.loadtxt('../data/ex1.csv', delimiter=',')
		for i in range(500):
			l1= random.sample(range(50),15)
			for k in l1:
				row1 = data[50*i + k]
				spamwriter.writerow(row1)
					
					

"""			
