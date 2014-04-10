#!/usr/bin/env python3.3


import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
import decimal
data = np.genfromtxt('../data/ex1.csv',delimiter=',', dtype = float)

a = [column[0] for column in data]
b = [column[3] for column in data]
c = [column[7] for column in data]
a1=[]
b1=[]
c1=[]


fig= plt.figure()

for i in range(4000):
	
	b_1 = 0
	c_1 = 0
	
	for j in range(5):
		b_1 = b[5*i + j] + b_1
		c_1 = c[5*i + j] + c_1
		
	a1.append(i)
	b1.append(b_1/5)
	c1.append(c_1/5)
	

fig , p1 = plt.subplots()
p11 = p1.twinx()
p1.bar(a1,b1,0.01,color='r',label="steptime")
p11.plot(a1,c1,color='b',label="Loop Time Averaged")
p1.set_title("Iterations Vs. Step and Loop Times")
p1.set_xlabel("Iteration Values")
p11.set_ylabel("Loop Time Averaged over reruns")
p1.set_ylabel("Step time Averaged over reruns")
p1.legend()
p11.legend()
fig.savefig('../plots/g24_lab24_plot01.png')

#############################################################################################################

a = [column[0] for column in data]
b = [column[3] for column in data]
c = [column[4] for column in data]
d = [column[5] for column in data]
e = [column[6] for column in data]

a1=[]
b1=[]
c1=[]
d1=[]
e1=[]
f1=[]

for i in range(4000):
	
	b_1 = 0
	c_1 = 0
	d_1 = 0
	e_1 = 0
	
	for j in range(5):
		b_1 = b[5*i + j] + b_1
		c_1 = c[5*i + j] + c_1
		d_1 = d[5*i + j] + d_1
		e_1 = e[5*i + j] + e_1
		
	a1.append(i)
	b1.append(b_1/5)
	c1.append(c_1/5)
	d1.append(d_1/5)
	e1.append(e_1/5)
	f1.append((c_1+d_1+e_1)/5)
	
	
	
	
fig2 , p2 = plt.subplots()
p2.plot(a1,b1,color='b',label="steptime")
p2.plot(a1,c1,color='r',label="collisiontime")
p2.plot(a1,d1,color='g',label="velocitytime")
p2.plot(a1,e1,color='m',label="positiontime")
p2.plot(a1,f1,color='y',label="sum")

p2.set_title("Iterations vs Various Times")
p2.set_xlabel("Iteration Values")
p2.set_ylabel("Time (ms)")
p2.legend()
fig2.savefig('../plots/g24_lab24_plot02.png')

##########################################################################################

a = [column[0] for column in data]
b = [column[3] for column in data]

a1=[]
b1=[]

for i in range(4000):
	
	b_1 = 0
	
	
	for j in range(5):
		b_1 = b[5*i + j] + b_1
		
		
	a1.append(i)
	b1.append(b_1/5)
	
b_s = []

for i in range(4000):
	
	b_1s = 0
	
	
	for j in range(5):
		b_1s = b_1s + (b[5*i + j] - b1[i])**2
		
	b_s.append(b_1s**0.5)
	

fig3 , p3 = plt.subplots()
p3.errorbar(a1,b1,yerr=b_s,color='r',label="errorbar")

p3.set_title("Step Times with Errors")
p3.set_xlabel("Iteration Values")
p3.set_ylabel("Step Times")
p3.legend()
fig3.savefig('../plots/g24_lab24_plot03.png')
"""
####################################################################################

a1=[]
b1=[]

roll = 77.0


a = [column[0] for column in data]
b = [column[3] for column in data]

width = 0.01
b1 = []
for i in range(25000):
    if a[i] == roll:
        b1.append(b[i])


fig4 , p4 = plt.subplots()

p4.hist(b1,10)
p4.hist(b1,10,cumulative=True,histtype="step")
p4.set_title("Frequency and Cumulative Frequency of Step time")
p4.set_xlabel("Iteration Values")
p4.set_ylabel("Frequency")
p4.legend()
fig4.savefig('../plots/g24_lab24_plot04.png')
"""
##################################################################3
"""
a1 = [column[0] for column in data]
b1 = [column[3] for column in data]

a_1=[]
b_1=[]

a_2=[]
b_2=[]


data = np.genfromtxt('../data/ex2.csv',delimiter=',', dtype = float)

a2 = [column[0] for column in data]
b2 = [column[3] for column in data]

for i in range(500):
	
	b_t = 0
	
	
	for j in range(50):
		b_t = b1[50*i + j] + b_t
		
		
	a_1.append(i)
	b_1.append(b_t/50)
	
	
for i in range(500):
	
	b_t = 0
	
	
	for j in range(15):
		b_t = b2[15*i + j] + b_t
		
	a_2.append(i)
	b_2.append(b_t/15)
	
	
fig5 , p5 = plt.subplots()
p5.plot(a_1,b_1,'.r',label="normal")
#p2.plot(a_1,b_1,'kx')
p5.plot(a_2,b_2,'.b',label="random")	

fit = np.polyfit(a_1,b_1,1)
fit_fn = np.poly1d(fit) # fit_fn is now a function which takes in x and returns an estimate for y

p5.plot(a_1,b_1,'yo', a_1, fit_fn(a_1), '--k',color='r')


fit = np.polyfit(a_2,b_2,1)
fit_fn = np.poly1d(fit) # fit_fn is now a function which takes in x and returns an estimate for y

p5.plot(a_2,b_2,'yo',a_2, fit_fn(a_2), '--k',color='b')



p5.set_title("Best fits for step times in random and complete samples")
p5.set_xlabel("Iteration Values")
p5.legend()
fig5.savefig('../plots/g24_lab24_plot05.png') """
