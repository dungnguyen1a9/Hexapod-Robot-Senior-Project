#! /usr/bin/python
import smbus
import time
import struct
from evdev import InputDevice, categorize, ecodes
#from multiprocessing import Process, Queue
from threading import Thread
from numpy import *
from numpy.linalg import inv

from math import *
import sys
bus = smbus.SMBus(1)

ADDRESS = [0xa, 0xb, 0xc, 0xd, 0xe , 0xf]

############Initial values ##############
#########################################
btn_array_cur = [0,0,0,0]
btn_array_pre = [0,0,0,0]


tau = 3.0
beta = 0.05*tau
h = 0.20 #meter
Z_0 = 0.34 #meter
Y_f = 0.3 
Y_b = -0.3
X_0 = 0.32
phase_init = 0 
t_0 = time.time()
time_t = time.time() - t_0 + phase_init

delta_min = 5*pi/180
delta_max = 25*pi/180

theorical_record  = open('recording_theorical_data.txt','w')
measured_record  = open('recording_measured_data.txt','w')
L_femur = 0.27978
L_tibia = 0.36830
C_1 = zeros((3,1), float)
L_1 = zeros((3,1), float)

num_of_legs = 1
leg_info = zeros((3,num_of_legs),float)
cmd_position = zeros((num_of_legs,3),float)
data = [0,0,0]


def RC_input():
	#python /usr/local/lib/python2.7/dist-packages/evdev/evtest.py
	
	#run the command above to check which event or channel the RC controller
	# will be used.  
	gamepad = InputDevice('/dev/input/event4')
	
	#button code variables (change to suit your device)
	aBtn = 304
	bBtn = 305
	xBtn = 307
	yBtn = 308
	 
	left = 312
	right = 313
	
	start = 315
	select = 314
	
	lTrig = 310
	rTrig = 311
	x = 0
	y = 0
	a = 0
	b = 0
	#prints out device info at start
	print(gamepad)
	#loop and filter by event code and print the mapped label
	for event in gamepad.read_loop():
	    if event.type == ecodes.EV_KEY:
	        if event.code == yBtn:
	            if event.value == 1:
	                y=1
	            elif event.value == 0:
					y=0
	                            
	            
	    if event.type == ecodes.EV_KEY:  
	        if event.code == aBtn:
	            if event.value == 1:
	                a=1
	            elif event.value == 0:
					a=0
					
	       ############
	    if event.type == ecodes.EV_KEY:  
	        if event.code == xBtn:
	            if event.value == 1:
	                x=1
	            elif event.value == 0:
					x=0 
					
					
	        ##########
	    if event.type == ecodes.EV_KEY:  
	        if event.code == bBtn:
	            if event.value == 1:
	                b=1
	            elif event.value == 0:
					b=0 
		global btn_array_cur
		btn_array_cur = [a,b,x,y]


def writeNumber(address,x):
	print(address)
	bus.write_i2c_block_data(address, 0x01 , x)
	return -1


def get_data(address):
	#read data from sub-controller and convert it into floats
	#temp holds the 12 bits that bus.read_i2c_block_data() outputs
	#the third argument of bus.read_i2c_block_data(address, 0,12) sets
	#the number of bytes to read from the i2c interface
	temp=bus.read_i2c_block_data(address, 0,12);
	#print(temp)
	#temp2 changes the data in locations 0 to 11 into a character string
	temp2=''.join([chr(i) for i in temp[0:12]])
	#temp3 unpacks the character string into 3 float numbers ("<3f")
	temp3=struct.unpack("<3f",temp2)
	
	
	return temp3;


# convert input value => string => byte to send to sub-controller	
def str_to_byte(value_to_byte):
	str_converter = str(value_to_byte)
	converted = []
	for b in str_converter:
		converted.append(ord(b))
	return converted


#############################################################
###### SENDING and RECEIVING VALUES TO THE SUB-CONTROL#######
#############################################################
def i2c_com(address, index ,input_x,input_y,input_z):
	global leg_info
	global data
	#rewrites the data that must be sent into the correct format using struct.
	#converts input values into floats using float()
	#packs floats into memory data using struct.pack()
	#unpacks memory data into integers using struct.unpack()
	#converts the tuple output from struct.unpack into a list using list()
	#currently set to use 3 floats ("!3f") which becomes 12 bytes ("!12b")
	byte_to_send = list(struct.unpack("12b",struct.pack("<3f",float(input_x),float(input_y),float(input_z))))
	
	# send to sub_controller.
	writeNumber(address , byte_to_send)

	############################################################
	###### RECEIVING VALUES TO THE SUB-CONTROL and PRINT #######
	############################################################				
	## each column of the matrix will represent information of the leg
	## in order [gamma,phi,theta, force , proximity] respectively
		
	#print("Data from subcontroller: ")
	
	data = get_data(address)
	#print('data: ')
	#print(data)
	leg_info[:,index] = [ data[0] , data[1] , data[2] ]
	

def desired_position(phase, time_t):
	## X_0 , Y_f, Y_b , Y_0, tau, beta need to be specific
	t = time_t + phase*tau 
	global x,y,z
	global cmd_position
	######## X - direction is gonna be the same ###############
	######## Z - direction ####################################
	
	if t >= (-6*tau) and t <= (-5*tau + beta):
		nu = -6*tau - beta 
		z = Z_0 - h/2 *(1 - cos((t-nu)*2*pi/(tau + 2*beta)))
		
	elif t >= (-5*tau + beta) and t <=  -beta:
		z = Z_0
		
	elif t >= (-beta) and t <= (tau + beta):
		nu = -beta
		z = Z_0 - h/2 *(1 - cos((t-nu)*2*pi/(tau + 2*beta)))
		
	elif t >= (tau+beta) and t <= 6*tau - beta:
		z =Z_0
		
	elif t >= (6*tau-beta) and t <= (7*tau + beta):	
		nu = 6*tau - beta
		z = Z_0 - h/2 *(1 - cos((t-nu)*2*pi/(tau + 2*beta)))
	
	elif t >= (7*tau + beta) and t <= (12*tau - beta):
		z = Z_0
		
	elif t >= (12*tau - beta) and t <= (12*tau) :
		nu = 12*tau - beta
		z = Z_0 - h/2 *(1 - cos((t-nu)*2*pi/(tau + 2*beta)))
		
	############If statements Y-direction ########################
	##############################################################

	if t >= (-6*tau) and t <= (-5*tau):
		mu = -6*tau  
		y = (Y_f - Y_b) / tau *(t - mu) + Y_b 
		
	elif t >= (-5*tau ) and t <=  0:
		mu = -5*tau
		y = (Y_b - Y_f) / (5*tau)*(t- mu) + Y_f
		
	elif t >= 0 and t <= tau :
		mu = 0
		y = (Y_f - Y_b) / tau *(t - mu) + Y_b
		
	elif t >= (tau) and t <= 6*tau :
		mu = tau
		y = (Y_b - Y_f) / (5*tau)*(t- mu) + Y_f
		
	elif t >= (6*tau) and t <= (7*tau ):	
		mu = 6*tau
		y = (Y_f - Y_b) / tau *(t - mu) + Y_b
	
	elif t >= (7*tau) and t <= (12*tau ):
		mu = 7*tau
		y = (Y_b - Y_f) / (5*tau)*(t- mu) + Y_f
		
	#################---X---##############################		
	x = X_0
		
	cmd_position[phase,:] = [x,y,z]
	#time.sleep(0.5)
	#print('Desired Position function: ')
	#print(cmd_position)
	
#################FEEDBACK FROM THE SUBCONTROLLER #################
	
def angle_to_coordinate(gamma, phi, theta):
	
	R_H_A = [[cos(phi) , -sin(phi), 0],[sin(phi) , cos(phi), 0],[0 , 0, 1]]
	R_H_A = reshape(R_H_A,(3,3))
	#print(R_H_A)
	
	R_L_H = [[cos(theta) ,0 , -sin(theta) ],[ 0 , 1 , 0],[sin(theta) , 0 , cos(theta) ]]
	R_L_H = reshape(R_L_H,(3,3))
	#print(R_L_H)
	
	R_B_L = [[-1, 0 , 0],[0 , 1 , 0],[0 , 0, -1]]
	R_B_L = reshape(R_B_L,(3,3))
	#print(R_B_L)
	
	R_C_B = [[cos(gamma), 0 , -sin(gamma) ],[0 , 1 , 0],[sin(gamma) , 0 , cos(gamma)]]
	R_C_B = reshape(R_C_B,(3,3))
	#print(R_C_B)
	
	L_1 = [[1,0,0]]
	L_1 = reshape(L_1,(3,1))

	C_1 = [1,0,0]
	C_1 = reshape(C_1,(3,1))
	
	#print('vector P: ')
	P = L_femur*dot(dot(R_H_A , R_L_H),L_1) + dot(L_tibia * dot(dot(R_H_A , R_L_H) , R_B_L), dot(R_C_B , C_1))

	return P

	
def bounding_box(Z):
	# using Z_0 as Z
	# Z is variable, the height of the robot respect to the ground
				
	theta_min = asin(L_tibia / L_femur * cos(delta_min) - Z / L_femur )
	theta_max = asin(L_tibia / L_femur * cos(delta_max) - Z / L_femur )
		
	L1 = L_femur*cos(theta_max) + L_tibia*sin(delta_min)
	L2 = L_femur*cos(theta_min) + L_tibia*sin(delta_max)
		
	bound = [L1,L2]
	#print(bound)
	return bound
		

def determine_Y_f_and_Y_b(Z) :
	# Z_0 = 0.34
	L_1 = (bounding_box(Z_0))[0]
	L_2 = (bounding_box(Z_0))[1]
	
	phi_max = acos(L_1/L_2)
	phi_min = phi_max*(-1) #same as phi_max but symmetric
	#print('phi_max: ' + str(phi_max*180/pi) + '   phi_min: ' + str(phi_min*180/pi))
	
	# Y_f : Y_forward 
	# Y_b : Y_backward
	Y_f = L_2*sin(phi_max)
	Y_b = L_2*sin(phi_min)
	Y_position = [Y_f , Y_b]
	
	#print( 'Y_f and Y_b')
	#print(Y_position)
	return Y_position


def sending_and_receive_info():
	exit = False
	while not exit: 
		theorical_record.write(str(time.time()) + ' ' + str(time_t) + ' ' + str(cmd_position[0,0] )+ ' '+ str(cmd_position[0,1] ) + ' ' + str(cmd_position[0,2] ) + '\n')	
		for i in range(num_of_legs):	
			input_x = cmd_position[i,0]
			input_y = cmd_position[i,1]
			input_z = cmd_position[i,2]
			( i2c_com(ADDRESS[i], i , (input_x),(input_y),(input_z) ) )
		#time.sleep(0.2)
		
			
def loop_time_offset(): 	
	global btn_array_pre
	global phase_init, t_0, time_t 
	
	if (btn_array_cur[0] == 1): 
		print('button X')
	elif (btn_array_cur[1] == 1):
		print('button cricle')
	elif (btn_array_cur[2] == 1):
		print('button triangle')
	elif (btn_array_cur[3] == 1):
		print('button square')	


	#print('loop_time_offset')
	################____forward_____#######################
	if (btn_array_cur[2] == 1) and (btn_array_pre[2] == 0):
		t_0 = time.time()
		print(t_0)
	
	
	elif (btn_array_cur[2] == 0 and btn_array_pre[2] == 1):
		phase_init = time_t
		print(phase_init)
		
	elif (btn_array_cur[2] == 1 and btn_array_cur[0] == 1):
		print("nothing")
		t_0 = time.time()
		phase_init = time_t
	
	if (btn_array_cur[2] == 1 and btn_array_cur[0] != 1 ):  
		time_t = time.time() - t_0 + phase_init
		print(time_t)
		
		if (time_t > 6*tau ) or (time_t < -6*tau):
			t_0 = time.time()
			phase_init = 0
	
	
	###############____backward_____#######################
	if (btn_array_cur[0] == 1) and (btn_array_pre[0] == 0):
		t_0 = time.time()
		print(t_0)
	
	elif (btn_array_cur[0] == 0 and btn_array_pre[0] == 1):
		phase_init = time_t
		print(phase_init)
	
	elif (btn_array_cur[2] == 1 and btn_array_cur[0] == 1):
		print('nothing')
		t_0 = time.time()
		phase_init = time_t
		
	if (btn_array_cur[0] == 1 and btn_array_cur[2] != 1 ):  
		time_t = -(time.time() - t_0) + phase_init
		print(time_t)
		if (time_t > 6*tau ) or (time_t < -6*tau):
			t_0 = time.time()
			print(t_0)
			phase_init = 0
	
	#time.sleep(0.1)	
	btn_array_pre = btn_array_cur
	#print('previous button')
	#print(btn_array_pre)
	
	
def main():
	exit = False
	
	while not exit: 
		loop_time_offset()
		for i in range(num_of_legs):
			desired_position(i, time_t)
			
		feed_back = angle_to_coordinate(data[0]*pi/180,data[1]*pi/180,data[2]*pi/180)

		#print ('FEED_BACK: ')
		#print (str(feed_back[0,0]) + ' ' + str(feed_back[1,0]) + ' '+ str(-feed_back[2,0]) )
		
		measured_record.write(str(time.time()) + ' ' + str(time_t) + ' ' + str(feed_back[0,0] )+ ' '+ str(feed_back[1,0] ) + ' ' + str(-feed_back[2,0]) + '\n')
		#time.sleep(0.2)
		
print('First run')	
print(bounding_box(Z_0))
print(determine_Y_f_and_Y_b(Z_0))
#time.sleep(3)
print('Second run')
p1 = Thread(target = RC_input )
p1.daemon = True

p2 = Thread(target = main )	
p2.daemon = True

p3 = Thread( target = sending_and_receive_info)
p3.daemon = True

p1.start()	
p2.start()
p3.start()

while True:
	time.sleep(1)
