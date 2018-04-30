#Takes the input of the node from the Mathattan map and generats the corresponding node values for the MAP Vectors
from collections import defaultdict
from heapq import *
import RFM69
from RFM69registers import *
import datetime
import random
import sys
import os
import numpy as np
import smbus
import time

#sys.path.append('/home/pi/MAPPING')

##-------------------------------------------------------------------
test = RFM69.RFM69(RF69_915MHZ, 2, 10, True)

# for RPI version 1, use "bus = smbus.SMBus(0)"	
bus = smbus.SMBus(1)

	# This is the address we setup in the Arduino Program
	#Slave Address 1
address = 0x04

#_____________________________________________________________________________________________________________________________

def writeNumber(value):
    bus.write_byte(address, value)

    # bus.write_byte_data(address, 0, value)
    return -1

def readarduino():
	
    # number = bus.read_byte(address)
    number = bus.read_i2c_block_data(address,0,16)
    print("reading information from arduino")
    print(number)
    time.sleep(.01)
    return number
 
def listen():
	DEC=0
	while (DEC==0):
		dec=readarduino()
		print (dec[0])
		if dec[0]==49:
			DEC=dec
			print 'works'
		else: 
			print 'not yet'
		

def speakpi(data):

		#Receives the data from the User
		#data = raw_input("Enter the data to be sent : ")
	data_list = list(data)
	for i in data_list:
		#Sends to the Slaves 
		writeNumber(int(ord(i)))
		time.sleep(.1)
		writeNumber(int(0x0A))

	return 1

#___________________________________________________________________________________________________________________________________________________________________

def all_same(items):
	return all(x == items[0] for x in items)
	
	
def gpsgo():	
  
	validate=False 




	while (validate==False):
		test.receiveBegin()
	   
									
		while not test.receiveDone():
			time.sleep(2)      
			
			D1=[]
			D2=[]
			D3=[]
		
			i=0
			
			
			while (i<=2):
			
				GPS=("".join([chr(letter) for letter in test.DATA]))
				GPS_DATA=GPS.split()
				
				rx=int(GPS_DATA[2])
				ry=int(GPS_DATA[1])
				rt=int(GPS_DATA[4])
				px=int(GPS_DATA[8])
				py=int(GPS_DATA[7])
				pt=int(GPS_DATA[10])
				dx=int(GPS_DATA[13])
				dy=int(GPS_DATA[13])
				dt=int(90)
			
				ROB_DATA=[rx,ry,rt]
				PSG_DATA=[px,py,pt]
				DES_DATA=[dx,dy,dt]
				D1.append(ROB_DATA)
				D2.append(PSG_DATA)
				D3.append(DES_DATA)
				
				i+=1
		
			
		#print "%s from %s RSSI:%s" % ("".join([chr(letter) for letter in test.DATA]), test.SENDERID, test.RSSI)
		#print(ROB_DATA,PSG_DATA,DES_DATA) 
		validateROB=all_same(D1)
		validatePSG=all_same(D2)
		validateDES=all_same(D3)
		validateVEC=[validateROB,validatePSG,validateDES]
		validate=all_same(validateVEC)
		#print(validateVEC)
		#print(validate)
	
		if test.ACKRequested():
			test.sendACK()
	#print "shutting down"
	
	return np.array([ROB_DATA,PSG_DATA,DES_DATA])
	

#______________________________________________________________________________________________________


def dijkstra(f, t):
	edges = [
			('a1', 'b1', 1),
            ('a2', 'a1', 1),
            ('a3', 'b3', 1),
            ('a3', 'a2', 1),
            ('a4', 'a3', 1),
            ('b1', 'c1', 1),
            ('b2', 'a2', 1),
            ('b3', 'c3', 1),
            ('b4', 'a4', 1),
            ('c1', 'd1', 1),
            ('c1', 'f1', 1),
            ('c2', 'b2', 1),
            ('c3', 'd3', 1),
            ('c3', 'f3', 1),
            ('c4', 'b4', 1),
            ('d1', 'd2', 1),
            ('d1', 'f1', 1),
            ('d2', 'd3', 1),
            ('d2', 'f3', 1),
            ('d3', 'd4', 1),
            ('d4', 'c4', 1),
            ('e1', 'f1', 1),
            ('e2', 'e1', 1),
            ('e3', 'e2', 1),
            ('e3', 'f3', 1),
            ('e3', 'c2', 1),
            ('e4', 'e3', 1),
            ('f1', 'g1', 1),
            ('f2', 'd3', 1),
            ('f2', 'c2', 1), 
            ('f2', 'e2', 1),
            ('f3', 'g3', 1),
            ('f4', 'e4', 1),
            ('f4', 'c4', 1),
            ('g1', 'i1', 1),
            ('g2', 'f2', 1),
            ('g3', 'h2', 1),
            ('g3', 'i3', 1),
            ('g4', 'f4', 1),
            ('h1', 'i4', 1),
            ('h2', 'h1', 1),
            ('h3', 'h2', 1),
            ('h3', 'i3', 1),
            ('h4', 'h3', 1),
            ('i1', 'j1', 1),
            ('i2', 'g2', 1),
            ('i2', 'h2', 1),
            ('i3', 'j3', 1),
            ('i4', 'h4', 1),
            ('i4', 'g4', 1),
            ('j1', 'k1', 1),
            ('j2', 'i2', 1),
            ('j3', 'k3', 1),
            ('j4', 'i4', 1),
            ('k1', 'k2', 1),
            ('k2', 'j2', 1),
            ('k2', 'k3', 1),
            ('k3', 'k4', 1),
            ('k4', 'j4', 1)
             ]
             
	g = defaultdict(list)
	for l,r,c in edges:
		g[l].append((c,r))

	q, seen = [(0,f,())], set()
	while q:
		(cost,v1,path) = heappop(q)
		if v1 not in seen:
			seen.add(v1)
			path = (v1, path)
			
			if v1 == t: 
				return (path)

			for c, v2 in g.get(v1, ()):
				if v2 not in seen:
					heappush(q, (cost+c, v2, path))

	return float("inf")
#_______________________________________________________________________________________________________________________
def pathdata(start,end):
	
	out = dijkstra(start, end)
	#print(out)
	data = {}
	aux=[]
	while len(out)>1:
		aux.append(out[0])
		out = out[1]
	
	aux.reverse()
	data['path']=aux
	path=aux
	return path


def nodecreater():

#checks to correct the difference between coordinates in case the node inputs from the user are not in the desired orientation

     
        
        #Distance values from a0 inches (x,y)
        a1=(35,24)  
        a2=(69,21)
        a3=(129,23)
        a4=(165,27)
        b1=(16,40)
        b2=(89,34)
        b3=(113,36)
        b4=(184,44)
        c1=(5,100)
        c2=(89,99)
        c3=(112,98)
        c4=(189,103)
        d1=(24,119)
        d2=(64,122)
        d3=(135,145)
        d4=(172,123)
        e1=(19,145)
        e2=(58,142)
        e3=(137,145)
        e4=(167,147)
        f1=(2,161)
        f2=(80,168)
        f3=(107,172)
        f4=(187,173)
        g1=(3,198)
        g2=(77,203)
        g3=(107,208)
        g4=(176,210)
        h1=(20,219)
        h2=(58,223)
        h3=(126,226)
        h4=(157,227)
        i1=(5,234)
        i2=(77,242)
        i3=(105,244)
        i4=(166,244)
        j1=(6,263)
        j2=(77,273)
        j3=(103,275)
        j4=(152,273)
        k1=(23,281)
        k2=(59,285)
        k3=(120,288)
        k4=(129,289)

        cordname= np.array(['a1','a2','a3','a4','b1','b2','b3','b4','c1','c2','c3','c4','d1','d2','d3','d4','e1','e2','e3','e4','f1','f2','f3','f4','g1','g2','g3','g4','h1','h2','h3','h4','i1','i2','i3','i4','j1','j2','j3','j4','k1','k2','k3','k4'])
        cordname = np.reshape(cordname,[11,4])
        datainch=[a1,a2,a3,a4,b1,b2,b3,b4,c1,c2,c3,c4,d1,d2,d3,d4,e1,e2,e3,e4,f1,f2,f3,f4,g1,g2,g3,g4,h1,h2,h3,h4,i1,i2,i3,i4,j1,j2,j3,j4,k1,k2,k3,k4]
        cord=datainch

        A=cord[0:4]
        B=cord[4:8]
        C=cord[8:12]
        D=cord[12:16]
        E=cord[16:20]
        F=cord[20:24]
        G=cord[24:28]
        H=cord[28:32]
        I=cord[32:36]
        J=cord[36:40]
        K=cord[40:44]

        node_array = np.array([A,B,C,D,E,F,G,H,I,J,K])
        row_avg = np.average(node_array[:,:,1],1) # average y value of each row (a, b, etc)
        col_avg = np.average(node_array[:,:,0],0) # average x value of each col (1, 2, etc)

        node_data = np.dstack([cordname,node_array])
        return node_array, cordname, row_avg, col_avg, node_data
        
        


##-------------------------------------------------------------------

## Function returns the name and distance to closest node from current
def closest_node(current_loc):
    [node_array,cordname,row_avg,col_avg,node_data]=nodecreater()
    [x_cur,y_cur,ang_cur] = np.array(current_loc)
    
    node_locs = node_array
    dist = np.zeros(node_locs.shape[0:2])

    #Find distance between current loc and closest node and name of
    for i in range(node_locs.shape[0]): #for every row 
        for j in range(node_locs.shape[1]): # for every column
            dist[i,j] = np.linalg.norm([x_cur,y_cur] - node_locs[i,j]) # take the norm of the distance vector

    min_index = np.unravel_index(np.argmin(dist),node_locs.shape[0:2]) # index of minimum value (convert from 'flat' index)

    close_dist = dist[min_index]
    close_node = cordname[min_index]

    #Check direction, find row and column nearest then backtrack to prev node
    if -10<ang_cur<10: #Right (road is up)
        #find out what row is associated (closest in positive y direction)
        close_row = np.array(np.where(row_avg > y_cur+2)).item(0)
        #find out what column is previous (closest in negative x direction)
        close_col = np.array(np.where(col_avg < x_cur)).item(-1)
    elif 80<ang_cur<100: #Up (road is left)
        close_row = np.array(np.where(row_avg < y_cur)).item(-1) #closest in negative y direction
        close_col = np.array(np.where(col_avg < x_cur)).item(-1) #closest in negative x direction
    elif 170<ang(ang_cur)<190: #Left (road is down)
        close_row = np.array(np.where(row_avg < y_cur-2)).item(-1) #closest in negative y direction (correct for sidewalk)
        close_col = np.array(np.where(col_avg > x_cur)).item(0) #closest in negative x direction
    else: #Down (road is right)
        close_row = np.array(np.where(row_avg > y_cur)).item(0) #closest in pos y direction
        close_col = np.array(np.where(col_avg > x_cur)).item(0) #closest in pos x direction
        
    prev_node = cordname[close_row,close_col]
        
    return np.array([close_dist, close_node, prev_node])

##[close_dist, close_node, prev_node] = closest_node([5,0,0])

##-------------------------------------------------------------------

##Parses the directions into individual connections

def connections(directions):
    direc = np.array(directions)
    begs = direc[0:len(direc)-1]
    ends = direc[1:len(direc)]
    connec = np.array([begs,ends])
    return connec

##direc = np.array(['a','b','c','d','e'])
##connec = connections(direc)
##print connec
##print connec[:,0]

##-------------------------------------------------------------------

## Distance to specific locations

def distance(current_loc,target_loc):
    
    [x_cur,y_cur,ang_cur] = np.array(current_loc)
##    [x_tar,y_tar,ang_tar] = np.array(target_loc)
##    [x_tar,y_tar] = np.array(target_loc) 
    [x_tar,y_tar] = target_loc
    dist = np.linalg.norm(np.array([x_cur,y_cur]) - np.array([x_tar,y_tar])) # take the norm of the distance vector

    return dist

##-------------------------------------------------------------------

# fixes angles

def ang(cur_ang):
    if cur_ang < 0:
        cur_ang = 360 + cur_ang
    elif cur_ang > 360:
        cur_ang = cur_ang - (cur_ang/360)*360
    else:
        cur_ang = cur_ang
    return cur_ang


##-------------------------------------------------------------------

##Difference in angle

def ang_compare(current_loc,ang_des):
    ang_cur = ang(current_loc.item(2))
    ang_dif = abs(ang_cur-ang_des)
    return ang_dif

##-------------------------------------------------------------------

## Figures out what instructions to follow for the current node pair
right = [['a1','b1'],
         ['c1','d1'],
         ['e1','f1'],
         ['j1','k1'],
         ['b2','a2'],
         ['d2','c2'],
         ['f2','e2'],
         ['i2','h2'],
         ['k2','j2'],
         ['a3','b3'],
         ['c3','d3'],
         ['e3','f3'],
         ['h3','i3'],
         ['j3','k3'],
         ['b4','a4'],
         ['d4','c4'],
         ['f4','e4'],
         ['i4','h4'],
         ['k4','j4']]

left = [['d2','f3'],
        ['c3','e2'],
        ['e3','c2'],
        ['f2','d3']]

## g->h is short then long, h->g is long then short (check map)

left_tight = [['g3','h2'],
              ['h3','g2']] 


def instructions(connections):
    instruct_list = []
    i = 0
    while i < connections.shape[1]:
        node_pair = connections[:,i].tolist()
        if node_pair in right:
            instruction = 'right'
        elif node_pair in left:
            instruction = 'left'
        elif node_pair in left_tight:
            instruction = 'left_tight'
        else:
            instruction = 'straight'
        instruct_list.append(instruction)
        i = i+1
    return instruct_list

##-------------------------------------------------------------------

# returns coordinates for node name

def node2cord(node_name):
	#index = np.array(np.where(np.isin(node_data,node_name)))
	[node_array,cordname,row_avg,col_avg,node_data]=nodecreater()
	ind=np.isin(node_data,node_name)
	index = np.where(np.isin(node_data,node_name))
	cord = node_array[index[1:]]
	cord = cord[0]
	return cord

##-------------------------------------------------------------------
# STRAIGHT
# Finds target coordinate and steps forward until that is reached

def go_straight(target_loc):
    current_loc = gpsgo().item(0)
    cur_dist = distance(current_loc, target_loc)
    while cur_dist > 3:
        speakpi('s')
        listen()
        current_loc = gpsgo().item(0)
        cur_dist = distance(current_loc, target_loc)

##-------------------------------------------------------------------
# RIGHT
# Finds target angle and steps forward until that is reached
        
def go_right():
    current_loc = gpsgo().item(0)
    ang_des = ang(current_loc.item(2)-90)
    ang_dif = ang_compare(current_loc,ang_des)
    while ang_dif > 3:
        speakpi('r')
        listen()
        current_loc = gpsgo().item(0)
        ang_dif = ang_compare(current_loc,ang_des)

##-------------------------------------------------------------------
# LEFT
# Finds target angle and steps forward until that is reached
        
def go_left():
    current_loc = gpsgo().item(0)
    ang_des = ang(current_loc.item(2)+90)
    ang_dif = ang_compare(current_loc,ang_des)
    while ang_dif > 3:
        speakpi('l')
        listen()
        current_loc = gpsgo().item(0)
        ang_dif = ang_compare(current_loc,ang_des)

##-------------------------------------------------------------------

def go_left_tight():
    current_loc = gpsgo().item(0)
    ang_des = ang(current_loc.item(2)+90)
    ang_dif = ang_compare(current_loc,ang_des)
    while ang_dif > 3:
        speakpi('L')
        listen()
        current_loc = gpsgo().item(0)
        ang_dif = ang_compare(current_loc,ang_des)

##-------------------------------------------------------------------   
 

