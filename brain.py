#import the correct files and registers
from commands import gpsgo,nodecreater,closest_node,connections,distance,ang,ang_compare,instructions,node2cord,go_straight,go_right,go_left,go_left_tight,listen,speakpi,writeNumber,readarduino,all_same,dijkstra,pathdata
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

address = 0x04
bus = smbus.SMBus(1)

#MAP___________________________________________________________________________________________________________________

#generate coordinate system using MAPPING/node_generator.py
#node_generator.py  COMMAND FUNCTION: node_generator(x,y) INPUTS: a1 locations measured by camera
print('Phase 1 MAPPING')
[node_array,cordname,row_avg,col_avg,node_data]=nodecreater()
#MAP=nodecreater
#GPS_______________________________________________________________________________________________________________________________________________________

#check the gps coordinates using RFM69/parsedGPS.py
#parsedGPS.py COMMAND FUNCTION: gpsgo()
print('Phase 2 GPS')
[ROB,PSG,DES]=gpsgo()
print('Passenger Found!')
print(ROB,PSG,DES)


#CLOSEST NODE___________________________________________________________________________________________________________________

#find closest node information using node finder
'''END=closest_node(PSG).item(2)
START_DIST=closest_node(ROB).item(0)
print START_DIST
#set TOLERANCE greater than motor step
TOL=10
#for start location move robot forward some increment and check GPS and use closest node until we get a start position

while (START_DIST>TOL):
  speakpi('straight')
  #print readarduino()
  print 'spoke'
  [ROB,PSG,DES]=gpsgo()
  print 'good'
  START_DIST=closest_node(ROB).item(0)
  print 'checked'
START=closest_node(ROB).item(1)
'''
START='a1'
END='c1'
#PATH AND DRIVE INSTRUCTIONS___________________________________________________________________________________________________________________

#use pathfinder algorithm ART_DIST=closest_node(ROB).item(0)for proper driver location Dijkstra
#dijkstra.py COMMAND FUNCTION: pathdata("start","end") INPUTS: ex. pathdata("a1","d2")
print('Phase 3 Path finding')
PATH=pathdata(START,END)

#asses drive instructions per node. Continuous loop to send node case until next node is in range. Then send next node drive instructions. 
#connections.py  COMMAND FUNCTION: connections(directions)
print('Phase 4 connections')
CONNECT=connections(PATH)
INSTRUCT=instructions(CONNECT)
print('Phase 5 node connection drive instructions')
    #Send drive instructions to arduino using i2c communications
    #PIi2c.py  COMMAND FUNCTION: speakpi("hello"), readarduino() 
i=0

print(node_data)
print(CONNECT[1,0])
print np.isin(node_data,CONNECT[1,0])
while (i<(len(PATH)-1)):
    TARGET = node2cord(CONNECT[1,i].tolist)
    if (INSTRUCT[i]=='straight'):
      go_straight(TARGET)
      i+=1
    elif (INSTRUCT[i]=='right'):
      go_right()
      i+=1         
    elif (INSTRUCT[i]=='left'):
      go_left()
      i+=1
    elif (INSTRUCT[i]=='left_tight'):
      go_left_tight()
      i+=1
    else:
      print('final node before passenger reached')
      i+=1

#GET TO PASSENGER_____________________________________________________________________________________________________              
      
#once final node before passenger is achieved run algorithm for Arduino to drive until passenger is found
PSG_LOC=PSG[:-1]
go_straight(PSG_LOC)
#pick up passenger using Lasso commands
speakpi('p')
listen()

#DRIVE TO NEXT NODE_____________________________________________________________________________________________________
[ROB,PSG,DES]=gpsgo()
END=closest_node(DES).item(0)
START_DIST=closest_node(ROB).item(0)

#dijkstra to find shortest path
PATH=pathdata(START,END)
CONNECT=connections(PATH)
INSTRUCT=instruction(CONNECT)
#send drive instructions to arudino
i=0
while (i<(len(PATH)-1)):
    TARGET = node2cord(CONNECT[1,i])
    if (INSTRUCT[i]=='straight'):
      go_straight(TARGET)
      i+=1
    elif (INSTRUCT[i]=='right'):
      go_right()
      i+=1         
    elif (INSTRUCT[i]=='left'):
      go_left()
      i+=1
    elif (INSTRUCT[i]=='left_tight'):
      go_left_tight()
      i+=1
    else:
      print('final node before passenger reached')
      i+=1
      
#DRIVE SLOW THROUGH DROP OFF__________________________________________________________________________________________________
#once final node closest to destination is reached. drive and continuous check gps until range value is reached
FINAL_DEST=DES[:-1]
go_straight(FINAL_DEST)
#drop off passenger using lasso commands
speakpi('d')
listen()
print ('passenger eliminated')
NEWPSG=PSG
while (NEWPSG==PSG):
  print('waiting for new passenger location')
  [ROB,PSG2,DES]=gpsgo()
  NEWPSG=PSG2
  
#go back to top of while loop and start next passenger pick up!__________________________________________________________________________________________________
