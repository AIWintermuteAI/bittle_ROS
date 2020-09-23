#!/usr/bin/env python3
import rospy
import serial
import struct
import sys
import time
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist

dir_dict = {1:'kwk', -1:'kbk', 2:'kcrR', 3:'kcrL', 0:'kbalance'}

class Driver:

    def __init__(self, port='/dev/ttyS0'):
        self.dir = 0
        rospy.init_node('cmd_vel_listener')
        rospy.Subscriber("/cmd_vel", Twist, self.callback)
        self.ser = serial.Serial(
        port=port,
        baudrate=57600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
        )

    def callback(self, msg):
        rospy.loginfo("Received a /cmd_vel message!")
        rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
        
        if msg.linear.x > 0:
            dir = 1
        elif msg.linear.x < 0:
            dir = -1
        elif msg.angular.z > 0:
            dir = 2
        elif msg.angular.z < 0:
            dir = 3
        else:
            dir = 0
 
        if self.dir != dir:
            self.wrapper([dir_dict[dir],0])
            self.dir = dir

    def run(self):
        rospy.spin()
    
    def wrapper(self, task):  #Structure is [token, var=[], time]
        print(task)
        if len(task)==2:
            self.serialWriteByte([task[0]])
        elif isinstance(task[1][0],int):
            self.serialWriteNumToByte(task[0],task[1])
        else:
            self.serialWriteByte(task[1])
        time.sleep(task[-1])
        
    def serialWriteNumToByte(self, token, var=[]): # Only to be used for c m u b i l o within Python
        #print("Num Token "); print(token);print(" var ");print(var);print("\n\n");
        if token == 'l' or token=='i':
            var=list(map(lambda x:int(x), var))
            instrStr=token+struct.pack('b' * len(var), *var)+'~'
        elif token =='c' or token =='m' or token =='u' or token =='b':
            instrStr = token + str(var[0])+" "+str(var[1])+'\n'
        print("!!!!"+ instrStr)
        self.ser.write(instrStr.encode())

    def serialWriteByte(self, var=[]):
        token = var[0][0]
        if (token == 'c' or token == 'm' or token=='b' or token=='u') and len(var)>=2:
            instrStr=""
            for element in var:
                instrStr=instrStr +element+" "
        elif token == 'l' or token=='i' :
            if(len(var[0])>1):
                var.insert(1,var[0][1:])       
            var[1:]=list(map(lambda x:int(x), var[1:]))
            instrStr = token+struct.pack('b' * len(var[1:]), *var[1:])+'~'
        elif token == 'w' or token == 'k':
            instrStr = var[0] + '\n'
        else:
            instrStr = token
        print("!!!!!!! "+instrStr)
        self.ser.write(instrStr.encode())
    
if __name__ == '__main__':
    driver = Driver()
    driver.run()
