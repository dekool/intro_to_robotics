#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import socket
import json
import time
import errno
import threading
from errnames import get_error_name

def get_ip():
    from netifaces import interfaces, ifaddresses, AF_INET
    for interface in interfaces():
        try:
            for link in ifaddresses(interface)[AF_INET]:
                ip=str(link['addr'])
                if ip.startswith('192.168.1.'):
                    return ip
        except KeyError:
            pass
    return '127.0.0.1'

class RClient(object):
    """ 
    Robot python interface class
    Typical usage involves:
        
        r=RClient("192.168.1.151",2777)
        if not r.connect(): print error and exit
        while main_loop:
            r.drive(left_speed,right_speed)
            sensors=r.sense()
            some_calculations()
        r.terminate()
        
    """
    def __init__(self,host,port,user_deprecate='',id_deprecate=''):
        #self.ip = get_ip()
        self.ip = ''
        self.robot=(host,port)
        self.lock=threading.RLock()
        self.done=False
        self.sensors=[0.0,0.0,0.0,0.0,0.0]
        
    def connect(self):
        """ Connect to server and create processing thread """
        try:
            self.recv_thread=threading.Thread(target=self.recv_loop)
            self.recv_thread.start()
            return True
        except socket.error,e:
            reason=get_error_name(e[0])
            print "Socket Error: "+reason
        return False
            
    def recv_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.ip,9209))
        sock.setblocking(0)
        while not self.done:
            try:
                data, addr = sock.recvfrom(256)
                if len(data)==0:
                    time.sleep(0.05)
                else:
                    #print "Received from '{}' data='{}'".format(addr,data)
                    try:
                        self.sensors=[float(s) for s in data.split()]
                    except ValueError:
                        pass
            except socket.error,e:
                errnum=e[0]
                if errnum!=errno.EAGAIN:
                    reason=get_error_name(errnum)
                    print "Socket Error ({}): {}".format(errnum,reason)
                time.sleep(0.05)
                
            
    def sendmsg(self,msg):
        with self.lock:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(msg, self.robot)
                return True
            except socket.error:
                return False
            
    def terminate(self):
        """ Call before your program ends, for a clean exit """
        self.done=True
        self.recv_thread.join()
        
    def drive(self,left,right):
        """ Make the robot move.  Send 2 integers for motors [-1000 : 1000] """
        self.sendmsg("{} {}".format(left,right))
        
    def sense(self):
        """ Get a list of sensor readings.  5 floating point values:  X,Y, 3 sonars """
        return self.sensors


#
#  Following code is a simple test main that allows to control the robot
#  from the keyboard, and see raw sersor readings on the screen
#
done=False
cmd=''

def kbd():
    global cmd
    while cmd!='q':
        cmd=sys.stdin.readline().strip()
    cmd=''
    global done
    done=True

def test():
    global done
    global cmd
    r=RClient("192.168.1.159",2777)
    counter=0
    if r.connect():
        kbd_thread=threading.Thread(target=kbd)
        kbd_thread.start()
        while not done:
            # Keyboard input can be a pair of speeds, or q to exit
            if cmd:
                s=cmd.split()
                cmd=''
                if len(s)==2:
                    try:
                        r.drive(int(s[0]),int(s[1]))
                    except ValueError:
                        print "Invalid speeds"
            time.sleep(0.1)
            counter+=1
            #if (counter%10)==0:
            print r.sense()
        r.terminate()
        print "Done"
        kbd_thread.join()
    else:
        print "Failed to connect"


if __name__=='__main__':
    test()

