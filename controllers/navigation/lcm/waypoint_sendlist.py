#!/usr/bin/env python
"""
   waypoint_sendlist_bm - publish timestamped waypoint list from bonnmotion traces
   
    Copyright (C) 2014 Eduardo Feo
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import lcm
from waypointlist import timestamped_waypoint_list_t as waypoint_list
from waypointlist import timestamped_waypoint_t as waypoint
import sys
import time
from xml.dom import minidom
from optparse import OptionParser
#############################################################
#############################################################
class WaypointScenarioBM():
#############################################################
#############################################################

    #############################################################
    def __init__(self, scenario, traces, shiftX, shiftY, shiftT):
    #############################################################
        if not scenario:
            print "Error: Missing scenario file"
            print scenario
            exit(1)
        print "Reading scenario from ",scenario
        self.configureScenario(scenario)
        #self.x_shift = rospy.get_param("~x_shift", 0.0)
        #self.y_shift = rospy.get_param("~y_shift", 0.0)
        self.x_shift = float(shiftX);
        self.y_shift = float(shiftY);
        self.t_shift = float(shiftT)
        #traces = rospy.get_param("~bm_traces", None)
        if not traces:
            print "Error: Missing trace file"
            exit(1)
        print "reading traces from",traces
        self.readTraces(traces)
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1") 

    #############################################################
    def readTraces(self, tfile):
        f = open(tfile)
        self.wps = []
        for line in f.readlines():
            if len(self.wps) >= len(self.robots):
                break
            wplist=[]
            s = line.split()
            i=0
            while i<len(s):
                wplist.append( (float(s[i]), float(s[i+1]),float(s[i+2]),float(s[i+3])) )
                i+=4
            print "read ",len(wplist)," waypoints for robot ",self.robots[len(self.wps)]
            self.wps.append(wplist)


    #############################################################
    def configureScenario(self, cfile):
        xmldoc = minidom.parse(cfile)
        itemlist = xmldoc.getElementsByTagName('robot') 
        print len(itemlist)
        self.goal_pub = []
        self.robot_to_ix = {}
        self.robots = []
        ix = 0
        for s in itemlist :
            rid = int(s.attributes['robotid'].value)
            rtype = s.attributes['type'].value
            """ comment to send also to bs and rn """
            if rtype != 's':
                if not s.attributes.has_key('force_send_wp'):
                    continue

            self.robot_to_ix[rid] = ix
            self.robots.append(rid)
            ix+=1

    def sendWPLists(self, N=None, s=None):
        for rid in self.robots:
            self.sendWPList(rid,N, s)

    def sendWPList(self,rid, N=None, s=None):
        if N:
            if s:
                print "sending %d wps from ix %d of robot "%(N,s), rid
            else:
                print "sending %d wps of robot "%(N), rid
        else:
            print "sending wps of robot ", rid
        msg = waypoint_list()
        msg.timestamp = self.t_shift
        msg.robotid = rid
        if s:
            nl = s
        else:
            nl = 0

        if N:
            nu = nl+N
        else:
            nu = len(self.wps[self.robot_to_ix[rid]])

        msg.n = len(self.wps[ self.robot_to_ix[rid]][s:nu])
        wps=[]
        for (t,x,y,o) in self.wps[self.robot_to_ix[rid]][s:nu]:
            wp = waypoint()
            """ in milliseconds """
            wp.timestamp = (t+self.t_shift)*1e3
            """ in mm """
            wp.position = [(x+self.x_shift)*1000.0,(y+self.y_shift)*1000.0,-1]
            wp.orientation = [o,0,0,0]
            wps.append(wp)
        msg.waypoints = wps
        self.lc.publish("TARGET", msg.encode())
        print "sent ", (nu-nl), " waypoints"
    #############################################################

    #############################################################
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """

    if len(sys.argv) < 3:
        print "Invalid number of arguments"
        exit(1)
    scene = sys.argv[1]
    traces = sys.argv[2]
    """ default values"""
    dx = 0.0
    dy = 0.0
    dt = 0

    if len(sys.argv) > 3:
        dx = float(sys.argv[3])
        if len(sys.argv) > 4:
            dy = float(sys.argv[4])
            if len(sys.argv) > 5:
                dt = int(sys.argv[5])
    bmscene = WaypointScenarioBM(scene, traces,dx, dy, dt)
    if len(sys.argv) <= 6:
        bmscene.sendWPLists()
    elif len(sys.argv) == 7:
        bmscene.sendWPLists(int(sys.argv[6]))
    elif len(sys.argv) == 8:
        bmscene.sendWPLists(int(sys.argv[6]), int(sys.argv[7]))
    else:
        print "WTF?"
