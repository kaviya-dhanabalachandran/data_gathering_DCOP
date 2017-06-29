#!/usr/bin/env python
"""
   waypointlist_listener - listen timestamped waypoint lists
   
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
#############################################################
#############################################################

def my_handler(channel, data):
    msg = waypoint_list.decode(data)
    print("Received message on channel \"%s\"" % channel)
    print("Waypoint list for robot %d" % msg.robotid)
    print("number of waypoints: %d" % len(msg.waypoints))
    print("")



 #############################################################
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """

    lcm = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    subscription = lcm.subscribe(sys.argv[1], my_handler)
    try:
        while True:
            lcm.handle()
    except KeyboardInterrupt:
        pass

    lcm.unsubscribe(subscription)
