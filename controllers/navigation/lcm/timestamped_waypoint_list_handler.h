#ifndef __TIMESTAMPED_WAYPOINTLIST_HANDLER_H
#define __TIMESTAMPED_WAYPOINTLIST_HANDLER_H
/*
 * Copyright (C) 2014, IDSIA (Institute Dalle Molle for Artificial Intelligence), http://http://www.idsia.ch/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <argos2/common/utility/datatypes/datatypes.h>
#include "protectedmutex.h"
#include <stdio.h>
#include <map>
#include <set>
#include <lcm/lcm-cpp.hpp>

#include <pthread.h>
#include <argos2/common/utility/math/vector2.h>
#include <argos2/common/utility/math/vector3.h>

using namespace std;
using namespace argos;

/**
 * @brief LCM engine - (see more information at https://code.google.com/p/lcm/)
 *
 * @details Basically this class provides the necessary methods for using LCM
 * based engine.
 *
 * @author Roberto Magán Carrión, rmagan@ugr.es, rmagan@idsia.ch
 * @date 05/11/2014
 *
 */

namespace waypointlist
{
  class timestamped_waypoint_list_t;
}
struct Waypoint
{
  CVector3 pos;
  CVector3 ori;
  Real GetX(){ return pos.GetX();}
  Real GetY(){ return pos.GetY();}
  Real GetZ(){ return pos.GetZ();}
  Real GetYaw(){ return ori.GetX();}

};

bool operator<(const Waypoint &p1, const Waypoint &p2) ;

typedef std::pair< UInt64, Waypoint> TimestampedWaypoint;
struct TimestampedWaypointCompare
{
  bool 
    operator()(const TimestampedWaypoint & left, const TimestampedWaypoint & right) const
    {
      return left.first < right.first;
    }
};


typedef 
std::set< TimestampedWaypoint, TimestampedWaypointCompare > TimestampedWaypointList;

class TimestampedWaypointListHandler 
{

  public:

    /**
     * Default LCM constructor
     */
    TimestampedWaypointListHandler();

    /**
     * LCM constructor.
     *
     * @param url char, the LCM URL
     * @param channel string, the LCM channel
     */
    TimestampedWaypointListHandler(const char * url, const string &channel, bool autorun);

    bool run();
    /**
     * This method is in charge of to handle the message obtained from a specific LCM channel.
     *
     * @param rbuf
     * @param chan string, the specified channel.
     * @param msg poselcm::pose_list_t, the message obtained from the channel (automatically created by LCM engine from a previous specification)
     */
    void handleMessage(const lcm::ReceiveBuffer* rbuf, 
		       const std::string& chan, 
		       const waypointlist::timestamped_waypoint_list_t* msg);


    /**
     * Checks if LCM is ready.
     *
     * @return true if yes or false it not
     */
    inline bool isLCMReady(); 
    /**
     * To subscribe to a specific LCM channel to get messages from it.
     *
     * @param channel string, name of the channel.
     */
    inline void subscribeToChannel(const string & channel) ;

    /**
     * Checks if there are messages from the channel for a pre-specified time.
     *
     * @param timeout
     * @return status int, > 0 there were messages, == 0 timeout expired and < 0 an error occurred
     */
    int getAvailableMessagesTimeout(int timeout) 
    {
      return m_lcm.handleTimeout(timeout);
    }

    /**
     * Blocking call for waiting messages
     * @return status int, > 0 there were messages, == 0 timeout expired and < 0 an error occurred
     */
    int getAvailableMessages() 
    {
      return m_lcm.handle();
    }


    /* Getters & setters */

    const string& getLcmChannel() const {
      return m_lcmChannel;
    }

    void setLcmChannel(const string& lcmChannel) 
    {
      this->m_lcmChannel = lcmChannel;
    }

    const char* getLcmUrl() const 
    {
      return m_lcmURL;
    }

    void setLcmUrl(const char* lcmUrl) 
    {
      m_lcmURL = lcmUrl;
    }

    bool hasWaypoints(UInt8);

    std::pair< bool, TimestampedWaypoint >  getNextWaypoint(UInt8, UInt64 t);
    std::pair< bool, TimestampedWaypoint > popWaypoint(UInt8 id);
    void clear();

  private:

    static UInt64 getTime();

    map<UInt8, TimestampedWaypointList> m_waypointLists;

    /**
     * LCM URL
     */
    const char * m_lcmURL;

    /**
     * LCM channel
     */
    string m_lcmChannel;

    /**
     * LCM instance
     */
    lcm::LCM m_lcm;

    /**
     * Mutex to control the access to the listAllNodes
     */
    pthread_mutex_t m_mutex;
    /**
     * Thread
     */
    pthread_t m_thread;

    static void * internalThreadEntryFunc(void * ptr) 
    {
      (( TimestampedWaypointListHandler *) ptr)->internalThreadEntry();
      return NULL;
    }
    void internalThreadEntry();
};

#endif
