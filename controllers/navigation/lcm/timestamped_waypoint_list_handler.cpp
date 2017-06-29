#include "timestamped_waypoint_list_handler.h"
#include <waypointlist/timestamped_waypoint_list_t.hpp>

using namespace waypointlist;


bool operator<(const Waypoint &p1, const Waypoint &p2) 
{
  return (p1.pos < p2.pos);
}


TimestampedWaypointListHandler::TimestampedWaypointListHandler()
{
}

TimestampedWaypointListHandler::TimestampedWaypointListHandler(const char * url, 
							       const string &channel,
							       bool autorun)
{
  /// Create a new LCM instance
  m_lcm = lcm_create(url);
  /// Set LCM URL
  m_lcmURL = url;
  /// Set LCM channel
  m_lcmChannel = channel;
  /// FIXME: better outside?
  if (isLCMReady()) 
  {
    subscribeToChannel(channel);
  }
  pthread_mutex_init(&m_mutex, NULL);
  if( autorun )
    run();
}

bool
TimestampedWaypointListHandler::run()
{
//  printf("running\n");
  int status = pthread_create(&m_thread, NULL, internalThreadEntryFunc, this);
  return (status == 0);
}

void 
TimestampedWaypointListHandler::internalThreadEntry() 
{
  while (true) 
  {
    //printf("Waiting for messages\n");
    m_lcm.handle();
    //getAvailableMessages();
    //printf("Messages are received\n");
  }
}

inline bool 
TimestampedWaypointListHandler::isLCMReady() 
{
  if (!m_lcm.good()) 
  {
    printf("LCM is not ready :(");
    return false;
  } 
  else 
  {
    printf("LCM is ready :)");
    return true;
  }
}

inline bool
TimestampedWaypointListHandler::hasWaypoints(UInt8 id)
{
  return (m_waypointLists[id].size()>0);
}


std::pair< bool, TimestampedWaypoint >
TimestampedWaypointListHandler::popWaypoint(UInt8 id)
{
  TimestampedWaypointList &tlist = m_waypointLists[id];
  TimestampedWaypoint wp;
  if( !tlist.size() )
    return make_pair(false, wp);
  TimestampedWaypointList::iterator it = 
    tlist.begin();
  wp = *it;
  tlist.erase(it);
  return make_pair(true, wp);
}

void
TimestampedWaypointListHandler::clear()
{
  m_waypointLists.clear();
}

std::pair< bool, TimestampedWaypoint >  
TimestampedWaypointListHandler::getNextWaypoint(UInt8 id, UInt64 t)
{
  TimestampedWaypointList &tlist = m_waypointLists[id];
  TimestampedWaypoint wp;
  if( !tlist.size() )
    return make_pair(false, wp);
  TimestampedWaypointList::iterator it = 
    tlist.begin();
  if( t < (*it).first )
  {
    /// first point in list is ahead
    return make_pair(false, (*it));
  }
  wp = (*it);
  while( it != tlist.end() )
  {
    if( t < (*it).first )
      break;
    wp = (*it);
    it++;
  }
  return make_pair(true, wp);
}

inline void 
TimestampedWaypointListHandler::subscribeToChannel(const string & channel)
{
  printf("Listening to channel %s\n", channel.c_str());
  m_lcm.subscribe(channel, &TimestampedWaypointListHandler::handleMessage, this);
}

void 
TimestampedWaypointListHandler::handleMessage(const lcm::ReceiveBuffer* rbuf, 
						   const std::string& chan, 
						   const timestamped_waypoint_list_t* msg)
{
  // printf("Got wp list for robot %d with %d waypoints\n",
  //	 msg->robotid, msg->n);
 fflush(stdout);
  m_waypointLists[msg->robotid].clear();
  for (int i = 0; i < msg->n; i++) 
  {
    const timestamped_waypoint_t &wp = msg->waypoints[i];
    Waypoint mwp;
    mwp.pos=CVector3(wp.position[0], wp.position[1], wp.position[2]);
    mwp.ori=CVector3(wp.orientation[0], wp.orientation[1], wp.orientation[2]);
    mwp.pos /= 1000.0;
    UInt64 ts(wp.timestamp);
    m_waypointLists[msg->robotid].insert( make_pair(ts, mwp));
  }
}


