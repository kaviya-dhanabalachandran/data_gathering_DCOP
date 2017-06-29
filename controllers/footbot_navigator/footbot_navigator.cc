#include "footbot_navigator.h"


#ifndef FOOTBOT_LQL_SIM
#else
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/simulator.h>
#endif


FootbotNavigator::FootbotNavigator() :
  RandomSeed(12345),
  m_Steps(0)
{
}


  void 
FootbotNavigator::Init(TConfigurationNode& t_node) 
{
  /// The first thing to do, set my ID
#ifdef FOOTBOT_LQL_SIM
  m_myID = 
    atoi(GetRobot().GetRobotId().substr(3).c_str());
#else
  m_myID = 
    atoi(GetRobot().GetRobotId().substr(7).c_str());
#endif
  printf("MyID %d\n", m_myID);

  /// Random
  GetNodeAttributeOrDefault(t_node, "RandomSeed", RandomSeed, RandomSeed);

  /// create the client and pass the configuration tree to it
  m_navClient = new RVONavClient(m_myID, GetRobot());
  m_navClient->init(t_node);
  m_navClient->start();

}


  void 
FootbotNavigator::ControlStep() 
{
  m_Steps+=1;

  /* do whatever */
  m_navClient->setTime(getTime());
  m_navClient->update();
}


  void 
FootbotNavigator::Destroy() 
{
  DEBUG_CONTROLLER("FootbotNavigator::Destroy (  )\n");
}

/**************************************/

bool 
FootbotNavigator::IsControllerFinished() const 
{
  return false;
}


std::string
FootbotNavigator::getTimeStr()
{
#ifndef FOOTBOT_LQL_SIM
  char buffer [80];
  timeval curTime;
  gettimeofday(&curTime, NULL);
  int milli = curTime.tv_usec / 1000;
  strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", localtime(&curTime.tv_sec));
  char currentTime[84] = "";
  sprintf(currentTime, "%s:%d", buffer, milli);
  std::string ctime_str(currentTime);
  return ctime_str;
#else
  return "mytime";
#endif
}


/// returns time in milliseconds
  UInt64 
FootbotNavigator::getTime()
{
#ifndef FOOTBOT_LQL_SIM
  struct timeval timestamp;
  gettimeofday(&timestamp, NULL);

  UInt64 ms1 = (UInt64) timestamp.tv_sec;
  ms1*=1000;

  UInt64 ms2 = (UInt64) timestamp.tv_usec;
  ms2/=1000;

  return (ms1+ms2);
#else
  return m_Steps * CPhysicsEngine::GetSimulationClockTick() * 1000;
#endif
}


  
REGISTER_CONTROLLER(FootbotNavigator, "footbot_navigator")

