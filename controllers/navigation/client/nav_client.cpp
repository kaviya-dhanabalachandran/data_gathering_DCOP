#include "nav_client.h"
#include <navigation/lcm/lcmhandler.h>

/// to handle debug and report macros
#define STR_EXPAND(tok) #tok
#define STR(tok) STR_EXPAND(tok)

#define __USE_DEBUG_NAV 0
#define __USE_ERROR_NAV 1

#if __USE_DEBUG_NAV
#define DEBUGNAV(m, ...) \
{\
  std::string t_str = getTimeStr();\
  fprintf(stderr, "%s DEBUGNAV[%d]: " m,\
	  t_str.c_str(),\
	  (int) m_myID,\
          ## __VA_ARGS__);\
  fflush(stderr);\
}
#else
#define DEBUGNAV(m, ...)
#endif

#if __USE_ERROR_NAV
#define ERRORNAV(m, ...) \
{\
  fprintf(stderr, "ERRORNAV[%d]: " m,\
	  (int) m_myID,\
          ## __VA_ARGS__);\
  fflush(stderr);\
}
#else
#define ERRORNAV(m, ...)
#endif


#ifdef USE_WP_LIST
TimestampedWaypointListHandler *RVONavClient::m_wpControl =
  new TimestampedWaypointListHandler("udpm://239.255.76.67:7667?ttl=1", 
				 "TARGET", true);
#else
LCMThread * RVONavClient::lcmThreadCommand = new LCMThread();
#endif

LCMThread * RVONavClient::lcmThread = new LCMThread();
bool RVONavClient::m_lcm_isset = false;

RVONavClient::RVONavClient(UInt8 robot_id, CCI_Robot &robot) :
  m_myID(robot_id),
  m_randomGen(NULL),
  m_randomSeed(12345),
  m_Steps(0),
  m_state(STATE_INIT),
  m_pcWheels(NULL), 
  m_ledsActuator(NULL), 
  m_targetMinPointDistance(0.12f),
  m_targetMinPopDistance(0.24f),
  m_cAlpha(10.0f), 
  m_fDelta(1.0f), 
  m_fWheelVelocity(5.0f), 
  m_cGoStraightAngleRange(-ToRadians(m_cAlpha), 
			  ToRadians(m_cAlpha)) 
{
  m_robot = &robot;
  m_ledMode=LEDS_MOBILITY;
  m_mobMode=MOBILITY_LCM;
  m_speedMode = SPEED_FIXED;
  m_beaconMode = BEACON_NONE;
  m_doTargetOrientation = false;
  m_targetOrientation = 0.0;
  m_spinVel = 10.0;
  m_enforcing = false;
  m_enforcingSpeed = 0.25;
  m_agentSpeed = 0.0;
  m_started = false;
  m_speedRandomLow = 0.05;
  m_speedRandomHigh = 0.25;
  m_targetTimeout = false;
  m_cntTargetTime = getTime();
  m_wpTimeout = 10*1000; /// in ms
  m_obstacleAvoidanceCounter = 0;
  m_obstacleAvoidanceAllowance = 10; /// in steps
}

void
RVONavClient::setAgentSpeed(Real speed)
{
  printf("RVONavClient setAgentSpeed %f\n", speed);
  m_agentSpeed = speed;
  setOptimalSpeed(m_agentSpeed);
}

/// trick: we do not need a state STOPPED
/// instead, we only stop the wheels and disable updating the velocity
/// in this way, the module works normally, just skipping the setVelocity
void
RVONavClient::start()
{
  if( m_started )
    return;
  m_randomGen->Reset();
  m_started = true;
  m_enforcing = false;
  if( m_speedMode == SPEED_RANDOM_UNIFORM )
    {
      Real randSpeed =
	m_randomGen->Uniform(CRange<Real>(m_speedRandomLow, m_speedRandomHigh));
      setAgentSpeed(randSpeed);
    }
  else
    {
      setOptimalSpeed(m_agentSpeed);
    }
  setState(STATE_INIT_READY);
}

void
RVONavClient::stop()
{
  if( !m_started )
    return;
  m_pcWheels->SetLinearVelocity(0,0);
  m_enforcing = false;
  m_started = false;
#ifdef USE_WP_LIST
  m_wpControl->clear();
#endif
}
UInt8
RVONavClient::robotId()
{
  return m_myID;
}

  bool 
RVONavClient::isNodeValid( const Node &n )
{
#ifndef FOOTBOT_SIM
  UInt64 dt = getTime() - n.getTimestamp();
  return (dt < NODE_VALIDITY_THRESHOLD);
#else
  /// in simulation, assume everything is valid
  /// i.e., we do not lose track of the robots
  return true;
#endif
}


bool
RVONavClient::hasValidInfo( UInt8 nid)
{
  map<UInt8, Node>::const_iterator it = m_nodes.find(nid);
  if( it == m_nodes.end())
    return false;
  return isNodeValid(it->second);
}

///TODO: preserve encapsulation
const map<UInt8, Node> &
RVONavClient::nodes()
{
  return m_nodes;
}


void 
RVONavClient::setLeds()
{
  if( m_ledMode == LEDS_MOBILITY )
  {
    switch (m_state )
    {
    case STATE_INIT_READY:
      m_ledsActuator->SetAllColors(CColor::BLUE);
      break;
    case STATE_ARRIVED_AT_TARGET:
      m_ledsActuator->SetAllColors(CColor::RED);
      break;
    case STATE_MOVING:
      m_ledsActuator->SetAllColors(CColor::GREEN);
      break;
    case STATE_ORIENTING:
      m_ledsActuator->SetAllColors(CColor::YELLOW);
      break;
    default:
      break;
    }
  }
  else if( m_ledMode == LEDS_FIXED )
  {
    m_ledsActuator->SetAllColors(m_ledFixedColor);
  }
}

void
RVONavClient::setBeacon()
{
  if( m_beaconMode == BEACON_MOBILITY || 
      m_beaconMode == BEACON_MOBILITY_LETTER_DEMO )
  {
    switch (m_state )
    {
    case STATE_INIT_READY:
      m_beaconActuator->SetColor(CColor::BLUE);
      break;
    case STATE_ARRIVED_AT_TARGET:
      if( m_beaconMode == BEACON_MOBILITY_LETTER_DEMO )
      {
	/// check if it is outside area
	position = CVector2(mySelf.getPosition().GetX(), mySelf.getPosition().GetY());
	if( position.GetX() < -0.3 ||
	    position.GetY() < -0.3 )
	{
	  m_beaconActuator->SetColor(CColor::BLACK);
	}
	else
	{
	  m_beaconActuator->SetColor(CColor::RED);
	}
      }
      else
      {
	m_beaconActuator->SetColor(CColor::RED);
      }
      break;
    case STATE_MOVING:
      m_beaconActuator->SetColor(CColor::GREEN);
      break;
    case STATE_ORIENTING:
      m_beaconActuator->SetColor(CColor::YELLOW);
      break;
    default:
      break;
    }
  } 

}

void
RVONavClient::setOptimalSpeed(Real speed)
{
  agent->optimalSpeed = speed;
}

void 
RVONavClient::setState(RobotNavState s)
{
  if( s == m_state )
    return;
  /// change of state
  m_state = s;
  if( s == STATE_ARRIVED_AT_TARGET || s == STATE_INIT_READY)
  {
    setLeds();
    setBeacon();
    /// we assume that robot arrived to enforced pos
    /// BUG: we need to update navigation once to stop the robot
    /*
    if( m_enforcing )
      {
	stay();
	// first, update nav
	updateNavigation();
	/// then, clear the flag
	m_enforcing = false;
	/// and go back to original speed
	setOptimalSpeed(m_agentSpeed);
      }
    */
  }

  if( s == STATE_MOVING || s == STATE_ORIENTING)
  {
    setLeds();
    setBeacon();
  
  }

}

RobotNavState
RVONavClient::state()
{
  return m_state;
}

  void 
RVONavClient::init(TConfigurationNode& t_node) 
{
  DEBUGNAV("init\n");
  /// Get actuators and sensors
  m_pcWheels = dynamic_cast<CCI_FootBotWheelsActuator*>(m_robot->GetActuator("footbot_wheels"));
  m_ledsActuator = dynamic_cast<CCI_FootBotLedsActuator*>(m_robot->GetActuator("footbot_leds"));
  m_beaconActuator          = dynamic_cast<CCI_FootBotBeaconActuator*>   (m_robot->GetActuator("footbot_beacon"));

  /// Random
  GetNodeAttributeOrDefault(t_node, "m_randomSeed", m_randomSeed, m_randomSeed);
  if( m_randomSeed == 12345)
    m_randomSeed+=m_myID;
  printf("RVONavClient randomSeed %d\n", m_randomSeed);


  /// distance threshold to determine that robot reached target point
  std::string text;

  if (NodeExists(t_node, "targetMinPointDistance")) 
  {
    GetNodeText(GetNode(t_node, "targetMinPointDistance"), text);
    sscanf(text.c_str(), "%f", &m_targetMinPointDistance);
    /// default for pop distance, double
    m_targetMinPopDistance = 2*m_targetMinPointDistance;
  }
  if (NodeExists(t_node, "targetMinPopDistance")) 
  {
    GetNodeText(GetNode(t_node, "targetMinPopDistance"), text);
    sscanf(text.c_str(), "%f", &m_targetMinPointDistance);
  }

  if (NodeExists(t_node, "originAreaX")) 
  {
    GetNodeText(GetNode(t_node, "originAreaX"), text);
    sscanf(text.c_str(), "%f", &originAreaX);
  }

  if (NodeExists(t_node, "originAreaY")) 
  {
    GetNodeText(GetNode(t_node, "originAreaY"), text);
    sscanf(text.c_str(), "%f", &originAreaY);
  }

  if (NodeExists(t_node, "destinationAreaX")) {
    GetNodeText(GetNode(t_node, "destinationAreaX"), text);
    sscanf(text.c_str(), "%f", &destinationAreaX);
  }

  if (NodeExists(t_node, "destinationAreaY")) {
    GetNodeText(GetNode(t_node, "destinationAreaY"), text);
    sscanf(text.c_str(), "%f", &destinationAreaY);
  }

  DEBUGNAV("Area bounds-> ORIGIN (%f,%f), DESTINATION (%f,%f)\n", 
	   originAreaX, originAreaY, destinationAreaX, destinationAreaY);
  originAreaCoord.SetX(originAreaX);
  originAreaCoord.SetY(originAreaY);
  destinationAreaCoord.SetX(destinationAreaX);
  destinationAreaCoord.SetY(destinationAreaY);

  CARGoSRandom::CreateCategory("navclient", m_randomSeed);
  m_randomGen = CARGoSRandom::CreateRNG("navclient");

  printf("checking for led conf\n");
  if (NodeExists(t_node, "ledcontrol")) 
  {
    TConfigurationNode node = GetNode(t_node, "ledcontrol");
    string ledMode("DEFAULT");
    GetNodeAttributeOrDefault<std::string>(node, "mode", ledMode, ledMode);
    if ( ledMode == "DEFAULT" )
      m_ledMode = LEDS_NONE;
    else if ( ledMode == "MOBILITY" )
      m_ledMode = LEDS_MOBILITY;
    else if ( ledMode == "FIXED" )
      m_ledMode = LEDS_FIXED;
    else
      m_ledMode = LEDS_NONE;

    string ledFixedColor("black");
    GetNodeAttributeOrDefault<std::string>(node, "fixedColor", 
					   ledFixedColor, ledFixedColor);
    stringstream ss(ledFixedColor);
    ss >> m_ledFixedColor;
    cout << "ledMode " << ledMode << " fcolor " << ledFixedColor << endl;
  }
  else
  {
  }

  if (NodeExists(t_node, "beaconcontrol")) 
  {
    TConfigurationNode node = GetNode(t_node, "beaconcontrol");
    string beaconMode("DEFAULT");
    GetNodeAttributeOrDefault<std::string>(node, "mode", beaconMode, beaconMode);
    if ( beaconMode == "DEFAULT" )
      m_beaconMode = BEACON_NONE;
    else if ( beaconMode == "MOBILITY" )
      m_beaconMode = BEACON_MOBILITY;
    else if ( beaconMode == "MOBILITY_LETTER_DEMO" )
      m_beaconMode = BEACON_MOBILITY_LETTER_DEMO;
    else if ( beaconMode == "FIXED" )
      m_beaconMode = BEACON_FIXED;
    else
      m_beaconMode = BEACON_NONE;

    string beaconFixedColor("black");
    GetNodeAttributeOrDefault<std::string>(node, "fixedColor", 
					   beaconFixedColor, beaconFixedColor);
    stringstream ss(beaconFixedColor);
    ss >> m_beaconFixedColor;
  }
  else
  {
  }

  if (NodeExists(t_node, "speedcontrol")) 
  {
    TConfigurationNode node = GetNode(t_node, "speedcontrol");
    string speedMode("DEFAULT");
    GetNodeAttributeOrDefault<std::string>(node, "mode", speedMode, speedMode);
    if ( speedMode == "DEFAULT" )
      m_speedMode = SPEED_FIXED;
    else if ( speedMode == "RANDOM_UNIFORM" )
      m_speedMode = SPEED_RANDOM_UNIFORM;

    GetNodeAttributeOrDefault(node, "speedRandomLow", 
			      m_speedRandomLow,
			      m_speedRandomLow);
    GetNodeAttributeOrDefault(node, "speedRandomHigh", 
			      m_speedRandomHigh,
			      m_speedRandomHigh);
  }
  else
  {
    printf("NO speed CONF\n");
  }

  fflush(stdout);
  setLeds();
  setBeacon();


  if (NodeExists(t_node, "mobilitycontrol")) 
  {
    TConfigurationNode node = GetNode(t_node, "mobilitycontrol");
    string mobMode("DEFAULT");
    GetNodeAttributeOrDefault<std::string>(node, "mode", mobMode, mobMode);
    if ( mobMode == "LCM" )
      m_mobMode = MOBILITY_LCM;
    else if ( mobMode == "DEFAULT" || mobMode == "NONE" )
      m_mobMode = MOBILITY_NONE;
    else if ( mobMode == "LCM_POP" )
      m_mobMode = MOBILITY_LCM_POP;
    else if ( mobMode == "MANUAL" )
      {
	m_mobMode = MOBILITY_MANUAL;
      }
    else if ( mobMode == "SPIN" )
      {
	m_mobMode = MOBILITY_SPIN;
	double spinVel=10.0;
	GetNodeAttributeOrDefault<double>(node, "spinVel", spinVel, spinVel);
	m_spinVel = spinVel;	
      }
    else
      {
	printf("invalid mobility mode %s\n", mobMode.c_str());
	m_mobMode = MOBILITY_NONE;
      }
    GetNodeAttributeOrDefault(node, "wpTimeout", m_wpTimeout, m_wpTimeout);
    DEBUGNAV("Mobility Mode %s\n", mobMode.c_str());
  }
  else
  {
  }


  /** LCM engine */
  /// LCM Thread
  std::string track_chan("TRACK");
  std::string target_chan("TARGET");
  if (NodeExists(t_node, "lcm")) 
  {
    TConfigurationNode node = GetNode(t_node, "lcm");
    GetNodeAttributeOrDefault<std::string>(node, "track", track_chan, track_chan);
  }
  else
  {
    printf("NO LCM CONF\n");
  }
  if( m_mobMode == MOBILITY_LCM || m_mobMode == MOBILITY_LCM_POP )
    {
#ifndef USE_WP_LIST
      lcmThreadCommand->setLCMEngine("udpm://239.255.76.67:7667?ttl=1", "TARGET");
      lcmThreadCommand->startInternalThread();
#else
  //m_wpControl = 
  //new TimestampedWaypointListHandler("udpm://239.255.76.67:7667?ttl=1", 
  //	       "TARGET");
//  if( !m_wpControl->run() )
//    DEBUGERROR("error while running waypoint control\n");
#endif
    }

  /// lcm is set ONCE
  /// there's a maximum  number of lcm instances that can be created
  /// when this number is reached, you get
  /// assertion in lcm_udp
  if(!m_lcm_isset )
    {
      lcmThread->setLCMEngine("udpm://239.255.76.67:7667?ttl=1", track_chan.c_str());
      lcmThread->startInternalThread();
      m_lcm_isset = true;
    }
  /** NAVIGATION AND AVOIDING COLLISION */
  /* Additional sensors */
  // we do not have encoderSensor in simulation
  //encoderSensor = dynamic_cast<CCI_FootBotEncoderSensor*>(m_robot->GetSensor("footbot_encoder"));

 
  /// Init navigation methods 
  initOdometry();
  initLocalNavigation(t_node);

    

}

void
RVONavClient::addExternalCircularObstacle(UInt32 id, CVector3 pos, Real rad)
{
  //   cout << "RVONavClient: addExternalCircularObstacle " << id
  //      << " " << pos.GetX() << " " << pos.GetY()
  //     << " r: " << rad <<  endl;
   fflush(stdout);
  m_extObstacles[id] = Obstacle(id, pos,rad);
}

void
RVONavClient::clearExternalObstacles()
{
  m_extObstacles.clear();
}

void
RVONavClient::processConfig(std::string msg)
{
  std::stringstream ss(msg);
  std::string cmd;
  ss >> cmd;
  //  cout << "RVONavClient: got config cmd " << cmd << endl;
  if( cmd == "OBSTACLE" )
    {
      std::string action;
      ss >> action;
      if( action == "ADD" )
	{
	  UInt32 obstacle_id;
	  Real x,y,z,rad;
	  ss >> obstacle_id >> x >> y >> z >> rad;
	  addExternalCircularObstacle(obstacle_id,CVector3(x,y,z),rad);
	}
      if( action == "RESET_LIST" )
	{
	  clearExternalObstacles();
	  UInt32 m;
	  ss >> m;
	  while(m--)
	    {
	      UInt32 obstacle_id;
	      Real x,y,z,rad;
	      ss >> obstacle_id >> x >> y >> z >> rad;
	      addExternalCircularObstacle(obstacle_id,CVector3(x,y,z),rad);
	    }
	}

      if( action == "CLEAR" )
	{
	  clearExternalObstacles();
	}
    }
  if( cmd == "FORCE_POS" )
    {
      Real x,y,ori;
      ss >> x >> y >> ori;
      //      cout << "RVONavClient: FORCE_POS " << x << ", " << y;
      printf("FORCE_POS (%f,%f):%.2f\n", 
	       x, y, ori);
      DEBUGNAV("FORCE_POS (%f,%f):%.2f\n", 
	       x, y, ori);
      m_targetPosition.Set( x, y);
      m_targetOrientation = ori;
      //m_agentSpeed = agent->optimalSpeed;
      setOptimalSpeed(m_enforcingSpeed);
      m_enforcing = true;
    }
    if( cmd == "FORCE_POS_VEL" )
    {
      Real x,y,ori,vel;
      ss >> x >> y >> ori >> vel;
      //      cout << "RVONavClient: FORCE_POS " << x << ", " << y;
      DEBUGNAV("FORCE_POS_VEL (%f,%f):%.2f v: %.2f\n", 
	       x, y, ori, vel);
      m_targetPosition.Set( x, y);
      m_targetOrientation = ori;
      //m_agentSpeed = agent->optimalSpeed;
      if( vel > 0 )
	{
	  if( vel > 0.3 )
	    vel = 0.3;
	  setOptimalSpeed(vel);
	  m_enforcing = true;
	}
    }
}


void
RVONavClient::updateTargetPosition(bool pop_wp)
{
  if( m_enforcing )
    {
      DEBUGNAV("SetTargetPosition -REJECTED because enforcing\n");
      return;
    }
  m_targetTimeout = false;
  m_cntTargetTime = getTime();

  if( m_mobMode == MOBILITY_LCM )
    {
      CRadians oA;
      CVector3 axis;

      
  /// New target point from the COMMAND engine
#ifdef USE_WP_LIST
      std::pair< bool, TimestampedWaypoint> c_wp =
	m_wpControl->getNextWaypoint(m_myID,getTime());
      if( c_wp.first )
	{
	  CVector2 n_wp(c_wp.second.second.GetX(), c_wp.second.second.GetY());
	  Real n_ori = c_wp.second.second.GetYaw();
	  if( m_targetPosition != n_wp || m_targetOrientation != n_ori)    
	    {
	      DEBUGNAV("NEW_WP (%f,%f):%.2f valid from %llu\n", 
		       n_wp.GetX(), n_wp.GetY(), n_ori, c_wp.second.first);
	      m_targetPosition.Set( n_wp.GetX(), n_wp.GetY());
	      m_targetOrientation = n_ori;
	    }
	} 
      else
	{
	  DEBUGNAV("STAY\n");
	  /// stay in same pos
	  stay();
 	}
#else
      if (lcmThreadCommand->getLcmHandler()->existNode(m_myID)) 
	{
	  Node nodeCommand = 
	    lcmThreadCommand->getLcmHandler()->getNodeById(m_myID);
	  m_targetPosition.Set(nodeCommand.getPosition().GetX(), 
			       nodeCommand.getPosition().GetY());
	  DEBUGNAV("ID %d - SET_TARGET: (%f,%f)\n", 
		   (int) m_myID, 
		   nodeCommand.getPosition().GetX(), 
		   nodeCommand.getPosition().GetY());
	} 
      else 
	{
	  if (lcmThread->getLcmHandler()->existNode(m_myID)) 
	    {
	      stay();
	      // Node nodeCommand = 
	      // 	lcmThread->getLcmHandler()->getNodeById(m_myID);
	      // m_targetPosition.Set(nodeCommand.getPosition().GetX(), 
	      // 			   nodeCommand.getPosition().GetY());
	      // DEBUGNAV("ID %d - REMAIN_IN_POS: (%f,%f)\n", 
	      // 	       (int) m_myID, 
	      // 	       nodeCommand.getPosition().GetX(), 
	      // 	       nodeCommand.getPosition().GetY());
	    }
	}
#endif
    }
  else if( m_mobMode == MOBILITY_LCM_POP  )
    {
#ifdef USE_WP_LIST
      DEBUGNAV("updateTarget LCM_POP, pop? %d\n", pop_wp);
      if( pop_wp )
	{
	  CRadians oA;
	  CVector3 axis;
	  std::pair< bool, TimestampedWaypoint> c_wp =
	    m_wpControl->popWaypoint(m_myID);
	  if( c_wp.first )
	    {
	      CVector2 n_wp(c_wp.second.second.GetX(), c_wp.second.second.GetY());
	      Real n_ori = c_wp.second.second.GetYaw();
	      if( m_targetPosition != n_wp || m_targetOrientation != n_ori)    
		{
		  DEBUGNAV("NEW_WP (%f,%f):%.2f valid from %llu\n", 
			   n_wp.GetX(), n_wp.GetY(), n_ori, c_wp.second.first);
		  m_targetPosition.Set( n_wp.GetX(), n_wp.GetY());
		  m_targetOrientation = n_ori;
		}
	    } 
	  else
	    {
	      DEBUGNAV("STAY\n");
	      setState( STATE_ARRIVED_AT_TARGET );
	      /// stay in same pos
	      stay();
	    }
	}
#else
      ERRORNAV("LCM_POP not supported");
#endif
    }
  else if( m_mobMode == MOBILITY_NONE )
    {
      stay();
    }
  else if( m_mobMode == MOBILITY_SPIN )
    {
      spin();
    }
}


void
RVONavClient::spin()
{
  m_pcWheels->SetLinearVelocity(100 * m_spinVel, 
				-100 * m_spinVel);

}
void
RVONavClient::stay()
{
  CRadians oA;
  CVector3 axis;

  /// just stay in same pos
  if (lcmThread->getLcmHandler()->existNode(m_myID)) 
    {
      Node nodeCommand = 
	lcmThread->getLcmHandler()->getNodeById(m_myID);
      setTargetPosition(nodeCommand.getPosition()); 
    }
}

void
RVONavClient::checkObstacles()
{
  /// this counter is used to prevent the robot to oscillate while
  /// avoiding an obstacle
  if( m_obstacleAvoidanceCounter > 0 )
    {
      m_obstacleAvoidanceCounter--;
      return;
    }
    
  CVector2 pos = CVector2(mySelf.getPosition().GetX(), mySelf.getPosition().GetY());
    for (map<UInt32, Obstacle>::const_iterator it = m_extObstacles.begin(); 
       it != m_extObstacles.end(); it++) 
    {
      CVector2 opos((it->second).m_pos.GetX(), (it->second).m_pos.GetY());
      CVector2 va = pos-opos;
      CQuaternion ori = currentOrientation();
      CRadians oA;
      CVector3 axis;
      ori.ToAngleAxis(oA, axis);
      if (axis.GetZ() < 0)
	oA = -oA;
      CVector2 vb;
      vb.SetFromAngleAndLength(oA, 1.0);

      Real o_rad = (it->second).m_r;
      if( (pos - opos).Length() < o_rad )
	{
	  /// we are INSIDE one obstacle!!
	  /// check orientation
	  CRadians angle = ACos(va.DotProduct(vb)/ (va.Length()*vb.Length()));
	  fflush(stdout);
	  if (angle > CRadians::PI_OVER_TWO )
	    {
	      /// ok, we need to get out
	      CVector3 nexttpos(pos.GetX() + Cos(oA + CRadians::PI)*o_rad,
				pos.GetY() + Sin(oA + CRadians::PI)*o_rad,
				0);
	      setTargetPosition(nexttpos);
	      m_obstacleAvoidanceCounter=m_obstacleAvoidanceAllowance;
	      return;
	    }
	}
    }
}

void 
RVONavClient::update() 
{
  CRadians oA;
  CVector3 axis;
  m_Steps+=1;
#if LCM_CONFIG_ENABLED
  /// there was a nasty bug here, we should pop everything 
  for(;;)
    {
      std::pair< bool, TimestampedConfigMsg> c_msg =
	m_configHandler->popNextConfigMsg(m_myID, getTime());
      if( c_msg.first )
	{
	  processConfig(c_msg.second.msg);
	}
      else
	break;
    }
#endif

  /*  */
  if (lcmThread->getLcmHandler()->existNode(m_myID) &&
      (m_mobMode == MOBILITY_LCM || m_mobMode == MOBILITY_MANUAL || m_mobMode == MOBILITY_LCM_POP)) 
  {
    DEBUGNAV("lcmThread has my INFO (%d)\n", m_myID);
    updateTargetPosition(state() == STATE_INIT_READY );

#if __USE_DEBUG_NAV
    //    lcmThread->getLcmHandler()->printNodeListElements();
#endif

    /** LCM related Node information */
    //To get the other nodes locations through LCM
    m_nodes = lcmThread->getLcmHandler()->retrieveNodeList();

    mySelf = lcmThread->getLcmHandler()->getNodeById(m_myID);
    
    mySelf.getOrientation().ToAngleAxis(oA, axis);
    CQuaternion quat = mySelf.getOrientation();
    CRadians qz,qx,qy;
    quat.ToEulerAngles(qz,qy,qx);
    if (axis.GetZ() < 0)
      {
      oA = -oA;
    }

    // Position
    position = CVector2(mySelf.getPosition().GetX(), mySelf.getPosition().GetY());
    // Angle
    angle = oA;

    // Velocity
    //velocity = CVector2(speed, 0);
    velocity = CVector2(mySelf.getVelocity(), 0);

    /** NAVIGATION AND AVOIDING COLLISION */

    updateAgent(agent);
    agent->clearObstacles();
    addNodesAsObstacles(m_nodes);
    addExternalObstaclesToAgent();
    
    if ((m_targetPosition - position).Length() < m_targetMinPointDistance) 
      {
	if( m_doTargetOrientation )
	  {
	    Real oAd = ToDegrees(oA).GetValue();
	    if ( oAd < 0 )
	      oAd = 360.0 + oAd;
	    /// now, make them between -PI and PI
	    if( oAd > 180.0)
	      oAd = oAd - 360.0;
	    /// distance between angles
	    Real cdev = m_targetOrientation - oAd;
	    cdev += (cdev>180) ? -360 : (cdev<-180) ? 360 : 0;
	    if( fabs(cdev)  < 20.0 )
	      {
		setState( STATE_ARRIVED_AT_TARGET );
	      }
	    else
	      {
		setState( STATE_ORIENTING );
	      }
	
	  }	
	else
	  {
	    setState( STATE_ARRIVED_AT_TARGET );
	  }
	DEBUGNAV("State: STOPPED\n");
	if( m_mobMode == MOBILITY_LCM_POP)
	  {
	    updateTargetPosition(true);
	  }
      } 
    else 
      {
	DEBUGNAV("State: MOVING  -- cntTargetTime %ld currentTime %ld - from %f %f to %f %f\n",
		 m_cntTargetTime, getTime(),
		 position.GetX(), position.GetY(),
		 m_targetPosition.GetX(),
		 m_targetPosition.GetY());
	
	setState( STATE_MOVING );
	if( getTime() - m_cntTargetTime > m_wpTimeout )
	  {
	    m_targetTimeout = true;
	  }
	
	if( m_mobMode == MOBILITY_LCM_POP)
	  {
	    if ((m_targetPosition - position).Length() < m_targetMinPopDistance ||
		m_targetTimeout ) 
	      {
		updateTargetPosition(true);
	      }
	  }
	if( m_mobMode == MOBILITY_MANUAL )
	  {
	    /// this one checks if robot is inside an obstacle
	    // if it is, then another waypoint is selected to
	    // go out from the FIRST obstable found
	    checkObstacles();
	  }
      }
    if( m_started || m_enforcing )
      updateNavigation();
  }
}

CVector3
RVONavClient::currentPosition()
{
  return mySelf.getPosition();
}

CQuaternion
RVONavClient::currentOrientation()
{
  return mySelf.getOrientation();
}
bool
RVONavClient::validTarget(CVector2 pos)
{
  /// area
  if( originAreaCoord.GetX() + OBSTACLE_RADIUS  > pos.GetX() ||
      destinationAreaCoord.GetX() - OBSTACLE_RADIUS  < pos.GetX() ||
      originAreaCoord.GetY() + OBSTACLE_RADIUS > pos.GetY() ||
      destinationAreaCoord.GetY() - OBSTACLE_RADIUS  < pos.GetY())
    return false;
  /// obstacles
  for (map<UInt32, Obstacle>::const_iterator it = m_extObstacles.begin(); 
       it != m_extObstacles.end(); it++) 
    {
      CVector2 opos((it->second).m_pos.GetX(), (it->second).m_pos.GetY());
      if( (pos - opos).Length() < (it->second).m_r )
	return false;
    }
  return true;
}


void
RVONavClient::setTargetPosition(CVector3 pos)
{
  if( m_enforcing )
    {
      DEBUGNAV("SetTargetPosition (%f, %f) -REJECTED because enforcing\n", pos.GetX(), pos.GetY());
      return;
    }
  DEBUGNAV("SetTargetPosition (%f, %f)\n", pos.GetX(), pos.GetY());
  CVector2 tpos(pos.GetX(), pos.GetY());
  if( validTarget(tpos))
    m_targetPosition =  tpos;
  else
    {
      ERRORNAV("Invalid TargetPosition (%f, %f)\n", pos.GetX(),
	       pos.GetY());
    }
}

void
RVONavClient::setTargetOrientation(CQuaternion ori)
{
  CRadians oA;
  CVector3 axis;
  if( m_enforcing )
    {
      DEBUGNAV("SetTargetOrientation  -REJECTED because enforcing\n");
      return;
    }
    
  ori.ToAngleAxis(oA, axis);
  if (axis.GetZ() < 0)
    oA = -oA;

  m_targetOrientation = ToDegrees(oA).GetValue();
}
  void 
RVONavClient::initLocalNavigation(TConfigurationNode& t_node) 
{
  hlAgent.Init(t_node);
  hlAgent.axisLength = axisLength;
  orcaAgent.Init(t_node);
  orcaAgent.axisLength = axisLength;
  hrvoAgent.Init(t_node);
  hrvoAgent.axisLength = axisLength;


  localNavigationType = "HL";

  if (NodeExists(t_node, "local_navigation")) 
  {
    TConfigurationNode node = GetNode(t_node, "local_navigation");
    if (node.HasAttribute("type"))
      GetNodeAttribute(node, "type", localNavigationType);
    GetNodeAttributeOrDefault(node, "doTargetOrientation", 
			      m_doTargetOrientation, 
			      m_doTargetOrientation);
    GetNodeAttributeOrDefault(node, "targetOrientation", 
			      m_targetOrientation, 
			      m_targetOrientation);

  }

  if (localNavigationType == "HL") 
  {
    localNavigationIndex = 2;
    setAgent(hlAgent);
  } 
  else if (localNavigationType == "ORCA") 
  {
    localNavigationIndex = 0;
    setAgent(orcaAgent);
  } 
  else if (localNavigationType == "HRVO") 
  {
    localNavigationIndex = 0;
    setAgent(hrvoAgent);
  } 
  else 
  {
    throw "Navigation type not defined!";
    return;
  }
  /// retrieve the optimal speed of the agent
  m_agentSpeed =  agent->optimalSpeed;

}

  void 
RVONavClient::initOdometry() 
{
  position = CVector2(0, 0);
  angle = CRadians(0);
  velocity = CVector2(0, 0);
  axisLength=0.136;
  //  axisLength = encoderSensor->GetReading().WheelAxisLength * 0.01;
  printf("INIT axis length %.3f", axisLength);
  //DEBUGNAV("INIT axis length %.3f", axisLength);

}

  void 
RVONavClient::setAgent(Agent &a) 
{
  if (agent == &a)
    return;
  agent = &a;
}

  void 
RVONavClient::updateAgent(Agent *a) 
{
  a->position = position;
  a->velocity = velocity;
  a->angularSpeed = angularSpeed;
  a->angle = angle;
}

void
RVONavClient::addExternalObstaclesToAgent()
{
  CVector2 auxPosition;
  for (map<UInt32, Obstacle>::const_iterator it = m_extObstacles.begin(); 
       it != m_extObstacles.end(); it++) 
    {

      auxPosition.Set((it->second).m_pos.GetX(), 
		      (it->second).m_pos.GetY());
      
      // For a dynamic obstacle we need to use the addObstacleAtPoint(position,velocity,radius)
      agent->addObstacleAtPoint(auxPosition, (it->second).m_r);
    }
}
  void 
RVONavClient::addNodesAsObstacles(const map<UInt8, Node> &listNodeObstacles) 
{
  /// Aux variables
  CVector2 auxPosition;
  CVector2 auxVelocity;

  /// Create a list of Nodes
  for (map<UInt8, Node>::const_iterator it = listNodeObstacles.begin(); 
       it != listNodeObstacles.end(); it++) 
  {
    // I'm not and obstacle for myself!
    if ((it->second).getId() != mySelf.getId()) 
    {

      //printf("Agent \n");
      auxPosition.Set((it->second).getPosition().GetX(), 
		      (it->second).getPosition().GetY());
      auxVelocity.Set((it->second).getVelocity(), 0);

      // For a dynamic obstacle we need to use the addObstacleAtPoint(position,velocity,radius)
      //agent->addObstacleAtPoint(auxPosition, OBSTACLE_RADIUS);
      agent->addObstacleAtPoint(auxPosition, auxVelocity, OBSTACLE_RADIUS);

      //DEBUGNAV("Adding obstacle : %d\n", (it->second).getId());
    }
  }
  /// Init fixed obstacles
  setFixedObstacles(addAreaBounds(originAreaCoord, destinationAreaCoord));
}

std::vector<CVector2> 
RVONavClient::addAreaBounds(CVector2 bottomLeftCoord, CVector2 upperRightCoord) 
{

  //All obstacle points
  std::vector<CVector2> obstaclePoints;

  CVector2 upperLeftCoord;
  upperLeftCoord.Set(bottomLeftCoord.GetX(), upperRightCoord.GetY());

  CVector2 bottomRightCoord;
  bottomRightCoord.Set(upperRightCoord.GetX(), bottomLeftCoord.GetY());

  //Distance from bottom left to upper left.
  CVector2 d1;
  d1 = upperLeftCoord - bottomLeftCoord;

  double numObsD1;
  modf(d1.Length() / (2 * OBSTACLE_RADIUS), &numObsD1);
  double interDistanceD1 = d1.Length() / numObsD1;

  CVector2 pointAux;
  pointAux.SetX(bottomLeftCoord.GetX());
  for (int i = 0; i < numObsD1; i++) {
    pointAux.SetY(bottomLeftCoord.GetY() + i * interDistanceD1);

    //		printf("D1, P(%d) = (%f,%f)\n", i, pointAux.GetX(), pointAux.GetY());
    obstaclePoints.push_back(pointAux);
  }

  //Distance from upper left to upper right
  CVector2 d2;
  d2 = upperLeftCoord - upperRightCoord;
  //printf("D2 %f\n", d2.Length());

  double numObsD2;
  modf(d2.Length() / (2 * OBSTACLE_RADIUS), &numObsD2);
  double interDistanceD2 = d2.Length() / numObsD2;

  pointAux.SetY(upperLeftCoord.GetY());
  for (int i = 0; i < numObsD2; i++) {
    pointAux.SetX(upperLeftCoord.GetX() + i * interDistanceD2);

    //printf("D2, P(%d) = (%f,%f)\n", i, pointAux.GetX(), pointAux.GetY());
    obstaclePoints.push_back(pointAux);
  }

  //Distance from upper right to bottom right
  CVector2 d3;
  d3 = upperRightCoord - bottomRightCoord;
  //printf("D3 %f\n", d3.Length());

  double numObsD3;
  modf(d3.Length() / (2 * OBSTACLE_RADIUS), &numObsD3);
  double interDistanceD3 = d3.Length() / numObsD3;

  pointAux.SetX(upperRightCoord.GetX());
  for (int i = 0; i < numObsD3; i++) {
    pointAux.SetY(upperRightCoord.GetY() - i * interDistanceD3);

    //printf("D3, P(%d) = (%f,%f)\n", i, pointAux.GetX(), pointAux.GetY());
    obstaclePoints.push_back(pointAux);
  }

  //Distance from bottom right to bottom left
  CVector2 d4;
  d4 = bottomRightCoord - bottomLeftCoord;
  //printf("D4 %f\n", d4.Length());

  double numObsD4;
  modf(d4.Length() / (2 * OBSTACLE_RADIUS), &numObsD4);
  double interDistanceD4 = d4.Length() / numObsD4;

  pointAux.SetY(bottomRightCoord.GetY());
  for (int i = 0; i < numObsD4; i++) {
    pointAux.SetX(bottomRightCoord.GetX() - i * interDistanceD4);

    //printf("D4, P(%d) = (%f,%f)\n", i, pointAux.GetX(), pointAux.GetY());
    obstaclePoints.push_back(pointAux);
  }

  return obstaclePoints;

}

  void 
RVONavClient::setFixedObstacles(std::vector<CVector2> obstaclesPoints) 
{
  std::vector<CVector2>::iterator it;
  for (unsigned int i = 0; i < obstaclesPoints.size(); i++) 
  { 
    CVector2 points = obstaclesPoints[i];
    //printf("[%d] Obstacle points / area bounds, %f, %f\n", m_myID,points.GetX(), points.GetY());
    agent->addObstacleAtPoint(obstaclesPoints[i], OBSTACLE_RADIUS);
  }
}

  void 
RVONavClient::updateNavigation() 
{
  if( m_state == STATE_ORIENTING )
  {
    CVector3 axis;
    CRadians oA;
    mySelf.getOrientation().ToAngleAxis(oA, axis);
    if (axis.GetZ() < 0)
      oA = -oA;
    DEBUGNAV("STATE_ORIENTING: Current Orientation %f\n", ToDegrees(oA).GetValue());
    Real oAd = ToDegrees(oA).GetValue();
    if ( oAd < 0 )
      oAd = 360.0 + oAd;
    /// now, make them between -PI and PI
    if( oAd > 180.0)
      oAd = oAd - 360.0;
    /// distance between angles
    Real cdev = m_targetOrientation - oAd;
    cdev += (cdev>180) ? -360 : (cdev<-180) ? 360 : 0;

    agent->desideredAngle = ToRadians(CDegrees(cdev));

    agent->desideredSpeed = 0;
    agent->desideredVelocity = CVector2(0, 0);

  }
  else  if (m_state == STATE_ARRIVED_AT_TARGET) 
  {
    DEBUGNAV("STATE_ARRIVED - stopping\n");
    agent->desideredAngle = CRadians::ZERO;
    agent->desideredSpeed = 0;
    agent->desideredVelocity = CVector2(0, 0);
  } 
  else 
  {
    updateDesideredVelocity();
  }
  agent->updateVelocity();
  updateVelocity();
}

  void 
RVONavClient::updateDesideredVelocity() 
{
  agent->targetPosition = m_targetPosition;
  agent->updateDesideredVelocity();
}

  void 
RVONavClient::updateVelocity() 
{
  m_pcWheels->SetLinearVelocity(
    100 * agent->leftWheelTargetSpeed / ODOMETRY_CORRECTION_FACTOR_LEFT, 
    100 * agent->rightWheelTargetSpeed / ODOMETRY_CORRECTION_FACTOR_RIGHT);
}



std::string
RVONavClient::getTimeStr()
{
#ifndef FOOTBOT_SIM
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

void
RVONavClient::setTime(UInt64 ctime)
{
  m_currentTime = ctime;
}

/// returns time in milliseconds
  UInt64 
RVONavClient::getTime()
{
#ifndef FOOTBOT_SIM
  struct timeval timestamp;
  gettimeofday(&timestamp, NULL);

  UInt64 ms1 = (UInt64) timestamp.tv_sec;
  ms1*=1000;

  UInt64 ms2 = (UInt64) timestamp.tv_usec;
  ms2/=1000;

  return (ms1+ms2);
#else
  return m_currentTime;
#endif
}



