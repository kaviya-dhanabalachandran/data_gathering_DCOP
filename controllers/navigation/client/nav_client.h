#ifndef _RVONAVCLIENT_H_
#define _RVONAVCLIENT_H_

#include <iostream>
#include <fstream>

#include <argos2/common/control_interface/ci_controller.h>
#include <argos2/common/utility/logging/argos_log.h>
#include <argos2/common/utility/argos_random.h>
#include <argos2/common/utility/datatypes/color.h>

#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>
#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_beacon_actuator.h>

#include <argos2/common/control_interface/swarmanoid/ci_range_and_bearing_sensor.h>

/* Definition of the foot-bot proximity sensor */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_proximity_sensor.h>

#include <argos2/common/utility/argos_random.h>


#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <termios.h>
#include <math.h>

/** LCM engine */
#include <navigation/lcm/lcmthread.h>
#include <navigation/beans/node.h>
/** NAVIGATION AND AVOIDING COLLISION */
/* Navigations agents */
#include <navigation/agent/Agent.h>
#include <navigation/agent/ORCAAgent.h>
#include <navigation/agent/HRVOAgent.h>
#include <navigation/agent/HLAgent.h>
///Additional sensors
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_encoder_sensor.h>
#include <argos2/common/control_interface/ci_wifi_sensor.h>
#include <argos2/common/control_interface/ci_wifi_actuator.h>

/* Obstacle radius */
#define OBSTACLE_RADIUS 0.12
#define ODOMETRY_CORRECTION_FACTOR_RIGHT 1
#define ODOMETRY_CORRECTION_FACTOR_LEFT 1


using namespace argos;
using namespace std;

#include <argos2/common/utility/datatypes/datatypes.h>

/// use waypoint lists instead of single waypoint for control
#define USE_WP_LIST
#ifdef USE_WP_LIST
#include <navigation/lcm/timestamped_waypoint_list_handler.h>
#endif

#define NODE_VALIDITY_THRESHOLD 1000
  //< State of the robot
typedef 
    enum RobotNavState 
  {
    STATE_INIT,
    STATE_INIT_READY,
    STATE_MOVING, 
    STATE_ARRIVED_AT_TARGET, 
    STATE_ORIENTING,
  } RobotNavState;

class RVONavClient
{
 private:

  typedef 
    enum LedsControlMode
  {
    LEDS_NONE,
    LEDS_MOBILITY,
    LEDS_FIXED,
  } LedsControlMode;

  typedef 
    enum BeaconControlMode
  {
    BEACON_NONE,
    BEACON_MOBILITY,
    BEACON_MOBILITY_LETTER_DEMO,
    BEACON_FIXED,
  } BeaconControlMode;

  typedef
    enum MobilityControlMode
  {
    MOBILITY_NONE,
    MOBILITY_LCM,
    MOBILITY_LCM_POP,
    MOBILITY_MANUAL,
    MOBILITY_SPIN,
  } MobilityControlMode;

  typedef
    enum SpeedControlMode
  {
    SPEED_FIXED,
    SPEED_RANDOM_UNIFORM,
  } SpeedControlMode;
  

  struct Obstacle {
    UInt32 m_id;
    CVector3 m_pos;
    Real m_r;
    Obstacle()
    {
      m_r=0;
      m_id=-1;
      m_pos=CVector3(0,0,0);
    }
      
  Obstacle(UInt32 id, CVector3 pos, Real r):
    m_id(id), m_pos(pos), m_r(r){}

  };
  std::map<UInt32, Obstacle> m_extObstacles;
  
  UInt8 m_myID;
  CCI_Robot* m_robot;
  CARGoSRandom::CRNG* m_randomGen;
  UInt32 m_randomSeed;
  std::string m_MyIdStr;
  UInt64 m_Steps;
  bool m_started;

  /// important flag - if true - ignored any attempt ton control until arriving to
  /// enforced position
  bool m_enforcing;

  RobotNavState m_state;
  LedsControlMode m_ledMode;
  BeaconControlMode m_beaconMode;
  MobilityControlMode m_mobMode;
  SpeedControlMode m_speedMode;
  Real m_speedRandomLow, m_speedRandomHigh;
  CColor m_ledFixedColor;
  CColor m_beaconFixedColor;

  UInt64 m_currentTime;
  

  //! Actuators:
  CCI_FootBotWheelsActuator* m_pcWheels;
  CCI_FootBotLedsActuator *m_ledsActuator;
  CCI_FootBotBeaconActuator* m_beaconActuator;

  // Internal functionality:
  //  RobotAddressType RobotIdToAddress(const string& id);
  //  RobotAddressType RobotIdToAddress(const char *id);


  Real m_enforcingSpeed;
  Real m_agentSpeed;
  bool m_targetTimeout;
  UInt64 m_cntTargetTime;
  UInt64 m_wpTimeout;
  UInt32 m_obstacleAvoidanceCounter;
  UInt32 m_obstacleAvoidanceAllowance;
  void setLeds();
  void setBeacon();


  /** LCM engine */
  // LCM thread
  static bool m_lcm_isset;
  static LCMThread *lcmThread;

  /// make it static because in simulation robots can share the same config handler
  //TODO: enable online configuration
  //  static TimestampedConfigMsgHandler *m_configHandler;

#ifndef USE_WP_LIST
  // LCM thread command channel
  static LCMThread *lcmThreadCommand;
#else
  static TimestampedWaypointListHandler *m_wpControl;
#endif
  /// List of node obstacles retrieved by LCM-tracking system. The
  /// other nodes are an obstacles for me.
  map<UInt8, Node> m_nodes;
  
  // Myself Node
  Node mySelf;

  /// NAVIGATION AND AVOIDING COLLISION 
  /// Additional sensors 
  CCI_FootBotEncoderSensor* encoderSensor;

  /// Navigation agents 
  Agent * agent;
  HLAgent hlAgent;
  ORCAAgent orcaAgent;
  HRVOAgent hrvoAgent;

  /// Local navigation
  std::string localNavigationType;
  int localNavigationIndex;

  /// Mobility parameters 
  Real axisLength;
  CVector2 position;
  CVector2 velocity;
  CRadians angle;
  CRadians angularSpeed;
  CVector2 deltaPosition;
  Real speed;
  Real m_targetMinPointDistance;
  Real m_targetMinPopDistance;
  Real m_targetOrientation; //! in degrees
  bool m_doTargetOrientation;

  /* Target -> next (x,y) point */
  CVector2 m_targetPosition; // in METERS

  // Area bounds as fixed obstacles
  CVector2 originAreaCoord;
  CVector2 destinationAreaCoord;

  Real originAreaX;
  Real originAreaY;
  Real destinationAreaX;
  Real destinationAreaY;

  /* Maximum tolerance for the angle between
   * the robot heading direction and
   * the closest obstacle detected. */
  CDegrees m_cAlpha;
  /* Maximum tolerance for the proximity reading between
   * the robot and the closest obstacle.
   * The proximity reading is 0 when nothing is detected
   * and grows exponentially to 1 when the obstacle is
   * touching the robot.
   */
  Real m_fDelta;
  /* Wheel speed. */
  Real m_fWheelVelocity;
  Real m_spinVel;
  /* Angle tolerance range to go straight.
   * It is set to [-alpha,alpha]. */
  CRange<CRadians> m_cGoStraightAngleRange;

  void checkObstacles();
 public:

  /* Class constructor. */
  RVONavClient(UInt8 robot_id, CCI_Robot &);

  /* Class destructor. */
  virtual ~RVONavClient() {
  }

  void spin();
  void stay();

  void processConfig(std::string);
  UInt8 robotId();
  /// state 
  void setState(RobotNavState s);
  RobotNavState state();
  void setOptimalSpeed(Real speed);
  void setAgentSpeed(Real speed);


  void start();
  void stop();
  /** NAVIGATION AND AVOIDING COLLISION */
  /* Navigation agents */
  void setAgent(Agent &a);
  void updateAgent(Agent *a);

  /* Local navigation */
  void initLocalNavigation(TConfigurationNode& t_tree);
  void initGlobalNavigation(TConfigurationNode& t_tree);
  void initOdometry();
#ifndef FOOTBOT_SIM
    static UInt64 getTime();
#else
    UInt64 getTime();
#endif

  static std::string getTimeStr();

  static bool isNodeValid( const Node &n );
  /* Obstacles */
  void addNodesAsObstacles(const map<UInt8, Node> &listNodeObstacles);
  void addExternalObstaclesToAgent();
  std::vector<CVector2> addAreaBounds(CVector2 bottomLeftCoord, CVector2 upperRightCoord);
  void setFixedObstacles(std::vector<CVector2> obstaclesPoints);

  const map<UInt8, Node> &nodes();
  /* Update navigation */
  void updateNavigation();
  void updateDesideredVelocity();
  void updateVelocity();
  void updateTargetPosition(bool);


  void init(TConfigurationNode& t_tree);

  void addExternalCircularObstacle(UInt32 id, CVector3 pos, Real rad);
  void clearExternalObstacles();
    
  void update();
  void destroy();

  CVector3 currentPosition();
  CQuaternion currentOrientation();

  bool hasValidInfo( UInt8 nid);

  void setTargetPosition(CVector3);
  bool validTarget(CVector2);
  void setTargetOrientation(CQuaternion);
  
  void setTime(UInt64);

  static UInt8 ConvertValueToByte(Real value);
  static Real ConvertByteToValue(UInt8 byte);
};

#endif
