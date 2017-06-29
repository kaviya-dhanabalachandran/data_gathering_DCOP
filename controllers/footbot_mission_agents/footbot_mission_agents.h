#ifndef _FOOTBOTMISAGENT_H_
#define _FOOTBOTMISAGENT_H_

#include <iostream>
#include <fstream>
#include <argos2/common/control_interface/ci_controller.h>
#include <argos2/common/utility/logging/argos_log.h>
#include <argos2/common/utility/argos_random.h>
#include <argos2/common/utility/datatypes/color.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_beacon_actuator.h>
#include <argos2/common/utility/argos_random.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <math.h>
#include <unistd.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_encoder_sensor.h>
#include <argos2/common/utility/datatypes/datatypes.h>
#include <navigation/client/nav_client.h>
#include <fstream>
#include <deque>


#define PI 3.14159265

using namespace argos;
using namespace std;


#define MAX_UDP_SOCKET_BUFFER_SIZE 1500



class FootbotMissionAgents: public CCI_Controller
{
 
 public:
    
    struct Position
    {
        float x;
        float y;
    };

    struct dataWrite
    {
      ofstream data_file;
      string filename;
    };
    // Agent can send neighboring agent's pos and time met

    struct SStateData {
      /* possible states in which the controller can be */
      enum EState {
         STATE_RESTING = 0,
         STATE_EXPLORING
      } State;

      enum ReceivedDataType {
         RELAY_HELLO_MESSAGE = 0,
         RELAY_ACCEPTANCE_FOR_DATA
         //AGENT_DATA
      } ReceivedData;

      enum SentDataType {
        PROFILE_DATA = 0,
        COLLECTED_DATA
        // AGENT_TO_AGENT
      } SentData;

      SStateData();

      RobotNavState target_state;
      int wait_time;
      Position goal_loc;
      Position current_loc;
      
      uint8_t id;
      uint64_t size_of_data_available;
      uint32_t ttl;
      map<uint64_t, uint64_t> data_generatedTimeSizeMap;
      vector<uint64_t> size_of_data_sent;
      uint32_t discarded_data_count;
     

      std::vector< std::vector<int> > dataGenerationTime;
      std::vector< std::vector<float> > randomPos; 
    };

    struct SRelayData{
      uint8_t id;
      uint64_t time_profile_data_sent;
      uint64_t time_gathered_data_sent;
      SRelayData();
    };


 private:
    UInt32 RandomSeed;
    std::string m_MyIdStr;
    UInt64 m_Steps;
    CARGoSRandom::CRNG* m_randomGen;
    UInt64 m_sendPackets;
    UInt8 m_myID;
    float speed;
    RVONavClient *m_navClient;

    CCI_WiFiSensor* m_pcWifiSensor;
    CCI_WiFiActuator* m_pcWifiActuator;

    CCI_FootBotLedsActuator* m_pcLEDs;
    
    /* The controller state information */
    SStateData stateData;
    SRelayData relayData;
    map<uint8_t, SRelayData> relayMap;

    int numberOfFutureTargetsToBeSent;
    vector<Position> targetPositions;

  public:

    /* Class constructor. */
    FootbotMissionAgents();

    /* Class destructor. */
    virtual ~FootbotMissionAgents() {
    }

    virtual void Init(TConfigurationNode& t_tree);

    virtual void ControlStep();
    virtual void Destroy();
    virtual bool IsControllerFinished() const;
    
    void updateState();
    void Rest();
    void Explore();

    CVector3 randomWaypoint();

    static std::string getTimeStr();
    UInt64 getTime();
    uint32_t getTimeLimit(float,float);

    void initialiseAgentData(TConfigurationNode& t_node, int numberOfAgents, int goalN);
    
    void generateData();

    void SendData(uint8_t type_of_message, uint8_t relay_id);
    void ParseMessage(uint8_t id, std::vector<char> &v);

    void ParseRelayMessage(vector<char> &incoming_agent_message);
    void ParseRelayAcceptance(vector<char> &incoming_agent_message);
    
    // This is used for caculating fairness in meeting agents and for how many agents were met by relay in 'X' timesteps
    dataWrite timeStepsWhenRelayMet;
    dataWrite agentPositions;

    //dataWrite dataInformation;
    // ParseNeighborData
    size_t SendProfileData(char* ptr, uint8_t id, uint8_t relay_id);
    size_t SendCollectedData(char* ptr, uint8_t id, uint8_t relay_id);
};
#endif