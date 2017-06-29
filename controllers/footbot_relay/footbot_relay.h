#ifndef _FOOTBOTRELAY_H_
#define _FOOTBOTRELAY_H_

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
#include <string>
#include <termios.h>
#include <math.h>
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_encoder_sensor.h>
#include <navigation/client/nav_client.h>
#include <map>
#include <set>
#include <vector>
#include <queue>
#include <sstream>
#include <include/constants.hpp>
#include <iterator>   
#include <random>
#include <deque>
#include <exception>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/StdVector>
#include <unordered_set>

#include "probabilityCalculation.h"
#include "utils.h"


#define MAX_UDP_SOCKET_BUFFER_SIZE 1500


using namespace argos;
using namespace std;



#include <argos2/common/utility/datatypes/datatypes.h>

class FootbotRelay: public CCI_Controller
{

 public:
    
    typedef Util::Position Position;

    
    Util util;
    
    struct dataWrite
    {
      ofstream data_file;
      string filename;
    };

  


    struct SAgentData
    {
        /*** Profile message details ***/
        Position current_location;
        Position goal_location;
        vector<Util::Position> goalLocationList;

        uint64_t time_last_visited;  // time when profile message is sent
        uint64_t time_last_data_collected; // this is from agent side
        uint64_t data_available; // size of data to be transmitted
        uint8_t id;
       //uint8_t number_of_neighbours;

        /*** Details of data transmitted ***/
        uint64_t transmitted_data_time; // this is what the relay stores when data is gathered
        uint32_t transmitted_data_size;

        // Time between data collected and 
        uint64_t delay_time;

        bool IsGoalSet;
        bool IsAgentStatusAvailable;
        bool IsDataReceived;
        
        uint64_t delay_in_collection;
        
        float weight;

        SAgentData();        
    };
    
   
    struct SStateData
    {   

      vector <uint8_t> parents;
      vector <uint8_t> children;

      set<uint8_t> ParentMessages;
      set<uint8_t> ChildrenMessages;
      set<uint8_t> RelaysReachedBaseStation;

      uint8_t recentlyVisitedAgentId;
      uint8_t recentlyProfileDataCollectedId;

      enum EState {
         STATE_NOGOAL = 0,
         STATE_SEARCH,  // this is the state when relay knows which agent to go to
         STATE_DATA_GATHERING, // when agent is detected
         STATE_RETURN_TO_BASESTATION, // moving to base station
         STATE_HOME
        } State;

      enum RState {
        STATE_INITIAL = 0,
        STATE_ASSIGN,
        STATE_DECIDE
      } RoundState;

      /**** received data identifier 0 - agent profile data, 1- agent collected data, 2- relay profile data ****/
      enum ReceivedDataType {
         AGENT_PROFILE_DATA = 0,
         AGENT_COLLECTED_DATA,
         RELAY_TASK_ASSIGNED_DATA,
         RELAY_NEW_DATA,
         RELAY_TRANSMITTED_DATA_TIME,
         RELAY_MOVING_TO_BASESTATION,
         RELAY_ROUND_TASK_ASSIGNED_DATA,
         RELAY_ROUND_COST_DATA,
         RELAY_START_ROUND,
         RELAY_STOP_ROUND
      } ReceivedData;

      enum SentDataType {
        RELAY_HELLO_TO_AGENT = 0,
        RELAY_SENDING_ACCEPTANCE_TO_AGENT,
        TASK_ASSIGNED,
        NEW_DATA,
        TRANSMITTED_DATA_TIME,
        MOVING_TO_BASESTATION,
        ROUND_TASK_ASSIGNED_TO_CHILDREN,
        ROUND_COST_TO_PARENTS,
        START_ROUND,
        STOP_ROUND,
        RELAY_TO_BASESTATION
      } SentData;

      //Position base_station;
      uint8_t NUMBER_OF_BASESTATION;
      map<int,Position> base_station;
      bool IsDataSentToBaseStation;
      bool MovingToBaseStation;

      uint8_t NUMBER_OF_RELAY;
      uint8_t NUMBER_OF_AGENT;
      
      bool IsGoalSet;
      bool IsAgentDetected;
      bool IstargetAgentSet; // Setting a target agent Id 
      bool visited_baseStation; // This is to check whether task is assigned initially or later

      uint8_t targetAgentId;
      vector<uint8_t> detectedAgentId;
      bool agentfound;
      vector<uint8_t> InRange; /// Id of the agents in range to be used in Data Gather State

      //time limit to come back to BS
      uint64_t time_limit;
      uint64_t time_for_each_agent;
      uint64_t min_data_size;
      uint64_t collected_data_size;

      
      // contains the time last visited as key and agent id as value
      map<uint64_t, uint8_t> timeMap; 

      
      SStateData();
      void Init(TConfigurationNode& t_node, int);
    };



private:
     
    //contains key: agent id, value: the distance to largest probability value of agents from relay's current position
    map<uint8_t, float> distance;
    
    UInt8 m_myID;
    UInt64 m_Steps;
    uint8_t timeInSearch;
    
    //agent Map containing details of agent id -> key , structure as value
    map<uint8_t, std::shared_ptr<SAgentData> > agentMap;

    set<uint8_t> visitedAgent;

    UInt32 RandomSeed;
    std::string m_MyIdStr;
    
    uint64_t search_time; 
    

    CARGoSRandom::CRNG* m_randomGen;
    //UInt64 m_sendPackets;
    Position startPos;
    
    float size_x;
    float size_y;
    float speed;

    RVONavClient *m_navClient;
    RobotNavState target_state;

    CCI_WiFiSensor* m_pcWifiSensor;
    CCI_WiFiActuator* m_pcWifiActuator;

    CCI_WiFiSensor* m_pcWifiSensorLongRange;
    CCI_WiFiActuator* m_pcWifiActuatorLongRange;
    
    CCI_FootBotLedsActuator* m_pcLEDs;
    
    SAgentData agentData;
    SStateData stateData;
    
    
    CVector3 getSpiralTargetPos(double x, double y);

    

    // Structure with agent details in Probability class
    typedef Prob::SAgentInfo AgentDetails;
    
    Prob prob;
    
    // creating a shared pointer for map with key - Agent Id , value - agent details 
    // filled in probability class
    using map_dataType = std::map <uint8_t,std::shared_ptr<AgentDetails> >;
    std::shared_ptr< std::map <uint8_t,std::shared_ptr<AgentDetails> > > PagentProbabilityMap = std::make_shared<map_dataType>();

    

    uint8_t counter = 0;

    Util::Position spiralPos;
    bool spiralInitial;
    int numberOfFutureTargetPositionsKnown;
    
    bool taskAssigned;
    bool taskDone;
    bool onlyIamFree;
    int numberOfRounds;
    map<uint8_t,float> costMap;
    int currentRoundNumber;
    bool initialiseRound;
    float distanceToBaseStation;
    uint8_t timeInDataGather;
    bool stopR;
    set<uint8_t> roundStopId;
    int counterInitialise = 0;
    vector<uint8_t> roundVector;
    set<uint8_t> idsChosenbyParents;
    bool initialRound;
    
    float localCost;
    float globalCost;
    uint8_t agent_considered;
    set<uint8_t> freeRelayIds;

    vector<uint8_t> receivedMessageFromRelay;
    std::vector<uint8_t> receivedMessageFromRelayVector;

    uint8_t idToSendData;
    Position base_loc;
    map<uint8_t,uint8_t> tempRelayMap;
    
public:

    /* Class constructor. */
    FootbotRelay();

    /* Class destructor. */
    virtual ~FootbotRelay() {
    }

    virtual void Init(TConfigurationNode& t_tree);

    void initialiseAgentData(TConfigurationNode& t_node, int NumberOfAgents, int goalN);

    virtual void ControlStep();
    virtual void Destroy();
    virtual bool IsControllerFinished() const;

    uint64_t getTimeLimit(float,float);
    bool UpdateState();
    void findDistanceToAgents();

    static std::string getTimeStr();
    UInt64 getTime();
    
    void SendMessage(uint8_t,uint8_t);
    void ParseMessage(std::vector<char> &v,uint8_t id, uint8_t sender);
    void Rounds(uint8_t state);

    void SetGoal();
    void Search(bool);
    void DataGather(uint8_t agentId);
    void Return();
    void Home();

    void SetState();
     /*** communication ***/

    // To Agents
    size_t HelloToAgent(uint8_t id,char* out_to_agent);
    size_t AcceptanceToAgent(uint8_t id,char* out_to_agent);
    
    //To BS
    size_t ToBaseStation(uint8_t id,char* out_to_agent);
    
    // To other relays
    size_t TaskAssignedToRelay(uint8_t identifier, char* out_to_relay);
    size_t NewDataInformationToRelay(uint8_t identifier, char* out_to_relay);
    size_t SendingTransmittedDataTimeToRelay(uint8_t identifier, char* out_to_relay);
    size_t AnnouncingTaskcompletion(uint8_t identifier, char* out_to_relay);
    size_t RoundStop(uint8_t identifier, char* out_to_relay);
    size_t RoundStart(uint8_t identifier, char* out_to_relay);

    //Round data exchange
    size_t RoundTaskAssignedToRelayChildren(uint8_t identifier, char* out_to_relay);
    size_t RoundCostToRelayParents(uint8_t identifier, char* out_to_relay);

    void ParseAgentProfile(vector<char> &incoming_agent_message);
    void ParseAgentCollectedData(vector<char> &incoming_agent_message);

    void ParseRelayTaskInfo(vector<char> &incoming_agent_message);
    void ParseRelayNewInformation(vector<char> &incoming_agent_message);
    void ParseRelayTransmittedDataTime(vector<char> &incoming_agent_message);
    void ParseRelayFinalData(vector<char> &incoming_agent_message);

    //Round data parse
    void ParseRoundTaskData(vector<char> &incoming_agent_message);
    void ParseRoundCostData(vector<char> &incoming_agent_message);
    void ParseRoundStopData(vector<char> &incoming_agent_message);
    void ParseRoundStartData(vector<char> &incoming_agent_message);

    void reinitiliseRoundVariables();
    uint8_t findNearestBaseStation();
    
    dataWrite relayPositions;
    dataWrite timeStepToMeet;
    dataWrite delayTime;
    dataWrite timeForACycle;

    map<uint8_t,uint8_t> agentIdMap; // contains numbers from 0 as key and ids as value
    map<uint8_t,uint8_t> relayIdMap; // contains id as key and corresponding n as value Eg : map <2,0>,<3,1>
   
    std::vector<uint8_t> agentIdList;
    std::vector<uint8_t> agentIdToConsider;

    std::vector<uint8_t> roundAgentIdToConsider;
    std::vector<uint8_t> relayIdList;

    uint64_t datasizeToBaseStation;

    void announce();
    void stopRoundAndReinitialise();

    std::tuple<uint8_t, float> assignAgent(vector <uint8_t> agentIds);


  private:
    float checkBounds(float,float);
    float checkLowerBounds(float);
    float checkUpperBounds(float,float);
    

};

#endif


//v.erase( std::remove( v.begin(), v.end(), 5 ), v.end() ); 