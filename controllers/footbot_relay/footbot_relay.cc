#include "footbot_relay.h"


#ifndef FOOTBOT_SIM
#else
#include <argos2/simulator/physics_engines/physics_engine.h>
#include <argos2/simulator/simulator.h>
#endif


#define MAX_UDP_SOCKET_BUFFER_SIZE 1500

#define __USE_DEBUG_COMM 1
#if __USE_DEBUG_COMM
#define DEBUGCOMM(m, ...) \
{\
  fprintf(stderr, "%.2f DEBUGCOMM[%d]: " m,\
    (float) m_Steps,\
    (int) m_myID, \
          ## __VA_ARGS__);\
  fflush(stderr);\
}
#else
#define DEBUGCOMM(m, ...)
#endif


FootbotRelay::FootbotRelay():
  RandomSeed(12345),
  m_Steps(0),
  target_state(STATE_ARRIVED_AT_TARGET),
  search_time(0),
  spiralInitial(true),
  taskAssigned(false),
  taskDone(false),
  globalCost(5000),
  initialiseRound(true),
  timeInDataGather(0),
  stopR(false),
  initialRound(true),
  timeInSearch(0)
  {}



FootbotRelay::SStateData::SStateData()
{
  IsGoalSet = false;
  IstargetAgentSet = false;
  IsAgentDetected = false; 
  collected_data_size = 0;
  State = SStateData::STATE_NOGOAL;
  MovingToBaseStation = false;
  visited_baseStation = false;
  agentfound  = false;
  IsDataSentToBaseStation = false; // Data sent to base station initially false
  /****Calculate time limit and min data size ****/

}


FootbotRelay::SAgentData::SAgentData()
{
  IsGoalSet = true;
  IsDataReceived = false;
  delay_time = 0;
  time_last_visited = 0;
  time_last_data_collected = 0;
  transmitted_data_time = 0;
  delay_in_collection = 0;
}

void
FootbotRelay::SStateData::Init(TConfigurationNode& t_node, int rId)
{
  
  printf(" number of base station  %d \n", NUMBER_OF_BASESTATION);
  Position loc; 
  
  //NUMBER_OF_BASESTATION ;
  for(int i=0; i< NUMBER_OF_BASESTATION; i++)
  { 
    
    ostringstream temp_str; 

    temp_str << "base_station" <<  i+1;
  
    GetNodeAttribute(GetNode(t_node, temp_str.str()), "x", loc.x);
    //cout << loc.x << endl;
    GetNodeAttribute(GetNode(t_node, temp_str.str()), "y", loc.y);
    //cout << loc.y << endl;
    Position tempB;
  
    if(loc.x + 0.5 > 24)
    {
      tempB.x = loc.x -0.4 -(rId*0.15);
      tempB.y = loc.y -0.4 -(rId*0.15);
    }
    else
    {
      tempB.x = loc.x +0.4 +(rId*0.15);
      tempB.y = loc.y +0.4 +(rId*0.15);
    }
    base_station.emplace(i+1,tempB);  
    
  }

}




void 
FootbotRelay::Init(TConfigurationNode& t_node) 
{ 
  

  /// The first thing to do, set my ID
#ifdef FOOTBOT_SIM
  m_myID = 
    atoi(GetRobot().GetRobotId().substr(3).c_str());
#else
  m_myID = 
    atoi(GetRobot().GetRobotId().substr(7).c_str());
#endif
  
  printf("MyID %d\n", m_myID);

  /// Random
  GetNodeAttributeOrDefault(t_node, "RandomSeed", RandomSeed, RandomSeed);

  string temp;
  GetNodeText(GetNode(t_node, "destinationAreaX"), temp);
  size_x = atof(temp.c_str());

  GetNodeText(GetNode(t_node, "destinationAreaY"),temp);
  size_y = atof(temp.c_str());

  GetNodeText(GetNode(t_node, "numberofrelay"), temp);
  stateData.NUMBER_OF_RELAY = atoi(temp.c_str());

  GetNodeText(GetNode(t_node, "numberofBS"), temp);
  stateData.NUMBER_OF_BASESTATION = atoi(temp.c_str());

  printf("Number of base_station: %d \n ", stateData.NUMBER_OF_BASESTATION);
  
  GetNodeText(GetNode(t_node, "numberofAgent"), temp);
  stateData.NUMBER_OF_AGENT = atoi(temp.c_str());

  GetNodeText(GetNode(t_node, "NumberOfGoalKnown"), temp);
  numberOfFutureTargetPositionsKnown = atoi(temp.c_str());
  
  //prob(Prob(size_x,size_y));

  string speed_string;
  if (NodeExists(t_node, "optimalSpeed")) 
  {
    GetNodeText(GetNode(t_node, "optimalSpeed"), speed_string);
    sscanf(speed_string.c_str(), "%f", &speed);
  }
  
      
    
  if( m_randomGen == NULL )
    {
      CARGoSRandom::CreateCategory("rwp",
           RandomSeed+m_myID);
      m_randomGen = CSimulator::GetInstance().GetRNG();
    }


  //////////////////////////////////////////////////////////////
  // Initialize things required for communications
  //////////////////////////////////////////////////////////////
  m_pcWifiSensor = dynamic_cast<CCI_WiFiSensor* >(GetRobot().GetSensor("wifi"));
  m_pcWifiActuator = dynamic_cast<CCI_WiFiActuator* >(GetRobot().GetActuator("wifi"));

  //  m_pcWifiSensorLongRange = dynamic_cast<CCI_WiFiSensor* >(GetRobot().GetSensor("wifilongrange"));
  //  m_pcWifiActuatorLongRange = dynamic_cast<CCI_WiFiActuator* >(GetRobot().GetActuator("wifilongrange"));

  //Led actuator
  m_pcLEDs   = dynamic_cast<CCI_FootBotLedsActuator*>(GetRobot().GetActuator("footbot_leds"));
   
  /// create the client and pass the configuration tree (XML) to it
  m_navClient = new RVONavClient(m_myID, GetRobot());
  m_navClient->init(t_node);
 
  /// start the navigation client
  m_navClient->start(); 
  m_pcLEDs->SetAllColors(CColor::MAGENTA); 
  
  //printf("Random num %d\n", m_randomGen->Uniform(CRange<UInt32>(1,50)));
  //agentMap[stateData.detectedAgentId]->time_last_visited = m_randomGen->Uniform(CRange<UInt32>(1,50));
  
  
  int NumberOfAgents = stateData.NUMBER_OF_AGENT;
  
  printf("Initialising agent and base station positions \n");
  /****Initialising Agent and Base Station positions ****/
  stateData.Init(GetNode(t_node, "state"), m_myID);

  initialiseAgentData(GetNode(t_node, "agent"),NumberOfAgents, numberOfFutureTargetPositionsKnown);
 
  // Records Positions of Agents
  relayPositions.filename = "outputFiles"+to_string(stateData.NUMBER_OF_RELAY)+"/position" + to_string(m_myID)+".csv";
  relayPositions.data_file.open(relayPositions.filename, ios::out | ios::ate);
  relayPositions.data_file << "x" << "," << "y" << "\n";

 // Records the time and position at which a relay meets the agent
  timeStepToMeet.filename = "outputFiles"+to_string(stateData.NUMBER_OF_RELAY)+"/timestep"+ to_string(m_myID)+".csv";
  timeStepToMeet.data_file.open(timeStepToMeet.filename, ios::out | ios::ate);
  timeStepToMeet.data_file << "TimeWhenMet" << "," << "x" << "," << "y" << "," << "AgentId" << "\n";

  delayTime.filename = "outputFiles"+to_string(stateData.NUMBER_OF_RELAY)+"/delayTime"+ to_string(m_myID)+".csv";
  delayTime.data_file.open(delayTime.filename, ios::out | ios::ate);

  delayTime.data_file << "Agents" << "," << "DelayTime" << "\n" ;
  
  timeForACycle.filename = "outputFiles"+to_string(stateData.NUMBER_OF_RELAY)+"/CycleTime"+ to_string(m_myID)+".csv";
  timeForACycle.data_file.open(timeForACycle.filename, ios::out | ios::ate);
  timeForACycle.data_file << "Start" << "," << "End" << "\n";
  timeForACycle.data_file << 5 << ",";

  uint8_t relayBeginningId = (uint8_t)(stateData.NUMBER_OF_AGENT +stateData.NUMBER_OF_BASESTATION +1);
  
  

  for(int i= 0; i< stateData.NUMBER_OF_RELAY; i++)
  {
    relayIdMap.emplace(relayBeginningId,i);

    if(relayBeginningId != m_myID)
    { 
      stateData.RelaysReachedBaseStation.insert(relayBeginningId);
      relayIdList.push_back(relayBeginningId);
    }
  
    relayBeginningId = relayBeginningId + 1;

    roundVector.push_back(i);
  }
 
  
  numberOfRounds = stateData.NUMBER_OF_RELAY;
  if(numberOfRounds == 1)
  {
    onlyIamFree = true;
    stateData.RoundState = SStateData::STATE_ASSIGN;
  }
  else if(numberOfRounds > 1)
  {
    currentRoundNumber = 1;
    stateData.RoundState = SStateData::STATE_INITIAL;
  }


int id = (int)relayIdMap[m_myID];

/*for(int i= 0; i < roundVector[id]; i++)
{ 
  stateData.parents.push_back(relayIdList[i]);
}*/

if(roundVector[id] != 0)
{ 
  int i = roundVector[id]-1;
  printf("relayIdList parent %d \n",(int)relayIdList[i]);
  stateData.parents.push_back(relayIdList[i]);
}

if(roundVector[id] != stateData.NUMBER_OF_RELAY-1)
{
  int i = roundVector[id];
  printf("relayIdList children %d \n",(int)relayIdList[i]);
  stateData.children.push_back(relayIdList[i]);
}
//std::sort(relayIdList.begin(), relayIdList.end());
//std::sort(stateData.parents.begin(), stateData.parents.end());

//std::set_difference(relayIdList.begin(), relayIdList.end(), stateData.parents.begin(), stateData.parents.end(), std::back_inserter(stateData.children));


printf("iD %d parents size %d children size %d \n",(int)m_myID, stateData.parents.size(),stateData.children.size());
for(auto &p:stateData.parents)
{
  printf("parents %d \n",(int)p);
}

for(auto &c:stateData.children)
{
  printf("children %d \n",(int)c);
}

printf("number Of Rounds %d \n",numberOfRounds);
printf("Number of agents to visit before going to basestation %d \n",int(ceil(stateData.NUMBER_OF_AGENT/(stateData.NUMBER_OF_RELAY*1.0))));
}


void
FootbotRelay::initialiseAgentData(TConfigurationNode& t_node, int NumberOfAgents, int NumberOfGoal)
{
  
  Position loc; 
  printf("number agetns %d\n", NumberOfAgents);
  //NUMBER_OF_BASESTATION ;
  for(int i=0; i< NumberOfAgents; i++)
  {  
    std::shared_ptr<SAgentData> agentTemp(new SAgentData());

    ostringstream temp_str;
    temp_str << "agent" << i+1;

    GetNodeAttribute(GetNode(t_node, temp_str.str()), "id",  agentTemp->id);
   
    GetNodeAttribute(GetNode(t_node, temp_str.str()), "x", agentTemp->current_location.x);
    GetNodeAttribute(GetNode(t_node, temp_str.str()), "y", agentTemp->current_location.y);
    
    for(int j= 1; j <= NumberOfGoal; j++)
    { 
      ostringstream posX;
      ostringstream posY;

      posX << "goal" << j << "x";
      posY << "goal" << j << "y";

      Position goal;
      GetNodeAttribute(GetNode(t_node, temp_str.str()), posX.str(), goal.x);
      GetNodeAttribute(GetNode(t_node, temp_str.str()), posY.str(), goal.y);

      agentTemp->goalLocationList.emplace_back(goal);
    }
    agentIdMap.emplace(i,agentTemp->id);
    agentIdList.push_back(agentTemp->id);

    agentTemp->goal_location  =  agentTemp->goalLocationList[0];
    
    agentMap.emplace(agentTemp->id,agentTemp);
   
    

    prob.initialise(agentTemp->id,agentTemp->current_location,agentTemp->goalLocationList);
  }
  
  for(auto &agi:agentIdList)
  {
     printf("agents in list %d\n",(uint8_t)agi);
  }
 agentIdToConsider = agentIdList;
 roundAgentIdToConsider = agentIdToConsider;
}



size_t 
FootbotRelay::HelloToAgent(uint8_t identifier, char* out_to_agent)
{  
  /*** This message is sent to detect agent in range ***/

  long unsigned int initial_address =  (long unsigned int)&(*out_to_agent);
  long unsigned int final_address;
    try
    {
      /// identifier - 0 relay to agent hello message 
      //uint8_t identifier = 0; 
      printf("Size of character %d\n", sizeof(identifier));
      memcpy(out_to_agent,&identifier,sizeof(identifier));
      out_to_agent = out_to_agent + sizeof(identifier);
      
      uint8_t id = (uint8_t)m_myID;
      memcpy(out_to_agent,&id,sizeof(id));
      out_to_agent = out_to_agent + sizeof(id);
      
      final_address = (long unsigned int)&(*out_to_agent);
      
    }
    catch(exception& e)
    {
      printf("Exception: %s\n",e.what());
    }
    printf("message created \n");
  return size_t(final_address-initial_address);
}

size_t 
FootbotRelay::AcceptanceToAgent(uint8_t identifier,char* out_to_agent)
{
  /*** This message is sent as acceptance to collect data from agent in range ***/
  printf("Creating message to request data from agent\n");
  long unsigned int initial_address =  (long unsigned int)&(*out_to_agent);

  //uint8_t identifier = 1;
  memcpy(out_to_agent,&identifier,sizeof(identifier));
  out_to_agent += sizeof(identifier);

  uint8_t id_relay = m_myID;
  memcpy(out_to_agent,&id_relay,sizeof(id_relay));
  out_to_agent+= sizeof(id_relay);
    
  long unsigned int final_address =  (long unsigned int)&(*out_to_agent);

  return size_t(final_address-initial_address);

}

size_t
FootbotRelay::ToBaseStation(uint8_t identifier,char* data_to_basestation_ptr)
{
  long unsigned int initial_address =  (long unsigned int)&(*data_to_basestation_ptr);
  printf("Creating message to send data to base station\n");

  uint8_t messId = identifier;
  memcpy(data_to_basestation_ptr, &messId, sizeof(messId));
  data_to_basestation_ptr = data_to_basestation_ptr + sizeof(messId);
  printf("Identifier data to base station %d\n", messId);


  uint8_t relay_id = m_myID;
  memcpy(data_to_basestation_ptr, &relay_id, sizeof(relay_id));
  data_to_basestation_ptr = data_to_basestation_ptr + sizeof(relay_id);
  printf("Relay Id %d\n", (int)relay_id);

  uint8_t numberOfAgents = visitedAgent.size();
  memcpy(data_to_basestation_ptr, &numberOfAgents, sizeof(numberOfAgents));
  data_to_basestation_ptr = data_to_basestation_ptr + sizeof(numberOfAgents);
  printf("number of agents %d\n", (int)numberOfAgents);

  for(auto &agentId:visitedAgent)
  {
      uint8_t id = agentMap[agentId]->id;
      memcpy(data_to_basestation_ptr, &id, sizeof(id));
      data_to_basestation_ptr = data_to_basestation_ptr + sizeof(id);
      printf("Agent Id:  %d\n", id);

      uint64_t time_last_data_collected = agentMap[agentId]->transmitted_data_time;
      memcpy(data_to_basestation_ptr, &time_last_data_collected, sizeof(time_last_data_collected));
      data_to_basestation_ptr = data_to_basestation_ptr + sizeof(time_last_data_collected);
      printf("Time Message Sent:  %d\n", time_last_data_collected);

      uint32_t agent_message_size = agentMap[agentId]->transmitted_data_size;
      memcpy(data_to_basestation_ptr, &agent_message_size, sizeof(agent_message_size));
      data_to_basestation_ptr = data_to_basestation_ptr + sizeof(agent_message_size);
      printf("Agent Message Size:  %d\n", agent_message_size);

      // delay time considers the time between the data received to the relay and delivered to base station
      printf("Agent ID %d delay in gathering data %d \n", (int)m_myID,agentMap[agentId]->delay_in_collection);
      printf("Delay in deliverign data %d \n", (m_Steps-agentMap[agentId]->transmitted_data_time));

      agentMap[agentId]->delay_time  = 0;

      if(agent_message_size != 0)
      {
        agentMap[agentId]->delay_time = (m_Steps-agentMap[agentId]->transmitted_data_time) + agentMap[agentId]->delay_in_collection;
      }
      
      printf("Total Delay Time: %d\n", agentMap[agentId]->delay_time);
      delayTime.data_file << agentMap[agentId]->id << "," << agentMap[agentId]->delay_time << "\n";
  }

  
  
  long unsigned int final_address =  (long unsigned int)&(*data_to_basestation_ptr);
  uint32_t data_to_BS_size = (final_address-initial_address); 
  
  /// 8.Message size (4)
  memcpy(data_to_basestation_ptr, &data_to_BS_size,sizeof(data_to_BS_size));
  data_to_basestation_ptr = data_to_basestation_ptr + sizeof(data_to_BS_size);
  data_to_BS_size = data_to_BS_size + sizeof(data_to_BS_size);
  
  printf("%d message_size \n", data_to_BS_size);
  stateData.IsDataSentToBaseStation = true;

  return size_t(data_to_BS_size);
}


size_t 
FootbotRelay::TaskAssignedToRelay(uint8_t identifier, char* out_to_relay)
{  
  /*** This message is sent to other relays to announce the agents assigned to children ***/

  long unsigned int initial_address =  (long unsigned int)&(*out_to_relay);
  long unsigned int final_address;
    

  /// identifier - 2 relay to relay tasks assigned
  uint8_t message_id = identifier;
  printf("Size of character %d\n", sizeof(message_id));
  memcpy(out_to_relay,&message_id,sizeof(message_id));
  out_to_relay = out_to_relay + sizeof(message_id);
  
  uint8_t id = (uint8_t)m_myID;
  memcpy(out_to_relay,&id,sizeof(id));
  out_to_relay = out_to_relay + sizeof(id);
  
  
  uint8_t agId = stateData.targetAgentId;
  memcpy(out_to_relay,&agId,sizeof(agId));
  out_to_relay = out_to_relay + sizeof(agId);
  

  final_address = (long unsigned int)&(*out_to_relay);
    
  printf("message created \n");
  return size_t(final_address-initial_address);
}


size_t 
FootbotRelay::NewDataInformationToRelay(uint8_t identifier, char* out_to_relay)
{  
  /*** This message is sent  to other relays to announce agents met and their information ***/

  long unsigned int initial_address =  (long unsigned int)&(*out_to_relay);
  long unsigned int final_address;
    
  /// identifier - 2 relay to relay tasks assigned
  uint8_t message_id = identifier;
  printf("Size of character %d\n", sizeof(message_id));
  memcpy(out_to_relay,&message_id,sizeof(message_id));
  out_to_relay = out_to_relay + sizeof(message_id);
  
  uint8_t id = (uint8_t)m_myID;
  memcpy(out_to_relay,&id,sizeof(id));
  out_to_relay = out_to_relay + sizeof(id);
  
  //Recently visited agent's info
  uint8_t agId = (uint8_t)stateData.recentlyProfileDataCollectedId;
  memcpy(out_to_relay,&agId,sizeof(agId));
  out_to_relay = out_to_relay + sizeof(agId);

  stateData.recentlyProfileDataCollectedId = 0;
  
  // current Location
  float curr_x = agentMap[agId]->current_location.x;
  memcpy(out_to_relay,&curr_x,sizeof(curr_x));
  out_to_relay = out_to_relay + sizeof(curr_x);

  float curr_y = agentMap[agId]->current_location.y;
  memcpy(out_to_relay,&curr_y,sizeof(curr_y));
  out_to_relay = out_to_relay + sizeof(curr_y);

  //current goal 
  float goal_x = agentMap[agId]->goal_location.x;
  memcpy(out_to_relay,&goal_x,sizeof(goal_x));
  out_to_relay = out_to_relay + sizeof(goal_x);

  float goal_y = agentMap[agId]->goal_location.y;
  memcpy(out_to_relay,&goal_y,sizeof(goal_y));
  out_to_relay = out_to_relay + sizeof(goal_y);
 
  // goal Location list
  for(int i=0; i < numberOfFutureTargetPositionsKnown; i++)
  {
      //future goal 
    float fgoal_x = agentMap[agId]->goalLocationList[i].x;
    memcpy(out_to_relay,&fgoal_x,sizeof(fgoal_x));
    out_to_relay = out_to_relay + sizeof(fgoal_x);

    float fgoal_y = agentMap[agId]->goalLocationList[i].y;
    memcpy(out_to_relay,&fgoal_y,sizeof(fgoal_y));
    out_to_relay = out_to_relay + sizeof(fgoal_y);
  }
  
  
  final_address = (long unsigned int)&(*out_to_relay);
    
  printf("message created \n");
  return size_t(final_address-initial_address);
}


size_t 
FootbotRelay::SendingTransmittedDataTimeToRelay(uint8_t identifier, char* out_to_relay)
{

  long unsigned int initial_address =  (long unsigned int)&(*out_to_relay);
  long unsigned int final_address;
    
  /// identifier - 2 relay to relay tasks assigned
  uint8_t message_id = identifier;
  printf("Size of character %d\n", sizeof(message_id));
  memcpy(out_to_relay,&message_id,sizeof(message_id));
  out_to_relay = out_to_relay + sizeof(message_id);
  
  uint8_t id = (uint8_t)m_myID;
  memcpy(out_to_relay,&id,sizeof(id));
  out_to_relay = out_to_relay + sizeof(id);
  
 //Recently visited agent's info
  uint8_t agId = (uint8_t)stateData.recentlyVisitedAgentId;
  memcpy(out_to_relay,&agId,sizeof(agId));
  out_to_relay = out_to_relay + sizeof(agId);

  stateData.recentlyVisitedAgentId = 0;

  // time when data gathered
  uint64_t dataReceivedTime = agentMap[agId]->transmitted_data_time;
  memcpy(out_to_relay,&dataReceivedTime,sizeof(dataReceivedTime));
  out_to_relay = out_to_relay + sizeof(dataReceivedTime);

  final_address = (long unsigned int)&(*out_to_relay);
    
  printf("message created \n");
  return size_t(final_address-initial_address);
}



size_t 
FootbotRelay::AnnouncingTaskcompletion(uint8_t identifier, char* out_to_relay)
{  
  /*** This message is sent  to other relays to announce agents met and their information ***/

  long unsigned int initial_address =  (long unsigned int)&(*out_to_relay);
  long unsigned int final_address;
    
  /// identifier - 2 relay to relay tasks assigned
  uint8_t message_id = identifier;
  printf("Size of character %d\n", sizeof(message_id));
  memcpy(out_to_relay,&message_id,sizeof(message_id));
  out_to_relay = out_to_relay + sizeof(message_id);
  
  uint8_t id = (uint8_t)m_myID;
  memcpy(out_to_relay,&id,sizeof(id));
  out_to_relay = out_to_relay + sizeof(id);

  float d = distanceToBaseStation;
  memcpy(out_to_relay,&d,sizeof(d));
  out_to_relay = out_to_relay + sizeof(d);

  final_address = (long unsigned int)&(*out_to_relay);
    
  printf("message created \n");
  return size_t(final_address-initial_address);
}

size_t 
FootbotRelay::RoundTaskAssignedToRelayChildren(uint8_t identifier, char* out_to_relay)
{  
  /*** This message is sent to other relays to announce the agents assigned to children ***/

  long unsigned int initial_address =  (long unsigned int)&(*out_to_relay);
  long unsigned int final_address;
    

  /// identifier - 2 relay to relay tasks assigned
  uint8_t message_id = identifier;
  printf("Size of character %d\n", sizeof(message_id));
  memcpy(out_to_relay,&message_id,sizeof(message_id));
  out_to_relay = out_to_relay + sizeof(message_id);
  
  uint8_t id = (uint8_t)m_myID;
  memcpy(out_to_relay,&id,sizeof(id));
  out_to_relay = out_to_relay + sizeof(id);
  
  uint8_t roundNum = (uint8_t)currentRoundNumber;
  memcpy(out_to_relay,&roundNum,sizeof(roundNum));
  out_to_relay = out_to_relay + sizeof(roundNum);
  
  idsChosenbyParents.insert(agent_considered);

  uint8_t setSize = idsChosenbyParents.size();
  printf("size of agents assigned to parents %d \n",(int)setSize);
  memcpy(out_to_relay,&setSize,sizeof(setSize));
  out_to_relay = out_to_relay + sizeof(setSize);
  
  for(auto &agentId:idsChosenbyParents)
  {
    uint8_t agId = agentId;
    printf("agents chosen %d by Relays %d \n",(int)agId, (int)m_myID);
    memcpy(out_to_relay,&agId,sizeof(agId));
    out_to_relay = out_to_relay + sizeof(agId);
  }
  float cost;
  if(stateData.parents.size() > 0)
  {
    cost = costMap[stateData.parents[0]]+localCost;
  }
  else
  {
    cost = localCost;
  }
  
  memcpy(out_to_relay,&cost,sizeof(cost));
  out_to_relay = out_to_relay + sizeof(cost);

  final_address = (long unsigned int)&(*out_to_relay);
    
  printf("message created \n");
  return size_t(final_address-initial_address);
}


size_t 
FootbotRelay::RoundCostToRelayParents(uint8_t identifier, char* out_to_relay)
{  
  /*** This message is sent to other relays to announce the agents assigned to children ***/

  long unsigned int initial_address =  (long unsigned int)&(*out_to_relay);
  long unsigned int final_address;
    

  /// identifier - 2 relay to relay tasks assigned
  uint8_t message_id = identifier;
  printf("Size of character %d\n", sizeof(message_id));
  memcpy(out_to_relay,&message_id,sizeof(message_id));
  out_to_relay = out_to_relay + sizeof(message_id);
  
  uint8_t id = (uint8_t)m_myID;
  memcpy(out_to_relay,&id,sizeof(id));
  out_to_relay = out_to_relay + sizeof(id);
  
  uint8_t roundNum = (uint8_t)currentRoundNumber;
  memcpy(out_to_relay,&roundNum,sizeof(roundNum));
  out_to_relay = out_to_relay + sizeof(roundNum);
  
  float cost;
  if(stateData.children.size() > 0)
  {
    cost = costMap[stateData.children[0]]+localCost;
  }
  else
  {
    cost = localCost;
  }

  memcpy(out_to_relay,&cost,sizeof(cost));
  out_to_relay = out_to_relay + sizeof(cost);

  final_address = (long unsigned int)&(*out_to_relay);
    
  printf("message created \n");
  return size_t(final_address-initial_address);
}


size_t 
FootbotRelay::RoundStop(uint8_t identifier, char* out_to_relay)
{
   /*** This message is sent to other relays to announce the agents assigned to children ***/

  long unsigned int initial_address =  (long unsigned int)&(*out_to_relay);
  long unsigned int final_address;
    

  /// identifier - 2 relay to relay tasks assigned
  uint8_t message_id = identifier;
  printf("Size of character %d\n", sizeof(message_id));
  memcpy(out_to_relay,&message_id,sizeof(message_id));
  out_to_relay = out_to_relay + sizeof(message_id);
  
  uint8_t id = (uint8_t)m_myID;
  memcpy(out_to_relay,&id,sizeof(id));
  out_to_relay = out_to_relay + sizeof(id);

   final_address = (long unsigned int)&(*out_to_relay);
    
  printf("message created to stop round \n");
  return size_t(final_address-initial_address);
}

size_t 
FootbotRelay::RoundStart(uint8_t identifier, char* out_to_relay)
{
   /*** This message is sent to other relays to announce the agents assigned to children ***/

  long unsigned int initial_address =  (long unsigned int)&(*out_to_relay);
  long unsigned int final_address;
    

  /// identifier - 2 relay to relay tasks assigned
  uint8_t message_id = identifier;
  printf("Size of character %d\n", sizeof(message_id));
  memcpy(out_to_relay,&message_id,sizeof(message_id));
  out_to_relay = out_to_relay + sizeof(message_id);
  
  uint8_t id = (uint8_t)m_myID;
  memcpy(out_to_relay,&id,sizeof(id));
  out_to_relay = out_to_relay + sizeof(id);
  
  uint8_t sizeofFreeRelays = (uint8_t)(receivedMessageFromRelay.size()-1);
  memcpy(out_to_relay,&sizeofFreeRelays,sizeof(sizeofFreeRelays));
  out_to_relay = out_to_relay + sizeof(sizeofFreeRelays);

  for(auto &fRelay : receivedMessageFromRelay)
  {
    if(fRelay != m_myID)
    {
      uint8_t freeReId = fRelay;
      memcpy(out_to_relay,&freeReId,sizeof(freeReId));
      out_to_relay = out_to_relay + sizeof(freeReId);
    }
  }
  final_address = (long unsigned int)&(*out_to_relay);
    
  printf("message created to start round \n");
  return size_t(final_address-initial_address);
}

void 
FootbotRelay::ParseAgentProfile(vector<char> &incoming_agent_message)
{   

    char* agent_mes_ptr = (char*)&incoming_agent_message[0];
    

    uint8_t mes_type = (uint8_t)agent_mes_ptr[0];
    agent_mes_ptr = agent_mes_ptr + sizeof(mes_type);

    //neighbour_count = neighbour_count + 1;
   

   printf("profile message from agent\n");
    
    // Agent id
    uint8_t id = (uint8_t)agent_mes_ptr[0];
    agent_mes_ptr = agent_mes_ptr + sizeof(id);
    agentMap[id]->id = id;
    printf("agent id %d\n",agentMap[id]->id);
    
    // Time when message is sent
   
    memcpy(&agentMap[id]->time_last_visited, agent_mes_ptr, sizeof(agentMap[id]->time_last_visited));
    //printf("time sent %u\n",agentMap[id]->time_last_visited);
    agent_mes_ptr+=sizeof(agentMap[id]->time_last_visited);

    // Agent pos
    memcpy(&agentMap[id]->current_location.x, agent_mes_ptr, sizeof(agentMap[id]->current_location.x));
    agent_mes_ptr+= sizeof(agentMap[id]->current_location.x);

    memcpy(&agentMap[id]->current_location.y, agent_mes_ptr, sizeof(agentMap[id]->current_location.y));
    agent_mes_ptr+= sizeof(agentMap[id]->current_location.y);
    
    //printf("Agent Position %f %f \n", agentMap[id]->current_location.x, agentMap[id]->current_location.y);


    // last time when hte data is transmitted
    memcpy(&agentMap[id]->time_last_data_collected,agent_mes_ptr,sizeof(agentMap[id]->time_last_data_collected));
    agent_mes_ptr+= sizeof(agentMap[id]->time_last_data_collected);
    //printf("LAST DATA transmitted %u\n",agentMap[id]->time_last_data_collected);

    // number of neighbours
   /* agent_message.number_neighbors = (uint8_t)agent_mes_ptr[0];
    agent_mes_ptr = agent_mes_ptr+ sizeof(agent_message.number_neighbors);
    //printf("number of neighbours for agent %d\n",agent_message.number_neighbors);
  
   //timestep
    memcpy(&agent_message.timestep, agent_mes_ptr, sizeof(agent_message.timestep));
    agent_mes_ptr = agent_mes_ptr + sizeof(agent_message.timestep);
    //printf("timestep %d\n", agent_message.timestep);
     */
    // future position
    agentMap[id]->goal_location.x = 0.0;
    agentMap[id]->goal_location.y = 0.0;

    memcpy(&agentMap[id]->goal_location.x, agent_mes_ptr, sizeof(agentMap[id]->goal_location.x));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentMap[id]->goal_location.x);
    //printf("target X %f\n", agentMap[id]->goal_location.x);

    memcpy(&agentMap[id]->goal_location.y, agent_mes_ptr, sizeof(agentMap[id]->goal_location.y));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentMap[id]->goal_location.y);
    //printf("target Y %f\n", agentMap[id]->goal_location.y);
    agentMap[id]->IsGoalSet = true;

    vector <Util::Position> goalList;
    goalList.emplace_back(agentMap[id]->goal_location);

    //Future Goal Locations - 4 + 1 (current target)
    for(int i=1; i<= numberOfFutureTargetPositionsKnown; i++)
    {
        Position goal;

        memcpy(&goal.x, agent_mes_ptr, sizeof(goal.x));
        agent_mes_ptr = agent_mes_ptr + sizeof(goal.x);
        //printf("target X %f\n", goal.x);

        memcpy(&goal.y, agent_mes_ptr, sizeof(goal.y));
        agent_mes_ptr = agent_mes_ptr + sizeof(goal.y);
        //printf("target Y %f\n", goal.y);

        goalList.emplace_back(goal);
    }
    
      agentMap[id]->goalLocationList = goalList;
    
    
     // amount of data available from the agent
    memcpy(&agentMap[id]->data_available, agent_mes_ptr, sizeof(agentMap[id]->data_available));
    agent_mes_ptr = agent_mes_ptr + sizeof(agentMap[id]->data_available);
    //printf("Amount of data available %lu\n", agentMap[id]->data_available);
    
    uint32_t message_size;
    memcpy(&message_size, agent_mes_ptr, sizeof(message_size));
    agent_mes_ptr = agent_mes_ptr+sizeof(message_size);
    //printf("message_size %d\n",message_size);

    //if(agentMap[id]->data_available > 0)
    
       
       //stateData.detectedAgentId.push_back(id);
       vector<uint8_t>::iterator itr = std::find(stateData.InRange.begin(),stateData.InRange.end(),id);
       

       if( itr == stateData.InRange.end() && stateData.targetAgentId == id && agentMap[id]->data_available > 0)
       {
          stateData.InRange.push_back(id);
          stateData.IsAgentDetected = true;
       }
       
    
    //update agent locations in the probability map
    //if(not prob.getAgentState(id) == 0)
    { 
      //printf("Tell me what is happeningggg \n");
      prob.updateAgentPosition(id, agentMap[id]->current_location, agentMap[id]->goalLocationList);
    }
    
    stateData.recentlyProfileDataCollectedId = id;

    if(stateData.NUMBER_OF_RELAY > 1)
    {
      stateData.SentData = SStateData::NEW_DATA;
      SendMessage(stateData.SentData, 0);
    }

    timeStepToMeet.data_file << m_Steps << "," << m_navClient->currentPosition().GetX() << "," << m_navClient->currentPosition().GetY()<< "," << id << "\n" ;
    printf("done parsing agent message\n");
}



void 
FootbotRelay::ParseAgentCollectedData(vector<char> &incoming_agent_message)
{   
  char* agent_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)agent_mes_ptr[0];
  agent_mes_ptr = agent_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,agent_mes_ptr,sizeof(id));
  agent_mes_ptr = agent_mes_ptr + sizeof(id);


  memcpy(&agentMap[id]->transmitted_data_time, agent_mes_ptr, sizeof(agentMap[id]->transmitted_data_time));
  agent_mes_ptr = agent_mes_ptr + sizeof(agentMap[id]->transmitted_data_time);

  printf("Received data from agents\n");
    /// Receiving and storing data
   
  memcpy(&agentMap[id]->transmitted_data_size,agent_mes_ptr,sizeof(agentMap[id]->transmitted_data_size));
  agent_mes_ptr = agent_mes_ptr + sizeof(agentMap[id]->transmitted_data_size);
  
  //printf("Relay knows the size of data: %d sent \n",(agentMap[id]->transmitted_data_size));

  datasizeToBaseStation = agentMap[id]->transmitted_data_size + datasizeToBaseStation; 

  memcpy(&agentMap[id]->delay_in_collection,agent_mes_ptr,sizeof(agentMap[id]->delay_in_collection));
  agent_mes_ptr = agent_mes_ptr + sizeof(agentMap[id]->delay_in_collection);
      
  agentIdToConsider.erase(std::remove(agentIdToConsider.begin(), agentIdToConsider.end(), id), agentIdToConsider.end()); 
    
  for(auto &ag:agentIdToConsider)
  {
    printf("afterr erasing %d \n", (int)ag);
  }

  stateData.InRange.erase(std::remove(stateData.InRange.begin(), stateData.InRange.end(), id), stateData.InRange.end());
  
  int i = id;
  
  //agentMap[i]->data_available = 0;
  agentMap[i]->IsDataReceived = true;
  visitedAgent.insert(id);
  stateData.recentlyVisitedAgentId = id;

  //stateData.State = SStateData::STATE_HOME;
  //printf("Switching to home state after gathering data in data gather state \n");
  taskDone = true;
}

void 
FootbotRelay::ParseRelayTaskInfo(vector<char> &incoming_agent_message)
{   
  char* relay_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,relay_mes_ptr,sizeof(id));
  relay_mes_ptr = relay_mes_ptr + sizeof(id);

  //stateData.ParentMessages.insert(id);

  uint8_t agentAssi;
  memcpy(&agentAssi,relay_mes_ptr,sizeof(agentAssi));
  relay_mes_ptr = relay_mes_ptr + sizeof(agentAssi);

  printf(" Removed Agent %d \n", (int)agentAssi);

  agentIdToConsider.erase(std::remove(agentIdToConsider.begin(), agentIdToConsider.end(), agentAssi), agentIdToConsider.end());
  
  
  for(auto &rem: agentIdToConsider)
  {
     printf("%d Remaining agents %d \n", (int)m_myID, rem);
  }

}


void 
FootbotRelay::ParseRelayNewInformation(vector<char> &incoming_agent_message)
{   
  char* relay_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,relay_mes_ptr,sizeof(id));
  relay_mes_ptr = relay_mes_ptr + sizeof(id);

  uint8_t agId;
  memcpy(&agId,relay_mes_ptr,sizeof(agId));
  relay_mes_ptr = relay_mes_ptr + sizeof(agId);

  if(find(agentIdToConsider.begin(),agentIdToConsider.end(),agId) == agentIdToConsider.end())
  {
    agentIdToConsider.push_back(agId);
  }

  float curr_agX;
  memcpy(&curr_agX,relay_mes_ptr,sizeof(curr_agX));
  relay_mes_ptr = relay_mes_ptr + sizeof(curr_agX);

  float curr_agY;
  memcpy(&curr_agY,relay_mes_ptr,sizeof(curr_agY));
  relay_mes_ptr = relay_mes_ptr + sizeof(curr_agY);

  float goal_agX;
  memcpy(&goal_agX,relay_mes_ptr,sizeof(goal_agX));
  relay_mes_ptr = relay_mes_ptr + sizeof(goal_agX);

  float goal_agY;
  memcpy(&goal_agY,relay_mes_ptr,sizeof(goal_agY));
  relay_mes_ptr = relay_mes_ptr + sizeof(goal_agY);

  agentMap[agId]->current_location.x = curr_agX;
  agentMap[agId]->current_location.y = curr_agY;

  agentMap[agId]->goal_location.x = goal_agX;
  agentMap[agId]->goal_location.y = goal_agY;

  for(int i= 0; i < numberOfFutureTargetPositionsKnown; i++)
  {
    float goalX;
    memcpy(&goalX,relay_mes_ptr,sizeof(goalX));
    relay_mes_ptr = relay_mes_ptr + sizeof(goalX);

    float goalY;
    memcpy(&goalY,relay_mes_ptr,sizeof(goalY));
    relay_mes_ptr = relay_mes_ptr + sizeof(goalY);

    agentMap[agId]->goalLocationList[i].x = goalX;
    agentMap[agId]->goalLocationList[i].y = goalY;
  }
  
 
  prob.updateAgentPosition(agId, agentMap[agId]->current_location, agentMap[agId]->goalLocationList);

}

void 
FootbotRelay::ParseRelayTransmittedDataTime(vector<char> &incoming_agent_message)
{   
  char* relay_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,relay_mes_ptr,sizeof(id));
  relay_mes_ptr = relay_mes_ptr + sizeof(id);

  uint8_t agId;
  memcpy(&agId,relay_mes_ptr,sizeof(agId));
  relay_mes_ptr = relay_mes_ptr + sizeof(agId);

  uint64_t timeAgentDataGathered;
  memcpy(&timeAgentDataGathered,relay_mes_ptr,sizeof(timeAgentDataGathered));
  relay_mes_ptr = relay_mes_ptr + sizeof(timeAgentDataGathered);
  
  agentMap[agId]->transmitted_data_time = timeAgentDataGathered;

  
}


void 
FootbotRelay::ParseRelayFinalData(vector<char> &incoming_agent_message)
{   
  char* relay_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,relay_mes_ptr,sizeof(id));
  relay_mes_ptr = relay_mes_ptr + sizeof(id);

  float distBS;
  memcpy(&distBS,relay_mes_ptr,sizeof(distBS));
  relay_mes_ptr = relay_mes_ptr + sizeof(distBS);
  
  printf("distance difference %f \n", abs(distanceToBaseStation-distBS));

  if(stateData.MovingToBaseStation && currentRoundNumber == 0)
  { 
    uint8_t sizeBefore, sizeAfter;
    sizeBefore = receivedMessageFromRelay.size();
    if(find(receivedMessageFromRelay.begin(),receivedMessageFromRelay.end(),id) == receivedMessageFromRelay.end())
    {
      receivedMessageFromRelay.push_back(id);
    }
    
    
    sizeAfter = receivedMessageFromRelay.size();
    printf("Relay %d is free \n", (int)id);

    if(sizeAfter > sizeBefore)
    {
      receivedMessageFromRelayVector.push_back(id);
    }
    //freeRelayIds.insert(id);

    stateData.SentData = SStateData::MOVING_TO_BASESTATION;
    SendMessage(stateData.SentData, 0);
  }
  
  stateData.RelaysReachedBaseStation.insert(id);
}

void 
FootbotRelay::ParseRoundTaskData(vector<char> &incoming_agent_message)
{   
  char* relay_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,relay_mes_ptr,sizeof(id));
  relay_mes_ptr = relay_mes_ptr + sizeof(id);
  
  printf("Id %d received task assigned in rounds from parent \n", int(m_myID));
  stateData.ParentMessages.insert(id);
  printf("Parent message size %d \n", stateData.ParentMessages.size());

  uint8_t roundnum;
  memcpy(&roundnum,relay_mes_ptr,sizeof(roundnum));
  relay_mes_ptr = relay_mes_ptr + sizeof(roundnum);
  
  if(roundnum == currentRoundNumber)
  { 
    uint8_t numberAgentsAssignedToParents;
    memcpy(&numberAgentsAssignedToParents,relay_mes_ptr,sizeof(numberAgentsAssignedToParents));
    relay_mes_ptr = relay_mes_ptr + sizeof(numberAgentsAssignedToParents);

    
    for(int i= 0; i < numberAgentsAssignedToParents; i++)
    {
      uint8_t agid;
      memcpy(&agid,relay_mes_ptr,sizeof(agid));
      relay_mes_ptr = relay_mes_ptr + sizeof(agid);
      printf("Agent received from parent %d \n",(int)agid);
      idsChosenbyParents.insert(agid);
      roundAgentIdToConsider.erase(std::remove(roundAgentIdToConsider.begin(), roundAgentIdToConsider.end(), agid), roundAgentIdToConsider.end());
    }
    

    float Lcost;
    memcpy(&Lcost,relay_mes_ptr,sizeof(Lcost));
    relay_mes_ptr = relay_mes_ptr + sizeof(Lcost);

    std::map<uint8_t,float>::iterator it = costMap.find(id);
    if(it!= costMap.end())
    {
      costMap[id] = Lcost;
    }
    else
      costMap.emplace(id,Lcost);
  }

 

  for(auto &rem: roundAgentIdToConsider)
  {
     printf("%d Remaining agents %d \n", (int)m_myID, rem);
  }

}

void 
FootbotRelay::ParseRoundCostData(vector<char> &incoming_agent_message)
{   
  char* relay_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,relay_mes_ptr,sizeof(id));
  relay_mes_ptr = relay_mes_ptr + sizeof(id);

  stateData.ChildrenMessages.insert(id);

  uint8_t roundnum;
  memcpy(&roundnum,relay_mes_ptr,sizeof(roundnum));
  relay_mes_ptr = relay_mes_ptr + sizeof(roundnum);
  
  
  if(roundnum == currentRoundNumber)
  {
    float Lcost;
    memcpy(&Lcost,relay_mes_ptr,sizeof(Lcost));
    relay_mes_ptr = relay_mes_ptr + sizeof(Lcost);

    std::map<uint8_t,float>::iterator it = costMap.find(id);
    if(it!= costMap.end())
    {
      costMap[id] = Lcost;
    }
    else
      costMap.emplace(id,Lcost);
  }
   
}


void 
FootbotRelay::ParseRoundStartData(vector<char> &incoming_agent_message)
{
  char* relay_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,relay_mes_ptr,sizeof(id));
  relay_mes_ptr = relay_mes_ptr + sizeof(id);

  uint8_t freeRelaysize = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(freeRelaysize);

  for(int i=0; i< freeRelaysize; i++)
  {
    uint8_t freeRelId = (uint8_t)relay_mes_ptr[0];
    relay_mes_ptr = relay_mes_ptr + sizeof(freeRelId);

    if(find(receivedMessageFromRelay.begin(),receivedMessageFromRelay.end(),id) == receivedMessageFromRelay.end())
    {
      receivedMessageFromRelay.push_back(id);
      freeRelayIds.insert(id);
    }
  }


}


void 
FootbotRelay::ParseRoundStopData(vector<char> &incoming_agent_message)
{
  char* relay_mes_ptr = (char*)&incoming_agent_message[0];
    
  uint8_t mes_type = (uint8_t)relay_mes_ptr[0];
  relay_mes_ptr = relay_mes_ptr + sizeof(mes_type);

  uint8_t id;
  memcpy(&id,relay_mes_ptr,sizeof(id));
  relay_mes_ptr = relay_mes_ptr + sizeof(id);

  freeRelayIds.erase(id);
  roundStopId.insert(id);
  
  if(freeRelayIds.size() == 0)
  {
     reinitiliseRoundVariables();
     onlyIamFree = true;
  }

}

void 
FootbotRelay::ParseMessage(vector<char> &incoming_agent_message, uint8_t received_data_id, uint8_t sender_id)
{
  switch(received_data_id) {
      case SStateData::AGENT_PROFILE_DATA: {
        printf("differ %d or initially %d \n", m_Steps - agentMap[sender_id]->time_last_visited, agentMap[sender_id]->time_last_visited);
        printf(" initial %s \n",agentMap[sender_id]->time_last_visited < 0 ? "true" : "false");
        if((m_Steps - agentMap[sender_id]->time_last_visited) > 50 || agentMap[sender_id]->time_last_visited == 0)
        {
         ParseAgentProfile(incoming_agent_message);
        }
        else
        {
          printf("Agent %d recently visited \n",(int)sender_id);
        }
         break;
      }
      case SStateData::AGENT_COLLECTED_DATA: {

         ParseAgentCollectedData(incoming_agent_message); // explore is not necessary for one-one-one case

         if(stateData.NUMBER_OF_RELAY > 1)
          {
           stateData.SentData = SStateData::TRANSMITTED_DATA_TIME;
           SendMessage(stateData.SentData, 0);
          } 
        
         break;
      }
       case SStateData::RELAY_TASK_ASSIGNED_DATA: {
        printf("PArsing received Task Assigned message \n");
         ParseRelayTaskInfo(incoming_agent_message); 
         break;
      } 
      
      case SStateData::RELAY_NEW_DATA: {
        printf("PArsing received new data message \n");
         ParseRelayNewInformation(incoming_agent_message); 
         break;
      } 

       case SStateData::RELAY_TRANSMITTED_DATA_TIME: {
        printf("PArsing received TRANSMITTED_DATA_TIME message \n");
         ParseRelayTransmittedDataTime(incoming_agent_message); 
         break;
      } 

      case SStateData::RELAY_MOVING_TO_BASESTATION: {
        printf("PArsing received reached BS message \n");

         if( find(receivedMessageFromRelay.begin(),receivedMessageFromRelay.end(),sender_id) == receivedMessageFromRelay.end() && stateData.State == 3)
         {
            ParseRelayFinalData(incoming_agent_message); 
         }
         
         break;
      } 

      case SStateData::RELAY_ROUND_TASK_ASSIGNED_DATA: {
        printf("PArsing received round task data \n");
         ParseRoundTaskData(incoming_agent_message); 
         break;
      } 

      case SStateData::RELAY_ROUND_COST_DATA: {
        printf("PArsing received round costdata \n");
         ParseRoundCostData(incoming_agent_message); 
         break;
      } 

      case SStateData::RELAY_START_ROUND: {
        printf("Parsing Relay Start message \n");
         ParseRoundStartData(incoming_agent_message); 
         break;
      } 

      case SStateData::RELAY_STOP_ROUND: {
        printf("Parsing Relay Stop message \n");
         ParseRoundStopData(incoming_agent_message); 
         break;
      } 

      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
  } 
}

void
FootbotRelay::SendMessage(uint8_t send_data_id, uint8_t id)
{ 
  std::ostringstream str_tmp(ostringstream::out);
  str_tmp << "fb_" << id;
  string str_Dest = str_tmp.str();
  
  switch(send_data_id) {
      
      case SStateData::RELAY_HELLO_TO_AGENT: {
         printf("sending hello \n" );
         char agent_socket_msg[20];
         size_t mes_size =  HelloToAgent(send_data_id,agent_socket_msg);
         printf("size of hello message %d\n", (int)mes_size);

         // m_pcWifiActuator->BroadcastMessage_Extern(agent_socket_msg,mes_size);
         m_pcWifiActuator->SendBinaryMessageTo_Extern("-1",agent_socket_msg,mes_size);
       
         
         break;
      }
      
      case SStateData::RELAY_SENDING_ACCEPTANCE_TO_AGENT: {
         
         char agent_data_msg[20];
         size_t mes_size = AcceptanceToAgent(send_data_id,agent_data_msg); 

         m_pcWifiActuator->SendBinaryMessageTo_Extern(str_Dest.c_str(),agent_data_msg,mes_size);
        
        break;
      }
      case SStateData::RELAY_TO_BASESTATION: {
         
         char base_socket_msg[MAX_UDP_SOCKET_BUFFER_SIZE*2];
         size_t collected_data_size = ToBaseStation(send_data_id,base_socket_msg); 

          printf("Base Station Target %s \n", str_Dest.c_str());
          m_pcWifiActuator->SendBinaryMessageTo_Extern(str_Dest.c_str(),base_socket_msg,collected_data_size);
          break;
      }
      
      // Task Assigned info is sent only to agents which are not parents
      case SStateData::TASK_ASSIGNED: {

         char task_assigned_msg[20];
         size_t data_size = TaskAssignedToRelay(send_data_id,task_assigned_msg); 
        
         printf(" Sending message to all \n");
         
         m_pcWifiActuator->SendBinaryMessageTo_Local("-1",task_assigned_msg,data_size);
      
         break;
      }

      // new data info is sent to all other relays
      case SStateData::NEW_DATA: {

         char new_data_msg[MAX_UDP_SOCKET_BUFFER_SIZE];
         size_t data_size = NewDataInformationToRelay(send_data_id,new_data_msg); 

         printf("NEW DATA SENDING \n");
         m_pcWifiActuator->SendBinaryMessageTo_Local("-1",new_data_msg,data_size);
         
         break;
      }
      
       case SStateData::TRANSMITTED_DATA_TIME: 
      {
         char new_data_msg[MAX_UDP_SOCKET_BUFFER_SIZE];
         size_t data_size = SendingTransmittedDataTimeToRelay(send_data_id,new_data_msg); 

         printf("TRANSMITTED_DATA_TIME DATA SENDING \n");
         m_pcWifiActuator->SendBinaryMessageTo_Local("-1",new_data_msg,data_size);
         
         break;
      }

      // new data info is sent to all other relays
      case SStateData::MOVING_TO_BASESTATION: {

         char dest_data_msg[100];
         size_t data_size = AnnouncingTaskcompletion(send_data_id,dest_data_msg); 

         for(auto &r:relayIdList)
         {  
          if(roundStopId.find(r) == roundStopId.end())
           {
              std::ostringstream str_tmpId(ostringstream::out);
              str_tmpId << "fb_" << r;
              string str_relayDest = str_tmpId.str();
              printf("Message from %d to %s \n", (int)m_myID, str_relayDest.c_str());
              printf("SENDING REACHED BASE STATION \n");
              m_pcWifiActuator->SendBinaryMessageTo_Local(str_relayDest.c_str(),dest_data_msg,data_size);
            }
         }
         break;
      }
      
      case SStateData::ROUND_TASK_ASSIGNED_TO_CHILDREN:
      {  
        char round_data_msg[20];
        size_t data_size = RoundTaskAssignedToRelayChildren(send_data_id,round_data_msg); 

        printf("SENDING task assigned in round to children \n");
        printf("Id %d currentRoundNumber %d agent considered %d ;data exchange task in round\n",(int)m_myID, currentRoundNumber, agent_considered);

        for(auto &child:stateData.children)
         { 
            std::ostringstream str_tmpId(ostringstream::out);
            str_tmpId << "fb_" << child;
            string str_relayDest = str_tmpId.str();
            printf("Message from %d to %s \n", (int)m_myID, str_relayDest.c_str());
            m_pcWifiActuator->SendBinaryMessageTo_Local(str_relayDest.c_str(),round_data_msg,data_size);
         }
        break;
      }
      case SStateData::ROUND_COST_TO_PARENTS:
      {  
        char round_data_msg[20];
        size_t data_size = RoundCostToRelayParents(send_data_id,round_data_msg); 

        printf("SENDING cost in round to parents \n");

        for(auto &parent:stateData.parents)
         { 
            std::ostringstream str_tmpId(ostringstream::out);
            str_tmpId << "fb_" << parent;
            string str_relayDest = str_tmpId.str();
            printf("Message from %d to %s \n", (int)m_myID, str_relayDest.c_str());
            m_pcWifiActuator->SendBinaryMessageTo_Local(str_relayDest.c_str(),round_data_msg,data_size);
         }
        break;
      }
      
      case SStateData::START_ROUND:
      {  
        char round_start_msg[40];
        size_t data_size = RoundStart(send_data_id,round_start_msg); 
        stopR = true;
        printf("SENDING START round \n");

        for(auto &parent:receivedMessageFromRelay)
         { 
            std::ostringstream str_tmpId(ostringstream::out);
            str_tmpId << "fb_" << parent;
            string str_relayDest = str_tmpId.str();
            printf("Message from %d to %s \n", (int)m_myID, str_relayDest.c_str());
            m_pcWifiActuator->SendBinaryMessageTo_Local(str_relayDest.c_str(),round_start_msg,data_size);
         }
        break;
      }


      case SStateData::STOP_ROUND:
      {  
        char round_stop_msg[20];
        size_t data_size = RoundStop(send_data_id,round_stop_msg); 
        stopR = true;
        printf("SENDING STOP round \n");

        for(auto &parent:freeRelayIds)
         { 
            std::ostringstream str_tmpId(ostringstream::out);
            str_tmpId << "fb_" << parent;
            string str_relayDest = str_tmpId.str();
            printf("Message from %d to %s \n", (int)m_myID, str_relayDest.c_str());
            m_pcWifiActuator->SendBinaryMessageTo_Local(str_relayDest.c_str(),round_stop_msg,data_size);
         }
        break;
      }


      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
  }
}

bool
FootbotRelay::UpdateState()
{ 
  bool goalCalculated = false;
  printf("In update \n");
  
  // Sending hello message to agent
  if(m_Steps%10 == 0 && not(stateData.MovingToBaseStation))
  {   
    stateData.SentData = SStateData::RELAY_HELLO_TO_AGENT;
    try{
      SendMessage(stateData.SentData, 0);
    }
    catch(exception& e)
    {
      printf("%s\n",e.what());
    }
   
  }

  if(stateData.MovingToBaseStation && freeRelayIds.size() < (stateData.NUMBER_OF_RELAY-1) && m_Steps%20 == 0 && not stopR)
  {
    stateData.SentData = SStateData::MOVING_TO_BASESTATION;
    SendMessage(stateData.SentData, 0);
  }

  
  /**
  *  The code below updates probability matrix values and finds the final belief vector
  *
  */
  
  if(m_Steps % (10) == 0 || m_Steps == 1)  // converting seconds to timestep 1 second = 10 timestep
  { 
    printf("Here finding prob \n");
    
    for (auto const &i:agentMap) 
      {
         if(prob.getAgentState(i.first) != 1)
        {
          Util::Position agentProbablePos;
          agentMap[i.first]->current_location = prob.update(i.first);
          printf("Id %d Position %f %f \n",(int)i.first, agentMap[i.first]->current_location.x, agentMap[i.first]->current_location.y);
         
          printf("update done \n");

         goalCalculated = true;
        }
      }
  }
  return goalCalculated;
}

void
FootbotRelay::announce()
{
  printf("Id %d state announce, currentRoundNumber %d \n", (int)m_myID, currentRoundNumber);
        
  if(onlyIamFree && currentRoundNumber == 0)
  { 
    stateData.targetAgentId = agent_considered;
    stateData.SentData = SStateData::TASK_ASSIGNED;
    SendMessage(stateData.SentData, 0);
    printf("Only one relay free \n");
    
  }
  else if( currentRoundNumber == numberOfRounds+1)
  { 
    
    stateData.SentData = SStateData::TASK_ASSIGNED;
    SendMessage(stateData.SentData, 0);
    
  }
  else if( currentRoundNumber <= numberOfRounds)
  { 
    if(stateData.children.size() > 0)
    { 
      printf("children size %d \n",stateData.children.size());
      for(auto &child:stateData.children)
      {
         printf("ID %d child %d \n",(int)m_myID,(int)child);
      }
      stateData.SentData = SStateData::ROUND_TASK_ASSIGNED_TO_CHILDREN;
      SendMessage(stateData.SentData, 0);
    }

    if(stateData.parents.size() > 0)
    { 
      for(auto &parent:stateData.parents)
      {
         printf("ID %d parent %d \n",(int)m_myID,(int)parent);
      }
      stateData.SentData = SStateData::ROUND_COST_TO_PARENTS;
      SendMessage(stateData.SentData, 0);
    }
    
    stateData.RoundState = SStateData::STATE_DECIDE;
  }
}

void FootbotRelay::Rounds(uint8_t currentRoundState)
{
  switch(currentRoundState) {

      case SStateData::STATE_INITIAL: 
      { 
        
        printf("Id %d state initial\n", (int)m_myID);
        
        map<uint8_t,uint8_t> roundRelayMap;
        
        idsChosenbyParents.clear();
        
         if(currentRoundNumber > 1)
        {
          std::rotate(roundVector.rbegin(), roundVector.rbegin() + 1, roundVector.rend());
          for(auto &r : roundVector)
          {
             printf("round Val %d \n",r);
          }

          roundAgentIdToConsider.clear();
          roundAgentIdToConsider = agentIdToConsider;
          
          for(auto &rAg:roundAgentIdToConsider)
          {
            printf("Agent ids after initialising %d\n",(int)rAg);
          }

         int id = (int)relayIdMap[m_myID];
         
         //roundRelayMap.emplace(roundVector[id],rId);

         if(initialRound)
         {  
            for(auto &rId : relayIdList)
            {
              int index  = relayIdMap[rId];
              roundRelayMap.emplace(roundVector[index],rId);
            }

            if(roundVector[id] != 0)
            { 
              int i = roundVector[id]-1;
              stateData.parents.push_back(roundRelayMap[i]);
            }

            if(roundVector[id] != stateData.NUMBER_OF_RELAY-1)
            {
              int i = roundVector[id]+1;
              stateData.children.push_back(roundRelayMap[i]);
            }
         } 
         else
         {  
            int id = tempRelayMap[m_myID];
            
            for(auto &rId : tempRelayMap)
            {
              int index  = rId.second;
              roundRelayMap.emplace(roundVector[index],rId.first);
            }

            if(roundVector[id] != 0)
            { 
              int i = roundVector[id]-1;
              stateData.parents.push_back(roundRelayMap[i]);
            }

            if(roundVector[id] != freeRelayIds.size())
            {
              int i = roundVector[id]+1;
              stateData.children.push_back(roundRelayMap[i]);
            } 
         }
        }
        stateData.RoundState = SStateData::STATE_ASSIGN;
        break;
      }
      case SStateData::STATE_ASSIGN: 
      { 
        printf("Id %d state assign \n", (int)m_myID);

        if(stateData.parents.size() > 0)
        {
          printf("parent message size %d parents size %d \n", stateData.ParentMessages.size(),stateData.parents.size());
        }

        
        if(stateData.ParentMessages.size() == stateData.parents.size())
        {
          int indexAg;
          tie(indexAg , localCost)  = assignAgent(roundAgentIdToConsider);
          agent_considered = roundAgentIdToConsider[indexAg];
          
          if(find(agentIdList.begin(),agentIdList.end(),agent_considered) == agentIdList.end() || localCost == 5000)
          {
            stateData.SentData = SStateData::STOP_ROUND;
            SendMessage(stateData.SentData,0);
            reinitiliseRoundVariables();

            onlyIamFree = true;
          }
          printf("id %d agent chosen %d local cost %f\n",(int)m_myID,(int)agent_considered,localCost);
          announce();
        }
        
       
        break;
      }
      
      case SStateData::STATE_DECIDE:
      { 
        printf("Id %d state decide\n", (int)m_myID);
        if((stateData.ParentMessages.size()+stateData.ChildrenMessages.size()) == (stateData.parents.size()+stateData.children.size()))
        {
           stateData.children.clear();
           stateData.parents.clear();

           stateData.ParentMessages.clear();
           stateData.ChildrenMessages.clear();

           printf("cleared parents and children in decide state \n");

           float currGlobalCost = 0.0;
           currGlobalCost = localCost;

           for(auto &c:costMap)
           {
              currGlobalCost = currGlobalCost + c.second;
           }
           printf("current cost %f, agent id chosen %d, current round number %d  \n", currGlobalCost, (int)agent_considered, currentRoundNumber);
           
           if(globalCost > currGlobalCost)
           {
              globalCost = currGlobalCost;
              stateData.targetAgentId = agent_considered;
           }
           printf("global cost %f, id %d \n", globalCost, (uint8_t)stateData.targetAgentId);

           currentRoundNumber = currentRoundNumber + 1;

           printf("currentRoundNumber %d , number of rounds %d \n",currentRoundNumber, numberOfRounds);

           if(currentRoundNumber <= numberOfRounds)
          { 
            stateData.children.clear();
            stateData.parents.clear();
            stateData.RoundState = SStateData::STATE_INITIAL;
          }
          else if(currentRoundNumber == numberOfRounds+1)
          { 
            //IstargetAgentSet = true;
            announce();
          }
        }

        
        break;
      }

      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }
    }
}


void FootbotRelay::SetGoal() 
{ 
  if(onlyIamFree) 
  { 
    printf("Only I am free ");
    if(agentIdToConsider.size() > 1)
    {
      int indexAg;
      tie(indexAg , localCost)  = assignAgent(agentIdToConsider);
      agent_considered = agentIdToConsider[indexAg];
    }
    else if(agentIdToConsider.size() == 1)
    {
       agent_considered = agentIdToConsider[0];
    }
    
    announce();
    printf("id %d agent chosen %d local cost %f\n",(int)m_myID,(int)agent_considered,localCost);
    stateData.State = SStateData::STATE_SEARCH;
  }
  else 
  {   

    if(currentRoundNumber == numberOfRounds+1)
    {
      stateData.State = SStateData::STATE_SEARCH;
      
      reinitiliseRoundVariables();
      printf("Target set %d\n",(int)stateData.targetAgentId);
      printf("cleared parents and children in set goal \n");
    }
    else
    {
      Rounds(stateData.RoundState);
    }

    printf("ID %d Round number %d , round state %d \n", (int)m_myID, currentRoundNumber, stateData.RoundState);
      
  }
  
  
}

void FootbotRelay::reinitiliseRoundVariables()
{
  currentRoundNumber = 0;
  numberOfRounds = 0;

  freeRelayIds.clear();
  receivedMessageFromRelayVector.clear();
  stateData.parents.clear();
  stateData.children.clear();

  stateData.ParentMessages.clear();
  stateData.ChildrenMessages.clear();

  costMap.clear();

  globalCost = 5000;
}

float FootbotRelay::checkUpperBounds(float num, float max)
{
  num = max-num < 0.5 ? num-1.0 : 0.0;
  return num;
}

float FootbotRelay::checkLowerBounds(float num)
{ 
  num = num-0.0 < 0.5 ? num+1.0 : 0.0;
  return num;
}

float FootbotRelay::checkBounds(float num, float max)
{  
   float test = (checkUpperBounds(num,max) || checkLowerBounds(num));
   if(test)
   {
      num = test;
   }
   return num;
}

void 
FootbotRelay::Search(bool goalCalculated)
{ 
  Position currentPos;
  currentPos.x = m_navClient->currentPosition().GetX();
  currentPos.y = m_navClient->currentPosition().GetY();

  if(stateData.IsAgentDetected)
  { 
    printf("AGENT DETECTEDDDDDDDD");
    for(auto &i : stateData.InRange)
    {
       printf("detected agent %d\n",(int)i);
    }
    stateData.State = SStateData::STATE_DATA_GATHERING;
    stateData.IsGoalSet = false;
    stateData.IsAgentDetected = false;
  }
  else
  {
    if(m_navClient->state() == target_state)
    { 
      printf("Agent not found but reached target\n");
      timeInSearch = timeInSearch + 1;
      if(timeInSearch > 35)
      {
         taskDone = true;
         stateData.State = SStateData::STATE_HOME;
         agentIdToConsider.erase(std::remove(agentIdToConsider.begin(), agentIdToConsider.end(), stateData.targetAgentId), agentIdToConsider.end()); 
      }
    }
    else
    {
      
      printf("Search State Target not reached \n");
    }
    Position target = agentMap[stateData.targetAgentId]->current_location;
    
    if(goalCalculated)
    {   
        target.x = checkBounds(target.x,size_x);
        target.y = checkBounds(target.y,size_x);
        //m_navClient->start();
        printf("Set a new target Position every 20 seconds\n");
        
        CVector3 targetPos(target.x,target.y,0.0);
        m_navClient->setTargetPosition(targetPos);
        printf("TARGET id and loc %d %f %f \n", (int)stateData.targetAgentId, targetPos[0], targetPos[1]);
    } 
  }
}

void
FootbotRelay::DataGather(uint8_t agentId)
{
  //send the acceptance to collect data after checking the available data size.
  // move towards agent's current location Potential field -- agent should attract until it sends data and once data is sent agent should repel

// collect data from agents assigned alone  

if(stateData.targetAgentId == agentId)
{
  
 
    if(not stateData.IsGoalSet)
    {
      //unordered_set<uint8_t>::iterator itr = stateData.InRange.begin();
      stateData.SentData = SStateData::RELAY_SENDING_ACCEPTANCE_TO_AGENT;
      SendMessage(stateData.SentData,agentId);

      printf("Agent Location set as target\n");
      CVector3 targetPos(agentMap[agentId]->current_location.x,agentMap[agentId]->current_location.y,0.0);
      m_navClient->setTargetPosition(targetPos);
      printf("TARGET %f %f \n", targetPos[0], targetPos[1]);
      stateData.IsGoalSet = true;
      timeInDataGather = 0;
     }
      
      
    
    //if(m_navClient->state() == target_state)
    else if(agentMap[agentId]->IsDataReceived)
    {  
       printf("Reached Agent Location\n");
       //stateData.collected_data_size = stateData.collected_data_size + agentData.data_available;
       //agentData.data_available = 0;
       
       stateData.IsGoalSet = false;
       stateData.IsAgentDetected = false;
       
       // if no agent in range 
      agentMap[agentId]->IsDataReceived = false;
      printf("Switching to home state from data gather state \n");
      stateData.State = SStateData::STATE_HOME;
      agentMap[agentId]->data_available = 0;
      //taskDone = true;
    }

    else if(stateData.IsGoalSet)
    {
      timeInDataGather = timeInDataGather + 1;
      if(timeInDataGather > 10)
      {
        stateData.IsGoalSet = false;
        DataGather(stateData.targetAgentId);
      }
    }

  }
  
}

uint8_t
FootbotRelay::findNearestBaseStation()
{  
   Position current;
   current.x = m_navClient->currentPosition().GetX();
   current.y = m_navClient->currentPosition().GetY();

   float dist = 500;
   uint8_t index = 0;
   for(auto &pos:stateData.base_station)
   {  
      float t = util.findDistance(&current, &pos.second);
      
      if(dist > t)
      {
         dist = t;
         index = pos.first;
      }
   }
  
  return index;
  
}

void
FootbotRelay::Return()
{   
    // initialising Return State -> set target position as position of one of the base station
   /* if(stateData.IsAgentDetected)
    {
      stateData.State = SStateData::STATE_DATA_GATHERING;
      stateData.IsAgentDetected = false;
    }*/

    if(not stateData.IsGoalSet && not stateData.IsDataSentToBaseStation) // If the goal is not set and data is not transferred to base station
    {

      idToSendData = findNearestBaseStation();
    
      printf("Target set to base location\n");
      CVector3 targetPos(stateData.base_station[idToSendData].x,stateData.base_station[idToSendData].y,0.0);
      m_navClient->setTargetPosition(targetPos);
      printf("TARGET %f %f \n", targetPos[0], targetPos[1]);
      stateData.IsGoalSet = true;
      stateData.MovingToBaseStation = true;

      
       stateData.InRange.clear();

       initialiseRound = true;
    }  
    
    else
    {
      Position currentL;
      currentL.x = m_navClient->currentPosition().GetX();
      currentL.y = m_navClient->currentPosition().GetY();
      


      double dist = util.findDistance(&stateData.base_station[idToSendData], &currentL);
      distanceToBaseStation = dist;
      printf("Distance: %f \n", dist);
      printf("Navigation state of agent %d\n", m_navClient->state());

      if( receivedMessageFromRelay.size() > 0)
      { 

        if(initialiseRound)
        { 
          onlyIamFree = false;
          roundAgentIdToConsider.clear();
          roundAgentIdToConsider = agentIdToConsider;
          initialRound = false;

          tempRelayMap.clear();
          roundVector.clear();
          receivedMessageFromRelay.push_back(m_myID);
          //freeRelayIds.insert(m_myID);


          printf("initialise ---- > task donee and more free relays\n");
          numberOfRounds = receivedMessageFromRelay.size();
          printf("numberOfRounds %d \n", numberOfRounds);
          currentRoundNumber = 1;
          globalCost = 5000;
          
          sort(receivedMessageFromRelayVector.begin(),receivedMessageFromRelayVector.end());
          sort(receivedMessageFromRelay.begin(),receivedMessageFromRelay.end());

          map <uint8_t,uint8_t> roundRelayMap;

          for(int i= 0; i < receivedMessageFromRelay.size(); i++)
          {
            roundVector.push_back(i);
            tempRelayMap.emplace(receivedMessageFromRelay[i],i);
            roundRelayMap.emplace(i,receivedMessageFromRelay[i]);

            printf("tempRelayMap %d %d \n", receivedMessageFromRelay[i], i);
            printf("roundRelayMap %d %d \n", i , receivedMessageFromRelay[i]);
          }
          
          for(int i=0; i< roundVector.size(); i++)
          {
            printf("Round  vector %d \n", roundVector[i]);
          }
          stateData.parents.clear();
          stateData.children.clear();
          
          int id = (int)tempRelayMap[m_myID];

          printf("cleared parents and children in initialise Return state \n");
         
          if(roundVector[id] != 0)
          { 
            int i = roundVector[id]-1;
            printf("[%d] roundVector Value %d parent %d \n", (int)m_myID, i, roundRelayMap[i]);
            stateData.parents.push_back(roundRelayMap[i]);
          }

          if(roundVector[id] != receivedMessageFromRelay.size()-1)
          {
            int i = roundVector[id]+1;
            printf("[%d] roundVector Value %d children %d \n", (int)m_myID, i, roundRelayMap[i]);
            stateData.children.push_back(roundRelayMap[i]);
          }

          stateData.RoundState = SStateData::STATE_INITIAL;
          Rounds(stateData.RoundState);
          initialiseRound  = false;

          stateData.SentData = SStateData::START_ROUND;
          SendMessage(stateData.SentData,0);
          counterInitialise = 0;
          
          costMap.clear();
        }
        else if(receivedMessageFromRelay.size()-1 == freeRelayIds.size() && currentRoundNumber <= numberOfRounds)
        {
          Rounds(stateData.RoundState);
        }
        else
        {
            counterInitialise = counterInitialise + 1;
            if(counterInitialise == 10)
            {
              // Remove the relays from receivedMessageFromRelay which are not present in freeRelayIds
              for(auto &f:freeRelayIds)
              {
                if(find(receivedMessageFromRelay.begin(),receivedMessageFromRelay.end(),f) == receivedMessageFromRelay.end())
                { 
                  receivedMessageFromRelay.erase(std::remove(receivedMessageFromRelay.begin(), receivedMessageFromRelay.end(), f), receivedMessageFromRelay.end());
                }
              }
            }
        }
      }
      
      if(stateData.IsGoalSet && dist <= 0.40) // if an object is stationary(Base Station) at that target location, it doesn't go beyond 0.28.0.35
      {
           // send data once reached base Station
          printf("Reached Base Station\n");
          stateData.SentData = SStateData::RELAY_TO_BASESTATION;
          SendMessage(stateData.SentData,idToSendData);
          m_navClient->stop();
          stateData.IsGoalSet = false;
          stateData.MovingToBaseStation = false;

      }
        
      
      else if(stateData.IsDataSentToBaseStation)
      {  
         timeForACycle.data_file << m_Steps << "\n";
         timeForACycle.data_file << m_Steps << ",";
         
         receivedMessageFromRelay.clear();
         printf("Data Sent to Base Station\n");
         m_navClient->start();
        
         stateData.State = SStateData::STATE_NOGOAL;
         stateData.IsDataSentToBaseStation = false;
         stateData.visited_baseStation = true;
    
        
         if(freeRelayIds.size() == 0)
          { 
            printf("task donee and only one free relay\n");
            onlyIamFree = true;
            freeRelayIds.clear();
            
           stopR = false;
           roundStopId.clear();
           currentRoundNumber = 0;
           numberOfRounds = 0;

          }

           // copying visited to toVisit list
          vector<uint8_t>::iterator itr; 
          for(auto &agVisited:visitedAgent)
          { 
            itr = find(agentIdToConsider.begin(), agentIdToConsider.end(), agVisited);

            if(itr == agentIdToConsider.end())
              agentIdToConsider.push_back(agVisited);
          }

          visitedAgent.clear();  
      }
    }
    
}


void 
FootbotRelay::Home() 
{ 
  stateData.targetAgentId = 0;
  

  stateData.IsGoalSet = false;
  stateData.IsAgentDetected = false;
  onlyIamFree = false;

  int numberOfAgentToVisit =  int(ceil(stateData.NUMBER_OF_AGENT/(stateData.NUMBER_OF_RELAY*1.0)));
  roundAgentIdToConsider = agentIdToConsider;
  if(visitedAgent.size() == numberOfAgentToVisit)
  { 
    printf("visited %d agents, number to visit %d \n",visitedAgent.size(), numberOfAgentToVisit);
    stateData.State = SStateData::STATE_RETURN_TO_BASESTATION;
  }
  else if(taskDone && freeRelayIds.size() > 0)
  { 
    printf("task donee and more free relays\n");
    numberOfRounds = (uint8_t)freeRelayIds.size();
    currentRoundNumber = 1;
    stateData.RoundState = SStateData::STATE_INITIAL;
    stateData.State = SStateData::STATE_NOGOAL;
    freeRelayIds.clear();
  }
  else if(taskDone)
  { 
    printf("task donee and only one free relay\n");
    onlyIamFree = true;
    
    stateData.State = SStateData::STATE_NOGOAL;
  }
  taskDone = false;
}

std::tuple<uint8_t, float> 
FootbotRelay::assignAgent(vector <uint8_t> agentIds) 
{ 
   Position relayPos;
   vector<Position> agentPositionList;
   vector<uint64_t> timeLastDataCollectedList;

   printf("before sending data to clustering class \n");

   for(int i= 0 ; i < agentIds.size() ; i++)
   {
      printf("agent ids to consider %d \n", (uint8_t)agentIds[i]);
   }

   for(auto &Id : agentIds)
   {
    if((m_Steps-agentMap[Id]->transmitted_data_time > 50) || agentMap[Id]->transmitted_data_time == 0) 
    {
      agentPositionList.push_back(agentMap[Id]->current_location);
      timeLastDataCollectedList.push_back(m_Steps-agentMap[Id]->transmitted_data_time);
    }
    
   }

   relayPos.x = m_navClient->currentPosition().GetX();
   relayPos.y = m_navClient->currentPosition().GetY();
   
   if(timeLastDataCollectedList.size() > 0)
   {
      return util.findBestAgent(relayPos, agentPositionList, timeLastDataCollectedList);
   }
   else
   {
     return std::make_tuple (0,5000);
   }

}

/***
*  initially begin with number of rounds equal to number of relays. Later number of rounds is equal to number of free relays
* 
***/
void 
FootbotRelay::ControlStep() 
{ 
  m_Steps+=1;

  printf("Id %d current state %d parents size %d children size %d \n",(int)m_myID,stateData.State, stateData.parents.size(), stateData.children.size());

  
  if(m_navClient->currentPosition().GetX() > 0  &&  m_navClient->currentPosition().GetY() > 0)
  {  
    
  if(m_Steps % 5 == 0)
  {
    relayPositions.data_file << m_navClient->currentPosition().GetX() << "," << m_navClient->currentPosition().GetY() << "\n";
  }
    
    bool goalCalculated = UpdateState();
    printf("target now %d\n ",(int)stateData.targetAgentId);
    printf("goal calculated %s \n", goalCalculated ? "true" : "false");
    //SetState();

  switch(stateData.State) 
  {

      case SStateData::STATE_NOGOAL: {
         printf("IN no goal set state \n");
         SetGoal();
         break;
      }
      case SStateData::STATE_SEARCH: {
         printf("IN SEARCH STATE \n");
         Search(goalCalculated);
         break;
      }
    
      case SStateData::STATE_DATA_GATHERING: {
         printf("IN DataGather STATE \n");

         if(agentMap[stateData.InRange[0]]->data_available > 0)
         {
            DataGather(stateData.InRange[0]); 
         
            printf("visiting agent \n");
            //agentMap[stateData.InRange[0]]->data_available = 0;

         }  
         else
         { 
           stateData.InRange.erase(stateData.InRange.begin());
           taskDone = true;
           stateData.State = SStateData::STATE_HOME;
         }
         break;
      }
      case SStateData::STATE_RETURN_TO_BASESTATION: {
         printf("IN Return STATE \n");
         Return(); // Once Reached BS send collected data
         break;
      }

      case SStateData::STATE_HOME: {
         printf("going home state \n");
         Home();
         break;
      }
      default: {
         LOGERR << "We can't be here, there's a bug!" << std::endl;
      }

     }
   }

  

 
  
  printf(" Relay Position %f %f \n",m_navClient->currentPosition().GetX(), m_navClient->currentPosition().GetY());
    
  
  

    /********  Receive messages from mission agents ********/
  
  TMessageList message_from_agents;
  m_pcWifiSensor->GetReceivedMessages_Extern(message_from_agents);
  for(TMessageList::iterator it = message_from_agents.begin(); it!=message_from_agents.end();it++)
  {   
    //send_message_to_relay = true;
    printf("Received %lu bytes to incoming buffer\n", it->Payload.size());

    vector<char> check_message = it->Payload;
    printf("Identifier of received message [extern], sender  %d %d \n",int(check_message[0]),int(check_message[1]));
    uint8_t senderId =  uint8_t(check_message[1]);
    
    std::vector<uint8_t>::iterator itr = find(agentIdList.begin(), agentIdList.end(), senderId);
    
    if(itr != agentIdList.end())
    {
      ParseMessage(it->Payload, uint8_t(check_message[0]),uint8_t(check_message[1]));
    }
    string sender = it->Sender;
    /*if((char)check_message[0] == 'a')
    {   
      //agents.push_back(atoi((it->Sender).c_str));
      meeting_data_file << sender << " ";
      //parse_agent_message(it->Payload);
      //uint8_t received_from = stoi(it->Sender)+1;
      received_message_file << m_Steps << "," << check_message[0] << "," << sender << "," << (m_navClient->currentPosition().GetX()) << "," << (m_navClient->currentPosition().GetY()) << "\n";
      //received_message_file << m_Steps << "," << check_message[0] << "," << it->Sender << "\n";
    }

    else if((char)check_message[0] == 'b')
    {
      meeting_data_file << "d_"+(sender) << " ";
      //data_exchange_agents.push_back(atoi((it->Sender).c_str));
      received_message_file << m_Steps << "," << check_message[0] << "," << sender << "," << (m_navClient->currentPosition().GetX()) << "," << (m_navClient->currentPosition().GetY()) << "\n";
    } */
  }


  /********  Receive messages from relays ********/

  TMessageList message_from_relays;
  m_pcWifiSensor->GetReceivedMessages_Local(message_from_relays);
  for(TMessageList::iterator it = message_from_relays.begin(); it!=message_from_relays.end();it++)
  {   
    //send_message_to_relay = true;
    printf("Received %lu bytes to incoming buffer from relay \n", it->Payload.size());

    vector<char> check_message = it->Payload;
    printf("Identifier of received message [local], sender  %d %d \n",int(check_message[0]),int(check_message[1]));

    uint8_t senderId =  uint8_t(check_message[1]); 
    bool checkState = false;

    if(uint8_t(check_message[1]) == 4 && stateData.State == 3)
      checkState = true;

    if(senderId != m_myID)
    {
      if(uint8_t(check_message[1]) != 4)
      {
        ParseMessage(it->Payload, uint8_t(check_message[0]),uint8_t(check_message[1]));
      }
      else if(checkState)
      {
         ParseMessage(it->Payload, uint8_t(check_message[0]),uint8_t(check_message[1]));
      }
    }
    
    
  }

  m_pcLEDs->SetAllColors(CColor::MAGENTA);
  m_navClient->setTime(getTime());
  m_navClient->update();
  
  ////cout <<"MyId: " << m_myID << "target: " << counter << endl;
  
}




void 
FootbotRelay::Destroy() 
{
  DEBUG_CONTROLLER("FootbotRelay::Destroy (  )\n");
}

/**************************************/

bool 
FootbotRelay::IsControllerFinished() const 
{
  return false;
}


std::string
FootbotRelay::getTimeStr()
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


/// returns time in milliseconds
  UInt64 
FootbotRelay::getTime()
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
  return m_Steps * CPhysicsEngine::GetSimulationClockTick() * 1000;
#endif
}


  
REGISTER_CONTROLLER(FootbotRelay, "footbot_relay_controller")