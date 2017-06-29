#ifndef _FOOTBOTBASESTATION_H_
#define _FOOTBOTBASESTATION_H_

#include <iostream>
#include <fstream>
#include <map>
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
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_encoder_sensor.h>
#include <navigation/client/nav_client.h>

using namespace argos;
using namespace std;

#include <argos2/common/utility/datatypes/datatypes.h>


class FootbotBaseStation: public CCI_Controller
{
  private:
    UInt32 RandomSeed;
    
    UInt64 m_Steps;
    CARGoSRandom::CRNG* m_randomGen;
    UInt64 m_sendPackets;
    UInt8 m_myID;
    RVONavClient *m_navClient;

    CCI_WiFiSensor* m_pcWifiSensor;
    CCI_WiFiActuator* m_pcWifiActuator;
    
    CCI_FootBotLedsActuator* m_pcLEDs;

    struct Agent_message_data
    {
        uint8_t id;
        uint32_t data_size;
        uint64_t time_data_sent;
    };
    uint32_t test_data_size;
    struct Agent_message_data agent_data;

    ofstream agent_data_file;
    string filename;
    void parse_relay_message(std::vector<char> &v);
    
  public:

    /* Class constructor. */
    FootbotBaseStation();

    /* Class destructor. */
    virtual ~FootbotBaseStation() {
    }

    virtual void Init(TConfigurationNode& t_tree);

    virtual void ControlStep();
    virtual void Destroy();
    virtual bool IsControllerFinished() const;

    UInt64 getTime();
    //void broadcastStringPacket(const CVector3& position);
    
    
};

#endif