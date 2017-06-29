#ifndef _FOOTBOTNAV_H_
#define _FOOTBOTNAV_H_

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
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_encoder_sensor.h>
#include <navigation/client/nav_client.h>

using namespace argos;
using namespace std;

#include <argos2/common/utility/datatypes/datatypes.h>


class FootbotNavigator: public CCI_Controller
{
  private:

    CARGoSRandom::CRNG* RandomGen;
    UInt32 RandomSeed;
    std::string m_MyIdStr;
    UInt64 m_Steps;
    UInt8 m_myID;
    RVONavClient *m_navClient;
    
  public:

    /* Class constructor. */
    FootbotNavigator();

    /* Class destructor. */
    virtual ~FootbotNavigator() {
    }

    virtual void Init(TConfigurationNode& t_tree);

    virtual void ControlStep();
    virtual void Destroy();
    virtual bool IsControllerFinished() const;

    static std::string getTimeStr();
    UInt64 getTime();

};

#endif
