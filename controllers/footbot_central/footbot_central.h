
#ifndef FOOTBOT_CENTRAL_H
#define FOOTBOT_CENTRAL_H

/*
 * Include some necessary headers.
 */
/* Definition of the CCI_Controller class. */
#include <argos2/common/control_interface/ci_controller.h>
/* Definition of the foot-bot wheel actuator */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_wheels_actuator.h>
/* Definition of the foot-bot LEDs actuator */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_leds_actuator.h>
/* Definition of the foot-bot proximity sensor */
#include <argos2/common/control_interface/swarmanoid/footbot/ci_footbot_proximity_sensor.h>

#include <argos2/common/control_interface/ci_wifi_sensor.h>
#include <argos2/common/control_interface/ci_wifi_actuator.h>
#include <argos2/simulator/space/space.h>
#include <argos2/common/utility/math/vector2.h>
#include <iostream>
#include <navigation/lcm/lcmthread.h>
#include <list>
#include <vector>
#include <sys/time.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;
using namespace std;

/*
 * A controller is simply an implementation of the CCI_Controller class.
 */
class CFootBotCentral: public CCI_Controller {

 public:

  std::map<UInt8, CVector3 > m_pPosition; 
  /* Class constructor. */
  CFootBotCentral();

  /* Class destructor. */
  virtual ~CFootBotCentral() {
  }

  /*
   * This function initializes the controller.
   * The 't_node' variable points to the <parameters> section in the XML file
   * in the <controllers><footbot_diffusion_controller> section.
   */
  virtual void Init(TConfigurationNode& t_node);

  /*
   * This function is called once every time step.
   * The length of the time step is set in the XML file.
   */
  virtual void ControlStep();

  /*
   * This function resets the controller to its state right after the Init().
   * It is called when you press the reset button in the GUI.
   * In this example controller there is no need for resetting anything, so
   * the function could have been omitted. It's here just for completeness.
   */
  virtual void Reset() {
  }

  /*
   * Called to cleanup what done by Init() when the experiment finishes.
   * In this example controller there is no need for clean anything up, so
   * the function could have been omitted. It's here just for completeness.
   */
  virtual void Destroy();

 private:

  CSpace& m_cSpace;

  /* LCM engine */
  LCMHandler * lcmHandler;

  int m_nsteps;
};

#endif
