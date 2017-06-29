/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _HRVOAGENT_H_
#define _HRVOAGENT_H_

#include "Agent.h"
#include "HRVO/HRVO.h"
#include "HRVO/Agent.h"


using namespace argos;

class HRVOAgent : public Agent
{
 public:

 virtual void updateDesideredVelocity();
 virtual void updateVelocity();
 virtual void Init(TConfigurationNode& t_tree);
 virtual void clearObstacles();

  virtual void addObstacleAtPoint(CVector2 p,CVector2 v,Real r);
  virtual void addObstacleAtPoint(CVector2 p,Real r);

 HRVOAgent(){};
 ~HRVOAgent(){};

 private:
 uint agentIndex;
 Real rangeSq;
 virtual void setup();
  HRVO::Agent *_HRVOAgent;
};

#endif
