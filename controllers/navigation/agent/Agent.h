/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _AGENT_H_
#define _AGENT_H_

#include <argos2/common/control_interface/ci_controller.h>
#include <argos2/common/utility/math/angles.h>
#include <argos2/common/utility/math/vector2.h>
#include <argos2/common/utility/math/vector3.h>
#include <list>
#include <algorithm>

using namespace argos;

//mobility

#define DEFAULT_LINEAR_SPEED_CONTINUOUS false
#define DEFAULT_MAX_ANGULAR_SPEED 10 //20 //cm/s
#define MIN_ANGULAR_SPEED 0.02
#define MAX_SPEED 0.3 //0.2
#define DEFAULT_ROTATION_TAU 0.5
#define DEFAULT_HORIZON 3
// configuration

#define DEFAULT_OPTIMAL_SPEED 0.2
#define DEFAULT_SOCIAL_SENSING_RATIO 1
#define DEFAULT_FOOTBOT_SOCIAL_RADIUS 20 //40
#define DEFAULT_HUMAN_SOCIAL_RADIUS 40
#define DEFAULT_OBSTACLE_SOCIAL_RADIUS 20
#define RADIUS 0.085036758 //Footbot's radius
#define HUMAN_RADIUS 0.05//0.25//m

#define NO_COLLISION -1
//Agent Types

enum {FOOTBOT=0,OBSTACLE=2,HUMAN=1};
#define NUMBER_OF_AGENT_TYPES 3

#define ANGULAR_SPEED_DOMINATE

class Agent
{
public:

// Mobility
Real rotationTau; 
bool linearSpeedIsContinuos; 
Real socialMargin;
Real socialRadius[NUMBER_OF_AGENT_TYPES];
Real safetyMargin;
Real radius;
Real ratioOfSocialRadiusForSensing;
Real optimalSpeed;
Real maxRotationSpeed;
Real maxSpeed;
CVector2 position;
CVector2 velocity;
CRadians angularSpeed;
CRadians angle;


CVector2 desideredVelocity;
Real desideredSpeed;
CRadians desideredAngle;
CVector2 targetPosition;

Real leftWheelSpeed;
Real rightWheelSpeed;

Real axisLength;

//Navigation

Real horizon;

Real leftWheelTargetSpeed;
Real rightWheelTargetSpeed;

  

virtual void updateDesideredVelocity()=0;
virtual void updateVelocity();
virtual void Init (TConfigurationNode& t_tree);
virtual void clearObstacles()=0;

virtual void addObstacleAtPoint(CVector2 p,CVector2 v,Real r)=0;
virtual void addObstacleAtPoint(CVector2 p,Real r)=0;

Agent(){};
~Agent(){};
 
protected:
Real marginForObstacleAtDistance(Real distance,Real obstacleRadius,Real safetyMargin,Real socialMargin);
CVector2 relativePositionOfObstacleAt(CVector2 &position,Real obstacleRadius, Real &distance); 

private:

virtual void setup()=0;

};


#endif
