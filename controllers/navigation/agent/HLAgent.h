/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef _HLAGENT_H_
#define _HLAGENT_H_

#include "Agent.h"

using namespace argos;

//trajectory planning

#define UNKNOWN -2
#define MAX_RESOLUTION 301

// configuration

#define DEFAULT_APERTURE CRadians::PI
#define DEFAULT_RESOLUTION 40
#define DEFAULT_TAU 0.1
#define MIN_TAU 0.1

typedef struct{
	Real centerDistance;
	Real radius;
	Real sensingMargin;
	Real agentSensingMargin;
	//Real socialMargin;
	//Real penetration;
	CVector2 dx;
	CVector2 va;
	CVector2 position;
	Real C;
	CRadians gamma,visibleAngle;
} AgentCache;


typedef std::list<AgentCache> obstacleList_t;
typedef obstacleList_t::iterator obstacleIterator_t;
typedef std::list<AgentCache> agentList_t;
typedef agentList_t::iterator agentIterator_t;


class HLAgent: public Agent
{

public:

// Mobility
// Navigation
 unsigned int resolution;
 
 Real effectiveHorizon;

 CRadians aperture;

 virtual void addObstacleAtPoint(CVector2 point,Real radius);
 virtual void addObstacleAtPoint(CVector2 point,CVector2 velocity,Real radius);

 virtual void clearObstacles();
 virtual void updateDesideredVelocity();
 virtual void updateVelocity();
 virtual void Init(TConfigurationNode& t_tree);
 
 Real *collisionMap();

HLAgent(){};
~HLAgent(){};

 std::vector<Real> getDistances();

Real tau;
Real eta;


private:

 virtual void setup();


 void prepareAgents();
 Real distanceCache[MAX_RESOLUTION];
 Real staticDistanceCache[MAX_RESOLUTION];
 CRadians angleResolution();
 void initDistanceCache();
 unsigned int indexOfRelativeAngle(CRadians relativeAngle);
 Real computeDistanceToCollisionAtRelativeAngle(CRadians relativeAngle, Real *staticCache);
 Real fearedDistanceToCollisionAtRelativeAngle(CRadians angle);
 Real distForAngle(AgentCache *agent,CRadians angle);
 Real staticDistForAngle(AgentCache *agent,CRadians angle);
 Real distanceToCollisionAtRelativeAngle(CRadians angle);

 AgentCache makeObstacleAtPoint(CVector2 p, CVector2 v, Real r);
 AgentCache makeObstacleAtPoint(CVector2 p, Real r);

 agentList_t nearAgents;
 obstacleList_t staticObstacles;

 void updateVelocityCartesian();
 void debugAgents();
 void debugStaticObstacles();

};


#endif
