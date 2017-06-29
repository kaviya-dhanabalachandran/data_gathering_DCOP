/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "ORCAAgent.h"




void ORCAAgent::clearObstacles()
{
  if(useEffectiveCenter)
    {
      D=axisLength*0.5;
      RVO::Vector2 delta=RVO::Vector2((float)Cos(angle),(float)Sin(angle))*D;
      _RVOAgent->position_=RVO::Vector2((float)position.GetX(),(float)position.GetY())+delta;
      _RVOAgent->radius_=radius+D;
      _RVOAgent->velocity_=RVO::Vector2((float)(velocity.GetX()-Sin(angle)*D*angularSpeed.GetValue()),(float)(velocity.GetY()+Cos(angle)*D*angularSpeed.GetValue()));
      _RVOAgent->maxSpeed_=optimalSpeed/sqrt(1+RVO::sqr(0.5*axisLength/D));
    }
  else
    {
      _RVOAgent->radius_=radius;
      _RVOAgent->velocity_=RVO::Vector2((float)velocity.GetX(),(float)velocity.GetY());
      _RVOAgent->position_=RVO::Vector2((float)position.GetX(),(float)position.GetY());
      _RVOAgent->maxSpeed_=optimalSpeed;
    }
  _RVOAgent->timeHorizon_=timeHorizon;
  _RVOAgent->neighborDist_=2*horizon;
       
  for(uint i=0;i<_RVOAgent->obstacleNeighbors_.size();i++)
    {
      delete _RVOAgent->obstacleNeighbors_[i].second;
    }
    
  rangeSq = (horizon*2)*(horizon*2);
    
  for(uint i=0;i<agentNeighbors.size();i++)
    {
      delete agentNeighbors[i];
    }
    
  agentNeighbors.clear();
    
  _RVOAgent->agentNeighbors_.clear();    
  _RVOAgent->obstacleNeighbors_.clear();    
}

void ORCAAgent::setup()
{

  //  printf("ORCA Target %.2f %.2f, Position %.2f %.2f\n",targetPosition.GetX(),targetPosition.GetY(),_RVOAgent->position_.x(),_RVOAgent->position_.y());
  RVO::Vector2 t=RVO::Vector2(targetPosition.GetX(),targetPosition.GetY())-_RVOAgent->position_;
  _RVOAgent->prefVelocity_=t*_RVOAgent->maxSpeed_/abs(t);
  // printf("pref v %.2f %.2f\n",_RVOAgent->prefVelocity_.x(),_RVOAgent->prefVelocity_.y());

}

void ORCAAgent::updateDesideredVelocity()
 {
   setup();
   _RVOAgent->computeNewVelocity();
   CVector2 newVelocity(_RVOAgent->newVelocity_.x(),_RVOAgent->newVelocity_.y());
   desideredAngle=(newVelocity.Angle()-angle).SignedNormalize();
   desideredSpeed=abs(_RVOAgent->newVelocity_);
   //  printf("des v v %.2f %.2f\n",_RVOAgent->newVelocity_.x(),_RVOAgent->newVelocity_.y());
 }

void ORCAAgent::updateVelocity()
 {
   if(useEffectiveCenter && desideredSpeed!=0)
     {
       leftWheelTargetSpeed=desideredSpeed*(Cos(desideredAngle)-axisLength*0.5/D*Sin(desideredAngle));
       rightWheelTargetSpeed=desideredSpeed*(Cos(desideredAngle)+axisLength*0.5/D*Sin(desideredAngle));
     }
   else
     {
       Agent::updateVelocity();
     } 

   //printf("target  %.2f %.2f\n",leftWheelTargetSpeed,rightWheelTargetSpeed);
}

void ORCAAgent::Init(TConfigurationNode& t_tree)
 {
   Agent::Init(t_tree);

   useEffectiveCenter=false;//true;
   timeHorizon=10.0;

   std::string text;
   Real d;
 if(NodeExists(t_tree,"timeHorizon"))
    {
      GetNodeText(GetNode(t_tree, "timeHorizon"), text);
      sscanf(text.c_str(),"%f",&d);
      timeHorizon=fmax(0.1,d);
    }
 if(NodeExists(t_tree,"local_navigation"))
    {
      GetNodeAttributeOrDefault(GetNode(t_tree,"local_navigation"),"NH",useEffectiveCenter,useEffectiveCenter);
    }



   _RVOAgent=new RVO::Agent(NULL);
   _RVOAgent->maxNeighbors_=1000; 
   _RVOAgent->timeStep_=TIME_STEP;
   _RVOAgent->timeHorizon_=timeHorizon;


   printf("ORCA Agent INIT: time Horizon %.2f, NH %d\n",timeHorizon,useEffectiveCenter);

   //_RVOAgent->timeHorizonObst_=timeHorizonStatic;
}


//TODO add non penetration check!

void ORCAAgent::addObstacleAtPoint(CVector2 p,CVector2 v,Real r)
{
  RVO::Agent *a=new RVO::Agent(NULL);
  a->velocity_=RVO::Vector2((float)v.GetX(),(float)v.GetY());


  
  a->position_=RVO::Vector2((float)p.GetX(),(float)p.GetY());
  Real distance;
  CVector2 relativePosition=relativePositionOfObstacleAt(p,r,distance);
  a->radius_=r+fmin(distance-r-_RVOAgent->radius_-0.001,marginForObstacleAtDistance(distance,r,safetyMargin,socialMargin));

  // printf("Obstacle (%.3f,%.3f) (%.3f,%.3f) %.5f -> radius %.3f\n",p.GetX(),p.GetY(),v.GetX(),v.GetY(),r,a->radius_);

  a->prefVelocity_=a->velocity_;
  agentNeighbors.push_back(a);
  _RVOAgent->insertAgentNeighbor(a, rangeSq);
}


void ORCAAgent::addObstacleAtPoint(CVector2 p,Real r)
{
  addObstacleAtPoint(p,CVector2(0,0),r);
}
