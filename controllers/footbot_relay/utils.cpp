#include "utils.h"

/****
Step:1  Order the agent based on their priority
Step:2  Send the best agent
****/

Util::Util()
{

}





float Util::weightCalculation(uint64_t t)
{ 
  float d = (t/tau);
  float e = exp (-d);
  
  //printf("WEIGHTS %f \n", 1-(a*e));
  return (0.03+(a*e));
}



std::tuple<uint8_t, float> 
Util::findBestAgent(Position relayPos, vector<Position> agentPositionList, vector<uint64_t> timeLastDataCollected)
{   
	float weight = 5000;
	uint8_t agentId;

	for(int i=0 ; i < agentPositionList.size(); i++)
	{   
		float w = findDistance(&relayPos,&agentPositionList[i]) * weightCalculation(timeLastDataCollected[i]);
		printf("weight %f id %d timeLastDataCollected %ld , distance %f \n", w, i,timeLastDataCollected[i], findDistance(&relayPos,&agentPositionList[i]));
		
		if(weight > w)
		{
			weight = w;
			agentId  = (uint8_t)i;


		}
		
	}

	return std::make_tuple(agentId, weight);
}

