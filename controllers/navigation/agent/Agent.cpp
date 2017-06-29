/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "Agent.h"

void Agent::updateVelocity() {

	CRadians delta = desideredAngle.SignedNormalize();

	Real targetAngularSpeed = (1.0 / rotationTau) * delta.GetValue() * 0.5 * axisLength;
	Real targetLinearSpeed = 0;

	if (targetAngularSpeed > maxRotationSpeed) {
		targetAngularSpeed = maxRotationSpeed;
	} else if (targetAngularSpeed < -maxRotationSpeed) {
		targetAngularSpeed = -maxRotationSpeed;
	} else {
		if (linearSpeedIsContinuos) {
			targetLinearSpeed = desideredSpeed * (1 - fabs(targetAngularSpeed) / maxRotationSpeed);
		} else {
			targetLinearSpeed = desideredSpeed;
		}
	}

#ifdef ANGULAR_SPEED_DOMINATE
	if (fabs(targetLinearSpeed) + fabs(targetAngularSpeed) > maxSpeed) {
		if (targetLinearSpeed < 0) {

			targetLinearSpeed = -maxSpeed + fabs(targetAngularSpeed);
		} else {
			targetLinearSpeed = maxSpeed - fabs(targetAngularSpeed);
		}
	}
#endif

	leftWheelTargetSpeed = targetLinearSpeed - targetAngularSpeed;
	rightWheelTargetSpeed = targetLinearSpeed + targetAngularSpeed;
	//printf("%p TV L:%.3f, R:%.3f\n",this,leftWheelTargetSpeed,rightWheelTargetSpeed);
}

void Agent::Init(TConfigurationNode& t_tree) {
	maxSpeed = MAX_SPEED;
	leftWheelSpeed = 0.0;
	rightWheelSpeed = 0.0;
	linearSpeedIsContinuos = DEFAULT_LINEAR_SPEED_CONTINUOUS;
	maxRotationSpeed = DEFAULT_MAX_ANGULAR_SPEED;
	radius = RADIUS;
	optimalSpeed = DEFAULT_OPTIMAL_SPEED;
	rotationTau = DEFAULT_ROTATION_TAU;

	socialRadius[HUMAN] = DEFAULT_HUMAN_SOCIAL_RADIUS;
	socialRadius[FOOTBOT] = DEFAULT_FOOTBOT_SOCIAL_RADIUS;
	socialRadius[OBSTACLE] = DEFAULT_OBSTACLE_SOCIAL_RADIUS;
	safetyMargin = 0.1;
	socialMargin = 0.1;
	ratioOfSocialRadiusForSensing = DEFAULT_SOCIAL_SENSING_RATIO;

	horizon = DEFAULT_HORIZON;

	if (NodeExists(t_tree, "mobility")) {
		TConfigurationNode node = GetNode(t_tree, "mobility");

		if (node.HasAttribute("continuous"))
			GetNodeAttribute(node, "continuous", linearSpeedIsContinuos);
		if (node.HasAttribute("rotation_max_speed"))
			GetNodeAttribute(node, "rotation_max_speed", maxRotationSpeed);

	}

	std::string text;

	if (NodeExists(t_tree, "horizon")) {
		GetNodeText(GetNode(t_tree, "horizon"), text);
		sscanf(text.c_str(), "%f", &horizon);
	}

	if (NodeExists(t_tree, "optimalSpeed")) {
		GetNodeText(GetNode(t_tree, "optimalSpeed"), text);
		sscanf(text.c_str(), "%f", &optimalSpeed);
	}

	if (NodeExists(t_tree, "maxSpeed")) {
		GetNodeText(GetNode(t_tree, "maxSpeed"), text);
		sscanf(text.c_str(), "%f", &maxSpeed);
	}

	if (NodeExists(t_tree, "safetyMargin")) {
		GetNodeText(GetNode(t_tree, "safetyMargin"), text);
		sscanf(text.c_str(), "%f", &safetyMargin);
		safetyMargin = fmax(0.01, safetyMargin);
	}

	if (NodeExists(t_tree, "socialMargin")) {
		GetNodeText(GetNode(t_tree, "socialMargin"), text);
		sscanf(text.c_str(), "%f", &socialMargin);
		//safetyMargin=fmax(safetyMargin,socialMargin);
	}

	if (NodeExists(t_tree, "socialRadius")) {
		TConfigurationNode node = GetNode(t_tree, "socialRadius");
		std::string text = "";
		if (node.HasAttribute("footbot")) {
			GetNodeAttribute(node, "foobot", text);
			Real d;
			scanf(text.c_str(), "%f", &d);
			socialRadius[FOOTBOT] = d;
		}
		if (node.HasAttribute("human")) {
			GetNodeAttribute(node, "human", text);
			Real d;
			scanf(text.c_str(), "%f", &d);
			socialRadius[HUMAN] = d;
		}
		if (node.HasAttribute("obstacle")) {
			GetNodeAttribute(node, "obstacle", text);
			Real d;
			scanf(text.c_str(), "%f", &d);
			socialRadius[OBSTACLE] = d;
		}
	}
	if (NodeExists(t_tree, "rotationTau")) {
		GetNodeText(GetNode(t_tree, "rotationTau"), text);
		sscanf(text.c_str(), "%f", &rotationTau);
		rotationTau = fmax(0.1, rotationTau);
	}

	printf("Agent Init: sM %.2f, SM %.2f, horizon %.2f optimal speed %.2f max speed %.2f rotation tau %.2f max rotation speed %.2f\n",
	       safetyMargin,
	       socialMargin,
	       horizon,
	       optimalSpeed,
	       maxSpeed,
	       rotationTau,
	       maxRotationSpeed);
	//DEBUG_CONTROLLER("Mobility Settings: maxRotationSpeed %.2f, linearSpeedIsContinuos %d \r\n",maxRotationSpeed,linearSpeedIsContinuos);

}

CVector2 Agent::relativePositionOfObstacleAt(CVector2 &obstaclePosition, Real obstacleRadius, Real &distance) {
	CVector2 relativePosition = obstaclePosition - position;
	distance = relativePosition.Length();
	Real minDistance = (radius + obstacleRadius) + 0.002;
	if (distance < minDistance) {
		//too near,  cannot penetrate in an obstacle (footbot or human)
		relativePosition = relativePosition / distance * minDistance;
		obstaclePosition = position + relativePosition;
		distance = minDistance;
	}
	return relativePosition;
}

Real Agent::marginForObstacleAtDistance(Real distance, Real obstacleRadius, Real safetyMargin, Real socialMargin) {

	//  printf("margin: dist %.2f or %.2f sM %.2f SM %.2f\n",distance,obstacleRadius,safetyMargin,socialMargin);

	double farMargin = 1;
	double distanceToBeSeparated = safetyMargin + radius + obstacleRadius;
	double distanceToBeFar = farMargin + radius + obstacleRadius;

	if (distance < distanceToBeSeparated) {
		return safetyMargin;
	} else if (distance > distanceToBeFar) {
		return socialMargin;
	} else {
		return (socialMargin - safetyMargin) / (distanceToBeFar - distanceToBeSeparated) * (distance - distanceToBeSeparated) + safetyMargin;
	}
}
