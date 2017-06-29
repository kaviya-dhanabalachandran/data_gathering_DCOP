/*
 * Copyright (C) 2014, IDSIA (Institute Dalle Molle for Artificial Intelligence), http://http://www.idsia.ch/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "lcmhandler.h"

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <navigation/lcm/poselcm/pose_list_t.hpp>


LCMHandler::LCMHandler(const char * url, const string &channel)
{
  // Create a new LCM instance
  lcm = lcm_create(url);

  // Set LCM URL
  lcmURL = url;

  // Set LCM channel
  lcmChannel = channel;

  // FIXME: better outside?
  if (isLCMReady()) {
    subscribeToChannel(channel);
  }

  pthread_mutex_init(&mutex, NULL);

}


  UInt64 
LCMHandler::getTime()
{
  struct timeval timestamp;
  gettimeofday(&timestamp, NULL);

  // cout << timestamp.tv_sec << " " << timestamp.tv_usec/1000 << endl;    
  UInt64 ms1 = (UInt64) timestamp.tv_sec;
  ms1*=1000;

  UInt64 ms2 = (UInt64) timestamp.tv_usec;
  ms2/=1000;

  //cout << timestamp.tv_sec << " " << timestamp.tv_usec/1000 << endl;    
  //cout << "getTime from lllcmhandler " << (ms1+ms2) << endl;
  //UInt64 ms = timestamp.tv_sec*1000 + timestamp.tv_usec/1000;
  return ms1+ms2;
}


bool LCMHandler::isLCMReady()
{
  if (!lcm.good()) {
    printf("LCM is not ready :(");
    return false;
  } else {
    printf("LCM is ready :)");
    return true;
  }
}


void
LCMHandler::subscribeToChannel(const string & channel)
{
  lcm.subscribe(channel, &LCMHandler::handleMessage, &(*this));
}


int
LCMHandler::getAvailableMessagesTimeout(int timeout) 
{
  return lcm.handleTimeout(timeout);
}

int
LCMHandler::getAvailableMessages()
{
  return lcm.handle();
}

void
LCMHandler::publishMessage(poselcm::pose_list_t *lcmNodeList)
{
  lcm.publish(lcmChannel, lcmNodeList);
}



  void 
LCMHandler::handleMessage(const lcm::ReceiveBuffer* rbuf, 
			  const std::string& chan, const poselcm::pose_list_t* msg) 
{
  //printf("gotLCMMsg \n");
  //fflush(stdout);

  // Node to insert or update
  Node node;
  // Node positions
  CVector3 positions;
  // Node orientation
  CQuaternion orientations;
  // To set the timestamp
  //struct timeval tp;

  //printf("gotLCMMsg %d\n", msg->n);
  for (int i = 0; i < msg->n; i++) 
  {
    const poselcm::pose_t &pose = msg->poses[i];

    // ID
    node.setId(pose.robotid);

    // Timestamp
    //gettimeofday(&tp, NULL);
    //UInt64 ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    node.setTimestamp(getTime());

    // Position
    positions.Set(pose.position[0], pose.position[1], pose.position[2]);
    positions/=1000.0f;
    node.setPosition(positions);

    // Orientation
    orientations = CQuaternion(pose.orientation[0]/10000.0f, 
			       pose.orientation[1]/10000.0f, 
			       pose.orientation[2]/10000.0f, 
			       pose.orientation[3]/10000.0f);
    node.setOrientation(orientations);

    //Velocity
    node.setVelocity(pose.velocity/1000.0f);

    // Add or Update to the list
    addOrUpdateNode(node);

  }
  //printf("\n");

  // Print new list status
  //printNodeListElements();

}

bool LCMHandler::existNode(const UInt8 id) {

  ProtectedMutex m(mutex);
  bool exist = listAllNodes.end() != listAllNodes.find(id);
  return exist;
}

void LCMHandler::addOrUpdateNode(Node node) {

  // If does not exist, add it!
  if (!existNode(node.getId())) 
  {
    ProtectedMutex m(mutex);
    listAllNodes.insert(pair<UInt8, Node>(node.getId(), node));
  } else {
    ProtectedMutex m(mutex);
    //Remove
    listAllNodes.erase(node.getId());
    //Insert
    listAllNodes.insert(pair<UInt8, Node>(node.getId(), node));
  }

}

map<UInt8, Node> LCMHandler::retrieveNodeList() {
  ProtectedMutex m(mutex);
  map<UInt8, Node> nodeList = listAllNodes;

  return nodeList;

}

void 
LCMHandler::setLCMMessageFromNodeList(map<UInt8, Node> nodeList, poselcm::pose_list_t* lcmNodeList) 
{

  //Timestamp
  struct timeval tp;
  gettimeofday(&tp, NULL);
  UInt64 ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
  lcmNodeList->timestamp = ms;

  // Size of the list
  lcmNodeList->n = nodeList.size();

  // LCM Node
  poselcm::pose_t lcmNode;

  // Node
  Node currentNode;

  //Make the LCM node list
  for (map<UInt8, Node>::iterator it = nodeList.begin(); it != nodeList.end(); it++) {

    // Current Node
    currentNode = (it->second);

    // LCM Node
    // ID
    lcmNode.robotid = currentNode.getId();
    // Position
    lcmNode.position[0] = currentNode.getPosition().GetX()*1000;
    lcmNode.position[1] = currentNode.getPosition().GetY()*1000;
    lcmNode.position[2] = currentNode.getPosition().GetZ()*1000;
    // Orientation
    lcmNode.orientation[0] = currentNode.getOrientation().GetW()*10000.;
    lcmNode.orientation[1] = currentNode.getOrientation().GetX()*10000.;
    lcmNode.orientation[2] = currentNode.getOrientation().GetY()*10000.;
    lcmNode.orientation[3] = currentNode.getOrientation().GetZ()*10000.;
    // Velocity
    lcmNode.velocity = currentNode.getVelocity();
    // Add the node to the list
    lcmNodeList->poses.push_back(lcmNode);
  }
}

void 
LCMHandler::publish(const map<UInt8, Node> nodeList) 
{
  //Making the message
  poselcm::pose_list_t lcmNodeList;
  setLCMMessageFromNodeList(nodeList, &lcmNodeList);
  //Publish
  publishMessage(&lcmNodeList);
}

/** overrides the timestamp */
void 
LCMHandler::publish(const map<UInt8, Node> nodeList, UInt64 timestamp) 
{
  //Making the message
  poselcm::pose_list_t lcmNodeList;
  setLCMMessageFromNodeList(nodeList, &lcmNodeList);
  /// set timestamp
  lcmNodeList.timestamp = timestamp;
  //Publish
  publishMessage(&lcmNodeList);
}

void 
LCMHandler::printNodeListElements() 
{

  ProtectedMutex m(mutex);
  printf("Node list with %d elements.\n", 
	 (int) listAllNodes.size());
  for (map<UInt8, Node>::iterator it = listAllNodes.begin(); it != listAllNodes.end(); it++) {

    printf("ID %d - timestamp %llu\n", (int) it->first, 
	   (it->second).getTimestamp());
    printf("ID %d - location (%f,%f,%f)\n", it->first, (it->second).getPosition().GetX(), (it->second).getPosition().GetY(), (it->second).getPosition().GetZ());
    printf("ID %d - orientation (%f,%f,%f,%f)\n", it->first, (it->second).getOrientation().GetW(), (it->second).getOrientation().GetX(), (it->second).getOrientation().GetY(),
	   (it->second).getOrientation().GetZ());
    printf("ID %d - velocity %d\n", it->first, (it->second).getVelocity());

  }

}

  Node 
LCMHandler::getNodeById(const UInt8 id) 
{
  //printf("GetNodeById %d\n", id);
  //fflush(stdout);
  ProtectedMutex m(mutex);
  Node node = listAllNodes.at(id);

  return node;
}
