#ifndef _LCMHANDLER_H_
#define _LCMHANDLER_H_

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

#include <argos2/common/utility/datatypes/datatypes.h>
#include <navigation/beans/node.h>
#include "protectedmutex.h"
#include <stdio.h>
#include <map>
#include <lcm/lcm-cpp.hpp>

#include <pthread.h>

using namespace std;
using namespace argos;

/**
 * @brief LCM engine - (see more information at https://code.google.com/p/lcm/)
 *
 * @details Basically this class provides the necessary methods for using LCM
 * based engine.
 *
 * @author Roberto Magán Carrión, rmagan@ugr.es, rmagan@idsia.ch
 * @date 05/11/2014
 *
 */


namespace poselcm
{
class pose_list_t;
}

class LCMHandler
{

public:

	/**
	 * Default LCM constructor
	 */
	LCMHandler();

	/**
	 * LCM constructor.
	 *
	 * @param url char, the LCM URL
	 * @param channel string, the LCM channel
	 */
	LCMHandler(const char * url, const string &channel);
	/**
	 * This method is in charge of to handle the message obtained from a specific LCM channel.
	 *
	 * @param rbuf
	 * @param chan string, the specified channel.
	 * @param msg poselcm::pose_list_t, the message obtained from the channel (automatically created by LCM engine from a previous specification)
	 */
	void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
			     const poselcm::pose_list_t* msg);

	/**
	 * Publish an specific list of nodes
	 *
	 * @param nodeList, list of nodes to publish
	 */
	void publish(const map<UInt8, Node> nodeList);

	void publish(const map<UInt8, Node> nodeList, UInt64 timestamp);
	/**
	 * Checks if LCM is ready.
	 *
	 * @return true if yes or false it not
	 */
	bool isLCMReady();
	/**
	 * To subscribe to a specific LCM channel to get messages from it.
	 *
	 * @param channel string, name of the channel.
	 */
	void subscribeToChannel(const string & channel);
	/**
	 * Checks if there are messages from the channel for a pre-specified time.
	 *
	 * @param timeout
	 * @return status int, > 0 there were messages, == 0 timeout expired and < 0 an error occurred
	 */
	int getAvailableMessagesTimeout(int timeout);
	/**
	 * Blocking call for waiting messages
	 * @return status int, > 0 there were messages, == 0 timeout expired and < 0 an error occurred
	 */
	int getAvailableMessages();
	/**
	 * Publish a specific list of nodes in LCM type format
	 *
	 * @param lcmNodeList, list of nodes in the LCM type format
	 */
	void publishMessage(poselcm::pose_list_t *lcmNodeList);
	/**
	 * Retrieve the node list previously obtained from the LCM channel.
	 *
	 * @return the node list
	 */
	map<UInt8, Node> retrieveNodeList();

	/**
	 * Checks if the node exists in the list of nodes.
	 *
	 * @param id UIn8, node ID
	 * @return true, if node exists, false if does not exists.
	 */
	bool existNode(const UInt8 id);

	/**
	 * Add a new node if is not already in the list or will update it if is already in the list.
	 *
	 * @param node, the Node to add or update
	 */
	void addOrUpdateNode(Node node);

	/**
	 * Gets the Node by its ID
	 *
	 * @param id UInt8, Node ID
	 * @return the Node
	 */
	Node getNodeById(const UInt8 id);

	/**
	 * Convert the list of Nodes to the corresponding LCM list format (see lcmtypes). This method
	 * is used to conform a LCM message for publishing it into a determined LCM channel.
	 *
	 * @param listAllNodes, a map of Nodes
	 * @param lcmNodeList, a list of LCM Nodes
	 */
	void setLCMMessageFromNodeList(map<UInt8, Node> listAllNodes,
				       poselcm::pose_list_t* lcmNodeList);

	/**
	 * Print all Node elements in the list
	 */
	void printNodeListElements();

	/* Getters & setters */

	const string& getLcmChannel() const
	{
		return lcmChannel;
	}

	void setLcmChannel(const string& lcmChannel) {
		this->lcmChannel = lcmChannel;
	}

	const char* getLcmUrl() const {
		return lcmURL;
	}

	void setLcmUrl(const char* lcmUrl) {
		lcmURL = lcmUrl;
	}

private:

	static UInt64 getTime();
	/**
	 * Contains the list of all nodes retrieved from the tracking system
	 */
	map<UInt8, Node> listAllNodes;

	/**
	 * Contains the list of all nodes that will be publish to a determined channel
	 */
	map<UInt8, Node> listAllNodesToPublish;

	/**
	 * LCM URL
	 */
	const char * lcmURL;

	/**
	 * LCM channel
	 */
	string lcmChannel;

	/**
	 * LCM instance
	 */
	lcm::LCM lcm;

	/**
	 * Mutex to control the access to the listAllNodes
	 */
	pthread_mutex_t mutex;

};
#endif
