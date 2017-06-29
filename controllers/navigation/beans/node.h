#ifndef _BEANS_NODE_H_
#define _BEANS_NODE_H_

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

#include <argos2/common/utility/math/vector3.h>
#include <argos2/common/utility/math/quaternion.h>

using namespace argos;

/**
 * @brief Node class
 *
 * @details This class is mapping the information of a real robot about its location and orientation
 * provided by the tracking system.
 *
 * @author Roberto Magán Carrión, rmagan@ugr.es, rmagan@idsia.ch
 * @date 05/11/2014
 *
 */
class Node {

public:

	/* Getters & Setters */

	inline UInt8 getId() const {
		return id;
	}

	inline void setId(UInt8 id) {
		this->id = id;
	}

	inline const CQuaternion& getOrientation() const {
		return orientation;
	}

	inline void setOrientation(CQuaternion& orientation) {
		this->orientation = orientation;
	}

	inline const CVector3& getPosition() const {
		return position;
	}

	inline void setPosition(CVector3& position) {
		this->position = position;
	}

	UInt64 getTimestamp() const {
		return timestamp;
	}

	void setTimestamp(UInt64 timestamp) {
		this->timestamp = timestamp;
	}

	UInt16 getVelocity() const {
		return velocity;
	}

	void setVelocity(UInt16 velocity) {
		this->velocity = velocity;
	}

private:
	/**
	 * Node ID
	 */
	UInt8 id;

	/**
	 * Node location
	 */
	CVector3 position;

	/**
	 * Node orientation
	 */
	CQuaternion orientation;

	/**
	 * Creation timestamp
	 */
	UInt64 timestamp;

	/**
	 * Node velocity
	 */
	UInt16 velocity;

};
#endif
