/*
 * protectedmutex.h
 *
 *  Created on: 19/11/2014
 *      Author: roberto
 */

#ifndef CONTROLLERS_FOOTBOT_AN_INCLUDES_LCM_PROTECTEDMUTEX_H_
#define CONTROLLERS_FOOTBOT_AN_INCLUDES_LCM_PROTECTEDMUTEX_H_

#include <pthread.h>

class ProtectedMutex {

	pthread_mutex_t & m;
public:
	explicit ProtectedMutex(pthread_mutex_t & _m) :
			m(_m) {
		pthread_mutex_lock(&m);
	}
	~ProtectedMutex() {
		pthread_mutex_unlock(&m);
	}

};

#endif /* CONTROLLERS_FOOTBOT_AN_INCLUDES_LCM_PROTECTEDMUTEX_H_ */
