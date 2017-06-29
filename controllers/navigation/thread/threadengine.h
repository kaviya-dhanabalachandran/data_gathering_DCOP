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

#include <pthread.h>
#include <stdio.h>

/**
 * @brief Thread POSIX based class
 *
 * @details Allows to inherent classes to instance and run POSIX based threads
 *
 * @author Roberto Magán Carrión, rmagan@ugr.es, rmagan@idsia.ch
 * @date 05/11/2014
 *
 */
class ThreadEngine {
public:

	/**
	 * Init the thread engine
	 *
	 * @return true, is everything was ok or false if something was wrong.
	 */
	bool startInternalThread() {

		int status = pthread_create(&_thread, NULL, internalThreadEntryFunc, this);

		return (status == 0);
	}

	/**
	 * Will not return until the internal thread has exited
	 */
	void waitForInternalThreadToExit() {
		(void) pthread_join(_thread, NULL);
	}

	ThreadEngine();
	virtual ~ThreadEngine();

protected:

	/**
	 * To implement by the subclass for running the thread
	 */
	virtual void internalThreadEntry() = 0;

private:

	/**
	 * This will call to the internalThreadEntry() method implemented in the subclass
	 *
	 * @param This
	 */
	static void * internalThreadEntryFunc(void * This) {

		((ThreadEngine *) This)->internalThreadEntry();
		return NULL;
	}

	/**
	 * Thread
	 */
	pthread_t _thread;
};

