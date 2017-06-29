#ifndef _LCMTHREAD_H_
#define _LCMTHREAD_H_

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

#include <stdio.h>
#include <string>
#include <lcm/lcm-cpp.hpp>

#include "../thread/threadengine.h"

using namespace std;
using namespace lcm;

/**
 * @brief LCM thread
 *
 * @details For getting the received messages in a separated thread.
 *
 * @author Roberto Magán Carrión, rmagan@ugr.es, rmagan@idsia.ch
 * @date 05/11/2014
 *
 */


class LCMHandler;
class LCMThread: public ThreadEngine
{
 public:

  /**
   * To set the LCM connections parameters
   *
   * @param url char, multicast URL
   * @param channel string, channel for hearing messages
   */
  void setLCMEngine(const char * url, const string &channel);
  /**
   * Is listening for messages on an specific channel that was previously established.
   */
  void internalThreadEntry();
  /* Getters & Setters */
  LCMHandler* getLcmHandler();

  void setLcmHandler(LCMHandler* lcmHandler);
 private:

  /**
   * LCM handler
   */
  LCMHandler * lcmHandler;

};
#endif
