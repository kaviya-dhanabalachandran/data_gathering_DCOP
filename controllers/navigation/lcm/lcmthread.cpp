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
#include "lcmthread.h"
#include "lcmhandler.h"



void
LCMThread::setLCMEngine(const char * url, const string &channel)
{
  lcmHandler = new LCMHandler(url, channel);
}

void
LCMThread::internalThreadEntry()
{
  while (true)
    {
      lcmHandler->getAvailableMessages();
    }
}

LCMHandler*
LCMThread::getLcmHandler()
{
  return lcmHandler;
}

void
LCMThread::setLcmHandler(LCMHandler* lcmHandler)
{
  this->lcmHandler = lcmHandler;
}



