//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __SDVN_SDVNPING_H_
#define __SDVN_SDVNPING_H_

#include <omnetpp.h>
#include "SdvnTypes.h"

using namespace omnetpp;

class SdvnPing : public cSimpleModule
{
  protected:
    // Gates
    int toSwitch;
    int fromSwitch;

    long vehicleId;
    double packetInterval;
    double warmUp;

    int msgSent;
    int msgRecv;

    cOutVector vMsgRate;
    cOutVector vLatency;

    cMessage* messagesEvent;
    cMessage* attackEvent;

    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();
    void schedule();
    void recordV();

  public:

    // Attacks Settings
    bool attacker;
    bool attacking;
    int attackMode;
    double attackerRate;
    double startTime;
    double duration;

};

#endif
