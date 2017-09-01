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

#ifndef __SDVN_ATTACKER_H_
#define __SDVN_ATTACKER_H_

#include <omnetpp.h>
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "modules/SdvnMobility.h"
#include "application/SdvnPing.h"
#include "SdvnTypes.h"

using namespace omnetpp;
using std::cout;
using std::vector;
using std::advance;
using Veins::TraCIScenarioManagerAccess;

class Attacker : public cSimpleModule
{
  protected:
    long numVictims;
    double victimWarmUp;
    vector<long> victims;

    double attackerRate;
    double attackScale;

    double startTime;
    double duration;
    double checkInterval;

    cMessage* check;

    virtual void initialize();
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    void checkAttackers();
    bool alreadyVictim(long vehicleId);
    void selectNewVictim();
    void stopAttacks();

  public:
    long getVictimId();
    double getAttackSize();
};

#endif
