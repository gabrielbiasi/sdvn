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

#include "Attacker.h"

Define_Module(Attacker);

void Attacker::initialize() {
    victims = vector<long>();
    numVictims = par("numVictims").longValue();
    victimWarmUp = par("victimWarmUp").doubleValue();

    attackerRate = par("attackerRate").doubleValue();
    attackScale = par("attackScale").doubleValue();

    startTime = par("startTime").doubleValue();
    duration = par("duration").doubleValue();

    checkInterval = par("checkInterval").doubleValue();
    check = new cMessage("Attacker Check", 123);

    if(numVictims > 0)
        scheduleAt(startTime + uniform(0,1), check);

}

void Attacker::handleMessage(cMessage *msg) {
    if(msg->isSelfMessage()) {
        switch(msg->getKind()) {
        case 123:
            if(simTime() >= startTime+(duration*victims.size())) {
                if(victims.size() < numVictims) {
                    selectNewVictim();
                } else {
                    stopAttacks();
                    return;
                }
            } else {
                checkAttackers();
            }
            scheduleAt(simTime() + checkInterval, check);
            break;
        }

    } else {
        error("Attacker receiving message from another module.");
    }
}

void Attacker::checkAttackers() {
    SdvnPing* appl;
    long counter = 0;
    vector<cModule*> non_attackers = vector<cModule*>();

    // Checking
    auto vehicles = TraCIScenarioManagerAccess().get()->getManagedHosts();
    auto it = vehicles.begin();
    while(it != vehicles.end()) {
        appl = dynamic_cast<SdvnPing*>(it->second->getSubmodule("appl"));
        if(appl->attacking) {
            counter++;
        } else {
            non_attackers.push_back(it->second);
        }
        it++;
    }

    if(counter >= attackerRate*vehicles.size()) return;
    counter = attackerRate*vehicles.size() - counter;

    // Restauring
    while(counter > 0) {
        auto iter = non_attackers.begin();
        advance(iter, intuniform(0, non_attackers.size()-1));
        appl = dynamic_cast<SdvnPing*>((*iter)->getSubmodule("appl"));
        if((*iter)->getIndex() != getVictimId()) {
            non_attackers.erase(iter);
            appl->attacking = true;
            counter--;
        }
    }
}

void Attacker::stopAttacks() {
    SdvnPing* appl;
    auto vehicles = TraCIScenarioManagerAccess().get()->getManagedHosts();
    auto it = vehicles.begin();
    while(it != vehicles.end()) {
        appl = dynamic_cast<SdvnPing*>(it->second->getSubmodule("appl"));
        appl->attacking = false;
        it++;
    }
}

void Attacker::selectNewVictim() {
    long vehicleId;
    auto vehicles = TraCIScenarioManagerAccess().get()->getManagedHosts();
    if(victims.size() >= numVictims)
        error("Victim requested when there is no attack sheduled");

    while(true) {
        auto iter = vehicles.begin();
        advance(iter, intuniform(0, vehicles.size()-1));
        SdvnPing* appl = dynamic_cast<SdvnPing*>(iter->second->getSubmodule("appl"));
        SdvnMobility* mobility = dynamic_cast<SdvnMobility*>(iter->second->getSubmodule("veinsmobility"));
        vehicleId = iter->second->getIndex();
        if(mobility->getTotalTime() >= victimWarmUp // old enough
                && mobility->getTotalTime() < 2*victimWarmUp // fresh enough
                && !alreadyVictim(vehicleId) // not an repetive vehicle
                && !appl->attacking) { // not an attacker
            victims.push_back(vehicleId);
            return;
        }
    }
}

bool Attacker::alreadyVictim(long vehicleId) {
    for(long i : victims) {
        if(i == vehicleId)
            return true;
    }
    return false;
}

double Attacker::getAttackSize(){
    return attackScale;
}

long Attacker::getVictimId() {
    if(victims.size() > 0) {
        return victims.back();
    } else {
        error("No Victim selected yet.");
        return NO_VEHICLE;
    }
}

void Attacker::finish() {
    cSimpleModule::finish();
    if(check->isScheduled()) {
        cancelAndDelete(check);
    } else {
        delete check;
    }

    cout << "[ATTACKER] Victim choose: [";
    for(long i : victims) cout << i << ", ";
    cout << "]" << endl;
}
