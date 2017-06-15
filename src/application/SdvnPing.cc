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

#include "SdvnPing.h"
#include "messages/AppMessage_m.h"

using namespace std;
using Veins::TraCIScenarioManagerAccess;

Define_Module(SdvnPing);

int SdvnPing::victimId = NO_VEHICLE;

void SdvnPing::initialize(int stage) {
    if (stage == 0) {

        attackMode = par("attackMode").longValue();
        attackerRate = par("attackerRate").doubleValue();
        startTime = par("startTime").doubleValue();
        duration = par("duration").doubleValue();
        attackSize = par("attackSize").longValue();

        toSwitch = findGate("toSwitch");
        fromSwitch = findGate("fromSwitch");

        vehicleId = getParentModule()->getIndex();
        packetInterval = par("packetInterval").doubleValue();
        warmUp = par("warmUp").doubleValue();

        msgSent = 0;
        msgRecv = 0;

        vLatency.setName("RTT");
        vMsgRate.setName("Packet Rate");

        messagesEvent = new cMessage("Send", 0);
        scheduleAt(simTime() + warmUp + uniform(0, 1), messagesEvent);

        attacking = false;
        attacker = attackMode != 0 && uniform(0,1) < attackerRate;
        if (attacker) {
            WATCH(victimId);
            attackEvent = new cMessage("Attack", 1);
            if (simTime() < startTime) { // Vehicle shows up before attack
                scheduleAt(startTime, attackEvent);
            } else if (simTime() > startTime && simTime() < startTime + duration) { // Vehicle shows up during the attack
                scheduleAt(simTime()+0.0001, attackEvent);
            } else { // Vehicle shows up after the attack
                delete attackEvent;
            }
        } else {
            attackEvent = nullptr;
        }
        WATCH(attacker);
    }
}

void SdvnPing::handleMessage(cMessage *msg) {
    if(msg->isSelfMessage()) {
           AppMessage* packet;
           std::stringstream ss;
           int sourceId, destinationId, repeat;

           switch(msg->getKind()) {
               case 0:
                   // Sending Message
                   // Checks if perform a flooding attack or not
                   if(attacker && attacking && attackMode == A_DDOS) {
                       sourceId = -171;
                       destinationId = victimId;
                       repeat = attackSize;
                   } else {
                       sourceId = vehicleId;
                       destinationId = getRandomVehicle();
                       repeat = 1;
                   }

                   for(int i = 0; i < repeat; i++) {
                       packet = new AppMessage();
                       packet->setSourceAddress(sourceId);
                       packet->setDestinationAddress(destinationId);
                       packet->setTTL(64);
                       packet->setId(msgSent);

                       // Payload
                       ss.str("");
                       ss << "PING Hello!";
                       packet->setPayload(ss.str().c_str());

                       // Sending message
                       EV_INFO << "Vehicle [" << vehicleId << "] Sending PING #" << msgSent <<" to Vehicle [" << destinationId << "]\n";
                       send(packet, toSwitch);

                       if(!attacking) msgSent++; // Do not register while attacking
                   }
                   if(!attacking) recordV(); // Do not register while attacking
                   schedule();
                   break;

               case 1: // Start or Stop Attack!
                   if(!attacking) { // Starting attacks
                       attacking = true;

                       if(victimId == NO_VEHICLE) // Initialize if needed
                           victimId = getRandomVehicle();

                       // Stop the message event for black hole and overflow attack
                       if(attackMode == A_BLACK_HOLE || attackMode == A_OVERFLOW) cancelEvent(messagesEvent);
                       scheduleAt(startTime + duration + 0.1, attackEvent);

                   } else { // Stopping attacks
                       attacking = false;
                       if(!messagesEvent->isScheduled()) schedule();
                   }

                   break;
           }
       } else if (msg->getArrivalGateId() == fromSwitch) {
           // Message received from switch.
           AppMessage* packet = (AppMessage*) msg;
           int packetId = packet->getId();
           int senderId = packet->getSourceAddress();
           std::string s = std::string(packet->getPayload());
           if(s.find("PING") != std::string::npos) {
               // PING received from a vehicle. Sending PONG.
               EV_INFO << "Vehicle [" << vehicleId << "] PING #"<< packetId <<" received from Vehicle [" << senderId << "]\n";
               EV_INFO << "Vehicle [" << vehicleId << "] Sending PONG...\n";

               packet->setSourceAddress(vehicleId);
               packet->setDestinationAddress(senderId);
               packet->setTTL(64);

                s.replace(0, 4, "PONG");
                packet->setPayload(s.c_str());

                send(packet, toSwitch);

          } else if(s.find("PONG") != std::string::npos) {
              // PONG received from a vehicle. Registering statistics.
              simtime_t latency = simTime()-packet->getTimestamp();
              vLatency.record(latency);
              EV_INFO << "Vehicle [" << vehicleId << "] PONG #"<< packetId <<" received from Vehicle [" << senderId << "]\n";
              EV_INFO << "Vehicle [" << vehicleId << "] RTT: " << latency << ".\n";
              msgRecv++;
              delete packet;
          } else {
              EV_INFO << "Something wrong!!";
              ASSERT(false);
          }
       }
}

void SdvnPing::schedule() {
    scheduleAt(simTime() + packetInterval, messagesEvent);
}

int SdvnPing::getRandomVehicle() {
    auto map = TraCIScenarioManagerAccess().get()->getManagedHosts();
    auto iter = map.begin();
    std::advance(iter, (int) uniform(0, map.size()));
    return iter->second->getIndex();
}

void SdvnPing::recordV() {
    double v = (double) (msgRecv/(double)msgSent)*100;
    vMsgRate.record(v);
}

void SdvnPing::finish() {
    cSimpleModule::finish();

    if(messagesEvent->isScheduled()) {
        cancelAndDelete(messagesEvent);
    } else {
        delete messagesEvent;
    }

    if(attacker) {
        recordScalar("Victim ID", victimId);
        if(attackEvent->isScheduled()) {
            cancelAndDelete(attackEvent);
        } else {
            delete attackEvent;
        }
    }

    recordScalar("Messages Sent", msgSent);
    recordScalar("Messages Received", msgRecv);
}
