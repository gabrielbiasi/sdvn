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

Define_Module(SdvnPing);

void SdvnPing::initialize(int stage) {
    if (stage == 0) {
        if(hasGUI()) {
            getModuleByPath("SdvnScenario.manager")->setDisplayString("p=1200,100;i=block/network2;is=s");
            getModuleByPath("SdvnScenario.obstacles")->setDisplayString("p=1400,100;i=misc/town;is=s");
            getModuleByPath("SdvnScenario.annotations")->setDisplayString("p=1200,200;i=msg/paperclip;is=s");
            getModuleByPath("SdvnScenario.world")->setDisplayString("p=1400,200;i=misc/globe;is=s");
            getModuleByPath("SdvnScenario.connectionManager")->setDisplayString("p=1200,300;i=abstract/multicast;is=s");
        }

        attackMode = par("attackMode").longValue();
        attackerRate = par("attackerRate").doubleValue();
        startTime = par("startTime").doubleValue();
        duration = par("duration").doubleValue();

        toSwitch = findGate("toSwitch");
        fromSwitch = findGate("fromSwitch");

        vehicleId = getParentModule()->getIndex();
        packetInterval = par("packetInterval").doubleValue();
        warmUp = par("warmUp").doubleValue();

        msgSent = 0;
        msgRecv = 0;

        vLatency.setName("RTT");
        vMsgRate.setName("Packet Rate");

        messagesEvent = nullptr;
        attackEvent = nullptr;

        messagesEvent = new cMessage("Send", 0);
        scheduleAt(simTime() + warmUp + uniform(0, 1), messagesEvent);

        attacking = false;
        attacker = attackMode != 0 && uniform(0,1) < attackerRate;
        if (attacker) {
            attackEvent = new cMessage("Attack", 1);
            if (simTime() < startTime) {
                scheduleAt(startTime, attackEvent);
            } else if (simTime() > startTime && simTime() < startTime + duration) {
                scheduleAt(simTime()+0.0001, attackEvent);
            } else {
                delete attackEvent;
            }
        }
    }
}

void SdvnPing::handleMessage(cMessage *msg) {
    if(msg->isSelfMessage()) {
           AppMessage* packet;
           std::stringstream ss;
           cModule* module;
           int destinationId, repeat;

           switch(msg->getKind()) {
               case 0:
                   // Send Message
                   destinationId = vehicleId - 1;
                   if(destinationId < 0) return;

                   // Check if vehicle is in the simulation
                   ss << "SdvnScenario.vehicle[" << destinationId << "].appl";
                   module = getModuleByPath(ss.str().c_str());
                   if(!module) return;

                   // checks if perform DDoS attack or not
                   repeat = (attacker && attacking && attackMode == A_DDOS) ? 25 : 1;

                   for(int i = 0; i < repeat; i++) {
                       packet = new AppMessage();
                       packet->setSourceAddress(vehicleId);
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

                       msgSent++;
                   }
                   recordV();
                   schedule();
                   break;

               case 1: // Start or Stop Attack!
                   if(!attacking) { // Starting attacks
                       attacking = true;

                       // Stop the message event for black hole and overflow attack
                       if(attackMode == A_BLACK_HOLE || attackMode == A_OVERFLOW) cancelEvent(messagesEvent);
                       scheduleAt(startTime + duration + 0.1, attackEvent);

                   } else { // Stopping attacks
                       attacking = false;
                       delete attackEvent;
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

void SdvnPing::recordV() {
    double v = (double) (msgRecv/(double)msgSent)*100;
    vMsgRate.record(v);
}

void SdvnPing::finish() {
    cSimpleModule::finish();

    cancelAndDelete(messagesEvent);

    recordScalar("Messages Sent", msgSent);
    recordScalar("Messages Received", msgRecv);
}
