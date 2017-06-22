#include "application/SdvnPing.h"

Define_Module(SdvnPing);

int SdvnPing::k = 0;
bool SdvnPing::first = false;

void SdvnPing::initialize(int stage) {
    char *token, buffer[100];
    if (stage == 0) {
        toSwitch = findGate("toSwitch");
        fromSwitch = findGate("fromSwitch");

        vehicleId = getParentModule()->getIndex();
        burstSize = par("burstSize").doubleValue();
        burstInterval = par("burstInterval").doubleValue();
        warmUp = par("warmUp").doubleValue();

        msgSent = 0;
        msgRecv = 0;

        vLatency.setName("RTT");
        vMsgRate.setName("Packet Rate");

        messagesEvent = new cMessage("Send", 0);
        scheduleAt(simTime() + warmUp + uniform(0, 1), messagesEvent);

        attackMode = par("attackMode").longValue();
        attackerRate = par("attackerRate").doubleValue();
        attacking = false;

        victims = vector<long>();
        strcpy(buffer, par("victims").stringValue());
        token = strtok(buffer, ",");
        while(token != NULL) {
            victims.push_back(atol(token));
            token = strtok(NULL, ",");
        }
        size = victims.size();

        auto me = victims.begin();
        while(me != victims.end() && *me != vehicleId) me++;
        attacker = attackMode != 0 && uniform(0,1) < attackerRate && me == victims.end();
        if (attacker) {
            attackEvent = new cMessage("Attack", 1);

            // supporting multiple attacks
            attackSize = vector<long>();
            strcpy(buffer, par("attackSize").stringValue());
            token = strtok(buffer, ",");
            while(token != NULL) {
                attackSize.push_back(atol(token));
                token = strtok(NULL, ",");
            }
            ASSERT(attackSize.size() == size);

            duration = vector<long>();
            strcpy(buffer, par("duration").stringValue());
            token = strtok(buffer, ",");
            while(token != NULL) {
                duration.push_back(atol(token));
                token = strtok(NULL, ",");
            }
            ASSERT(duration.size() == size);

            startTime = vector<long>();
            strcpy(buffer, par("startTime").stringValue());
            token = strtok(buffer, ",");
            while(token != NULL) {
                startTime.push_back(atol(token));
                token = strtok(NULL, ",");
            }
            ASSERT(startTime.size() == size);

            if (simTime() < startTime[k]) { // Vehicle shows up before attack
                scheduleAt(startTime[k], attackEvent);
            } else if (simTime() > startTime[k] && simTime() < startTime[k] + duration[k]) { // Vehicle shows up during the attack
                scheduleAt(simTime()+0.0001, attackEvent);
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
        int destinationId, repeat;

        switch(msg->getKind()) {
        case 0:
            // Sending Message
            // Checks if perform a flooding attack or not
            if(attacker && attacking && attackMode == A_DDOS) {
                // Source address will be spoofed on switch module
                destinationId = victims[k];
                repeat = attackSize[k];
            } else {
                destinationId = getRandomVehicle()->getIndex();
                repeat = burstSize;
            }

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

                if(!attacking) msgSent++; // Do not register while attacking
            }
            if(!attacking) recordV(); // Do not register while attacking
            schedule();
            break;

        case 1: // Start or Stop Attack!
            if(!attacking) { // Starting attacks
                attacking = true;
                std::cout << "Vehicle [" << vehicleId << "] starting attack on ["<< victims[k] << "]\n";
                if(!first) first = true;

                // Stop the message event for black hole and overflow attack
                //if(attackMode == A_BLACK_HOLE || attackMode == A_OVERFLOW) cancelEvent(messagesEvent);
                scheduleAt(startTime[k] + duration[k] + 0.1, attackEvent);

            } else { // Stopping attacks
                attacking = false;
                std::cout << "Vehicle [" << vehicleId << "] stopping attack on ["<< victims[k] << "]\n";
                if(first) {
                    k++;
                    first = false;
                }

                // Checking for future attacks
                if(k < size) {
                    if (simTime() < startTime[k]) { // Vehicle shows up before attack
                        scheduleAt(startTime[k], attackEvent);
                    } else if (simTime() > startTime[k] && simTime() < startTime[k] + duration[k]) { // Vehicle shows up during the attack
                        scheduleAt(simTime()+0.0001, attackEvent);
                    }
                }
                //if(!messagesEvent->isScheduled()) schedule();
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
    scheduleAt(simTime() + burstInterval, messagesEvent);
}

cModule* SdvnPing::getRandomVehicle() {
    auto map = TraCIScenarioManagerAccess().get()->getManagedHosts();
    auto iter = map.begin();
    std::advance(iter, intuniform(0, map.size()-1));
    return iter->second;
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
        recordScalar("attacker", true);
        if(attackEvent->isScheduled()) {
            cancelAndDelete(attackEvent);
        } else {
            delete attackEvent;
        }
    }
    recordScalar("Messages Sent", msgSent);
    recordScalar("Messages Received", msgRecv);
}
