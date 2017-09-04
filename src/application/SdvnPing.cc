#include "application/SdvnPing.h"

Define_Module(SdvnPing);

void SdvnPing::initialize(int stage) {
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

        // Attacker Settings
        attacking = false;
        attackSent = 0;
        attackRecv = 0;
    }
}

void SdvnPing::handleMessage(cMessage *msg) {
    Attacker* mod = dynamic_cast<Attacker*>(getSystemModule()->getSubmodule("attacker"));
    SdvnController* ctrl = dynamic_cast<SdvnController*>(getSystemModule()->getSubmodule("controller"));

    if(msg->isSelfMessage()) {
        AppMessage* packet;
        stringstream ss;
        int destinationId, repeat;

        // Sending Message
        // Checks if perform a flooding attack or not
        if(attacking) {
            // Source address will be spoofed on switch module
            destinationId = mod->getVictimId();
            repeat = burstSize * mod->getAttackSize();
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
            ss.clear();
            ss << "PING Hello!";
            packet->setPayload(ss.str().c_str());

            // Sending message
            EV_INFO << "Vehicle [" << vehicleId << "] Sending PING #" << msgSent <<" to Vehicle [" << destinationId << "]\n";
            send(packet, toSwitch);

            if(attacking) {
                auto victim = find(ctrl->confirmed.begin(),ctrl->confirmed.end(), mod->getVictimId());
                // only counts if the sentinel already knows about the attack
                if(victim != ctrl->confirmed.end()) attackSent++;
            } else {
                msgSent++;
            }
        }
        if(!attacking) recordV(); // Do not register while attacking
        scheduleAt(simTime() + burstInterval, messagesEvent);

    } else if (msg->getArrivalGateId() == fromSwitch) {
        // Message received from switch.
        AppMessage* packet = (AppMessage*) msg;
        int packetId = packet->getId();
        int senderId = packet->getSourceAddress();
        string s = string(packet->getPayload());

        if(s.find("PING") != string::npos) {
            // PING received from a vehicle. Sending PONG.
            EV_INFO << "Vehicle [" << vehicleId << "] PING #"<< packetId <<" received from Vehicle [" << senderId << "]\n";
            EV_INFO << "Vehicle [" << vehicleId << "] Sending PONG...\n";

            if(senderId == NO_VEHICLE) {
                attackRecv++;
                delete packet;
            } else {
                packet->setSourceAddress(vehicleId);
                packet->setDestinationAddress(senderId);
                packet->setTTL(64);

                s.replace(0, 4, "PONG");
                packet->setPayload(s.c_str());
                send(packet, toSwitch);
            }
        } else if(s.find("PONG") != string::npos) {
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

cModule* SdvnPing::getRandomVehicle() {
    auto map = TraCIScenarioManagerAccess().get()->getManagedHosts();
    auto iter = map.begin();
    advance(iter, intuniform(0, map.size()-1));
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

    if(attackRecv > 0) recordScalar("Spoofed Packets Received", attackRecv);
    if(attackSent > 0) recordScalar("Spoofed Packets Sent", attackSent);

    recordScalar("Messages Sent", msgSent);
    recordScalar("Messages Received", msgRecv);
}
