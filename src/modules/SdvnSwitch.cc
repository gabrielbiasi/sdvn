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
#include "modules/SdvnSwitch.h"

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

Define_Module(SdvnSwitch);

void SdvnSwitch::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        architecture = getSystemModule()->par("architecture").longValue();
        prefixRsuId = par("prefixRsuId").longValue();

        type = par("type").stdstringValue();
        if(isVehicle()) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();

            std::stringstream ss;
            ss << ".vehicle[" << myId << "].appl";
            appl = (SdvnPing*) getSystemModule()->getModuleByPath(ss.str().c_str());
        } else {
            appl = nullptr;
            BaseMobility* mobi = (BaseMobility*) getParentModule()->getSubmodule("mobility");
            ASSERT(mobi);
            myId += prefixRsuId;
        }

        dataPriority = par("dataPriority").longValue();

        maxPacketInBuffer = par("maxPacketInBuffer").longValue();
        maxPacketStandbyBuffer = par("maxPacketStandbyBuffer").longValue();
        maxFlowRules = par("maxFlowRules").longValue();

        standbyTime = par("standbyTime").doubleValue();

        fromApp = findGate("fromApp");
        toApp = findGate("toApp");

        fromRsu = findGate("fromRsu");
        toRsu = findGate("toRsu");

        fromController = findGate("fromController");
        toController = findGate("toController");

        controllerBeaconEvent = new cMessage("Send Neighbors", 010);
        controllerBeaconsInterval = par("controllerBeaconsInterval").doubleValue();

        double offSet = dblrand() * (controllerBeaconsInterval / 2);
        offSet = offSet + floor(offSet/0.050)*0.050;
        scheduleAt(simTime() + offSet, controllerBeaconEvent);

        currentNeighbors = std::vector<int>();
        flowTable = std::vector<ControllerMessage*>();
        packetInBuffer = std::vector<AppMessage*>();
        packetStandbyBuffer = std::vector<AppMessage*>();

        droppedRule = 0;
        droppedByTTL = 0;
        droppedByRule = 0;
        droppedByStandbyOverflow = 0;
        droppedByInOverflow = 0;
        WATCH(myId);
    }
}

void SdvnSwitch::cleanFlowTable() {
    simtime_t now = simTime();
    int c1 = 0, c2 = 0, c3 = 0;
    vector<ControllerMessage*>::iterator it = flowTable.begin();
    while(it < flowTable.end()) {
        ControllerMessage* cm = (ControllerMessage*) (*it);
        if (now > cm->getTimestamp() + cm->getHardTimeout()) {
            c1++;
            flowTable.erase(it);
            delete cm;
        } else if (now > cm->getLastUsed() + cm->getIdleTimeout()) {
            c2++;
            flowTable.erase(it);
            delete cm;
        } else if(cm->getFlowAction() == FORWARD) {
            // Small test to avoid using forward
            bool hasConnectivity = false;
            for(auto &neighbor : currentNeighbors) {
                if (neighbor == cm->getFlowId()) {
                    hasConnectivity = true;
                    break;
                }
            }
            if(!hasConnectivity) {
                c3++;
                flowTable.erase(it);
                delete cm;
            } else {
                it++;
            }

        } else {
            it++;
        }
    }
    EV_INFO << "Vehicle [" << myId << "] Flow Table Cleanup: " << c1 <<" Hard, " << c2 << " Idle Rules and "<< c3 << " Lost Connectivity Rules cleaned.\n";
}



void SdvnSwitch::onData(WaveShortMessage* wsm) {

    // Ignore data packet not assigned to me
    if(wsm->getRecipientAddress() != myId) {
        delete wsm;
        return;
    }

    findHost()->getDisplayString().updateWith("r=16,green");
    EV_INFO << "Vehicle [" << myId << "] SdvnSwitch onData received.\n";

    /* Limpeza de Flow Rules vencidas */
    cleanFlowTable();

    AppMessage* packet = (AppMessage*) wsm;

    /* Attackers Area */
    if(performAttack(packet)) return;

    // I am the destination, sending to upper layer.
    if(packet->getDestinationAddress() == myId) {
        EV_INFO << "Vehicle [" << myId << "] onData I am the destination.\n";
        sendApplication(packet);
    } else {

        // TTL Check.
        if(packet->getTTL() <= 0) {
            EV_INFO << "Vehicle [" << myId << "] onData Dropping by TTL.\n";
            delete packet;
            droppedByTTL++;
            return;
        }
        EV_INFO << "Vehicle [" << myId << "] Checking Flow Table...\n";
        // Checking Flow Table
        for(auto &cm : flowTable) {
            if(packet->getDestinationAddress() == cm->getDestinationAddress()) { // Flow Match
                EV_INFO << "Vehicle [" << myId << "] MATCH: ";
                cm->setLastUsed(simTime()); // Refresh the idle timeout

                // Perform the Action Flow
                switch(cm->getFlowAction()) {
                    case FORWARD:
                        packet->setTTL(packet->getTTL()-1);
                        wsm->setSenderAddress(myId);
                        wsm->setRecipientAddress(cm->getFlowId());

                        if(!isVehicle() && cm->getFlowId() >= prefixRsuId) {
                            // I2I
                            EV_INFO << "FORWARD! Sending using Ethernet to RSU[" << cm->getFlowId() << "].\n";
                            sendRSU(wsm);
                        } else {
                            // V2V or V2I
                            EV_INFO << "FORWARD! Sending using WAVE to [" << cm->getFlowId() << "].\n";
                            sendWSM(wsm);
                        }
                        return;
                    case STANDBY:
                        EV_INFO << "WAIT! Buffering message.\n";
                        if(packetStandbyBuffer.size() >= ((unsigned) maxPacketStandbyBuffer)) {
                            EV_INFO << "Vehicle [" << myId << "] Wait Buffer Overflow! Dropping packet.\n";
                            droppedByStandbyOverflow++;
                            delete packet;
                            return;
                        } else {
                            packetStandbyBuffer.push_back(packet);
                            EV_INFO << "Vehicle [" << myId << "] Wait Buffer now: [" << packetStandbyBuffer.size() << "]\n";
                            cMessage* check = new cMessage("Check Wait Buffer", 013);
                            scheduleAt(simTime() + standbyTime, check);
                        }
                        return;
                    case DROP:
                        EV_INFO << "DROP! message deleted. \n";
                        droppedByRule++;
                        delete packet;
                        return;
                }
            }
        }

        EV_INFO << "Vehicle [" << myId << "] MISS! Trying to buffering on Packet In...\n";

        // Packet In Buffer Overflow!
        if(packetInBuffer.size() >= ((unsigned) maxPacketInBuffer)) {
            EV_INFO << "Vehicle [" << myId << "] Packet In Buffer Overflow!\n";
            droppedByInOverflow++;
            delete packet;
            return;
        } else {
            // The PACKET_IN is sent and the packet and their similar are store in the buffer
            for(auto &p : packetInBuffer) {
                if(p->getDestinationAddress() == packet->getDestinationAddress()) {
                    EV_INFO << "Vehicle [" << myId << "] Similar packet found on Packet In Buffer, just buffering.\n";
                    packetInBuffer.push_back(packet);
                    EV_INFO << "Vehicle [" << myId << "] Packet In Buffer Now: " << packetInBuffer.size() << "\n";
                    return;
                }
            }
            EV_INFO << "Vehicle [" << myId << "] Buffered and sending PACKET_IN to Controller.\n";
            packetInBuffer.push_back(packet);
            ControllerMessage* cm = new ControllerMessage();
            cm->setKind(012); // Controller Message Kind
            cm->setSourceVehicle(myId);
            cm->setFlowAction(PACKET_IN);
            cm->setDestinationAddress(packet->getDestinationAddress());
            sendController(cm);
        }
    }
}


void SdvnSwitch::handleSelfMsg(cMessage* msg) {
    if (msg->getKind() == 010) {
        // Time to send my neighbors!

        EV_INFO << "Vehicle [" << myId << "] Sending my current neighbors! [";
        NeighborMessage* nm = new NeighborMessage();
        nm->setKind(011); // Neighbor Message Kind
        nm->setSourceVehicle(myId);
        nm->setTimestamp(simTime());

        nm->setNeighborsArraySize(currentNeighbors.size());
        for(unsigned int i=0; i < currentNeighbors.size(); i++) {
            nm->setNeighbors(i, currentNeighbors[i]);
            EV_INFO << currentNeighbors[i] << ",";
        }
        EV_INFO << "]\n";

        sendController(nm);
        currentNeighbors.clear();
        scheduleAt(simTime() + controllerBeaconsInterval, controllerBeaconEvent);

        if(!isVehicle()) addRsuNeighbors(); // adds the neighbors RSUs


    } else if(msg->getKind() == 013) {
        delete msg;
        EV_INFO << "Vehicle [" << myId << "] Recovering packet on Standby Buffer!";

        AppMessage* packet = *(packetStandbyBuffer.begin());
        packetStandbyBuffer.erase(packetStandbyBuffer.begin());

        onData(packet);

        EV_INFO << "Packet on Standby Buffer yet: " << packetStandbyBuffer.size() << " \n";
    } else {
        BaseWaveApplLayer::handleSelfMsg(msg);
    }
}


void SdvnSwitch::onController(ControllerMessage* msg) {

    // Ignore control packet not assigned to me
    if(msg->getSourceVehicle() != myId) {
        delete msg;
        return;
    }

    // New Flow Rule
    EV_INFO << "Vehicle [" << myId << "] ControllerMessage received.\n";
    if(msg->getMessageType() == FLOW_MOD && msg->getSourceVehicle() == myId) {

        // Cleanup
        cleanFlowTable();

        if(flowTable.size() >= ((unsigned) maxFlowRules)) {
            // Flow Table Overflow!
            EV_INFO << "Vehicle [" << myId << "] Flow Table Overflow! Dropping rule.\n";
            droppedRule++;
            delete msg;
            return;
        }

        if(msg->getFlowAction() == FORWARD) {
            EV_INFO << "Vehicle [" << myId << "] ControllerMessage FORWARD to ["<< msg->getFlowId() << "]!\n";
        } else if(msg->getFlowAction() == STANDBY) {
            EV_INFO << "Vehicle [" << myId << "] ControllerMessage STANDBY!\n";
        } else {
            EV_INFO << "Vehicle [" << myId << "] ControllerMessage DROP!\n";
        }

        flowTable.push_back(msg); // Put the new flow in the table
        msg->setTimestamp(simTime()); // Start timeouts
        msg->setLastUsed(simTime());

        vector<AppMessage*> selected = vector<AppMessage*>();
        vector<AppMessage*>::iterator it = packetInBuffer.begin();

        while(it < packetInBuffer.end()){
            AppMessage* ap = (AppMessage*) (*it);
            if(ap->getDestinationAddress() == msg->getDestinationAddress()) {
                EV_INFO << "Message packet to Vehicle [";
                EV_INFO << ap->getDestinationAddress();
                EV_INFO << "] recovered from Packet In Buffer.\n";
                selected.push_back(ap);
                packetInBuffer.erase(it);
            } else {
                it++;
            }
        }

        // Send all the selected packets to the L3 layer
        for(auto &packet : selected) onData(packet);

    } else {
        EV_INFO << "Vehicle [" << myId << "] ControllerMessage WRONG.\n";
        delete msg;
    }
}

void SdvnSwitch::onApplication(AppMessage* msg) {
    // AppMessage to WaveShortMessage
    WaveShortMessage* wsm = (WaveShortMessage*) msg;

    size_t message_size = std::string(msg->getPayload()).length();
    wsm->addByteLength(message_size);

    wsm->setName("data");
    wsm->addBitLength(headerLength);
    wsm->setChannelNumber(Channels::CCH);
    wsm->setPsid(0);
    wsm->setPriority(par("dataPriority").longValue());
    wsm->setWsmVersion(1);
    wsm->setTimestamp(simTime());
    wsm->setSenderAddress(myId);
    wsm->setRecipientAddress(myId); // simulates the packet arriving on the L3
    wsm->setSenderPos(curPosition);
    wsm->setSerial(0);

    onData(wsm); // send message to Switch
}

void SdvnSwitch::onBeacon(WaveShortMessage* wsm) {
    int sender = wsm->getSenderAddress();
    EV_INFO << "Vehicle [" << myId << "]" << " beacon received from vehicle [" << sender << "] Current: [";
    for (auto &i : currentNeighbors)
        EV_INFO << i << ", ";
    EV_INFO << "]\n";

    delete wsm;

    for (auto &i : currentNeighbors) // If neighbor already in the neighbors table, just ignore.
        if (i == sender)
            return;
    // Store the new neighbor.
    currentNeighbors.push_back(sender);
}

void SdvnSwitch::handleMessage(cMessage* msg) {
    std::string name = std::string(msg->getName());
    int gateId = msg->getArrivalGateId();

    if(architecture == DISTRIBUTED && name == "control") {
        WaveShortMessage* wsm = (WaveShortMessage*) msg;
        if(!isVehicle() && gateId != fromController && wsm->getRecipientAddress() == myId) { // RSU just passing message to controller
            send(msg, toController);
        } else if(gateId == fromController && wsm->getRecipientAddress() != myId) { // Controller wants to send a control message
            sendWSM(wsm);
        } else {
            onController((ControllerMessage*) msg);
        }
    }
    else if (gateId == fromApp) {
        onApplication((AppMessage*) msg);
    } else if(name == "control") {
        onController((ControllerMessage*) msg);
    } else if (name == "data" || gateId == fromRsu) {
        onData((WaveShortMessage*) msg);
    } else if (name == "beacon") {
        onBeacon((WaveShortMessage*) msg);
    } else {
        BaseLayer::handleMessage(msg);
    }
}

bool SdvnSwitch::isVehicle() {
    return (type == std::string("Vehicle"));
}

void SdvnSwitch::sendApplication(AppMessage* msg)  {
    cSimpleModule::send(msg, toApp);
}

void SdvnSwitch::sendController(cMessage* msg)  {
    if(architecture == CENTRALIZED) {
        cModule* lteBase = getSystemModule()->getModuleByPath(".base");
        double lteDelay = lteBase->par("lteDelay").doubleValue();
        cSimpleModule::sendDirect(msg, lteDelay, 0, lteBase, "radioIn");
    } else {
        // Distributed Architecture
        if(!isVehicle()) {
            // If node is a RSU, just send direct to the module.
            send(msg, toController);
        } else {
            for(auto node : currentNeighbors) {
                if(node >= prefixRsuId) {
                    // RSU found!
                    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);

                    wsm->setName("control");
                    wsm->addBitLength(headerLength);
                    wsm->setChannelNumber(Channels::CCH);
                    wsm->setPsid(0);
                    wsm->setPriority(par("dataPriority").longValue());
                    wsm->setWsmVersion(1);
                    wsm->setTimestamp(simTime());
                    wsm->setSenderAddress(myId);
                    wsm->setRecipientAddress(node);
                    wsm->setSenderPos(curPosition);
                    wsm->setSerial(0);

                    sendWSM(wsm);
                    return;
                }
            }
            std::stringstream ss;
            ss << "Vehicle [" << myId << "] did not find a RSU under Distributed Mode.\n";
            ss << "SimTime: "<< simTime() << "\n";
            perror(ss.str().c_str());
        }
    }
}

void SdvnSwitch::sendRSU(WaveShortMessage* msg) {
    std::stringstream ss;
    ss << ".rsu[" << (msg->getRecipientAddress()-prefixRsuId) << "].switcher";
    cModule* rsu = getSystemModule()->getModuleByPath(ss.str().c_str());
    double rsuDelay = rsu->getParentModule()->par("rsuDelay").doubleValue();
    cSimpleModule::sendDirect(msg, rsuDelay, 0, rsu, "fromRsu");
}

void SdvnSwitch::addRsuNeighbors() {
    int num = getSystemModule()->par("numRsu").longValue();
    for(int i = 0; i < num; i++) {
        currentNeighbors.push_back(prefixRsuId + i);
    }
}

void SdvnSwitch::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
}

void SdvnSwitch::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);
}

bool SdvnSwitch::performAttack(AppMessage* msg) {
    int i, j, victim;
    ControllerMessage* cm;
    if (appl)
        if (appl->attacker && appl->attacking)
            switch(appl->attackMode) {
            case A_BLACK_HOLE:
                // Black Hole Attacker
                // Just dropping every message that arrives
                delete msg;
                return true;

            case A_DDOS:
                // DDoS Attacker
                // does nothing on network layer
                return false;
            case A_OVERFLOW:
                // Overflow Attacker
                // Send maxFlowRules/2 PACKET_IN messages setting the source
                // vehicle with the victim ID.
                victim = msg->getSourceAddress();
                for(i = 0; i < maxFlowRules/2; i++) {
                    cm = new ControllerMessage();
                    cm->setKind(012);
                    cm->setSourceVehicle(victim);
                    cm->setFlowAction(PACKET_IN);
                    j = (victim-10 < 0) ? 0 : victim-10;
                    cm->setDestinationAddress(intuniform(10, victim+10));
                    sendController(cm);
                }
                return false;
            }

    return false;
}


void SdvnSwitch::finish() {
    BaseWaveApplLayer::finish();

    recordScalar("Flow Rules Dropped", droppedRule);
    recordScalar("Dropped by TTL", droppedByTTL);
    recordScalar("Dropped by Rule", droppedByRule);
    recordScalar("Dropped by Standby Overflow", droppedByStandbyOverflow);
    recordScalar("Dropped by Packet In Overflow", droppedByInOverflow);
    recordScalar("Standby Buffer Size", packetStandbyBuffer.size());
    recordScalar("Packet In Buffer Size", packetInBuffer.size());

    if(controllerBeaconEvent->isScheduled()) {
        cancelAndDelete(controllerBeaconEvent);
    } else {
        delete controllerBeaconEvent;
    }

    for(auto &packet : packetStandbyBuffer) delete packet; // free packets still in buffer
    packetStandbyBuffer.clear();
    for(auto &packet : packetInBuffer) delete packet; // free packets still in buffer
    packetInBuffer.clear();
    for(auto &flow : flowTable) delete flow; // free flows still in table
    flowTable.clear();
}
