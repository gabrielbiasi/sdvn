#include "modules/SdvnSwitch.h"

Define_Module(SdvnSwitch);

void SdvnSwitch::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        architecture = getSystemModule()->par("architecture").longValue();
        prefixRsuId = par("prefixRsuId").longValue();

        type = par("type").stdstringValue();
        if(isVehicle()) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
            appl = (SdvnPing*) getParentModule()->getSubmodule("appl");
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

        numPacketsV.setName("Packets Processed");
        numFlowsV.setName("Flows Generated");
        numPackets = 0;
        botFlowId = NO_VEHICLE;

        fromApp = findGate("fromApp");
        toApp = findGate("toApp");

        fromRsu = findGate("fromRsu");
        toRsu = findGate("toRsu");

        fromController = findGate("fromController");
        toController = findGate("toController");

        controllerBeaconEvent = new cMessage("Send Neighbors", 010);
        controllerBeaconsInterval = par("controllerBeaconsInterval").doubleValue();

        // WTF
        double offSet = dblrand() * (controllerBeaconsInterval / 2);
        offSet = offSet + floor(offSet/0.050)*0.050;
        scheduleAt(simTime() + offSet, controllerBeaconEvent);

        currentNeighbors = vector<int>();
        flowTable = vector<ControllerMessage*>();
        packetInBuffer = vector<AppMessage*>();
        packetStandbyBuffer = vector<AppMessage*>();

        rsuQueue = vector<WaveShortMessage*>();

        droppedRule = 0;
        droppedByTTL = 0;
        droppedByRule = 0;
        droppedByStandbyOverflow = 0;
        droppedByInOverflow = 0;
        WATCH(myId);
        WATCH(botFlowId);
    }
}

void SdvnSwitch::cleanFlowTable() {
    simtime_t now = simTime();
    int c1 = 0, c2 = 0, c3 = 0;
    auto it = flowTable.begin();
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
    EV_INFO << "Vehicle [" << myId << "] SdvnSwitch onData received.\n";

    /* Limpeza de Flow Rules vencidas */
    cleanFlowTable();

    AppMessage* packet = (AppMessage*) wsm;

    // Checking Attacker Actions
    if(performAttack(packet)) return;

    if(packet->getDestinationAddress() == myId) {
        // I am the destination, sending to upper layer.
        EV_INFO << "Vehicle [" << myId << "] onData I am the destination.\n";
        numPackets++;
        sendApplication(packet);
        return;
    }

    // Checking TTL.
    if(packet->getTTL() <= 0) {
        EV_INFO << "Vehicle [" << myId << "] onData Dropping by TTL.\n";
        delete packet;
        droppedByTTL++;
        numPackets++;
        return;
    }

    // Packet is for another vehicle, starting SDN process.
    EV_INFO << "Vehicle [" << myId << "] Checking Flow Table...\n";
    // Checking Flow Table
    for(auto &cm : flowTable) {
        if(packet->getDestinationAddress() == cm->getDestinationAddress()) { // Flow Match
            EV_INFO << "Vehicle [" << myId << "] MATCH: ";
            cm->setLastUsed(simTime()); // Refresh the idle timeout

            // Perform the Action Flow
            switch(cm->getFlowAction()) {
            case S_DROP:
                // Special DROP
                // If the packet was generated by me, just continue in FORWARD code.
                // But if this message come from another vehicle, drop.
                if(packet->getSourceAddress() != myId) {
                    EV_INFO << "Vehicle [" << myId << "] Attacker DROP!";
                    delete packet;
                    return;
                }
            case FORWARD:
                packet->setTTL(packet->getTTL()-1);
                wsm->setSenderAddress(myId);
                wsm->setRecipientAddress(cm->getFlowId());
                EV_INFO << "Packet [" << packet->getSourceAddress();
                EV_INFO << "->";
                EV_INFO << packet->getDestinationAddress();
                EV_INFO << "] ";
                EV_INFO << "FORWARD! Sending using ";
                if(!isVehicle() && cm->getFlowId() >= prefixRsuId) {
                    // I2I
                    EV_INFO << "Ethernet to RSU[" << cm->getFlowId() << "].\n";
                    sendRSU(wsm);
                    numPackets++;
                } else {
                    // V2V or V2I
                    EV_INFO << "WAVE to [" << cm->getFlowId() << "].\n";
                    sendWSM(wsm);
                    numPackets++;
                }
                return;
            case STANDBY:
                EV_INFO << "STANDBY! Buffering message.\n";
                if(packetStandbyBuffer.size() >= ((unsigned) maxPacketStandbyBuffer)) {
                    EV_INFO << "Vehicle [" << myId << "] Standby Buffer Overflow! Dropping packet.\n";
                    droppedByStandbyOverflow++;
                    delete packet;
                    numPackets++;
                } else {
                    packetStandbyBuffer.push_back(packet);
                    EV_INFO << "Vehicle [" << myId << "] Standby Buffer now: [" << packetStandbyBuffer.size() << "]\n";
                    cMessage* check = new cMessage("Check Standby Buffer", 013);
                    scheduleAt(simTime() + standbyTime, check);
                }
                return;
            case DROP:
                EV_INFO << "DROP! message deleted. \n";
                droppedByRule++;
                numPackets++;
                delete packet;
                return;
            }
        }
    }

    EV_INFO << "Vehicle [" << myId << "] MISS! Trying to buffering on Packet In...\n";
    if(packetInBuffer.size() >= ((unsigned) maxPacketInBuffer)) {
        // Packet In Buffer Overflow!
        EV_INFO << "Vehicle [" << myId << "] Packet In Buffer Overflow!\n";
        droppedByInOverflow++;
        numPackets++;
        delete packet;
        return;
    }

    // The PACKET_IN is sent and the packet and their similar are store in the buffer
    for(auto &p : packetInBuffer) {
        // Checking for similars...
        if(p->getDestinationAddress() == packet->getDestinationAddress()) {
            EV_INFO << "Vehicle [" << myId << "] Similar packet found on Packet In Buffer, just buffering.\n";
            packetInBuffer.push_back(packet);
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


void SdvnSwitch::handleSelfMsg(cMessage* msg) {
    if (msg->getKind() == 010) {
        // Time to send my neighbors!

        // RSU alone on simulation
        if(!hasActiveVehicle()){
            cancelEvent(sendBeaconEvt); // Stop beacons
            return;
        }

        EV_INFO << "Vehicle [" << myId << "] Sending my current neighbors! [";
        NeighborMessage* nm = new NeighborMessage();
        nm->setKind(011); // Neighbor Message Kind
        nm->setSourceVehicle(myId);
        nm->setTimestamp(simTime());
        nm->setNumPackets(numPackets); // statistics to controller

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
        numPacketsV.record(numPackets);

        int fuu;
        SdvnController* c = dynamic_cast<SdvnController*>(getSystemModule()->getModuleByPath(".controller"));
        fuu = c->eachNumFlow[myId];
        numFlowsV.record(fuu);
        numPackets = 0; // Reseting

    } else if(msg->getKind() == 013) {
        delete msg;
        EV_INFO << "Vehicle [" << myId << "] Recovering packet on Standby Buffer!\n";

        AppMessage* packet = *(packetStandbyBuffer.begin());
        packetStandbyBuffer.erase(packetStandbyBuffer.begin());

        onData(packet);

        EV_INFO << "Packet on Standby Buffer yet: " << packetStandbyBuffer.size() << " \n";
    } else {
        BaseWaveApplLayer::handleSelfMsg(msg);
    }
}


void SdvnSwitch::onController(ControllerMessage* msg) {

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
            EV_INFO << "Vehicle [" << myId << "] FORWARD to ["<< msg->getFlowId() << "]!\n";
        } else if(msg->getFlowAction() == STANDBY) {
            EV_INFO << "Vehicle [" << myId << "] STANDBY!\n";
        } else {
            EV_INFO << "Vehicle [" << myId << "] DROP!\n";
        }

        // Looking for a similar flow rule in flow table
        auto flow = flowTable.begin();
        while(flow != flowTable.end()) {
            if((*flow)->getDestinationAddress() == msg->getDestinationAddress()) {
                EV_INFO << "Vehicle [" << myId << "] Similar flow rule found on flow table and it was replaced.\n";
                delete *flow;
                flowTable.erase(flow);
                break;
            }
            flow++;
        }

        flowTable.push_back(msg); // Put the new flow in the table
        msg->setTimestamp(simTime()); // Start timeouts
        msg->setLastUsed(simTime());

        vector<AppMessage*> selected = vector<AppMessage*>();

        auto it = packetInBuffer.begin();
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

    size_t message_size = string(msg->getPayload()).length();
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

    if(architecture == DISTRIBUTED && !rsuQueue.empty() && sender >= prefixRsuId) {
        // A RSU was found with control messages on rsuQueue on Distributed Mode.
        // Sending right now.
        EV_INFO << "Vehicle [" << myId << "]" << " beacon received from RSU with |rsuQueue| = ["<< rsuQueue.size() <<"]. Sending control messages...\n";
        for(auto msg : rsuQueue) {
            msg->setRecipientAddress(sender);
            sendWSM(msg);
        }
        rsuQueue.clear();
    }
}

void SdvnSwitch::handleMessage(cMessage* msg) {
    string name = string(msg->getName());
    int gateId = msg->getArrivalGateId();

    if(architecture == DISTRIBUTED && name == "control") {
        WaveShortMessage* wsm = (WaveShortMessage*) msg;
        if(!isVehicle() && gateId != fromController && wsm->getRecipientAddress() == myId) { // RSU just passing message to controller
            send(msg, toController);
        } else if(gateId == fromController && wsm->getRecipientAddress() != myId) { // Controller wants to send a control message
            sendWSM(wsm);
        } else {
            if(ControllerMessage* cm = dynamic_cast<ControllerMessage*>(msg)) {
                onController(cm);
            } else {
                // Receiving Neighbor Message, just ignoring.
                delete msg;
            }
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
    return (type == string("Vehicle"));
}

bool SdvnSwitch::hasActiveVehicle() {
    return TraCIScenarioManagerAccess().get()->getManagedHosts().size() > 0;
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

        WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(msg);
        wsm->setName("control");
        wsm->addBitLength(headerLength);
        wsm->setChannelNumber(Channels::CCH);
        wsm->setPsid(0);
        wsm->setPriority(par("dataPriority").longValue());
        wsm->setWsmVersion(1);
        wsm->setTimestamp(simTime());
        wsm->setSenderAddress(myId);
        wsm->setSenderPos(curPosition);
        wsm->setSerial(0);

        if(!isVehicle()) {
            // If node is a RSU, just send direct to the module.
            wsm->setRecipientAddress(myId);
            send(msg, toController);
        } else {
            for(auto node : currentNeighbors) {
                if(node >= prefixRsuId) {
                    // RSU found!
                    wsm->setRecipientAddress(node);
                    EV_INFO << "Vehicle [" << myId << "] sending control message to RSU [" << node << "]\n";
                    sendWSM(wsm);
                    return;
                }
            }
            // If the vehicle cannot find a RSU,
            // the control message is queued until
            // a RSU is reached.
            rsuQueue.push_back(wsm);
            EV_INFO << "Vehicle [" << myId << "] sending control message to RsuQueue. Size now:["<< rsuQueue.size() <<"]\n";
        }
    }
}

void SdvnSwitch::sendRSU(WaveShortMessage* msg) {
    // TODO Change this
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
    SdvnPing* botPing;
    ControllerMessage* cm;
    WaveShortMessage* wsm;
    if (appl)
        if (appl->attacker && appl->attacking)
            switch(appl->attackMode) {
            case A_BLACK_HOLE:
                // Black Hole Attacker
                // Just dropping every message that arrives
                delete msg;
                return true;

            case A_DDOS:
                // If the message was not generate by me,
                // return false in order to process normally
                // and behaves like a normal vehicle.
                if(msg->getSourceAddress() != myId) return false;

                // DDoS Attacker
                // Send the message directly
                wsm = dynamic_cast<WaveShortMessage*>(msg);

                // Spoofed source address
                msg->setSourceAddress(-171); // spoofing on app layer
                wsm->setSenderAddress(-171); // spoofing on net layer

                // Getting a random neighbor to forward the traffic load
                if(currentNeighbors.size() > 0 && botFlowId == NO_VEHICLE) {
                    botFlowId = currentNeighbors.at(intuniform(0,currentNeighbors.size()-1));
                    auto map = TraCIScenarioManagerAccess().get()->getManagedHosts();
                    for(auto jk : map) {
                        if (jk.second->getIndex() == botFlowId) {
                            botPing = dynamic_cast<SdvnPing*>(jk.second->getModuleByPath(".appl"));
                            if(botPing->attacker) botFlowId = NO_VEHICLE; // If vehicle is an attacker
                            break;
                        }
                    }
                }

                if(botFlowId != NO_VEHICLE) {
                    // Checking if current neighbor is still reachable
                    for(int n : currentNeighbors) {
                        if(botFlowId == n) {
                            EV_INFO << "Vehicle [" << myId << "] sending attack burst via [" << botFlowId <<"]\n";
                            // Victim address
                            wsm->setRecipientAddress(botFlowId);
                            // Send directly without checking any flow rule
                            sendWSM(wsm);
                            return true;
                        }
                    }
                    // Flow lost, get another vehicle
                    botFlowId = NO_VEHICLE;
                }
                delete wsm;

                return true;
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
    recordScalar("RSU Queue Size", rsuQueue.size());

    if(controllerBeaconEvent->isScheduled()) {
        cancelAndDelete(controllerBeaconEvent);
    } else {
        delete controllerBeaconEvent;
    }

    for(auto &packet : packetStandbyBuffer) delete packet;
    packetStandbyBuffer.clear();
    for(auto &packet : packetInBuffer) delete packet;
    packetInBuffer.clear();
    for(auto &control : rsuQueue) delete control;
    rsuQueue.clear();
    for(auto &flow : flowTable) delete flow;
    flowTable.clear();
}
