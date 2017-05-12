#include "modules/SdvnController.h"

Define_Module(SdvnController);

using std::map;
using std::vector;
using std::stringstream;

void SdvnController::initialize(int stage) {
    if (stage == 0) {
        if(hasGUI()) {
            cModule* sys = getSystemModule();
            sys->getModuleByPath(".manager")->setDisplayString("p=1200,100;i=block/network2;is=s");
            sys->getModuleByPath(".obstacles")->setDisplayString("p=1400,100;i=misc/town;is=s");
            sys->getModuleByPath(".annotations")->setDisplayString("p=1200,200;i=msg/paperclip;is=s");
            sys->getModuleByPath(".world")->setDisplayString("p=1400,200;i=misc/globe;is=s");
            sys->getModuleByPath(".connectionManager")->setDisplayString("p=1200,300;i=abstract/multicast;is=s");
        }

        debug = par("debug").boolValue();

        // Timeouts
        hardTimeout = par("hardTimeout").doubleValue();
        idleTimeout = par("idleTimeout").doubleValue();
        dropAfter = par("dropAfter").doubleValue();

        // Gates
        inControl = findGate("inControl");
        outControl = findGate("outControl");

        // Adjacency matrix
        graph = map<int, vector<int>>();
        timestamps = map<int, simtime_t>();

        prefixRsuId = 10000;
        architecture = getSystemModule()->par("architecture").longValue();
    }
}

int SdvnController::runSimpleDijkstra(int source, int destination) {

     // Start all distances with infinite

     vector<int> listV = vector<int>();
     map<int,int> dist = map<int,int>();
     map<int,int> prev = map<int,int>();

     auto j = graph.begin();
     for(j = graph.begin(); j != graph.end(); j++) {
         dist[(*j).first] = INT_MAX;
         prev[(*j).first] = -1;
         listV.push_back((*j).first);
     }

     // Vehicle stack
     dist[source] = 0;
     prev[source] = source;
     while(listV.size() > 0) {

         // Getting the shortest distance
         int shortV = -1, shortDist = 9999;

         std::vector<int>::iterator j, i = listV.begin();
         for(; i != listV.end(); i++) {
             if(dist[*i] < shortDist) {
                 shortV = *i;
                 shortDist = dist[*i];
                 j = i;
             }
         }
         if(shortV == -1) { // disconnected: (send to RSU or standby)
             int target = -1;
             if(source != prefixRsuId) {
                 for(int i = 0; i < graph[source].size(); i++) { // Vehicle to RSU
                     int neighbor = graph[source][i];
                     if(neighbor >= prefixRsuId) { // RSU found!
                         target = neighbor;
                         break;
                     }
                 }
             }
             if(target != -1) {
                 EV_INFO << "SDVN Controller: NOT FOUND, forward to RSU: [" << target << "]\n";
             } else {
                 EV_INFO << "SDVN Controller: STANDBY! \n";
             }
             return target; // Return RSU or -1 to standby

         } else if(shortV == destination) { // target
             int target = destination;
             while(prev[target] != source) target = prev[target];
             EV_INFO << "SDVN Controller: FOUND!, forward to most closer: [" << target << "]\n";
             return target; // Return to the next hop!
         }

         // Remove from vector
         listV.erase(j);

         // Simple process for shortest path
         for(int i = 0; i < graph[shortV].size(); i++) {
             int neighbor = graph[shortV][i];
             if(dist[shortV]+1 < dist[neighbor]) {
                 dist[neighbor] = dist[shortV]+1;
                 prev[neighbor] = shortV;
             }
         }
     }

     EV_INFO << "SDVN Controller: ALONE, just drop it.\n";
     return -1;
}

ControllerMessage* SdvnController::newFlow(int sourceId, int destinationId, int flowAction, int flowDestination) {
    ControllerMessage* flow = new ControllerMessage();
    flow->setName("control");
    flow->setMessageType(FLOW_MOD);
    flow->setSourceVehicle(sourceId);
    flow->setDestinationAddress(destinationId);
    flow->setFlowAction(flowAction);
    flow->setFlowId(flowDestination);
    flow->setHardTimeout(hardTimeout);
    flow->setIdleTimeout(idleTimeout);
    return flow;
}

void SdvnController::updateNetworkGraph(cMessage* message) {
    NeighborMessage* m = (NeighborMessage*) message;
    vector<int> new_neighbors;
    int vehicle = m->getSourceVehicle();
    if(graph.find(vehicle) != graph.end()) graph.erase(vehicle);

    new_neighbors = vector<int>();
    for(unsigned int i = 0; i < m->getNeighborsArraySize(); i++) {
        // put the neighbors in the new vector
        new_neighbors.push_back(m->getNeighbors(i));
    }
    graph[vehicle] = new_neighbors;
    timestamps[vehicle] = m->getTimestamp();
}


void SdvnController::handleMessage(cMessage *msg) {
    if(msg->getArrivalGateId() == inControl) {
        if(msg->getKind() == 011) {
            // 011 -> Neighbor Message
            EV_INFO << "SDVN Controller: Neighbor Update Received\n";
            updateNetworkGraph(msg);
            delete msg;
            return;

        } else if(msg->getKind() == 012) {
            // 012 -> Controller Message
            int myId, sourceId, destinationId, flowId;
            WaveShortMessage *wsm, *ori;
            ControllerMessage *flow, *cm = dynamic_cast<ControllerMessage*>(msg);

            // Getting RSU ID
            ori = dynamic_cast<WaveShortMessage*>(msg);
            myId = ori->getRecipientAddress();

            sourceId = cm->getSourceVehicle();
            destinationId = cm->getDestinationAddress();
            EV_INFO << "SDVN Controller:  Packet In Received from ["<< sourceId << "]\n";

            if(architecture == CENTRALIZED || (architecture == DISTRIBUTED && findLocalNode(destinationId))) {
                // Checking if the vehicle is still "alive"
                if(isAlive(destinationId)) {
                    flowId = runSimpleDijkstra(sourceId, destinationId);
                    flow = newFlow(sourceId, destinationId, (flowId != NO_VEHICLE) ? FORWARD : STANDBY, flowId);
                } else {
                    // Vehicle stopped sending neighbor messages, creating a drop rule.
                    flow = newFlow(sourceId, destinationId, DROP, NO_VEHICLE);
                }
            } else {
                // Distributed and vehicle is not in local SDVN
                flowId = findTarget(destinationId);
                // if the vehicle is found in another RSU, send a FORWARD rule to
                // vehicle in order to send all messages to "me". After, "me" will
                // send another PACKET_IN in order to send to RSU and so on.
                if(sourceId < prefixRsuId) {
                    flow = newFlow(sourceId, destinationId, (flowId != NO_VEHICLE) ? FORWARD : DROP, (flowId != NO_VEHICLE) ? myId : flowId);
                } else {
                    flow = newFlow(sourceId, destinationId, (flowId != NO_VEHICLE) ? FORWARD : DROP, flowId);
                }
            }

            wsm = dynamic_cast<WaveShortMessage*>(flow);
            wsm->addBitLength(256);
            wsm->setChannelNumber(178);
            wsm->setPsid(0);
            wsm->setPriority(2);
            wsm->setWsmVersion(1);
            wsm->setTimestamp(simTime());
            wsm->setSenderAddress(myId);
            wsm->setRecipientAddress(sourceId);
            wsm->setSerial(0);

            sendController(flow);
            delete msg;
            return;
        }

    } else if(msg->isSelfMessage()) {
        EV_INFO << "CONTROLADOR: Self Message\n";
    } else {
        EV_INFO << "CONTROLADOR: Something wrong here!\n";
    }
}

void SdvnController::sendController(cMessage* msg) {
    sendDelayed(msg, 0.0001, outControl);
}

bool SdvnController::isAlive(int id) {
    return (simTime() < timestamps[id] + dropAfter);
}

void SdvnController::finish() {
    cSimpleModule::finish();
    for(auto &j : graph) j.second.clear(); // free neighbors lists
    graph.clear();
    timestamps.clear();
}

/*
 * Distributed Methods
 *
 */

bool SdvnController::findLocalNode(int id) {
    for(auto node : graph)
        if(node.first == id)
            return true;
    return false;
}

int SdvnController::findTarget(int id) {
    int i, numRsu;
    std::stringstream ss;
    SdvnController* ctr;
    cModule *sys = getSystemModule();

    // For each RSU on simulation
    numRsu = sys->par("numRsu").longValue();
    for(i = 0; i < numRsu; i++) {
        ss.str("");
        ss << ".rsu[" << i << "].controller";
        ctr = (SdvnController*) getSystemModule()->getModuleByPath(ss.str().c_str());

        if(ctr->findLocalNode(id)) { // Vehicle found!
            return ctr->isAlive(id) ? (i + prefixRsuId) : NO_VEHICLE;
        }
    }
    return NO_VEHICLE; // Return NO_VEHICLE to DROP
}
