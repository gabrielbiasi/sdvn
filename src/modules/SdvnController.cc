#include "modules/SdvnController.h"

Define_Module(SdvnController);

using std::map;
using std::vector;
using std::stringstream;

void SdvnController::initialize(int stage) {
    if (stage == 0) {
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

     // Pilha de veículos
     dist[source] = 0;
     prev[source] = source;
     while(listV.size() > 0) {

         // Seleciona o menor dist[]
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


ControllerMessage* SdvnController::prepareNewFlowMod(cMessage* message) {
    ControllerMessage* m = (ControllerMessage*) message;
    int flowDestination;
    int vehicle = m->getSourceVehicle();
    int destination = m->getDestinationAddress();

    ControllerMessage* flow = new ControllerMessage();
    flow->setMessageType(FLOW_MOD);
    flow->setSourceVehicle(vehicle);
    flow->setDestinationAddress(destination);

    if(simTime() < timestamps[destination] + dropAfter) { // Check if vehicle still sending neighbor messages
        flowDestination = runSimpleDijkstra(vehicle, destination);
        flow->setFlowAction(flowDestination != -1 ? FORWARD : STANDBY);
        flow->setFlowId(flowDestination);
    } else {
        flow->setFlowAction(DROP);
        flow->setFlowId(-1);
    }

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
        new_neighbors.push_back(m->getNeighbors(i)); // put the neighbors in the new vector
    }
    graph[vehicle] = new_neighbors;
    timestamps[vehicle] = m->getTimestamp();
}


void SdvnController::handleMessage(cMessage *msg) {

    if(msg->getArrivalGateId() == inControl) {
        if(msg->getKind() == 011) { // 011 -> Neighbor Message
            //EV_INFO << "SDVN Controller: Neighbor Update Received\n";
            updateNetworkGraph(msg);
            delete msg;
            return;
        }

        if(msg->getKind() == 012) { // 012 -> Controller Message
            EV_INFO << "SDVN Controller:  Packet In Received from ["<< ((ControllerMessage*) msg)->getSourceVehicle() << "]\n";
            ControllerMessage* flow_mod = prepareNewFlowMod(msg);
            sendController(flow_mod);
            delete msg;
            return;
        }

    } else if(msg->isSelfMessage()) {
        EV_INFO << "CONTROLADOR: Self Message\n";
    } else {
        EV_INFO << "CONTROLADOR: Something wrong here!\n";
    }
}

vector<int>* SdvnController::getNodesId() {
    vector<int>* result;
    result = new vector<int>();
    for(auto node : graph) result->push_back(node.first);
    return result;
}

void SdvnController::sendController(cMessage* msg) {
    send(msg, outControl);
}

void SdvnController::finish() {
    cSimpleModule::finish();
    for(auto &j : graph) j.second.clear(); // free neighbors lists
    graph.clear();
}
