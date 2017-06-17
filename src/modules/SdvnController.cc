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
        sentinel = par("sentinel").boolValue();

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

        // Sentinel maps and parameters
        numPackets = map<int,vector<long>>();
        numFlows = map<int,vector<long>>();
        flowMods = map<int,vector<ControllerMessage*>>();
        flowTree = map<int,vector<long>>();

        flowThreshold = par("flowThreshold").doubleValue();
        numThreshold = par("numThreshold").doubleValue();
        checkLast = par("checkLast").longValue();

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
    WaveShortMessage* wsm = dynamic_cast<WaveShortMessage*>(flow);

    flow->setName("control");
    flow->setMessageType(FLOW_MOD);
    flow->setSourceVehicle(sourceId);
    flow->setDestinationAddress(destinationId);
    flow->setFlowAction(flowAction);
    flow->setFlowId(flowDestination);
    flow->setHardTimeout(hardTimeout);
    flow->setIdleTimeout(idleTimeout);

    wsm->addBitLength(256);
    wsm->setChannelNumber(178);
    wsm->setPsid(0);
    wsm->setPriority(2);
    wsm->setWsmVersion(1);
    wsm->setTimestamp(simTime());
    wsm->setSerial(0);

    return flow;
}

void SdvnController::updateNetworkGraph(cMessage* message) {
    int vehicle;
    NeighborMessage* nm;
    vector<int> new_neighbors;

    nm = (NeighborMessage*) message;
    vehicle = nm->getSourceVehicle();
    if(graph.find(vehicle) != graph.end()) graph.erase(vehicle);

    new_neighbors = vector<int>();
    for(unsigned int i = 0; i < nm->getNeighborsArraySize(); i++) {
        // put the neighbors in the new vector
        new_neighbors.push_back(nm->getNeighbors(i));
    }
    graph[vehicle] = new_neighbors;
    timestamps[vehicle] = nm->getTimestamp();

    if(sentinel) {
        numPackets[vehicle].push_back(nm->getNumPackets());
        numFlows[vehicle].push_back(flowMods[vehicle].size());
        executeSentinel(vehicle);
    }
}

void SdvnController::executeSentinel(int vehicle) {
    int lastNumPacket, lastNumFlow;
    double stdDevPacket, stdDevFlow;
    ControllerMessage* cm, new_flow;

    // Register the amount of flow rules requested in this window
    // time and release the flow rules to the next window time.
    // If we do not have "checkLast" values to work, just ignore.
    if(numPackets[vehicle].size() > checkLast && numFlows[vehicle].size() > checkLast) {

        // Removing the firsts
        numPackets[vehicle].erase(numPackets[vehicle].begin());
        numFlows[vehicle].erase(numFlows[vehicle].begin());

        /*
         * Phase 1: Detecting the attack
         */

        lastNumPacket = numPackets[vehicle].back();
        lastNumFlow = numFlows[vehicle].back();
        stdDevPacket = getMeanPlusStdDev(numPackets[vehicle]);
        stdDevFlow = getMeanPlusStdDev(numFlows[vehicle]);

        // Checking thresholds...
        if((lastNumPacket > stdDevPacket*numThreshold) // Threshold for packet amount
                && (lastNumFlow > stdDevFlow*flowThreshold) // Threshold for flow amount
                && (vehicle < prefixRsuId) ) { // Check if is a RSU

            if(simTime() > 45.0) {
                recordScalar("lastNumPacket", lastNumPacket);
                recordScalar("lastNumFlow", lastNumFlow);
                recordScalar("stdDevPacket", stdDevPacket*numThreshold);
                recordScalar("stdDevFlow", stdDevFlow*flowThreshold);
            }

            /*
             * Phase 2: Mitigating the Attack
             */

            EV_INFO << "SDVN Controller: ATTACK ATTACK ATTACK" << endl;
            EV_INFO << "SDVN Controller: Vehicle [" << vehicle << "] is over attack!" << endl;
            EV_INFO << "Building flow tree..." << endl;

            recordScalar("Detectation Time", simTime());

            for(auto aa : flowMods[vehicle]) {
                std::cout << "[" << aa->getSourceVehicle() << "->" << aa->getDestinationAddress();
                std::cout << "] via [" << aa->getFlowId() << "], type [" << aa->getFlowAction();
                std::cout << "] created: [" << aa->getTimestamp() << "]\n";
            }

            buildFlowTree(vehicle);

            for(auto aa : flowTree) {
                std::cout << "[";
                for(auto bb : aa.second) {
                    std::cout << bb << ", ";
                }
                std::cout << "]\n";
            }

            /*
             * TODO Create flow rule
             *
            for(;;) {
                new_flow = newFlow(vehicle, vehicle, S_DROP, NO_VEHICLE);
                sendController(new_flow);
            }*/
        }
    }
    // releasing memory
    for(auto flow : flowMods[vehicle]) delete flow;
    flowMods[vehicle].clear();
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

            if(architecture == DISTRIBUTED)
                clearFarNeighbors();

            if(architecture == CENTRALIZED || (architecture == DISTRIBUTED && findLocalNode(destinationId))) {
                // Checking if the vehicle is still "alive"
                if(simTime() < timestamps[destinationId] + dropAfter) {
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
                if(sourceId != myId) {
                    flow = newFlow(sourceId, destinationId, (flowId != NO_VEHICLE) ? FORWARD : DROP, (flowId != NO_VEHICLE) ? myId : flowId);
                } else {
                    flow = newFlow(sourceId, destinationId, (flowId != NO_VEHICLE) ? FORWARD : DROP, flowId);
                }
            }

            // Sender and Recipient values is used on Distributed Mode
            wsm = dynamic_cast<WaveShortMessage*>(flow);
            wsm->setSenderAddress(myId);
            wsm->setRecipientAddress(sourceId);

            // A copy is stored in order to calculate a possible flow tree
            if(sentinel) {
                auto it = flowMods[destinationId].begin();
                while(it != flowMods[destinationId].end()) {
                    if(flow->getSourceVehicle() == (*it)->getSourceVehicle()) {
                        delete *it;
                        flowMods[destinationId].erase(it);
                        break;
                    }
                    it++;
                }
                flowMods[destinationId].push_back(flow->dup());
            }

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

double SdvnController::getMeanPlusStdDev(const vector<long>&  li) {
    // Std Dev code ignoring the last value
    unsigned int i, size;
    double sum, mean, stddev;
    size = li.size() - 1; // last value ignored
    sum = 0;
    stddev = 0;
    for(i = 0; i < size; i++) sum += li[i];
    mean = sum/size;
    for(i = 0; i < size; i++) stddev += pow(i-mean, 2);
    return (mean + sqrt(stddev/size));
}

void SdvnController::sendController(cMessage* msg) {
    sendDelayed(msg, 0.0001, outControl);
}

void SdvnController::finish() {
    cSimpleModule::finish();

    for(auto &j : graph) j.second.clear(); // free neighbors lists

    if(sentinel){
        for(auto vehicle : flowMods) // for all vehicles
            for(auto flow : vehicle.second) // for all flows on each vehicle
                delete flow;
    }

    graph.clear();
    timestamps.clear();
    numPackets.clear();
    numFlows.clear();
}

void SdvnController::buildFlowTree(int victim) {
    int i, counter, vehicle;
    vector<long> stack = vector<long>();

    flowTree.clear();
    flowTree[0].push_back(victim);
    stack.push_back(victim);
    i = 1;
    counter = 1;

    while(!stack.empty()) {
        vehicle = stack.front();
        stack.erase(stack.begin());

        for(auto flow : flowMods[victim]) {
            if(flow->getFlowId() == vehicle
                    && flow->getDestinationAddress() == victim
                    && flow->getFlowAction() == FORWARD) {
                flowTree[i].push_back(flow->getSourceVehicle());
                stack.push_back(flow->getSourceVehicle());
            }
        }
        counter--;
        if(counter == 0) {
            counter = flowTree[i].size();
            i++;
        }
    }
}

/*
 * Distributed Methods
 *
 */

bool SdvnController::findLocalNode(int id) {
    map<int,vector<int>>::iterator it = graph.find(id);
    return (it != graph.end());
}

void SdvnController::clearFarNeighbors() {
    vector<int> olders = vector<int>();
    simtime_t now = simTime();
    // For distributed mode, it is possible a vehicle
    // switch between RSUs. Because of that, the parameter
    // dropAfter is way more strict, in order to maintain
    // the vehicle over only one RSU management.
    for (auto vehicle : timestamps) {
        if(now > vehicle.second + 3) { // TODO
            olders.push_back(vehicle.first);
        }
    }
    for(auto node : olders) {
        EV_INFO << "Get OLDER! [" << node << "]\n";
        graph.erase(node);
        timestamps.erase(node);
    }
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
        ctr = dynamic_cast<SdvnController*>(sys->getModuleByPath(ss.str().c_str()));

        if(ctr->findLocalNode(id)) { // Vehicle found!
            return (i + prefixRsuId);
        }
    }
    return NO_VEHICLE; // Return NO_VEHICLE to DROP
}
