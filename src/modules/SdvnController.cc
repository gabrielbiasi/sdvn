#include "modules/SdvnController.h"

Define_Module(SdvnController);

void SdvnController::initialize(int stage) {
    if (stage == 0) {
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
        victims = map<int,long>();
        suspicious = vector<long>();
        flowTree = map<int, vector<long>>();
        flowMods = map<int,vector<ControllerMessage*>>();
        eachNumFlow = map<int,long>();

        abnormalPackets = map<long,long>();
        abnormalFlows = map<long,long>();
        numPackets = map<int,vector<long>>();
        numFlows = map<int,vector<long>>();

        confirmed = vector<long>();

        flowThreshold = par("flowThreshold").doubleValue();
        numThreshold = par("numThreshold").doubleValue();
        normalCheck = par("normalCheck").longValue();
        abnormalCheck = par("abnormalCheck").longValue();

        prefixRsuId = getSystemModule()->par("prefixRsuId").longValue();
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
         prev[(*j).first] = NO_VEHICLE;
         listV.push_back((*j).first);
     }

     // Vehicle stack
     dist[source] = 0;
     prev[source] = source;
     while(listV.size() > 0) {

         // Getting the shortest distance
         int shortV = -1, shortDist = 9999;

         auto i = listV.begin();
         auto j = listV.begin();
         for(; i != listV.end(); i++) {
             if(dist[*i] < shortDist) {
                 shortV = *i;
                 shortDist = dist[*i];
                 j = i;
             }
         }
         if(shortV == -1) { // disconnected: (send to RSU or standby)
             int target = NO_VEHICLE;
             if(source != prefixRsuId) {
                 for(int i = 0; i < graph[source].size(); i++) { // Vehicle to RSU
                     int neighbor = graph[source][i];
                     if(neighbor >= prefixRsuId) { // RSU found!
                         target = neighbor;
                         break;
                     }
                 }
             }
             if(target != NO_VEHICLE) {
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
     return NO_VEHICLE;
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
        executeSentinel(vehicle, nm->getNumPackets(), eachNumFlow[vehicle]);
    }
}

void SdvnController::executeSentinel(int vehicle, long packetValue, long flowValue) {
    bool is_suspect, is_victim;
    double stdDevPacket, stdDevFlow;
    char buf[25];
    ControllerMessage* new_flow;

    auto suspect = find(suspicious.begin(), suspicious.end(), vehicle);
    is_suspect = (suspect != suspicious.end());

    auto victim = victims.find(vehicle);
    is_victim = (victim != victims.end());

    // If we do not have "normalCheck" values to work, just add the values and ignore.
    if(numPackets[vehicle].size() >= normalCheck && numFlows[vehicle].size() >= normalCheck) {

        /*
         * Phase 1: Detecting the attack
         */
        stdDevPacket = getMeanPlusStdDev(numPackets[vehicle]);
        stdDevFlow = getMeanPlusStdDev(numFlows[vehicle]);

        // Checking thresholds in order to classify an abnormal value
        if((packetValue > stdDevPacket*numThreshold) // Threshold for packet amount
                && (flowValue > stdDevFlow*flowThreshold) // Threshold for flow amount
                && (vehicle < prefixRsuId)) { // Check if is a RSU

            // The abnormal values were detected, starting counting.
            abnormalPackets[vehicle]++;
            abnormalFlows[vehicle]++;

            // Check if vehicle is receiving enough abnormal values to consider an attack
            if(abnormalPackets[vehicle] >= abnormalCheck && abnormalFlows[vehicle] >= abnormalCheck) {
                /*
                 * Phase 2: Mitigating the Attack
                 */
                if(is_suspect) {
                    EV_INFO << "SDVN Controller: ATTACK!\n";
                    EV_INFO << "SDVN Controller: Attack confirmed on Vehicle [" << vehicle << "]\n";
                    suspicious.erase(suspect);

                    // -- Statistics
                    auto me = find(confirmed.begin(), confirmed.end(), vehicle);
                    if(me == confirmed.end()) confirmed.push_back(vehicle);
                    sprintf(buf, "Victim Detected [%d]", vehicle);
                    recordScalar(buf, simTime());
                    // --
                } else {
                    EV_INFO << "SDVN Controller: Vehicle [" << vehicle;
                    EV_INFO << "] still over attack.\n";
                }

                // Restarting the counter of victim or "adding" the
                // suspect vehicle to the victim vector.
                victims[vehicle] = 0;

                // Getting the the leafs of flow tree
                buildFlowTree(vehicle);

                // Creating flows rules and sending
                // to bot destinations.
                for(auto level : flowTree) {
                    for(auto bot : level.second) {
                        stdDevPacket = getMeanPlusStdDev(numPackets[vehicle]);
                        if(numPackets[bot].back() >= stdDevPacket) {
                            new_flow = newFlow(bot, vehicle, S_DROP, NO_VEHICLE);
                            sendController(new_flow);
                        }
                    }
                }

            } else {
                // Abnormal values detected but the vehicle do not
                // receive enough values to consider an attack.
                if(!is_suspect) {
                    EV_INFO << "SDVN Controller: New possible attack detected on vehicle [";
                    EV_INFO << vehicle << "], starting monitoring.\n";
                    suspicious.push_back(vehicle);
                } else {
                    EV_INFO << "SDVN Controller: Suspected vehicle [";
                    EV_INFO << vehicle << "], still with abnormal values.\n";
                }
            }

            // The return is called here in order to bypass the code
            // where the flow rules are released and keep all flow rules
            // until the possible attack be resolved.
            eachNumFlow[vehicle] = 0;
            return;
        } else {
            // Checking if vehicle is in possible victim vector in order to remove
            if(is_suspect && !is_victim) {
                EV_INFO << "SDVN Controller: Vehicle [";
                EV_INFO << vehicle << "] removed from possible victims.\n";
                abnormalPackets[vehicle] = 0;
                abnormalFlows[vehicle] = 0;
                suspicious.erase(suspect);
                is_suspect = false;
            } else if(is_victim && victims[vehicle] > 2*abnormalCheck) {
                EV_INFO << "SDVN Controller: Vehicle [";
                EV_INFO << vehicle << "] with enough normal values, attack has stopped.\n";
                abnormalPackets[vehicle] = 0;
                abnormalFlows[vehicle] = 0;
                victims.erase(victim);
                is_victim = false;

                // -- Statistics
                sprintf(buf, "Victim Removed [%d]", vehicle);
                recordScalar(buf, simTime());
                // --
            } else if(is_victim) {
                EV_INFO << "SDVN Controller: Vehicle [";
                EV_INFO << vehicle << "] victim with normal values but still a victim.\n";
                victims[vehicle]++;

                // The return is called here in order to bypass the code
                // where the flow rules are released and keep all flow rules
                // until the possible attack be resolved.
                eachNumFlow[vehicle] = 0;
                return;
            }

            if(!is_victim && !is_suspect){
                // Removing the first values
                numPackets[vehicle].erase(numPackets[vehicle].begin());
                numFlows[vehicle].erase(numFlows[vehicle].begin());

                // Adding the new values
                numPackets[vehicle].push_back(packetValue);
                numFlows[vehicle].push_back(flowValue);
            }
        }
    } else {
        // The vehicle does not have enough values yet, just adding.
        numPackets[vehicle].push_back(packetValue);
        numFlows[vehicle].push_back(flowValue);
    }
    // releasing memory
    for(auto flow : flowMods[vehicle]) delete flow;
    flowMods[vehicle].clear();
    eachNumFlow[vehicle] = 0;
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
                eachNumFlow[destinationId]++;
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
    unsigned int i, size = li.size();
    double sum = 0, mean = 0, stddev = 0;
    for(i = 0; i < size; i++) sum += li[i]; // sum all values
    mean = sum/size; // arithmetic mean
    for(i = 0; i < size; i++) stddev += pow(li[i]-mean, 2);
    return (mean + sqrt(stddev/size)); // mean + standard derivation
}

void SdvnController::sendController(cMessage* msg) {
    sendDelayed(msg, 0.0001, outControl);
}

void SdvnController::finish() {
    cSimpleModule::finish();

    for(auto j : graph) j.second.clear(); // free neighbors lists

    if(sentinel) {
        for(auto vehicle : flowMods) // for all vehicles
            for(auto flow : vehicle.second) // for all flows on each vehicle
                delete flow;
        numPackets.clear();
        numFlows.clear();
    }
    graph.clear();
    timestamps.clear();
}

void SdvnController::buildFlowTree(int victim) {
    int i, counter, vehicle;
    vector<long> stack = vector<long>();

    i = 0;
    counter = 0;
    flowTree.clear();
    flowTree[i].push_back(victim);
    stack.push_back(victim);

    // Checking the stack
    while(!stack.empty()) {
        vehicle = stack.front();
        stack.erase(stack.begin());

        if(counter == 0) {
            counter = flowTree[i].size();
            i++;
        }

        for(auto flow : flowMods[victim]) {
            if(flow->getFlowId() == vehicle
                    && flow->getDestinationAddress() == victim
                    && flow->getFlowAction() == FORWARD) {
                flowTree[i].push_back(flow->getSourceVehicle());
                stack.push_back(flow->getSourceVehicle());
            }
        }
        counter--;
    }
}

/*
 * Distributed Methods
 *
 */

bool SdvnController::findLocalNode(int id) {
    auto it = graph.find(id);
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
    stringstream ss;
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
