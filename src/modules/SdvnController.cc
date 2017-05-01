#include "modules/SdvnController.h"

Define_Module(SdvnController);

using std::vector;
using std::stringstream;

#include <stdio.h>

void SdvnController::initialize(int stage) {
    if (stage == 0) {
        debug = par("debug").boolValue();

        // Timeouts
        hardTimeout = par("hardTimeout").doubleValue();
        idleTimeout = par("idleTimeout").doubleValue();
        dropAfter = par("dropAfter").doubleValue();

        // Gates
        toLteBase = findGate("toLteBase");
        fromLteBase = findGate("fromLteBase");

        // Adjacency matrix
        graph = std::map<int, int*>();
        timestamps = std::map<int, simtime_t>();

        prefixRsuId = 10000;
    }
}

int SdvnController::runSimpleDijkstra(int source, int destination) {

     // Inicia os mapas com distâncias infinitas para
     // executar o Dijkstra.

     std::vector<int> listV = std::vector<int>();
     std::map<int,int> dist = std::map<int,int>();
     std::map<int,int> prev = std::map<int,int>();

     auto j = graph.begin();
     for(j = graph.begin(); j != graph.end(); j++) {
         dist[(*j).first] = 9999;
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
         if(shortV == -1) { // Desconexo: (envia para o RSU ou aguarde um RSU)
             int target = -1;
             if(source != prefixRsuId) {
                 for(int i = 1; i < graph[source][0]+1; i++) { // Vehicle to RSU
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
                 EV_INFO << "SDVN Controller: WAIT! \n";
             }
             return target; // Retorna o vizinho que possui mais conexões!

         } else if(shortV == destination) { // Alvo!
             int target = destination;
             while(prev[target] != source) target = prev[target];
             EV_INFO << "SDVN Controller: FOUND!, forward to most closer: [" << target << "]\n";
             return target; // Retorna o pulo mais próximo!
         }

         // Remove do vetor
         listV.erase(j);

         // Processo simples de menor caminho
         for(int i = 1; i < graph[shortV][0]+1; i++) { // graph[shortV][0] --> tamanho do vetor
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

    if(simTime() < timestamps[vehicle] + dropAfter) { // Check if vehicle still sending neighbor messages
        flowDestination = runSimpleDijkstra(vehicle, destination);
        flow->setFlowAction(flowDestination != -1 ? FORWARD : WAIT);
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
    int vehicle = m->getSourceVehicle();
    int* p = graph[vehicle]; // Captura o ponteiro
    graph.erase(vehicle); // Remove do map
    delete p; // free memory

    int* new_neighbors = new int[m->getNeighborsArraySize()];
    for(unsigned int i = 0; i < m->getNeighborsArraySize(); i++) {
        new_neighbors[i] = m->getNeighbors(i); // Coloca os novos vizinhos
    }
    graph[vehicle] = new_neighbors;
    timestamps[vehicle] = m->getTimestamp(); // talvez aqui
}


void SdvnController::handleMessage(cMessage *msg) {

    if(msg->getArrivalGateId() == fromLteBase) {
        if(msg->getKind() == 011) { // 011 -> Neighbor Message
            //EV_INFO << "SDVN Controller: Neighbor Update Received\n";
            updateNetworkGraph(msg);
            delete msg;
            return;
        }

        if(msg->getKind() == 012) { // 012 -> Controller Message
            EV_INFO << "SDVN Controller:  Packet In Received from ["<< ((ControllerMessage*) msg)->getSourceVehicle() << "]\n";
            ControllerMessage* flow_mod = prepareNewFlowMod(msg);
            sendLte(flow_mod);
            delete msg;
            return;
        }

    } else if(msg->isSelfMessage()) {
        EV_INFO << "CONTROLADOR: Self Message\n";
    } else {
        EV_INFO << "CONTROLADOR: Algo errado no controlador!\n";
    }
}

void SdvnController::sendLte(cMessage* msg) {
    send(msg, toLteBase);
}

void SdvnController::finish() {
    cSimpleModule::finish();
    for(auto &j : graph) delete j.second; // free neighbors lists
    graph.clear();
}
