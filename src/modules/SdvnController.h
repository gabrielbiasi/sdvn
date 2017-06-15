#ifndef SdvnController_H
#define SdvnController_H

#include <omnetpp.h>
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"

using Veins::TraCIScenarioManager;
using Veins::TraCIScenarioManagerAccess;

#include "SdvnTypes.h"
#include "messages/ControllerMessage_m.h"
#include "messages/NeighborMessage_m.h"

using std::map;
using std::vector;

class SdvnController : public cSimpleModule {

protected:
    // Gates
    int inControl;
    int outControl;

    cMessage* checkFlow;
    double checkFlowInterval;

    bool debug;
    map<int, vector<int>> graph;
    map<int, simtime_t> timestamps;
    map<int, long> numPackets;

    double hardTimeout;
    double idleTimeout;
    double dropAfter;

    int prefixRsuId;
    int architecture;

    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    void sendController(cMessage* msg);
    void updateNetworkGraph(cMessage* message);
    ControllerMessage* newFlow(int sourceId, int destinationId, int flowAction, int flowDestination);
    int runSimpleDijkstra(int source, int destination);

    void clearFarNeighbors();

public:
    int findTarget(int id);
    bool findLocalNode(int id);
};

#endif
