#ifndef SdvnController_H
#define SdvnController_H

#include <omnetpp.h>
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"

using Veins::TraCIScenarioManager;
using Veins::TraCIScenarioManagerAccess;

#include "messages/ControllerMessage_m.h"
#include "messages/NeighborMessage_m.h"

using std::map;
using std::vector;

class SdvnController : public cSimpleModule {

protected:
    // Gates
    int inControl;
    int outControl;

    bool debug;
    map<int, vector<int>> graph;
    map<int, simtime_t> timestamps;

    double hardTimeout;
    double idleTimeout;
    double dropAfter;

    int prefixRsuId;

    enum MESSAGETYPES {
        PACKET_IN = 0,
        FLOW_MOD = 1
    };

    enum ACTIONS {
        FORWARD = 0,
        DROP = 1,
        STANDBY = 2
    };

    enum ATTACKS {
        NO_ATTACK = 0,
        A_BLACK_HOLE = 1,
        A_DDOS = 2,
        A_OVERFLOW = 3
    };

    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    void sendController(cMessage* msg);
    void updateNetworkGraph(cMessage* message);
    ControllerMessage* prepareNewFlowMod(cMessage* message);
    int runSimpleDijkstra(int source, int destination);

public:
    vector<int>* getNodesId();
};

#endif
