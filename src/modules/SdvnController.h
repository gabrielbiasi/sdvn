#ifndef SdvnController_H
#define SdvnController_H

#include <omnetpp.h>
#include <cmath>
#include <algorithm>

#include "SdvnTypes.h"
#include "messages/ControllerMessage_m.h"
#include "messages/NeighborMessage_m.h"

using std::map;
using std::find;
using std::vector;
using std::stringstream;

class SdvnController : public cSimpleModule {

protected:
    // Gates
    int inControl;
    int outControl;

    bool debug;
    map<int, vector<int>> graph;

    // Sentinel Variables
    map<int, long> victims;
    vector<long> suspicious;
    map<int, vector<ControllerMessage*>> flowMods;
    map<long,long> abnormalPackets;
    map<long,long> abnormalFlows;
    map<int, vector<long>> numPackets;
    map<int, vector<long>> numFlows;
    map<int, vector<long>> flowTree;

    long normalCheck;
    long abnormalCheck;
    double flowThreshold;
    double numThreshold;

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

    void executeSentinel(int vehicle, long packetValue, long flowValue);
    double getMeanPlusStdDev(const vector<long>&  li);
    void buildFlowTree(int victim);

    void clearFarNeighbors();

public:
    int findTarget(int id);
    bool findLocalNode(int id);
    map<int, long> eachNumFlow;

    bool sentinel;
    map<int, simtime_t> timestamps;

    // Sentinel Statistics
    vector<long> confirmed;
};

#endif
