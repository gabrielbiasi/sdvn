#ifndef LteBase_H
#define LteBase_H

#include <omnetpp.h>
#include "messages/ControllerMessage_m.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"

class LteBase : public cSimpleModule {

protected:
    // Gates
    int toController;
    int fromController;
    int radioIn;

    bool debug;
    double power;
    double lteDelay;
    long prefixRsuId;

protected:
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void finish();

    void sendSwitch(cMessage* msg, int vehicle);
    void sendController(cMessage* msg);
    void sendRSU(cMessage* msg, int rsu_id);
};

#endif
