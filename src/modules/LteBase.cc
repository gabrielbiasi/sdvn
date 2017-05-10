#include "modules/LteBase.h"

Define_Module(LteBase);

void LteBase::initialize(int stage) {
    if (stage == 0) {
        debug = par("debug").boolValue();
        power = par("power").doubleValue();

        lteDelay = par("lteDelay").doubleValue();
        prefixRsuId = par("prefixRsuId").longValue();

        // Gates
        radioIn = findGate("radioIn");
        toController = findGate("toController");
        fromController = findGate("fromController");
    }
}

void LteBase::handleMessage(cMessage *msg) {

    if (msg->isSelfMessage()) {
        EV_INFO << "LTE Base: Self-Message!";
    } else if (msg->getArrivalGateId() == radioIn){
        EV_INFO << "LTE BASE: Message received from Vehicle/RSU, retransmitting...\n";
        sendController(msg);

    } else if (msg->getArrivalGateId() == fromController) {
        EV_INFO << "LTE BASE: Message received from Controller, retransmitting...\n";
        int destination = ((ControllerMessage*) msg)->getSourceVehicle();
        if(destination >= prefixRsuId) {
            sendRSU(msg, destination);
        } else {
            sendSwitch(msg, destination);
        }

    } else {
        EV_INFO << "LTE Base: unknown message!\n";
    }
}

void LteBase::sendController(cMessage* msg)  {
    send(msg, toController);
}

void LteBase::sendSwitch(cMessage* msg, int vehicle) {
    std::stringstream ss;
    ss << ".vehicle[" << vehicle << "].switcher";
    cModule* module = getSystemModule()->getModuleByPath(ss.str().c_str());
    if(module != nullptr) { // vehicle might be finished your route
        cSimpleModule::sendDirect(msg, lteDelay, 0, module, "fromLteBase");
    } else {
        delete msg;
    }
}

void LteBase::sendRSU(cMessage* msg, int rsu_id) {
    std::stringstream ss;
    ss << ".rsu[" << (rsu_id-prefixRsuId) << "].switcher";
    cModule* module = getSystemModule()->getModuleByPath(ss.str().c_str());
    cSimpleModule::sendDirect(msg, lteDelay, 0, module, "fromLteBase");
}


void LteBase::finish() {
    cSimpleModule::finish();
}

