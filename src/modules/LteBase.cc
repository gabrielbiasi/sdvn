#include "modules/LteBase.h"

Define_Module(LteBase);

void LteBase::initialize(int stage) {
    if (stage == 0) {
        debug = par("debug").boolValue();
        power = par("power").doubleValue();


        lteDelay = par("lteDelay").doubleValue();

        // Gates
        radioIn = findGate("radioIn");
        toController = findGate("toController");
        fromController = findGate("fromController");
    }
}

void LteBase::handleMessage(cMessage *msg) {

    if (msg->isSelfMessage()) {
        EV_INFO << "LTE Base: Self-Message!";
    } else if (msg->getArrivalGateId() == radioIn){ // Mensagem recebida por um veículo
        // Apenas repassa a mensagem
        EV_INFO << "LTE BASE: Message received from Vehicle, retransmitting...\n";

        if(ControllerMessage* v = dynamic_cast<ControllerMessage*>(msg)) {
            EV_INFO << "[" << v->getSourceVehicle() << "-" << v->getDestinationAddress() << "]\n";
        }

        sendController(msg);

    } else if (msg->getArrivalGateId() == fromController) { // Mensagem recebida pelo Controlador

        // Apenas repassa a mensagem
        EV_INFO << "LTE BASE: Message received from Controller, retransmitting...\n";
        int destination = ((ControllerMessage*) msg)->getSourceVehicle();
        if(destination >= 10000) {
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
    ss << "SdvnScenario.vehicle[" << vehicle << "].switcher";
    cModule* module = getModuleByPath(ss.str().c_str());
    if(module != nullptr) { // veículo pode ter terminado seu trajeto
        cSimpleModule::sendDirect(msg, lteDelay, 0, module, "fromLteBase");
    } else {
        delete msg;
    }
}

void LteBase::sendRSU(cMessage* msg, int rsu_id) {
    std::stringstream ss;
    ss << "SdvnScenario.rsu[" << (rsu_id-10000) << "].switcher";
    cModule* module = getModuleByPath(ss.str().c_str());
    cSimpleModule::sendDirect(msg, lteDelay, 0, module, "fromLteBase");
}


void LteBase::finish() {
    cSimpleModule::finish();
}

