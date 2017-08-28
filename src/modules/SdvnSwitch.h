//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//
#ifndef SdvnSwitch_H
#define SdvnSwitch_H

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"

#include "SdvnTypes.h"

#include "messages/AppMessage_m.h"
#include "messages/ControllerMessage_m.h"
#include "messages/NeighborMessage_m.h"
#include "application/SdvnPing.h"
#include "modules/SdvnController.h"

using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;
using Veins::TraCICommandInterface;

using std::vector;
using std::string;

/**
 * Software Defined Vehicular Network Switch
 */
class SdvnSwitch : public BaseWaveApplLayer {
    public:
        virtual void initialize(int stage);
        virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);
        int botFlowId;
    protected:
        TraCIMobility* mobility;
        TraCICommandInterface* traci;
        TraCICommandInterface::Vehicle* traciVehicle;

        int prefixRsuId;

        vector<AppMessage*> packetInBuffer;
        int maxPacketInBuffer;

        vector<WaveShortMessage*> rsuQueue;

        vector<AppMessage*> packetStandbyBuffer;
        int maxPacketStandbyBuffer;
        double standbyTime;

        double controllerBeaconsInterval;
        cMessage* controllerBeaconEvent;
        vector<int> currentNeighbors;

        vector<ControllerMessage*> flowTable;
        int maxFlowRules;

        // Information to controller about the traffic
        cOutVector numPacketsV;
        cOutVector numFlowsV;
        long numPackets;

        // The bot will select a neighbor to forward the attack

        // App Gates
        int fromApp;
        int toApp;

        // Controller Gates
        int fromController;
        int toController;

        // RSU Gates
        int fromRsu;
        int toRsu;

        // mini-controller for Distributed Mode
        int architecture;

        // Scalars
        int droppedRule;
        int droppedByTTL;
        int droppedByRule;
        int droppedByStandbyOverflow;
        int droppedByInOverflow;

        SdvnPing* appl;

    protected:
        virtual void handleMessage(cMessage* msg);

        virtual void onBeacon(WaveShortMessage* wsm);
        virtual void onData(WaveShortMessage* wsm);
        virtual void handlePositionUpdate(cObject* obj);
        virtual void handleSelfMsg(cMessage* msg);
        virtual void finish();

        bool performAttack(AppMessage* msg);

        void onApplication(AppMessage* msg);
        void sendApplication(AppMessage* msg);
        void onController(ControllerMessage* msg);
        void sendController(cMessage* msg);
        void cleanFlowTable();

        void addRsuNeighbors();
        void sendRSU(WaveShortMessage* msg);
        bool isVehicle();
        bool hasActiveVehicle();
};

#endif
