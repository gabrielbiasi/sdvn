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

#include "SdvnTypes.h"

#include "messages/AppMessage_m.h"
#include "messages/ControllerMessage_m.h"
#include "messages/NeighborMessage_m.h"
#include "application/SdvnPing.h"

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;

using std::vector;

/**
 * Software Defined Vehicular Network Switch
 */
class SdvnSwitch : public BaseWaveApplLayer {
    public:
        virtual void initialize(int stage);
        virtual void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details);

    protected:
        TraCIMobility* mobility;
        TraCICommandInterface* traci;
        TraCICommandInterface::Vehicle* traciVehicle;
        AnnotationManager* annotations;

        std::string type;
        int prefixRsuId;

        std::vector<AppMessage*> packetInBuffer;
        int maxPacketInBuffer;

        std::vector<AppMessage*> packetStandbyBuffer;
        int maxPacketStandbyBuffer;
        double standbyTime;

        double controllerBeaconsInterval;
        cMessage* controllerBeaconEvent;
        std::vector<int> currentNeighbors;

        std::vector<ControllerMessage*> flowTable;
        int maxFlowRules;

        // App Gates
        int fromApp;
        int toApp;

        // Controller Gates
        int fromLteBase;
        int toLteBase;

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
        void onLte(ControllerMessage* msg);
        void sendLte(cMessage* msg);
        void cleanFlowTable();

        void addRsuNeighbors();
        void sendRSU(WaveShortMessage* msg);
        bool isVehicle();
};

#endif
