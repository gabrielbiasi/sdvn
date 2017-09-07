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

#include "modules/SdvnMobility.h"

Define_Module(SdvnMobility);

void SdvnMobility::initialize(int stage) {
    TraCIMobility::initialize(stage);
    traciVehicle = getVehicleCommandInterface();

    speed = par("speed").doubleValue();
    EV_INFO << "[" << getSpeed() << "]" << endl;
    //traciVehicle->setSpeed(speed);
}

void SdvnMobility::updateDisplayString() {
    TraCIMobility::updateDisplayString();
}

simtime_t SdvnMobility::getTotalTime() {
    return statistics.totalTime;
}

void SdvnMobility::finish() {
    TraCIMobility::finish();
}
