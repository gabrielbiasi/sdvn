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
}

void SdvnMobility::updateDisplayString() {
    TraCIMobility::updateDisplayString();

    /*ASSERT(-M_PI <= angle);
    ASSERT(angle < M_PI);

    vehicleApp = dynamic_cast<SdvnPing*>(getParentModule()->getModuleByPath(".appl"));

    if(vehicleApp->attacker) {
        getParentModule()->getDisplayString().setTagArg("i", 0, "vehicle/red");
    } else {
        getParentModule()->getDisplayString().setTagArg("i", 0, "vehicle/white");
    }
    getParentModule()->getDisplayString().setTagArg("is", 0, "vs");

    if (angle < -M_PI + 0.5 * M_PI_4 * 1) {
        getParentModule()->getDisplayString().setTagArg("t", 0, "\u2190");
        getParentModule()->getDisplayString().setTagArg("b", 0, "4");
        getParentModule()->getDisplayString().setTagArg("b", 1, "2");
    }
    else if (angle < -M_PI + 0.5 * M_PI_4 * 3) {
        getParentModule()->getDisplayString().setTagArg("t", 0, "\u2199");
        getParentModule()->getDisplayString().setTagArg("b", 0, "3");
        getParentModule()->getDisplayString().setTagArg("b", 1, "3");
    }
    else if (angle < -M_PI + 0.5 * M_PI_4 * 5) {
        getParentModule()->getDisplayString().setTagArg("t", 0, "\u2193");
        getParentModule()->getDisplayString().setTagArg("b", 0, "2");
        getParentModule()->getDisplayString().setTagArg("b", 1, "4");
    }
    else if (angle < -M_PI + 0.5 * M_PI_4 * 7) {
        getParentModule()->getDisplayString().setTagArg("t", 0, "\u2198");
        getParentModule()->getDisplayString().setTagArg("b", 0, "3");
        getParentModule()->getDisplayString().setTagArg("b", 1, "3");
    }
    else if (angle < -M_PI + 0.5 * M_PI_4 * 9) {
        getParentModule()->getDisplayString().setTagArg("t", 0, "\u2192");
        getParentModule()->getDisplayString().setTagArg("b", 0, "4");
        getParentModule()->getDisplayString().setTagArg("b", 1, "2");
    }
    else if (angle < -M_PI + 0.5 * M_PI_4 * 11) {
        getParentModule()->getDisplayString().setTagArg("t", 0, "\u2197");
        getParentModule()->getDisplayString().setTagArg("b", 0, "3");
        getParentModule()->getDisplayString().setTagArg("b", 1, "3");
    }
    else if (angle < -M_PI + 0.5 * M_PI_4 * 13) {
        getParentModule()->getDisplayString().setTagArg("t", 0, "\u2191");
        getParentModule()->getDisplayString().setTagArg("b", 0, "2");
        getParentModule()->getDisplayString().setTagArg("b", 1, "4");
    }
    else if (angle < -M_PI + 0.5 * M_PI_4 * 15) {
        getParentModule()->getDisplayString().setTagArg("t", 0, "\u2196");
        getParentModule()->getDisplayString().setTagArg("b", 0, "3");
        getParentModule()->getDisplayString().setTagArg("b", 1, "3");
    }
    else {
        getParentModule()->getDisplayString().setTagArg("t", 0, "\u2190");
        getParentModule()->getDisplayString().setTagArg("b", 0, "4");
        getParentModule()->getDisplayString().setTagArg("b", 1, "2");
    }*/
}

void SdvnMobility::finish() {
    TraCIMobility::finish();
}
