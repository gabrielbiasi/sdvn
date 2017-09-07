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

#ifndef MODULES_SDVNMOBILITY_H_
#define MODULES_SDVNMOBILITY_H_

#include <TraCIMobility.h>
using Veins::TraCICommandInterface;

class SdvnMobility: public Veins::TraCIMobility {
protected:
    double speed;
    TraCICommandInterface::Vehicle* traciVehicle;

    virtual void initialize(int stage);
    virtual void updateDisplayString();
    virtual void finish();
public:
    simtime_t getTotalTime();

};

#endif /* MODULES_SDVNMOBILITY_H_ */
