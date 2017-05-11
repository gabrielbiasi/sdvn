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

package sdvn.simulations;

import sdvn.nodes.RSU_D;
import org.car2x.veins.nodes.Scenario;
import ned.DatarateChannel;

network DistributedSdvn extends Scenario
{	    
    parameters:
        int numRsu       = default(4);
        int architecture = default(1);
    	@display("bgb=1050,1050;bgi=background/map,s");

    submodules:
        rsu[numRsu]: RSU_D {
            parameters:
                @display("i=device/antennatower;is=n");
        }
    connections:
}
