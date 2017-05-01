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

import sdvn.modules.SdvnController;
import sdvn.modules.LteBase;
import sdvn.nodes.RSU;
import org.car2x.veins.nodes.Scenario;
import ned.DatarateChannel;

network SdvnScenario extends Scenario
{	    
    parameters:
        int numRsu = default(4);
    	@display("bgb=1050,1050;bgi=background/map,s");
	types:
        channel EthernetLine extends DatarateChannel {
            delay = 50ns;
            datarate = 1000Mbps;
        }
    submodules:
        rsu[numRsu]: RSU {
            parameters:
                @display("i=device/antennatower;is=n");
        }
        controller: SdvnController {
            parameters:
                @display("p=1400,700;b=9,10,oval;i=device/server");
        }
        base: LteBase {
            parameters:
                @display("p=1200,700;b=9,10,oval;i=device/antennatower");
        }
    connections:
        base.toController --> EthernetLine --> controller.fromLteBase;
        base.fromController <-- EthernetLine <--  controller.toLteBase;
}
