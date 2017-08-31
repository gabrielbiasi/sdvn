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

package sdvn.modules;
import org.car2x.veins.modules.application.ieee80211p.BaseWaveApplLayer;

simple SdvnSwitch extends BaseWaveApplLayer {

	parameters:
	    @class(SdvnSwitch);
		double controllerBeaconsInterval = default(1s) @unit(s);
		double standbyTime = default(5s) @unit(s);
		
		int maxPacketInBuffer = default(1000);
		int maxPacketStandbyBuffer = default(1000);
		int maxFlowRules = default(50);
		 
    gates:
        // Gates for Applications
        input fromApp;
        output toApp;
        
        // Gates for RSUs
        input fromRsu;
        output toRsu;
        
        // Gates for LTE
        input fromController;
        output toController;
}
