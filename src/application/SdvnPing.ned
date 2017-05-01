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

package sdvn.application;

simple SdvnPing {
	parameters:
	    int    attackMode = default(0);
	    double attackerRate = default(0.1);
	    double startTime = default(60s) @unit(s);
	    double duration = default(60s) @unit(s);
	    
	    
		double warmUp = default(0s) @unit(s);
		double packetInterval = default(1s) @unit(s);
    gates:
        output toSwitch;
        input  fromSwitch;
}
