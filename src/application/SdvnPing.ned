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
	    @class(SdvnPing);
		double warmUp        = default(0s) @unit(s);
		double burstSize     = default(1);
		double burstInterval = default(1s) @unit(s);

	    int    attackMode    = default(0);
	    double attackerRate  = default(0.1);
	    string startTime     = default("");
	    string duration      = default("");
	    string attackSize    = default("");
	    string victims       = default("");

    gates:
        output toSwitch;
        input  fromSwitch;
}
