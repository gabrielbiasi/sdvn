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

simple SdvnController {
   parameters:
        @class(SdvnController);
        bool   debug         = default(false);
        bool   sentinel      = default(true);

        double dropAfter     = default(10s) @unit(s);
		double hardTimeout   = default(10s) @unit(s);
		double idleTimeout   = default(5s)  @unit(s);

		int    normalCheck   = default(10);
		int    abnormalCheck = default(3);
		double flowThreshold = default(1);
		double numThreshold  = default(1);

    gates:
        input inControl;
        output outControl;
}
