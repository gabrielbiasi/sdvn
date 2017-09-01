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

simple Attacker {
    parameters:
        @display("i=old/telnet");
        double attackScale   = default(5);
        double attackerRate  = default(0.1);
        int    numVictims    = default(0);

        double checkInterval = default(1s)   @unit(s);
        double startTime     = default(60s)  @unit(s);
        double duration      = default(10s)  @unit(s);     
        double victimWarmUp  = default(11s)  @unit(s);
}
