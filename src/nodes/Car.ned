package sdvn.nodes;

import sdvn.modules.SdvnSwitch;
import sdvn.modules.SdvnMobility;
import sdvn.application.SdvnPing;
import org.car2x.veins.base.modules.*;
import org.car2x.veins.modules.nic.Nic80211p;

module Car
{
    parameters:
        @display("bgb=238,255");
        string nicType = default("Nic80211p");
        string mobilityType = default("SdvnMobility");
    gates:
        input veinsradioIn;

    submodules:
        appl: SdvnPing {
            parameters:
                @display("p=112,48;i=block/app2");
        }
        nic: <nicType> like org.car2x.veins.modules.nic.INic80211p {
            parameters:
                @display("p=50,207");
        }
        switcher: SdvnSwitch {
            parameters:
                @display("p=112,127;i=block/fork");
        }
        mobility: <mobilityType> like org.car2x.veins.base.modules.IMobility {
            parameters:
                @display("p=44,127;i=block/cogwheel");
        }

    connections allowunconnected:
        switcher.toApp --> appl.fromSwitch;
        switcher.fromApp <-- appl.toSwitch;

        nic.upperControlOut --> switcher.lowerControlIn;
        nic.upperControlIn <-- switcher.lowerControlOut;

        nic.upperLayerOut --> switcher.lowerLayerIn;
        nic.upperLayerIn <-- switcher.lowerLayerOut;

        veinsradioIn --> nic.radioIn;
}
