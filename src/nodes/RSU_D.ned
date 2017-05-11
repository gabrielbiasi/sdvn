package sdvn.nodes;

import org.car2x.veins.base.modules.*;
import org.car2x.veins.modules.nic.Nic80211p;
import sdvn.modules.SdvnSwitch;
import sdvn.modules.SdvnController;

module RSU_D
{
    parameters:
        string nicType      = default("Nic80211p");
        double rsuDelay     = default(0.01s) @unit(s);
    gates:
        input veinsradioIn;
    submodules:
        controller: SdvnController {
            parameters:
                @display("p=60,50");
        }
        switcher: SdvnSwitch {
            parameters:
                @display("p=60,50");
        }
        nic: <nicType> like org.car2x.veins.modules.nic.INic80211p {
            parameters:
                @display("p=60,166");
        }
        mobility: BaseMobility {
            parameters:
                @display("p=130,172;i=block/cogwheel");
        }
        
    connections allowunconnected:
        switcher.toController --> controller.inControl;
        switcher.fromController <-- controller.outControl;

        nic.upperLayerOut --> switcher.lowerLayerIn;
        nic.upperLayerIn <-- switcher.lowerLayerOut;
        nic.upperControlOut --> switcher.lowerControlIn;
        nic.upperControlIn <-- switcher.lowerControlOut;

        veinsradioIn --> nic.radioIn;
}
