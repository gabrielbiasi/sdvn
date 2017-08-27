package sdvn.simulations;

import sdvn.modules.SdvnController;
import sdvn.modules.LteBase;

network CentralizedSdvn extends Sdvn
{	    
    parameters:
        double controllerPosX = default(0);
        double controllerPosY = default(0);
        double basePosX = default(0);
        double basePosY = default(0);

    submodules:
        controller: SdvnController {
            parameters:
                @display("p=$controllerPosX,$controllerPosY;b=9,10,oval;i=device/server");
        }
        base: LteBase {
            parameters:
                @display("p=$basePosX,$basePosY;b=9,10,oval;i=device/antennatower");
        }
    connections:
        base.toController --> controller.inControl;
        base.fromController <--  controller.outControl;
}
