package sdvn.simulations;

import sdvn.nodes.IRsu;
import org.car2x.veins.nodes.Scenario;

import org.car2x.veins.base.connectionManager.ConnectionManager;
import org.car2x.veins.base.modules.BaseWorldUtility;
import org.car2x.veins.modules.mobility.traci.TraCIScenarioManagerLaunchd;
import org.car2x.veins.modules.obstacle.ObstacleControl;
import org.car2x.veins.modules.world.annotations.AnnotationManager;

network Sdvn
{
    parameters:
        double obstaclesPosX;
        double obstaclesPosY;
        double annotationPosX;
        double annotationPosY;
        double connectionPosX;
        double connectionPosY;
        double worldPosX;
        double worldPosY;
        double managerPosX;
        double managerPosY;
        
        double playgroundSizeX @unit(m);
        double playgroundSizeY @unit(m);
        double playgroundSizeZ @unit(m);

        string map;
        string rsuType;
        int architecture;
        int numRsu;
        @display("bgb=$playgroundSizeX,$playgroundSizeY;bgi=$map;bgu=m");

    submodules:
        rsu[numRsu]: <rsuType> like IRsu;

        obstacles: ObstacleControl {
            @display("p=$obstaclesPosX,$obstaclesPosY;i=misc/town;is=s");
        }
        annotations: AnnotationManager {
            @display("p=$annotationPosX,$annotationPosY;i=msg/paperclip;is=s");
        }
        connectionManager: ConnectionManager {
            parameters:
                @display("p=$connectionPosX,$connectionPosY;i=abstract/multicast;is=s");
        }
        world: BaseWorldUtility {
            parameters:
                playgroundSizeX = playgroundSizeX;
                playgroundSizeY = playgroundSizeY;
                playgroundSizeZ = playgroundSizeZ;
                @display("p=$worldPosX,$worldPosY;i=misc/globe;is=s");
        }
        manager: TraCIScenarioManagerLaunchd {
            parameters:
                @display("p=$managerPosX,$managerPosY;i=block/network2;is=s");
        }
    connections allowunconnected:
}
