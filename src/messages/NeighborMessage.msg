cplusplus {{
#include "veins/modules/messages/WaveShortMessage_m.h"
}}

class WaveShortMessage;

message NeighborMessage extends WaveShortMessage {
	int sourceVehicle;
	simtime_t timestamp;
    int neighbors[];
    int numPackets;
}
