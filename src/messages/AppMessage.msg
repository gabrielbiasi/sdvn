cplusplus {{
#include "veins/modules/messages/WaveShortMessage_m.h"
}}

class WaveShortMessage;

message AppMessage extends WaveShortMessage {
    long id;
	long sourceAddress;
	long destinationAddress;
	long TTL;
	string payload;
}
