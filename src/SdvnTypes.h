#ifndef SDVNTYPES_H_
#define SDVNTYPES_H_

enum MESSAGETYPES {
    PACKET_IN = 0,
    FLOW_MOD = 1
};

enum INVALIDTYPES {
    NO_VEHICLE = -1
};

enum ARCHITECTURE {
    CENTRALIZED = 0,
    DISTRIBUTED = 1
};

enum ACTIONS {
    FORWARD = 0,
    DROP = 1,
    STANDBY = 2,
    S_DROP = 3
};

enum ATTACK_MODES {
    NO_ATTACK = 0,
    DETECTION = 1,
    MITIGATION = 2
};

#endif /* SDVNTYPES_H_ */
