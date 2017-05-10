#ifndef SDVNTYPES_H_
#define SDVNTYPES_H_

enum MESSAGETYPES {
    PACKET_IN = 0,
    FLOW_MOD = 1
};

enum ACTIONS {
    FORWARD = 0,
    DROP = 1,
    STANDBY = 2
};

enum ATTACKS {
    NO_ATTACK = 0,
    A_BLACK_HOLE = 1,
    A_DDOS = 2,
    A_OVERFLOW = 3
};

#endif /* SDVNTYPES_H_ */
