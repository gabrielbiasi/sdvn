//
// Generated file, do not edit! Created by nedtool 5.0 from messages/ControllerMessage.msg.
//

#ifndef __CONTROLLERMESSAGE_M_H
#define __CONTROLLERMESSAGE_M_H

#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0500
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



// cplusplus {{
#include "veins/modules/messages/WaveShortMessage_m.h"
// }}

/**
 * Class generated from <tt>messages/ControllerMessage.msg:7</tt> by nedtool.
 * <pre>
 * message ControllerMessage extends WaveShortMessage
 * {
 *     int sourceVehicle;
 * 
 *     int messageType;
 *     int flowAction;
 *     int flowId;
 * 
 *     simtime_t timestamp;
 *     simtime_t lastUsed;
 * 
 *     simtime_t hardTimeout;
 *     simtime_t idleTimeout;
 * 
 *     int destinationAddress;
 * }
 * </pre>
 */
class ControllerMessage : public ::WaveShortMessage
{
  protected:
    int sourceVehicle;
    int messageType;
    int flowAction;
    int flowId;
    ::omnetpp::simtime_t timestamp;
    ::omnetpp::simtime_t lastUsed;
    ::omnetpp::simtime_t hardTimeout;
    ::omnetpp::simtime_t idleTimeout;
    int destinationAddress;

  private:
    void copy(const ControllerMessage& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const ControllerMessage&);

  public:
    ControllerMessage(const char *name=nullptr, int kind=0);
    ControllerMessage(const ControllerMessage& other);
    virtual ~ControllerMessage();
    ControllerMessage& operator=(const ControllerMessage& other);
    virtual ControllerMessage *dup() const {return new ControllerMessage(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b);

    // field getter/setter methods
    virtual int getSourceVehicle() const;
    virtual void setSourceVehicle(int sourceVehicle);
    virtual int getMessageType() const;
    virtual void setMessageType(int messageType);
    virtual int getFlowAction() const;
    virtual void setFlowAction(int flowAction);
    virtual int getFlowId() const;
    virtual void setFlowId(int flowId);
    virtual ::omnetpp::simtime_t getTimestamp() const;
    virtual void setTimestamp(::omnetpp::simtime_t timestamp);
    virtual ::omnetpp::simtime_t getLastUsed() const;
    virtual void setLastUsed(::omnetpp::simtime_t lastUsed);
    virtual ::omnetpp::simtime_t getHardTimeout() const;
    virtual void setHardTimeout(::omnetpp::simtime_t hardTimeout);
    virtual ::omnetpp::simtime_t getIdleTimeout() const;
    virtual void setIdleTimeout(::omnetpp::simtime_t idleTimeout);
    virtual int getDestinationAddress() const;
    virtual void setDestinationAddress(int destinationAddress);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const ControllerMessage& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, ControllerMessage& obj) {obj.parsimUnpack(b);}


#endif // ifndef __CONTROLLERMESSAGE_M_H

