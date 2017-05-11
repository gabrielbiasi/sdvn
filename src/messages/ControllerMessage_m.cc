//
// Generated file, do not edit! Created by nedtool 5.0 from messages/ControllerMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "ControllerMessage_m.h"

namespace omnetpp {

// Template pack/unpack rules. They are declared *after* a1l type-specific pack functions for multiple reasons.
// They are in the omnetpp namespace, to allow them to be found by argument-dependent lookup via the cCommBuffer argument

// Packing/unpacking an std::vector
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::vector<T,A>& v)
{
    int n = v.size();
    doParsimPacking(buffer, n);
    for (int i = 0; i < n; i++)
        doParsimPacking(buffer, v[i]);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::vector<T,A>& v)
{
    int n;
    doParsimUnpacking(buffer, n);
    v.resize(n);
    for (int i = 0; i < n; i++)
        doParsimUnpacking(buffer, v[i]);
}

// Packing/unpacking an std::list
template<typename T, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::list<T,A>& l)
{
    doParsimPacking(buffer, (int)l.size());
    for (typename std::list<T,A>::const_iterator it = l.begin(); it != l.end(); ++it)
        doParsimPacking(buffer, (T&)*it);
}

template<typename T, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::list<T,A>& l)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        l.push_back(T());
        doParsimUnpacking(buffer, l.back());
    }
}

// Packing/unpacking an std::set
template<typename T, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::set<T,Tr,A>& s)
{
    doParsimPacking(buffer, (int)s.size());
    for (typename std::set<T,Tr,A>::const_iterator it = s.begin(); it != s.end(); ++it)
        doParsimPacking(buffer, *it);
}

template<typename T, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::set<T,Tr,A>& s)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        T x;
        doParsimUnpacking(buffer, x);
        s.insert(x);
    }
}

// Packing/unpacking an std::map
template<typename K, typename V, typename Tr, typename A>
void doParsimPacking(omnetpp::cCommBuffer *buffer, const std::map<K,V,Tr,A>& m)
{
    doParsimPacking(buffer, (int)m.size());
    for (typename std::map<K,V,Tr,A>::const_iterator it = m.begin(); it != m.end(); ++it) {
        doParsimPacking(buffer, it->first);
        doParsimPacking(buffer, it->second);
    }
}

template<typename K, typename V, typename Tr, typename A>
void doParsimUnpacking(omnetpp::cCommBuffer *buffer, std::map<K,V,Tr,A>& m)
{
    int n;
    doParsimUnpacking(buffer, n);
    for (int i=0; i<n; i++) {
        K k; V v;
        doParsimUnpacking(buffer, k);
        doParsimUnpacking(buffer, v);
        m[k] = v;
    }
}

// Default pack/unpack function for arrays
template<typename T>
void doParsimArrayPacking(omnetpp::cCommBuffer *b, const T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimPacking(b, t[i]);
}

template<typename T>
void doParsimArrayUnpacking(omnetpp::cCommBuffer *b, T *t, int n)
{
    for (int i = 0; i < n; i++)
        doParsimUnpacking(b, t[i]);
}

// Default rule to prevent compiler from choosing base class' doParsimPacking() function
template<typename T>
void doParsimPacking(omnetpp::cCommBuffer *, const T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: no doParsimPacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

template<typename T>
void doParsimUnpacking(omnetpp::cCommBuffer *, T& t)
{
    throw omnetpp::cRuntimeError("Parsim error: no doParsimUnpacking() function for type %s", omnetpp::opp_typename(typeid(t)));
}

}  // namespace omnetpp


// forward
template<typename T, typename A>
std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec);

// Template rule which fires if a struct or class doesn't have operator<<
template<typename T>
inline std::ostream& operator<<(std::ostream& out,const T&) {return out;}

// operator<< for std::vector<T>
template<typename T, typename A>
inline std::ostream& operator<<(std::ostream& out, const std::vector<T,A>& vec)
{
    out.put('{');
    for(typename std::vector<T,A>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        if (it != vec.begin()) {
            out.put(','); out.put(' ');
        }
        out << *it;
    }
    out.put('}');
    
    char buf[32];
    sprintf(buf, " (size=%u)", (unsigned int)vec.size());
    out.write(buf, strlen(buf));
    return out;
}

Register_Class(ControllerMessage);

ControllerMessage::ControllerMessage(const char *name, int kind) : ::WaveShortMessage(name,kind)
{
    this->sourceVehicle = 0;
    this->messageType = 0;
    this->flowAction = 0;
    this->flowId = 0;
    this->timestamp = 0;
    this->lastUsed = 0;
    this->hardTimeout = 0;
    this->idleTimeout = 0;
    this->destinationAddress = 0;
}

ControllerMessage::ControllerMessage(const ControllerMessage& other) : ::WaveShortMessage(other)
{
    copy(other);
}

ControllerMessage::~ControllerMessage()
{
}

ControllerMessage& ControllerMessage::operator=(const ControllerMessage& other)
{
    if (this==&other) return *this;
    ::WaveShortMessage::operator=(other);
    copy(other);
    return *this;
}

void ControllerMessage::copy(const ControllerMessage& other)
{
    this->sourceVehicle = other.sourceVehicle;
    this->messageType = other.messageType;
    this->flowAction = other.flowAction;
    this->flowId = other.flowId;
    this->timestamp = other.timestamp;
    this->lastUsed = other.lastUsed;
    this->hardTimeout = other.hardTimeout;
    this->idleTimeout = other.idleTimeout;
    this->destinationAddress = other.destinationAddress;
}

void ControllerMessage::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::WaveShortMessage::parsimPack(b);
    doParsimPacking(b,this->sourceVehicle);
    doParsimPacking(b,this->messageType);
    doParsimPacking(b,this->flowAction);
    doParsimPacking(b,this->flowId);
    doParsimPacking(b,this->timestamp);
    doParsimPacking(b,this->lastUsed);
    doParsimPacking(b,this->hardTimeout);
    doParsimPacking(b,this->idleTimeout);
    doParsimPacking(b,this->destinationAddress);
}

void ControllerMessage::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::WaveShortMessage::parsimUnpack(b);
    doParsimUnpacking(b,this->sourceVehicle);
    doParsimUnpacking(b,this->messageType);
    doParsimUnpacking(b,this->flowAction);
    doParsimUnpacking(b,this->flowId);
    doParsimUnpacking(b,this->timestamp);
    doParsimUnpacking(b,this->lastUsed);
    doParsimUnpacking(b,this->hardTimeout);
    doParsimUnpacking(b,this->idleTimeout);
    doParsimUnpacking(b,this->destinationAddress);
}

int ControllerMessage::getSourceVehicle() const
{
    return this->sourceVehicle;
}

void ControllerMessage::setSourceVehicle(int sourceVehicle)
{
    this->sourceVehicle = sourceVehicle;
}

int ControllerMessage::getMessageType() const
{
    return this->messageType;
}

void ControllerMessage::setMessageType(int messageType)
{
    this->messageType = messageType;
}

int ControllerMessage::getFlowAction() const
{
    return this->flowAction;
}

void ControllerMessage::setFlowAction(int flowAction)
{
    this->flowAction = flowAction;
}

int ControllerMessage::getFlowId() const
{
    return this->flowId;
}

void ControllerMessage::setFlowId(int flowId)
{
    this->flowId = flowId;
}

::omnetpp::simtime_t ControllerMessage::getTimestamp() const
{
    return this->timestamp;
}

void ControllerMessage::setTimestamp(::omnetpp::simtime_t timestamp)
{
    this->timestamp = timestamp;
}

::omnetpp::simtime_t ControllerMessage::getLastUsed() const
{
    return this->lastUsed;
}

void ControllerMessage::setLastUsed(::omnetpp::simtime_t lastUsed)
{
    this->lastUsed = lastUsed;
}

::omnetpp::simtime_t ControllerMessage::getHardTimeout() const
{
    return this->hardTimeout;
}

void ControllerMessage::setHardTimeout(::omnetpp::simtime_t hardTimeout)
{
    this->hardTimeout = hardTimeout;
}

::omnetpp::simtime_t ControllerMessage::getIdleTimeout() const
{
    return this->idleTimeout;
}

void ControllerMessage::setIdleTimeout(::omnetpp::simtime_t idleTimeout)
{
    this->idleTimeout = idleTimeout;
}

int ControllerMessage::getDestinationAddress() const
{
    return this->destinationAddress;
}

void ControllerMessage::setDestinationAddress(int destinationAddress)
{
    this->destinationAddress = destinationAddress;
}

class ControllerMessageDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertynames;
  public:
    ControllerMessageDescriptor();
    virtual ~ControllerMessageDescriptor();

    virtual bool doesSupport(omnetpp::cObject *obj) const override;
    virtual const char **getPropertyNames() const override;
    virtual const char *getProperty(const char *propertyname) const override;
    virtual int getFieldCount() const override;
    virtual const char *getFieldName(int field) const override;
    virtual int findField(const char *fieldName) const override;
    virtual unsigned int getFieldTypeFlags(int field) const override;
    virtual const char *getFieldTypeString(int field) const override;
    virtual const char **getFieldPropertyNames(int field) const override;
    virtual const char *getFieldProperty(int field, const char *propertyname) const override;
    virtual int getFieldArraySize(void *object, int field) const override;

    virtual std::string getFieldValueAsString(void *object, int field, int i) const override;
    virtual bool setFieldValueAsString(void *object, int field, int i, const char *value) const override;

    virtual const char *getFieldStructName(int field) const override;
    virtual void *getFieldStructValuePointer(void *object, int field, int i) const override;
};

Register_ClassDescriptor(ControllerMessageDescriptor);

ControllerMessageDescriptor::ControllerMessageDescriptor() : omnetpp::cClassDescriptor("ControllerMessage", "WaveShortMessage")
{
    propertynames = nullptr;
}

ControllerMessageDescriptor::~ControllerMessageDescriptor()
{
    delete[] propertynames;
}

bool ControllerMessageDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<ControllerMessage *>(obj)!=nullptr;
}

const char **ControllerMessageDescriptor::getPropertyNames() const
{
    if (!propertynames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
        const char **basenames = basedesc ? basedesc->getPropertyNames() : nullptr;
        propertynames = mergeLists(basenames, names);
    }
    return propertynames;
}

const char *ControllerMessageDescriptor::getProperty(const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : nullptr;
}

int ControllerMessageDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 9+basedesc->getFieldCount() : 9;
}

unsigned int ControllerMessageDescriptor::getFieldTypeFlags(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeFlags(field);
        field -= basedesc->getFieldCount();
    }
    static unsigned int fieldTypeFlags[] = {
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<9) ? fieldTypeFlags[field] : 0;
}

const char *ControllerMessageDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldName(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldNames[] = {
        "sourceVehicle",
        "messageType",
        "flowAction",
        "flowId",
        "timestamp",
        "lastUsed",
        "hardTimeout",
        "idleTimeout",
        "destinationAddress",
    };
    return (field>=0 && field<9) ? fieldNames[field] : nullptr;
}

int ControllerMessageDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount() : 0;
    if (fieldName[0]=='s' && strcmp(fieldName, "sourceVehicle")==0) return base+0;
    if (fieldName[0]=='m' && strcmp(fieldName, "messageType")==0) return base+1;
    if (fieldName[0]=='f' && strcmp(fieldName, "flowAction")==0) return base+2;
    if (fieldName[0]=='f' && strcmp(fieldName, "flowId")==0) return base+3;
    if (fieldName[0]=='t' && strcmp(fieldName, "timestamp")==0) return base+4;
    if (fieldName[0]=='l' && strcmp(fieldName, "lastUsed")==0) return base+5;
    if (fieldName[0]=='h' && strcmp(fieldName, "hardTimeout")==0) return base+6;
    if (fieldName[0]=='i' && strcmp(fieldName, "idleTimeout")==0) return base+7;
    if (fieldName[0]=='d' && strcmp(fieldName, "destinationAddress")==0) return base+8;
    return basedesc ? basedesc->findField(fieldName) : -1;
}

const char *ControllerMessageDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeString(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "int",
        "int",
        "int",
        "int",
        "simtime_t",
        "simtime_t",
        "simtime_t",
        "simtime_t",
        "int",
    };
    return (field>=0 && field<9) ? fieldTypeStrings[field] : nullptr;
}

const char **ControllerMessageDescriptor::getFieldPropertyNames(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldPropertyNames(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

const char *ControllerMessageDescriptor::getFieldProperty(int field, const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldProperty(field, propertyname);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    }
}

int ControllerMessageDescriptor::getFieldArraySize(void *object, int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldArraySize(object, field);
        field -= basedesc->getFieldCount();
    }
    ControllerMessage *pp = (ControllerMessage *)object; (void)pp;
    switch (field) {
        default: return 0;
    }
}

std::string ControllerMessageDescriptor::getFieldValueAsString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldValueAsString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    ControllerMessage *pp = (ControllerMessage *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getSourceVehicle());
        case 1: return long2string(pp->getMessageType());
        case 2: return long2string(pp->getFlowAction());
        case 3: return long2string(pp->getFlowId());
        case 4: return simtime2string(pp->getTimestamp());
        case 5: return simtime2string(pp->getLastUsed());
        case 6: return simtime2string(pp->getHardTimeout());
        case 7: return simtime2string(pp->getIdleTimeout());
        case 8: return long2string(pp->getDestinationAddress());
        default: return "";
    }
}

bool ControllerMessageDescriptor::setFieldValueAsString(void *object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->setFieldValueAsString(object,field,i,value);
        field -= basedesc->getFieldCount();
    }
    ControllerMessage *pp = (ControllerMessage *)object; (void)pp;
    switch (field) {
        case 0: pp->setSourceVehicle(string2long(value)); return true;
        case 1: pp->setMessageType(string2long(value)); return true;
        case 2: pp->setFlowAction(string2long(value)); return true;
        case 3: pp->setFlowId(string2long(value)); return true;
        case 4: pp->setTimestamp(string2simtime(value)); return true;
        case 5: pp->setLastUsed(string2simtime(value)); return true;
        case 6: pp->setHardTimeout(string2simtime(value)); return true;
        case 7: pp->setIdleTimeout(string2simtime(value)); return true;
        case 8: pp->setDestinationAddress(string2long(value)); return true;
        default: return false;
    }
}

const char *ControllerMessageDescriptor::getFieldStructName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructName(field);
        field -= basedesc->getFieldCount();
    }
    switch (field) {
        default: return nullptr;
    };
}

void *ControllerMessageDescriptor::getFieldStructValuePointer(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructValuePointer(object, field, i);
        field -= basedesc->getFieldCount();
    }
    ControllerMessage *pp = (ControllerMessage *)object; (void)pp;
    switch (field) {
        default: return nullptr;
    }
}


