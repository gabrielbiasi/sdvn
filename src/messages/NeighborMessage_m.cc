//
// Generated file, do not edit! Created by nedtool 5.0 from messages/NeighborMessage.msg.
//

// Disable warnings about unused variables, empty switch stmts, etc:
#ifdef _MSC_VER
#  pragma warning(disable:4101)
#  pragma warning(disable:4065)
#endif

#include <iostream>
#include <sstream>
#include "NeighborMessage_m.h"

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

Register_Class(NeighborMessage);

NeighborMessage::NeighborMessage(const char *name, int kind) : ::WaveShortMessage(name,kind)
{
    this->sourceVehicle = 0;
    this->timestamp = 0;
    neighbors_arraysize = 0;
    this->neighbors = 0;
    this->numPackets = 0;
}

NeighborMessage::NeighborMessage(const NeighborMessage& other) : ::WaveShortMessage(other)
{
    neighbors_arraysize = 0;
    this->neighbors = 0;
    copy(other);
}

NeighborMessage::~NeighborMessage()
{
    delete [] this->neighbors;
}

NeighborMessage& NeighborMessage::operator=(const NeighborMessage& other)
{
    if (this==&other) return *this;
    ::WaveShortMessage::operator=(other);
    copy(other);
    return *this;
}

void NeighborMessage::copy(const NeighborMessage& other)
{
    this->sourceVehicle = other.sourceVehicle;
    this->timestamp = other.timestamp;
    delete [] this->neighbors;
    this->neighbors = (other.neighbors_arraysize==0) ? nullptr : new int[other.neighbors_arraysize];
    neighbors_arraysize = other.neighbors_arraysize;
    for (unsigned int i=0; i<neighbors_arraysize; i++)
        this->neighbors[i] = other.neighbors[i];
    this->numPackets = other.numPackets;
}

void NeighborMessage::parsimPack(omnetpp::cCommBuffer *b) const
{
    ::WaveShortMessage::parsimPack(b);
    doParsimPacking(b,this->sourceVehicle);
    doParsimPacking(b,this->timestamp);
    b->pack(neighbors_arraysize);
    doParsimArrayPacking(b,this->neighbors,neighbors_arraysize);
    doParsimPacking(b,this->numPackets);
}

void NeighborMessage::parsimUnpack(omnetpp::cCommBuffer *b)
{
    ::WaveShortMessage::parsimUnpack(b);
    doParsimUnpacking(b,this->sourceVehicle);
    doParsimUnpacking(b,this->timestamp);
    delete [] this->neighbors;
    b->unpack(neighbors_arraysize);
    if (neighbors_arraysize==0) {
        this->neighbors = 0;
    } else {
        this->neighbors = new int[neighbors_arraysize];
        doParsimArrayUnpacking(b,this->neighbors,neighbors_arraysize);
    }
    doParsimUnpacking(b,this->numPackets);
}

int NeighborMessage::getSourceVehicle() const
{
    return this->sourceVehicle;
}

void NeighborMessage::setSourceVehicle(int sourceVehicle)
{
    this->sourceVehicle = sourceVehicle;
}

::omnetpp::simtime_t NeighborMessage::getTimestamp() const
{
    return this->timestamp;
}

void NeighborMessage::setTimestamp(::omnetpp::simtime_t timestamp)
{
    this->timestamp = timestamp;
}

void NeighborMessage::setNeighborsArraySize(unsigned int size)
{
    int *neighbors2 = (size==0) ? nullptr : new int[size];
    unsigned int sz = neighbors_arraysize < size ? neighbors_arraysize : size;
    for (unsigned int i=0; i<sz; i++)
        neighbors2[i] = this->neighbors[i];
    for (unsigned int i=sz; i<size; i++)
        neighbors2[i] = 0;
    neighbors_arraysize = size;
    delete [] this->neighbors;
    this->neighbors = neighbors2;
}

unsigned int NeighborMessage::getNeighborsArraySize() const
{
    return neighbors_arraysize;
}

int NeighborMessage::getNeighbors(unsigned int k) const
{
    if (k>=neighbors_arraysize) throw omnetpp::cRuntimeError("Array of size %d indexed by %d", neighbors_arraysize, k);
    return this->neighbors[k];
}

void NeighborMessage::setNeighbors(unsigned int k, int neighbors)
{
    if (k>=neighbors_arraysize) throw omnetpp::cRuntimeError("Array of size %d indexed by %d", neighbors_arraysize, k);
    this->neighbors[k] = neighbors;
}

int NeighborMessage::getNumPackets() const
{
    return this->numPackets;
}

void NeighborMessage::setNumPackets(int numPackets)
{
    this->numPackets = numPackets;
}

class NeighborMessageDescriptor : public omnetpp::cClassDescriptor
{
  private:
    mutable const char **propertynames;
  public:
    NeighborMessageDescriptor();
    virtual ~NeighborMessageDescriptor();

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

Register_ClassDescriptor(NeighborMessageDescriptor);

NeighborMessageDescriptor::NeighborMessageDescriptor() : omnetpp::cClassDescriptor("NeighborMessage", "WaveShortMessage")
{
    propertynames = nullptr;
}

NeighborMessageDescriptor::~NeighborMessageDescriptor()
{
    delete[] propertynames;
}

bool NeighborMessageDescriptor::doesSupport(omnetpp::cObject *obj) const
{
    return dynamic_cast<NeighborMessage *>(obj)!=nullptr;
}

const char **NeighborMessageDescriptor::getPropertyNames() const
{
    if (!propertynames) {
        static const char *names[] = {  nullptr };
        omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
        const char **basenames = basedesc ? basedesc->getPropertyNames() : nullptr;
        propertynames = mergeLists(basenames, names);
    }
    return propertynames;
}

const char *NeighborMessageDescriptor::getProperty(const char *propertyname) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? basedesc->getProperty(propertyname) : nullptr;
}

int NeighborMessageDescriptor::getFieldCount() const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    return basedesc ? 4+basedesc->getFieldCount() : 4;
}

unsigned int NeighborMessageDescriptor::getFieldTypeFlags(int field) const
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
        FD_ISARRAY | FD_ISEDITABLE,
        FD_ISEDITABLE,
    };
    return (field>=0 && field<4) ? fieldTypeFlags[field] : 0;
}

const char *NeighborMessageDescriptor::getFieldName(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldName(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldNames[] = {
        "sourceVehicle",
        "timestamp",
        "neighbors",
        "numPackets",
    };
    return (field>=0 && field<4) ? fieldNames[field] : nullptr;
}

int NeighborMessageDescriptor::findField(const char *fieldName) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    int base = basedesc ? basedesc->getFieldCount() : 0;
    if (fieldName[0]=='s' && strcmp(fieldName, "sourceVehicle")==0) return base+0;
    if (fieldName[0]=='t' && strcmp(fieldName, "timestamp")==0) return base+1;
    if (fieldName[0]=='n' && strcmp(fieldName, "neighbors")==0) return base+2;
    if (fieldName[0]=='n' && strcmp(fieldName, "numPackets")==0) return base+3;
    return basedesc ? basedesc->findField(fieldName) : -1;
}

const char *NeighborMessageDescriptor::getFieldTypeString(int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldTypeString(field);
        field -= basedesc->getFieldCount();
    }
    static const char *fieldTypeStrings[] = {
        "int",
        "simtime_t",
        "int",
        "int",
    };
    return (field>=0 && field<4) ? fieldTypeStrings[field] : nullptr;
}

const char **NeighborMessageDescriptor::getFieldPropertyNames(int field) const
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

const char *NeighborMessageDescriptor::getFieldProperty(int field, const char *propertyname) const
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

int NeighborMessageDescriptor::getFieldArraySize(void *object, int field) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldArraySize(object, field);
        field -= basedesc->getFieldCount();
    }
    NeighborMessage *pp = (NeighborMessage *)object; (void)pp;
    switch (field) {
        case 2: return pp->getNeighborsArraySize();
        default: return 0;
    }
}

std::string NeighborMessageDescriptor::getFieldValueAsString(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldValueAsString(object,field,i);
        field -= basedesc->getFieldCount();
    }
    NeighborMessage *pp = (NeighborMessage *)object; (void)pp;
    switch (field) {
        case 0: return long2string(pp->getSourceVehicle());
        case 1: return simtime2string(pp->getTimestamp());
        case 2: return long2string(pp->getNeighbors(i));
        case 3: return long2string(pp->getNumPackets());
        default: return "";
    }
}

bool NeighborMessageDescriptor::setFieldValueAsString(void *object, int field, int i, const char *value) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->setFieldValueAsString(object,field,i,value);
        field -= basedesc->getFieldCount();
    }
    NeighborMessage *pp = (NeighborMessage *)object; (void)pp;
    switch (field) {
        case 0: pp->setSourceVehicle(string2long(value)); return true;
        case 1: pp->setTimestamp(string2simtime(value)); return true;
        case 2: pp->setNeighbors(i,string2long(value)); return true;
        case 3: pp->setNumPackets(string2long(value)); return true;
        default: return false;
    }
}

const char *NeighborMessageDescriptor::getFieldStructName(int field) const
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

void *NeighborMessageDescriptor::getFieldStructValuePointer(void *object, int field, int i) const
{
    omnetpp::cClassDescriptor *basedesc = getBaseClassDescriptor();
    if (basedesc) {
        if (field < basedesc->getFieldCount())
            return basedesc->getFieldStructValuePointer(object, field, i);
        field -= basedesc->getFieldCount();
    }
    NeighborMessage *pp = (NeighborMessage *)object; (void)pp;
    switch (field) {
        default: return nullptr;
    }
}


