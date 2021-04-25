///
/// Copyright (C) 2010-2013, Dependable Systems Laboratory, EPFL
/// All rights reserved.
///
/// Licensed under the Cyberhaven Research License Agreement.
///

#ifndef S2E_PLUGINS_NLPPeripheralModel_H
#define S2E_PLUGINS_NLPPeripheralModel_H

#include <s2e/CorePlugin.h>
#include <s2e/Plugin.h>
#include <s2e/Plugins/NLP/SymbolicPeripherals.h>
#include <s2e/S2EExecutionState.h>
#include <s2e/SymbolicHardwareHook.h>
#include <utility>
#include <boost/regex.hpp>

namespace s2e {
//type address reset
static const boost::regex MemoRegEx("(.+_.+_.+)", boost::regex::perl);
static const boost::regex TARegEx("([TRO\\*],[\\*\\d]+,[\\*\\d]+,[=><\\*]{1,2},[TRO\\d\\*]+)", boost::regex::perl);

namespace plugins {

typedef std::map<uint32_t, PeripheralReg> RegMap;
typedef std::vector<Equation> EquList;
typedef std::vector<std::pair<EquList, EquList>> TAMap;

class Equation {
    // a1 = a2
    // R  > T
    // Reg bit = 1
public:
    std::string type; //R: receive; T: transmit; O: other
    uint32_t phaddr;
    std::string bits;
    std::string eq;//= ; >;  <;  >=; <=
    //uint32_t* linkaddr;
    std::string type_a2;//V:value; R: receive; T: transmit
    uint32_t value;
    bool rel;
};

class PeripheralReg{
public:
    std::string type;//R: receive; T: transmit; O: other
    uint32_t phaddr;
    uint32_t reset;
    uint32_t cur;
    uint32_t t_size;
    uint32_t r_size;
    uint32_t t_value;
    uint32_t r_value;
};

uint32_t data_register;
std::string data_register_type = "R";

class NLPPeripheralModel : public Plugin {
    S2E_PLUGIN
public:
    NLPPeripheralModel(S2E *s2e) : Plugin(s2e) {
    }
    void initialize();
    sigc::signal<void, S2EExecutionState *, uint32_t /* irq_no */> onExternalInterruptEvent;

private:
    
    bool ReadKBfromFile(S2EExecutionState *state, std::string fileName);
    //bool ReadMemofromFile(std::string fileName);
    //bool ReadTAfromFile(std::string fileName);
    
    void SplitString(const std::string &s, std::vector<std::string> &v, const std::string &c);
    bool getMemo(std::string peripheralcache, PeripheralReg &reg);
    bool getTApairs(std::string peripheralcache, EquList &trigger, EquList &action);
    bool extractEqu(std::string peripheralcache, EquList &vec, bool rel);


    sigc::connection symbolicPeripheralConnection;
    void onPeripheralRead(S2EExecutionState *state, SymbolicHardwareAccessType type, uint32_t phaddr,
                     unsigned size, uint32_t *NLPsymbolicvalue);

    void onPeripheralWrite(S2EExecutionState *state, SymbolicHardwareAccessType type, uint32_t phaddr,
                     uint32_t  writeconcretevalue);
};

} // namespace plugins
} // namespace s2e

#endif // S2E_PLUGINS_NLPPeripheralModel_H
