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
#include <s2e/Plugins/NLP/InvalidStatesDetection.h>
#include <s2e/S2EExecutionState.h>
#include <s2e/SymbolicHardwareHook.h>
#include <utility>
#include <boost/regex.hpp>

namespace s2e {
//type address reset
static const boost::regex MemoRegEx("([A-Z]_[a-z\\d]+_[a-z\\d]+)", boost::regex::perl);
//static const boost::regex TARegEx("([TRPCO\\*],[\\*a-z\\d]+,[\\*\\d]+,[=><\\*]{1,2},[a-zTRPCO\\d\\*,]+)", boost::regex::perl);
static const boost::regex TARegEx("([a-zA-Z\\d\\*,=></]+)", boost::regex::perl);
static const boost::regex CounterEx("([a-zA-Z\\d\\*,/\\-]+)", boost::regex::perl);
namespace plugins {

typedef struct field {
    std::string type; //R: receive; T: transmit; O: other
    uint32_t phaddr;
    std::vector<int> bits;
} Field;

typedef struct equation {
    Field a1;
    std::string eq;//= ; >;  <;  >=; <=
    std::string type_a2;//V:value; R: receive; T: transmit; F: field
    uint32_t value;
    Field a2;
    int interrupt;
    bool rel;
} Equation;

typedef struct peripheralReg {
    std::string type;//R: receive; T: transmit; O: other
    uint32_t phaddr;
    uint32_t reset;
    uint32_t cur_value;
    uint32_t t_size;
    uint32_t r_size;
    uint32_t t_value;
    uint32_t r_value;
} PeripheralReg;

typedef struct counter {
	Field a;
	uint32_t freq;
        std::vector<int32_t> value;
	//int32_t value;
} Counter;

typedef std::map<uint32_t, PeripheralReg> RegMap;
typedef std::vector<Equation> EquList;
typedef std::vector<std::pair<EquList, EquList>> TAMap;
typedef std::vector<Counter> CounterList;

enum RWType { Write, Read };
//std::map<std::string, uint32_t> symbol_list = {
//    {"*",0},{"=",1},{">":2},{"<",3},{">=",4},{"<=",5}
//};
//0:= ; 1:>; 2: <; 3: >=; 4: <=

class NLPPeripheralModel : public Plugin {
    S2E_PLUGIN
public:
    NLPPeripheralModel(S2E *s2e) : Plugin(s2e) {
    }
    void initialize();
    sigc::signal<void, S2EExecutionState *, uint32_t /* irq_no */> onExternalInterruptEvent;
    sigc::signal<void, S2EExecutionState *,uint32_t /* physicalAddress */,                uint32_t  /* size */, uint32_t * /* return value */> onBufferInput;

private:
    InvalidStatesDetection *onInvalidStateDectionConnection;

    uint32_t rw_count;
    std::string NLPfileName;
    std::map<std::pair<uint32_t, uint32_t>, TAMap> TA_range;
    CounterList allCounters;
    std::vector<uint32_t> data_register;
    uint32_t timer;
    bool readNLPModelfromFile(S2EExecutionState *state, std::string fileName);
    bool getMemo(std::string peripheralcache, PeripheralReg &reg);
    bool getTApairs(std::string peripheralcache, EquList &trigger, EquList &action);
    bool extractEqu(std::string peripheralcache, EquList &vec, bool rel);
    bool extractCounter(std::string peripheralcache, Counter &counter);
    void UpdateGraph(S2EExecutionState *state, RWType type, uint32_t phaddr);


    void onExceptionExit(S2EExecutionState *state, uint32_t irq_no);
    void onInvalidStatesDetection(S2EExecutionState *state, uint32_t pc, InvalidStatesType type, uint64_t tb_num);
    void CountDown();
    void onForceIRQCheck(S2EExecutionState *state, uint32_t pc, uint64_t re_tb_num);
    uint32_t get_reg_value(RegMap &state_map, Field a);
    void set_reg_value(RegMap &state_map, Field a, uint32_t value);
    void SplitString(const std::string &s, std::vector<std::string> &v, const std::string &c);
    void SplitStringToInt(const std::string &s, std::vector<int> &v, const std::string &c, int dtype);
    bool compare(uint32_t a1, std::string sym, uint32_t a2);
    void onPeripheralRead(S2EExecutionState *state, SymbolicHardwareAccessType type, uint32_t phaddr,
                     unsigned size, uint32_t *NLPsymbolicvalue);
    void onPeripheralWrite(S2EExecutionState *state, SymbolicHardwareAccessType type, uint32_t phaddr,
                     uint32_t  writeconcretevalue);
};

} // namespace plugins
} // namespace s2e

#endif // S2E_PLUGINS_NLPPeripheralModel_H
