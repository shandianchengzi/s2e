///
/// Copyright (C) 2010-2013, Dependable Systems Laboratory, EPFL
/// All rights reserved.
///
/// Licensed under the Cyberhaven Research License Agreement.
///

#ifndef S2E_PLUGINS_NLPPeripheralModel_H
#define S2E_PLUGINS_NLPPeripheralModel_H

#include <boost/regex.hpp>
#include <s2e/CorePlugin.h>
#include <s2e/Plugin.h>
#include <s2e/Plugins/SEmu/InvalidStatesDetection.h>
#include <s2e/Plugins/SEmu/SymbolicPeripherals.h>
#include <s2e/S2EExecutionState.h>
#include <s2e/SymbolicHardwareHook.h>
#include <utility>
#include <queue>

namespace s2e {
static const boost::regex MemoRegEx("([A-Z]_[a-z\\d]+_[a-z\\d]+(?:_\\d+){0,3})", boost::regex::perl);
static const boost::regex TARegEx("([a-zA-Z\\d\\*,=></]+)", boost::regex::perl);
static const boost::regex FlagEx("([a-zA-Z\\d\\*,/\\-]+)", boost::regex::perl);
namespace plugins {

typedef struct field {
    std::string type; // R: receive; T: transmit; O: other
    uint32_t phaddr;
    std::vector<long> bits;
} Field;

typedef struct equation {
    std::string trigger_type; //
    uint32_t id;
    Field a1;
    std::string eq;      //= ; >;  <;  >=; <=
    std::string type_a2; // V:value; R: receive; T: transmit; F: field
    uint32_t value;
    Field a2;
    int32_t interrupt;
    bool rel;
} Equation;

typedef struct peripheralReg {
    std::string type; // R: receive; T: transmit; O: other
    uint32_t phaddr;
    uint32_t reset;
    uint32_t cur_value;
    uint8_t width;
    uint32_t t_size;
    uint32_t r_size;
    uint32_t t_value;
    std::queue<uint8_t> r_value;
} PeripheralReg;

typedef struct flag {
    uint32_t id;
    Field a;
    int freq;
    std::vector<long> value;
    // int32_t value;
} Flag;

typedef std::map<uint32_t, PeripheralReg> RegMap;
typedef std::vector<Equation> EquList;
typedef std::pair<EquList, EquList> TA;
typedef std::vector<TA> TAMap;
typedef std::vector<Flag> FlagList;

enum RWType { Write,
              Read,
              Rx,
              Unknown };

class NLPPeripheralModel : public Plugin {
    S2E_PLUGIN
public:
    NLPPeripheralModel(S2E *s2e) :
        Plugin(s2e) {
    }
    sigc::signal<void, S2EExecutionState *, uint32_t /* irq_no */, bool * /* actual trigger or not */> onExternalInterruptEvent;
    sigc::signal<void, S2EExecutionState *, uint32_t /* irq_no */, std::queue<uint8_t>, bool * /* actual trigger or not */> onDMAInterruptEvent;
    sigc::signal<void, S2EExecutionState *, uint32_t /* physicalAddress */, uint32_t * /* size */,
                 std::queue<uint8_t> * /* return value */>
        onBufferInput;
    sigc::signal<void, S2EExecutionState *, std::vector<uint32_t> * /* enable IRQ vector */>
        onEnableISER;

private:
    InvalidStatesDetection *onInvalidStateDectionConnection;

    hw::PeripheralMmioRanges nlp_mmio;
    uint32_t rw_count;
    std::string NLPfileName;
    std::map<std::set<uint32_t>, TAMap> TA_range;
    TAMap allTAs;
    std::vector<std::pair<std::set<uint32_t>, FlagList>> Flags_range;
    FlagList allFlags;
    std::map<uint32_t, uint32_t> rx_flags;
    std::map<uint32_t, uint32_t> write_action;
    std::map<uint32_t, std::vector<Field>> constraints;
    std::map<uint32_t, uint32_t> DR2SR;
    std::map<uint32_t, uint32_t> statistics;
    int ta_numbers = 0;
    int flags_numbers = 0;
    int read_numbers = 0;
    int write_numbers = 0;
    bool checked_SR = false;
    std::map<uint32_t, std::set<uint32_t>> unenabled_flag;
    std::map<uint32_t, std::set<uint32_t>> untriggered_irq;
    std::map<uint32_t, std::set<uint64_t>> read_unauthorized_freq;
    std::map<uint32_t, std::set<uint64_t>> write_unauthorized_freq;
    std::map<std::pair<uint32_t, uint32_t>, std::set<uint64_t>> read_access_freq;
    std::map<std::pair<uint32_t, uint32_t>, std::set<uint64_t>> write_access_freq;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> chain_freq;
    std::set<uint32_t> data_register;
    std::set<uint32_t> todo_registers;
    FlagList todo_flags;
    TAMap todo_tas;
    bool timer = 0;
    std::map<uint32_t, bool> disable_init_dr_value_flag;
    uint32_t fork_point;
    bool begin_irq_flag;
    bool fork_point_flag;
    bool init_dr_flag;
    uint64_t tb_num;

    bool parseConfig();
    void initialize();
    std::pair<std::string, uint32_t> getAddress(std::string addr);
    void CheckEnable(S2EExecutionState *state, std::vector<uint32_t> &irq_no);
    bool ExistInMMIO(uint32_t tmp);
    bool extractConstraints(std::string peripheralcache, Field &field);
    template <typename T>
    bool parseRangeList(ConfigFile *cfg, const std::string &key, T &result);
    bool readNLPModelfromFile(S2EExecutionState *state, std::string fileName);
    bool getMemo(std::string peripheralcache, PeripheralReg &reg);
    bool getTApairs(std::string peripheralcache, EquList &trigger, EquList &action);
    bool extractEqu(std::string peripheralcache, EquList &vec, bool rel);
    bool extractFlag(std::string peripheralcache, Flag &flag);
    void UpdateGraph(S2EExecutionState *state, RWType type, uint32_t phaddr);
    bool EmitIRQ(S2EExecutionState *state, int irq);
    std::pair<uint32_t, uint32_t> AddressCorrection(S2EExecutionState *state, uint32_t phaddr);
    void onStatistics();
    void onExceptionExit(S2EExecutionState *state, uint32_t irq_no);
    void onEnableReceive(S2EExecutionState *state, uint32_t pc, uint64_t tb_num);
    void UpdateFlag(uint32_t phaddr);
    void onForkPoints(S2EExecutionState *state, uint64_t pc);
    uint32_t get_reg_value(S2EExecutionState *state, RegMap &state_map, Field a);
    void set_reg_value(S2EExecutionState *state, RegMap &state_map, Field a, uint32_t value);
    void SplitString(const std::string &s, std::vector<std::string> &v, const std::string &c);
    void SplitStringToInt(const std::string &s, std::vector<long> &v, const std::string &c, int dtype);
    bool compare(uint32_t a1, std::string sym, uint32_t a2);
    void onPeripheralRead(S2EExecutionState *state, SymbolicHardwareAccessType type, uint32_t phaddr, unsigned size,
                          uint32_t *NLPsymbolicvalue, bool *flag);
    void onPeripheralWrite(S2EExecutionState *state, SymbolicHardwareAccessType type, uint32_t phaddr,
                           uint32_t writeconcretevalue);
    void onTranslateBlockStart(ExecutionSignal *signal, S2EExecutionState *state, TranslationBlock *tb, uint64_t pc);
    void onTranslateBlockEnd(ExecutionSignal *signal, S2EExecutionState *state, TranslationBlock *tb, uint64_t pc,
                             bool staticTarget, uint64_t staticTargetPc);
    void onBlockEnd(S2EExecutionState *state, uint64_t pc, unsigned source_type);
};

} // namespace plugins
} // namespace s2e

#endif // S2E_PLUGINS_NLPPeripheralModel_H
