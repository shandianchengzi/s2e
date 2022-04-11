///
/// Copyright (C) 2017, Cyberhaven
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction, including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
/// copies of the Software, and to permit persons to whom the Software is
/// furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all
/// copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
/// SOFTWARE.
///

#include <boost/regex.hpp>
#include <klee/util/ExprUtil.h>
#include <s2e/ConfigFile.h>
#include <s2e/S2E.h>
#include <s2e/SymbolicHardwareHook.h>
#include <s2e/Utils.h>
#include <s2e/cpu.h>
#include <s2e/opcodes.h>

#include <llvm/Support/CommandLine.h>

#include "SymbolicPeripherals.h"

using namespace klee;

namespace {
llvm::cl::opt<bool> DebugSymbHw("debug-symbolic-hardware", llvm::cl::init(true));
}

namespace s2e {
namespace plugins {
namespace hw {

extern "C" {
static bool symbhw_is_mmio_symbolic(struct MemoryDesc *mr, uint64_t physaddr, uint64_t size, void *opaque);
}

static klee::ref<klee::Expr> symbhw_symbread(struct MemoryDesc *mr, uint64_t physaddress,
                                             const klee::ref<klee::Expr> &value, SymbolicHardwareAccessType type,
                                             void *opaque);

static void symbhw_symbwrite(struct MemoryDesc *mr, uint64_t physaddress, const klee::ref<klee::Expr> &value,
                             SymbolicHardwareAccessType type, void *opaque);

static const boost::regex KBGeneralPeripheralRegEx("(.+)_(.+)_(.+)_(.+)_(.+)", boost::regex::perl);
static const boost::regex KBIRQPeripheralRegEx("(.+)_(.+)_(.+)_(.+)_(.+)_(.+)", boost::regex::perl);
static const boost::regex KBDRPeripheralRegEx("(.+)_(.+)_(.+)_(.+)", boost::regex::perl);
static const boost::regex PeripheralModelLearningRegEx("v\\d+_iommuread_(.+)_(.+)_(.+)", boost::regex::perl);

S2E_DEFINE_PLUGIN(SymbolicPeripherals, "SymbolicPeripherals S2E plugin", "SymbolicPeripherals",
                "InvalidStatesDetection", "ARMFunctionMonitor");
namespace {
class SymbolicPeripheralsState : public PluginState {
private:
    AllKnowledgeBaseMap lastforkphs;
    std::pair<uint32_t, std::vector<uint32_t>> last_fork_cond;
    std::map<uint32_t /* irq num */, AllKnowledgeBaseMap> irq_lastforkphs;
    std::map<uint32_t /* pc */, uint32_t /* count */> irqfork_count;
    WritePeripheralMap write_phs;
    ReadPeripheralMap read_phs;          // map pair with count rather that value
    TypeFlagPeripheralMap type_flag_phs; // use to indicate control phs map but don't store the value
    TypeFlagPeripheralMap all_rw_phs;    // use to indicate control phs map but don't store the value
    TypeFlagPeripheralMap condition_phs; // record all phs which meet conditions
    T1PeripheralMap symbolicpc_phs;         // 1 means this phs have been read as pc
    T1PeripheralMap symbolicpc_phs_fork_count;
    TypeFlagPeripheralMap type_irq_flag;
    PeripheralForkCount ph_forks_count;
    AllKnowledgeBaseNoMap allcache_phs; // save every value for all phs read (once for each read)
public:
    SymbolicPeripheralsState() {
        write_phs.clear();
    }

    virtual ~SymbolicPeripheralsState() {
    }

    static PluginState *factory(Plugin *, S2EExecutionState *) {
        return new SymbolicPeripheralsState();
    }

    SymbolicPeripheralsState *clone() const {
        return new SymbolicPeripheralsState(*this);
    }
    // irq fork count
    void incirqfork_count(uint32_t pc) {
        ++irqfork_count[pc];
    }

    uint32_t getirqfork_count(uint32_t pc) {
        return irqfork_count[pc];
    }
    // phs cache
    // type flag
    void insert_all_rw_phs(uint32_t phaddr, uint32_t flag) {
        all_rw_phs[phaddr] = flag;
    }

    TypeFlagPeripheralMap get_all_rw_phs() {
        return all_rw_phs;
    }

    // type flag
    void insert_type_flag_phs(uint32_t phaddr, uint32_t flag) {
        type_flag_phs[phaddr] = flag;
    }

    TypeFlagPeripheralMap get_type_flag_phs() {
        return type_flag_phs;
    }

    uint32_t get_type_flag_ph_it(uint32_t phaddr) {
        return type_flag_phs[phaddr];
    }

    // irq flag for irqs
    void insert_irq_flag_phs(uint32_t phaddr, uint32_t flag) {
        type_irq_flag[phaddr] = flag;
    }

    uint32_t get_irq_flag_ph_it(uint32_t phaddr) {
        return type_irq_flag[phaddr];
    }


    // read and write phs
    void inc_readphs(uint32_t phaddr, uint32_t size) {
        read_phs[phaddr].first = size;
        read_phs[phaddr].second++;
    }

    uint64_t get_readphs_count(uint32_t phaddr) {
        return read_phs[phaddr].second;
    }

    uint32_t get_readphs_size(uint32_t phaddr) {
        return read_phs[phaddr].first;
    }

    void update_writeph(uint32_t phaddr, uint32_t value) {
        write_phs[phaddr] = value;
    }

    ReadPeripheralMap get_readphs() {
        return read_phs;
    }

    // last fork conds
    void insert_last_fork_cond(uint32_t pc, std::vector<uint32_t> cond_values) {
        last_fork_cond = std::make_pair(pc, cond_values);
    }

    std::pair<uint32_t, std::vector<uint32_t>> get_last_fork_cond() {
        return last_fork_cond;
    }

    // last fork phs interrupt
    void irq_insertlastfork_phs(uint32_t irq_num, UniquePeripheral phc, uint64_t ch_value, NumPair value_no) {
        irq_lastforkphs[irq_num][phc][ch_value] = value_no;
    }

    AllKnowledgeBaseMap irq_getlastfork_phs(uint32_t irq_num) {
        return irq_lastforkphs[irq_num];
    }

    void irq_clearlastfork_phs(uint32_t irq_num) {
        irq_lastforkphs[irq_num].clear();
    }

    // last fork phs
    void insertlastfork_phs(UniquePeripheral phc, uint64_t ch_value, NumPair value_no) {
        lastforkphs[phc][ch_value] = value_no;
    }

    AllKnowledgeBaseMap getlastfork_phs() {
        return lastforkphs;
    }

    void clearlastfork_phs() {
        lastforkphs.clear();
    }

    // cache phs order by no
    void insert_cachephs(uint32_t phaddr, uint64_t no, uint32_t value) {
        allcache_phs[phaddr][no] = value;
    }

    NumMap get_cache_phs(uint32_t phaddr) {
        return allcache_phs[phaddr];
    }

    AllKnowledgeBaseNoMap get_all_cache_phs() {
        return allcache_phs;
    }

    // record all conditional phs
    void insert_condition_ph_it(uint32_t phaddr) {
        condition_phs[phaddr] = 1;
    }

    TypeFlagPeripheralMap get_condition_phs() {
        return condition_phs;
    }

    // update fork pc map in lastest read
    void inc_peripheral_fork_count(UniquePeripheral phc) {
        ph_forks_count[phc]++;
    }

    void clear_peripheral_fork_count(UniquePeripheral phc) {
        ph_forks_count[phc] = 0;
    }

    uint32_t get_peripheral_fork_count(UniquePeripheral phc) {
        return ph_forks_count[phc];
    }

    // update symbolic pc phs
    void insert_symbolicpc_ph_it(UniquePeripheral phc) {
        symbolicpc_phs[phc] = 1;
    }

    uint32_t get_symbolicpc_ph_it(UniquePeripheral phc) {
        return symbolicpc_phs[phc];
    }
    // update symbolic pc phs forking count
    void inc_symbolicpc_ph_count(UniquePeripheral phc) {
        symbolicpc_phs_fork_count[phc]++;
    }

    uint32_t get_symbolicpc_ph_count(UniquePeripheral phc) {
        return symbolicpc_phs_fork_count[phc];
    }

};
}

bool SymbolicPeripherals::configSymbolicMmioRange(void) {
    SymbolicMmioRange m;

    // ARM MMIO range 0x40000000-0x60000000
    m.first = 0x40000000;
    m.second = 0x5fffffff;

    getDebugStream() << "Adding symbolic mmio range: " << hexval(m.first) << " - " << hexval(m.second) << "\n";
    m_mmio.push_back(m);

    return true;
}

void SymbolicPeripherals::initialize() {
    if (!configSymbolicMmioRange()) {
        getWarningsStream() << "Could not parse config\n";
        exit(-1);
    }


    ARMMmioRange nlpph = std::make_pair(0x40000000,0x60000000);
    nlp_mmio.push_back(nlpph);
    all_peripheral_no = 0;
    firmwareName = s2e()->getConfig()->getString(getConfigKey() + ".firmwareName", "x.elf");
    getInfoStream() << "firmware name is " << firmwareName << "\n";
    enable_fuzzing = s2e()->getConfig()->getBool(getConfigKey() + ".useFuzzer", false);


    g_s2e_cache_mode = s2e()->getConfig()->getBool(getConfigKey() + ".disableDiagnosisMode", false);
    if (g_s2e_cache_mode) {
        getWarningsStream() << "Fuzzing(Concrete) mode\n";
    } else {
        onInvalidStateDectionConnection = s2e()->getPlugin<InvalidStatesDetection>();
        onInvalidStateDectionConnection->onInvalidStatesEvent.connect(
            sigc::mem_fun(*this, &SymbolicPeripherals::onInvalidStatesDetection));
        onInterruptExitonnection = s2e()->getCorePlugin()->onExceptionExit.connect(
            sigc::mem_fun(*this, &SymbolicPeripherals::onExceptionExit));
        onStateForkConnection =
            s2e()->getCorePlugin()->onStateFork.connect(sigc::mem_fun(*this, &SymbolicPeripherals::onFork));
        onStateSwitchConnection = s2e()->getCorePlugin()->onStateSwitch.connect(
            sigc::mem_fun(*this, &SymbolicPeripherals::onStateSwitch));
        onStateKillConnection =
            s2e()->getCorePlugin()->onStateKill.connect(sigc::mem_fun(*this, &SymbolicPeripherals::onStateKill));
        onStateForkDecideConnection = s2e()->getCorePlugin()->onStateForkDecide.connect(
            sigc::mem_fun(*this, &SymbolicPeripherals::onStateForkDecide));
        onSymbolicAddressConnection = s2e()->getCorePlugin()->onSymbolicAddress.connect(
            sigc::mem_fun(*this, &SymbolicPeripherals::onSymbolicAddress));
    }

    g_symbolicMemoryHook = SymbolicMemoryHook(symbhw_is_mmio_symbolic, symbhw_symbread, symbhw_symbwrite, this);
}


bool SymbolicPeripherals::parseConfig(void) {
    ConfigFile *cfg = s2e()->getConfig();
    PeripheralMmioRanges nlpphs;
    std::stringstream ss;
    ss << getConfigKey();
    getDebugStream() << ss.str() << "!!!!\n";
    if (!parseRangeList(cfg, ss.str() + ".nlp_mmio", nlpphs)) {
        return false;
    }

    for (auto nlpph : nlpphs) {
        getInfoStream() << "Adding nlp ph range " << hexval(nlpph.first) << " - "
                    << hexval(nlpph.second) << "\n";
        nlp_mmio.push_back(nlpph);
    }

    return true;
}

template <typename T> bool SymbolicPeripherals::parseRangeList(ConfigFile *cfg, const std::string &key, T &result) {
    bool ok;

    int ranges = cfg->getListSize(key, &ok);
    if (!ok) {
        getWarningsStream() << "Could not parse ranges: " << key << "\n";
        return false;
    }

    for (int i = 0; i < ranges; ++i) {
        std::stringstream ss;
        ss << key << "[" << (i + 1) << "]";
        uint64_t start = cfg->getInt(ss.str() + "[1]", 0, &ok);
        if (!ok) {
            getWarningsStream() << "Could not parse start address: " << ss.str() + "[1]"
                                << "\n";
            return false;
        }

        uint64_t end = cfg->getInt(ss.str() + "[2]", 0, &ok);
        if (!ok) {
            getWarningsStream() << "Could not parse end address: " << ss.str() + "[2]"
                                << "\n";
            return false;
        }

        if (!(start <= end)) {
            getWarningsStream() << hexval(start) << " is greater than " << hexval(end) << "\n";
            return false;
        }

        result.push_back(std::make_pair(start, end));
    }

    return true;
}

template <typename T, typename U> inline bool SymbolicPeripherals::isSymbolic(T ports, U port) {
    for (auto &p : ports) {
        if (port >= p.first && port <= p.second) {
            return true;
        }
    }

    return false;
}

bool SymbolicPeripherals::isMmioSymbolic(uint64_t physAddr) {
    return isSymbolic(m_mmio, physAddr);
}

static void SymbHwGetConcolicVector(uint64_t in, unsigned size, ConcreteArray &out) {
    union {
        // XXX: assumes little endianness!
        uint64_t value;
        uint8_t array[8];
    };

    value = in;
    out.resize(size);
    for (unsigned i = 0; i < size; ++i) {
        out[i] = array[i];
    }
}

struct CmpByCount {
    bool operator()(const ReadTUPLE &ph1, const ReadTUPLE &ph2) {
        return ph1.second.second > ph2.second.second;
    }
};

struct CmpByNo {
    bool operator()(const std::pair<uint64_t, uint32_t> &ph1, const std::pair<uint64_t, uint32_t> &ph2) {
        return ph1.first < ph2.first;
    }
};

klee::ref<klee::Expr> SymbolicPeripherals::onNLPLearningMode(S2EExecutionState *state, SymbolicHardwareAccessType type,
                                                         uint64_t address, unsigned size, uint64_t concreteValue) {

    std::stringstream ss;
    switch (type) {
        case SYMB_MMIO:
            ss << "iommuread_";
            break;
        case SYMB_DMA:
            ss << "dmaread_";
            break;
        case SYMB_PORT:
            ss << "portread_";
            break;
    }

    // record all read phs
    DECLARE_PLUGINSTATE(SymbolicPeripheralsState, state);
    plgState->inc_readphs(address, size);
    plgState->insert_all_rw_phs(address, 1);

    ss << hexval(address) << "@" << hexval(state->regs()->getPc());
    ss << "_" << hexval(size);

    uint32_t NLP_value = concreteValue;
    bool dr_flag = false;
    onSymbolicNLPRegisterReadEvent.emit(state, type, address, size, &NLP_value, &dr_flag);

    getWarningsStream(g_s2e_state) << ss.str() << " size " << hexval(size) << "dr flag = " << dr_flag
                                << " SYM NLP value = " << hexval(NLP_value) << "\n";

    if (dr_flag) {
        uint64_t LSB = ((uint64_t) 1 << (size * 8));
        getInfoStream() << "return concrete value phaddr = " << hexval(address) << "\n";
        uint32_t value = NLP_value & (LSB - 1);
        return klee::ConstantExpr::create(value, size * 8);
    }
    ConcreteArray concolicValue;
    SymbHwGetConcolicVector(NLP_value, size, concolicValue);
    return state->createSymbolicValue(ss.str(), size * 8, concolicValue);
}


klee::ref<klee::Expr> SymbolicPeripherals::onNLPFuzzingMode(S2EExecutionState *state, SymbolicHardwareAccessType type,
                                                         uint64_t address, unsigned size, uint64_t concreteValue) {

    std::stringstream ss;
    switch (type) {
        case SYMB_MMIO:
            ss << "iommuread_";
            break;
        case SYMB_DMA:
            ss << "dmaread_";
            break;
        case SYMB_PORT:
            ss << "portread_";
            break;
    }

    // record all read phs
    DECLARE_PLUGINSTATE(SymbolicPeripheralsState, state);
    plgState->inc_readphs(address, size);
    plgState->insert_all_rw_phs(address, 1);

    ss << hexval(address) << "@" << hexval(state->regs()->getPc());
    ss << "_" << hexval(size);

    uint32_t NLP_value = concreteValue;
    bool flag = false;
    onSymbolicNLPRegisterReadEvent.emit(state, type, address, size, &NLP_value, &flag);

    getDebugStream(g_s2e_state) << ss.str() << " size " << hexval(size)
                                << " NLP value = " << hexval(NLP_value) << "\n";

    uint64_t LSB = ((uint64_t) 1 << (size * 8));
    uint32_t value;
    value = NLP_value & (LSB - 1);
    return klee::ConstantExpr::create(value, size * 8);
}



//////////////////////////////////////////////////////////////////////
static bool symbhw_is_mmio_symbolic(struct MemoryDesc *mr, uint64_t physaddr, uint64_t size, void *opaque) {
    SymbolicPeripherals *hw = static_cast<SymbolicPeripherals *>(opaque);
    return hw->isMmioSymbolic(physaddr);
}

// XXX: remove MemoryDesc
static klee::ref<klee::Expr> symbhw_symbread(struct MemoryDesc *mr, uint64_t physaddress,
                                             const klee::ref<klee::Expr> &value, SymbolicHardwareAccessType type,
                                             void *opaque) {
    SymbolicPeripherals *hw = static_cast<SymbolicPeripherals *>(opaque);

    if (DebugSymbHw) {
        hw->getDebugStream(g_s2e_state) << "reading mmio " << hexval(physaddress) << " value: " << value << "\n";
    }

    unsigned size = value->getWidth() / 8;
    uint64_t concreteValue = g_s2e_state->toConstantSilent(value)->getZExtValue();
    if (!g_s2e_cache_mode) {
        return hw->onLearningMode(g_s2e_state, SYMB_MMIO, physaddress, size, concreteValue);
    } else {
        return hw->onFuzzingMode(g_s2e_state, SYMB_MMIO, physaddress, size, concreteValue);
    }
}

klee::ref<klee::Expr> SymbolicPeripherals::onLearningMode(S2EExecutionState *state, SymbolicHardwareAccessType type,
                                                         uint64_t address, unsigned size, uint64_t concreteValue) {
    // peripheral address bit-band alias
    if (address >= 0x42000000 && address <= 0x43fffffc) {
        uint32_t phaddr = (address - 0x42000000) / 32 + 0x40000000;
        getDebugStream() << "bit band alias address = " << hexval(address) << " alias address = " << hexval(phaddr) << "\n";
        address = phaddr;
    }
    // nlp regions
    //for (auto nlpph : nlp_mmio) {
    //    if (address >= nlpph.first && address <= nlpph.second) {
    return onNLPLearningMode(state, type, address, size, concreteValue);
    //    }
    //}
}

klee::ref<klee::Expr> SymbolicPeripherals::onFuzzingMode(S2EExecutionState *state, SymbolicHardwareAccessType type,
                                                         uint64_t address, unsigned size, uint64_t concreteValue) {
    // peripheral address bit-band alias
    if (address >= 0x42000000 && address <= 0x43fffffc) {
        uint32_t phaddr = (address - 0x42000000) / 32 + 0x40000000;
        getDebugStream() << "bit band alias address = " << hexval(address) << " alias address = " << hexval(phaddr) << "\n";
        address = phaddr;
    }
    // nlp regions
    //for (auto nlpph : nlp_mmio) {
    //    if (address >= nlpph.first && address <= nlpph.second) {
    return onNLPFuzzingMode(state, type, address, size, concreteValue);
    //    }
    //}
}

static void symbhw_symbwrite(struct MemoryDesc *mr, uint64_t physaddress, const klee::ref<klee::Expr> &value,
                             SymbolicHardwareAccessType type, void *opaque) {
    SymbolicPeripherals *hw = static_cast<SymbolicPeripherals *>(opaque);
    uint32_t curPc = g_s2e_state->regs()->getPc();

    if (DebugSymbHw) {
        hw->getDebugStream(g_s2e_state) << "writing mmio " << hexval(physaddress) << " value: " << value
                                        << " pc: " << hexval(curPc) << "\n";
    }

    hw->onWritePeripheral(g_s2e_state, physaddress, value);
}

void SymbolicPeripherals::onWritePeripheral(S2EExecutionState *state, uint64_t phaddr,
                                                const klee::ref<klee::Expr> &value) {
    DECLARE_PLUGINSTATE(SymbolicPeripheralsState, state);
    // peripheral address bit-band alias
    uint32_t temp_value = 0;
    uint32_t bit_loc = 0;
    bool bit_alias = false;
    if (phaddr >= 0x42000000 && phaddr <= 0x43fffffc) {
        bit_loc = ((phaddr - 0x42000000) % 32) / 4;
        phaddr = (phaddr - 0x42000000) / 32 + 0x40000000;
        bool flag = false;
        onSymbolicNLPRegisterReadEvent.emit(state, SYMB_MMIO, phaddr, 0x4, &temp_value, &flag);
        getDebugStream() << "write bit band alias address = " << hexval(phaddr)
                         << " bit loc = " << hexval(bit_loc) << " nlp value =" << hexval(temp_value) <<"\n";
        bit_alias = true;
    }

    uint32_t writeConcreteValue;
    if (isa<klee::ConstantExpr>(value)) {
        klee::ref<klee::ConstantExpr> ce = dyn_cast<klee::ConstantExpr>(value);
        writeConcreteValue = ce->getZExtValue();
        if (bit_alias) {
            if (writeConcreteValue) {
                temp_value |= (uint32_t)(1<< bit_loc);
            } else {
                temp_value &= (uint32_t)(~(1<< bit_loc));
            }
            writeConcreteValue = temp_value;
        }
        getDebugStream() << "writing mmio " << hexval(phaddr) << " concrete value: " << hexval(writeConcreteValue)
                         << "\n";
        plgState->update_writeph((uint32_t) phaddr, writeConcreteValue);
    } else {
        if (bit_alias) {
            getWarningsStream() << " bit band does not support symbolic value\n";
            exit(-1);
        }
        // evaluate symbolic regs
        klee::ref<klee::ConstantExpr> ce;
        ce = dyn_cast<klee::ConstantExpr>(g_s2e_state->concolics->evaluate(value));
        writeConcreteValue = ce->getZExtValue();
        getDebugStream() << "writing mmio " << hexval(phaddr) << " symbolic to concrete value: " << hexval(writeConcreteValue) << "\n";
        plgState->update_writeph((uint32_t) phaddr, writeConcreteValue);
        getWarningsStream() << "SEMU unmap peripheral writing!!!\n";
    }

}

void SplitString(const std::string &s, std::vector<std::string> &v, const std::string &c) {
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (std::string::npos != pos2) {
        v.push_back(s.substr(pos1, pos2 - pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
}

bool SymbolicPeripherals::getPeripheralExecutionState(std::string variablePeripheralName, uint32_t *phaddr,
                                                          uint32_t *pc, uint64_t *ch_value, uint64_t *no) {
    boost::smatch what;
    if (!boost::regex_match(variablePeripheralName, what, PeripheralModelLearningRegEx)) {
        getWarningsStream() << "match false"
                            << "\n";
        exit(0);
        return false;
    }

    if (what.size() != 4) {
        getWarningsStream() << "wrong size = " << what.size() << "\n";
        exit(0);
        return false;
    }

    std::string peripheralAddressStr = what[1];
    std::string pcStr = what[2];
    std::string noStr = what[3];

    std::vector<std::string> v;
    SplitString(peripheralAddressStr, v, "_");
    *phaddr = std::stoull(v[0].c_str(), NULL, 16);
    *pc = std::stoull(v[1].c_str(), NULL, 16);
    *ch_value = std::stoull(pcStr.c_str(), NULL, 16);
    *no = std::stoull(noStr.c_str(), NULL, 10);

    return true;
}

void SymbolicPeripherals::onStateForkDecide(S2EExecutionState *state, bool *doFork,
                                                const klee::ref<klee::Expr> &condition, bool *conditionFork) {
    DECLARE_PLUGINSTATE(SymbolicPeripheralsState, state);
    uint32_t curPc = state->regs()->getPc();

    ArrayVec results;
    findSymbolicObjects(condition, results);
    for (int i = results.size() - 1; i >= 0; --i) { // one cond multiple sym var
        uint32_t phaddr;
        uint32_t pc;
        uint64_t ch_value;
        uint64_t no;
        auto &arr = results[i];

        getPeripheralExecutionState(arr->getName(), &phaddr, &pc, &ch_value, &no);
        if (symbolic_address_count[curPc] > 0) {
            getDebugStream(state) << "can not fork at Symbolic Address: " << hexval(curPc) << "\n";
            plgState->insert_symbolicpc_ph_it(std::make_pair(phaddr, pc));
            getWarningsStream(state) << "Add symbolic phaddr = " << hexval(phaddr) << " pc = " << hexval(pc) << "\n";
            *doFork = false;
            *conditionFork = false;
            continue;
        }

        // let go circle of condition in symbolic pc forking
        if (plgState->get_symbolicpc_ph_it(UniquePeripheral(phaddr, pc)) == 1) {
            getWarningsStream(state) << "condition random in symbolic address " << hexval(phaddr)
                                     << " pc = " << hexval(pc) << "\n";
            plgState->inc_symbolicpc_ph_count(UniquePeripheral(phaddr, pc));
            *conditionFork = plgState->get_symbolicpc_ph_count(UniquePeripheral(phaddr, pc)) % 2;
            return;
        }

            //getWarningsStream() << " condition false for nlp phaddr = " << hexval(phaddr) << "\n";
            *conditionFork = false;
    }
}

bool comp(std::vector<uint32_t> &v1, std::vector<uint32_t> &v2) {
    for (int i = 0; i < v2.size(); i++) {
        if (std::find(v1.begin(), v1.end(), v2[i]) == v1.end())
            return false;
    }
    return true;
}

void SymbolicPeripherals::onFork(S2EExecutionState *state, const std::vector<S2EExecutionState *> &newStates,
                                     const std::vector<klee::ref<klee::Expr>> &newConditions) {
    DECLARE_PLUGINSTATE(SymbolicPeripheralsState, state);
    std::map<uint32_t, AllKnowledgeBaseMap> cachefork_phs;
    cachefork_phs.clear();
    bool NLP_flag = false;

    std::vector<uint32_t> fork_states_values;
    fork_states_values.clear();

    for (int k = newStates.size() - 1; k >= 0; --k) {
        DECLARE_PLUGINSTATE(SymbolicPeripheralsState, newStates[k]);
        ReadPeripheralMap read_size_phs = plgState->get_readphs();
        ArrayVec results;

        findSymbolicObjects(newConditions[0], results);
        for (int i = results.size() - 1; i >= 0; --i) { // one cond multiple sym var
            uint32_t phaddr;
            uint32_t pc;
            uint64_t ch_value;
            uint64_t no;
            auto &arr = results[i];
            std::vector<unsigned char> data;

            getPeripheralExecutionState(arr->getName(), &phaddr, &pc, &ch_value, &no);

            // getDebugStream() << "The symbol name of value is " << arr->getName() << "\n";
            for (unsigned s = 0; s < arr->getSize(); ++s) {
                ref<Expr> e = newStates[k]->concolics->evaluate(arr, s);
                if (!isa<ConstantExpr>(e)) {
                    getWarningsStream() << "Failed to evaluate concrete value\n";
                    pabort("Failed to evaluate concrete value");
                }

                uint8_t val = dyn_cast<ConstantExpr>(e)->getZExtValue();
                data.push_back(val);
            }

            uint32_t condConcreteValue =
                data[0] | ((uint32_t) data[1] << 8) | ((uint32_t) data[2] << 16) | ((uint32_t) data[3] << 24);

            UniquePeripheral uniquePeripheral = std::make_pair(phaddr, pc);
            uint64_t LSB = ((uint64_t) 1 << (read_size_phs[phaddr].first * 8));
            uint32_t value = condConcreteValue & (LSB - 1);

            // nlp regions
            for (auto nlpph : nlp_mmio) {
                if (phaddr >= nlpph.first && phaddr <= nlpph.second) {
                    NLP_flag = true;
                    break;
                }
            }

            if (NLP_flag) {
                // update cachefork after T3 check
                cachefork_phs[k][uniquePeripheral][ch_value] = std::make_pair(no, value);
                plgState->insert_cachephs(phaddr, no, value);
                fork_states_values.push_back(value);
                getInfoStream(newStates[k]) << " nlp cache phaddr = " << hexval(phaddr) << " pc = " << hexval(pc)
                                        << " value = " << hexval(value) << " no = " << no
                                        << " width = " << hexval(ch_value)  << "value = " << hexval(value) << "\n";
            } else {
                getWarningsStream(state) << "SEMU uncover peripheral address range\n";
            }

        } // each condition

        // push fork states in interrupt
        if (newStates[k]->regs()->getInterruptFlag()) {
            if (newStates[k] != state) {
                getDebugStream() << "push irq state" << newStates[k]->getID() << "\n";
                irq_states.push_back(newStates[k]);
            }
        }

    } // each new State

    uint32_t current_pc = state->regs()->getPc();
    std::pair<uint32_t, std::vector<uint32_t>> last_fork_cond = plgState->get_last_fork_cond();
    plgState->insert_last_fork_cond(current_pc, fork_states_values);
    if (last_fork_cond.first == current_pc && comp(last_fork_cond.second, fork_states_values) &&
        comp(fork_states_values, last_fork_cond.second)) {
        for (int k = newStates.size() - 1; k >= 0; --k) {
            DECLARE_PLUGINSTATE(SymbolicPeripheralsState, newStates[k]);
            // only update kb for new condition
            if (newStates[k]->regs()->getInterruptFlag()) {
                plgState->irq_clearlastfork_phs(newStates[k]->regs()->getExceptionIndex());
                for (auto &it : cachefork_phs[k]) {
                    for (auto &itch : it.second) {
                        plgState->irq_insertlastfork_phs(newStates[k]->regs()->getExceptionIndex(), it.first,
                                                         itch.first, itch.second);
                    }
                }
            } else {
                plgState->clearlastfork_phs();
                for (auto &it : cachefork_phs[k]) {
                    for (auto &itch : it.second) {
                        plgState->insertlastfork_phs(it.first, itch.first, itch.second);
                    }
                }
            }
            // push back new loop state
            if (newStates[k] != state) {
                unsearched_condition_fork_states.back().push_back(newStates[k]);
            }
        }
        getWarningsStream(state) << "fork cond in the loop !!" << hexval(current_pc) << "\n";
        return;
    } else {
        // set fork flag
        if (state->regs()->getInterruptFlag()) {
            irq_no_new_branch_flag = false;
        } else {
            no_new_branch_flag = false;
        }

        for (int k = newStates.size() - 1; k >= 0; --k) {
            // push back new states
            if (newStates[k] != state) {
                std::vector<S2EExecutionState *> condition_fork_states; // forking states in each condition
                condition_fork_states.clear();
                condition_fork_states.push_back(newStates[k]);
                unsearched_condition_fork_states.push_back(condition_fork_states);
            }
        }
    }

    // update KB
    for (int k = newStates.size() - 1; k >= 0; --k) {
        DECLARE_PLUGINSTATE(SymbolicPeripheralsState, newStates[k]);
        // cache the possiable status phs in corresponding state and insert lask fork state for further choices
        // interrupt mode
        if (newStates[k]->regs()->getInterruptFlag()) {
            plgState->irq_clearlastfork_phs(newStates[k]->regs()->getExceptionIndex());
            for (auto &it : cachefork_phs[k]) {
                for (auto &itch : it.second) {
                    plgState->irq_insertlastfork_phs(newStates[k]->regs()->getExceptionIndex(), it.first, itch.first,
                                                     itch.second);
                }
            }
        } else { // normal mode
            plgState->clearlastfork_phs();
            for (auto &it : cachefork_phs[k]) {
                for (auto &itch : it.second) {
                    plgState->insertlastfork_phs(it.first, itch.first, itch.second);
                }
            }
        }
    }
}

void SymbolicPeripherals::onStateKill(S2EExecutionState *state) {

    if (!g_s2e_cache_mode) {
        if (irq_states.size() > 0) {
            auto itirqs = irq_states.begin();
            for (; itirqs != irq_states.end();) {
                if (*itirqs == state) {
                    irq_states.erase(itirqs);
                } else {
                    itirqs++;
                }
            }
        }

        if (learning_mode_states.size() > 0) {
            auto itles = learning_mode_states.begin();
            for (; itles != learning_mode_states.end();) {
                if (*itles == state) {
                    learning_mode_states.erase(itles);
                } else {
                    itles++;
                }
            }
        }

        for (int i = 0; i < unsearched_condition_fork_states.size(); i++) {
            auto cfss = unsearched_condition_fork_states[i].begin();
            for (; cfss != unsearched_condition_fork_states[i].end();) {
                if (*cfss == state) {
                    getDebugStream() << i << " delete cache condition state " << state->getID() << "\n";
                    unsearched_condition_fork_states[i].erase(cfss);
                    if (unsearched_condition_fork_states[i].size() == 0) {
                        getInfoStream() << "the empty condition unique fork state is " << i
                                            << " total condtions is " << unsearched_condition_fork_states.size()
                                            << "\n";
                        ForkStateStack::iterator cdss = unsearched_condition_fork_states.begin() + i;
                        unsearched_condition_fork_states.erase(cdss);
                        i--;
                        break;
                    }
                } else {
                    cfss++;
                }
            }
        }
    }

    fs++;
    while (fs < false_type_phs_fork_states.size()) {
        std::string s;
        llvm::raw_string_ostream ss(s);
        ss << "Kill Fork State in false status phs:" << false_type_phs_fork_states[fs]->getID() << "\n";
        ss.flush();
        s2e()->getExecutor()->terminateState(*false_type_phs_fork_states[fs], s);
    }
    fs = -1;
    false_type_phs_fork_states.clear();
}

void SymbolicPeripherals::onExceptionExit(S2EExecutionState *state, uint32_t irq_no) {
    getInfoStream() << "Interrupt exit irq num = " << hexval(irq_no) << "\n";
    irq_no_new_branch_flag = true;
}

std::vector<uint32_t> identify_setbit_loc(uint32_t value) {
    std::vector<uint32_t> setbit_loc_vec;
    std::vector<bool> bin_vec;
    for (int j = value; j; j = j/2) {
		bin_vec.push_back(j%2 ? 1: 0);
	}

    for (int k = 0;k < bin_vec.size(); k++)
    {
        if (bin_vec[k] == 1) {
            setbit_loc_vec.push_back(k);
        }
    }

    return setbit_loc_vec;
}

void SymbolicPeripherals::onStateSwitch(S2EExecutionState *currentState, S2EExecutionState *nextState) {

    getDebugStream() << "next irq flag = " << nextState->regs()->getInterruptFlag()
                     << " previous irq flag = " << currentState->regs()->getInterruptFlag() << "\n";

    // phs model learning
    getDebugStream() << nextState->regs()->getInterruptFlag()
                     << " flag irq_no_new_branch_flag = " << irq_no_new_branch_flag
                     << nextState->regs()->getInterruptFlag() << " flag no_new_branch_flag = " << no_new_branch_flag
                     << "\n";

    if (find(learning_mode_states.begin(), learning_mode_states.end(), nextState) ==
            learning_mode_states.end()) {
        getWarningsStream() << "Learning state can not be switch to fuzzing state\n";
        exit(-1);
    }

    AllKnowledgeBaseMap wrong_last_fork_phs, correct_last_fork_phs;
    if (!nextState->regs()->getInterruptFlag() && !currentState->regs()->getInterruptFlag()) {
        wrong_last_fork_phs = getLastBranchTargetRegValues(currentState, 0);
        correct_last_fork_phs = getLastBranchTargetRegValues(nextState, 0);
        // update flag
        no_new_branch_flag = true;
    }

    // irq mode
    if (nextState->regs()->getInterruptFlag() && currentState->regs()->getInterruptFlag()) {
        wrong_last_fork_phs = getLastBranchTargetRegValues(currentState, currentState->regs()->getExceptionIndex());
        correct_last_fork_phs = getLastBranchTargetRegValues(nextState, nextState->regs()->getExceptionIndex());
        // update flag
        irq_no_new_branch_flag = true;
    }

    // nlp diagnosis
    getWarningsStream() << "=========== Unit test Failed! ==========\n";
    // based on wrong state to identify
    for (auto wrong_last_fork_ph : wrong_last_fork_phs) {
        for (auto size_no_values : wrong_last_fork_ph.second) {
            uint32_t wrong_bits = correct_last_fork_phs[wrong_last_fork_ph.first][size_no_values.first].second ^ size_no_values.second.second;
            getWarningsStream() << "Wrong Peripheral = " << hexval(wrong_last_fork_ph.first.first)
                << " at pc = " << hexval(wrong_last_fork_ph.first.second) << " wrong value = " << hexval(size_no_values.second.second)
                << " correct value = " << hexval(correct_last_fork_phs[wrong_last_fork_ph.first][size_no_values.first].second) << "\n";
            std::string wrongbitStr= "Wrong bit:";
            for (auto bit_loc : identify_setbit_loc(wrong_bits)) {
                 wrongbitStr += " " + std::to_string(bit_loc);
            }
            getWarningsStream() << wrongbitStr << "\n";
        }
    }
    exit(-1);
}

AllKnowledgeBaseMap SymbolicPeripherals::getLastBranchTargetRegValues(S2EExecutionState *state, uint32_t irq_num) {
    DECLARE_PLUGINSTATE(SymbolicPeripheralsState, state);
    if (irq_num == 0) {
        return plgState->getlastfork_phs();
    } else {
        return plgState->irq_getlastfork_phs(irq_num);
    }
}

void SymbolicPeripherals::onSymbolicAddress(S2EExecutionState *state, ref<Expr> virtualAddress,
                                                uint64_t concreteAddress, bool &concretize,
                                                CorePlugin::symbolicAddressReason reason) {
    uint32_t curPc;
    curPc = state->regs()->getPc();
    symbolic_address_count[curPc]++;
}


} // namespace hw
} // namespace plugins
} // namespace s2e
