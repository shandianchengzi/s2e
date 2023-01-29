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

S2E_DEFINE_PLUGIN(SymbolicPeripherals, "SymbolicPeripherals S2E plugin", "SymbolicPeripherals", "InvalidStatesDetection");
namespace {
class SymbolicPeripheralsState : public PluginState {
public:
    SymbolicPeripheralsState() {
    }

    virtual ~SymbolicPeripheralsState() {
    }

    static PluginState *factory(Plugin *, S2EExecutionState *) {
        return new SymbolicPeripheralsState();
    }

    SymbolicPeripheralsState *clone() const {
        return new SymbolicPeripheralsState(*this);
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

    g_symbolicMemoryHook = SymbolicMemoryHook(symbhw_is_mmio_symbolic, symbhw_symbread, symbhw_symbwrite, this);
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

klee::ref<klee::Expr> SymbolicPeripherals::onReadPeripheral(S2EExecutionState *state, SymbolicHardwareAccessType type,
                                                         uint64_t address, unsigned size, uint64_t concreteValue) {

    // peripheral address bit-band alias
    if (address >= 0x42000000 && address <= 0x43fffffc) {
        uint32_t phaddr = (address - 0x42000000) / 32 + 0x40000000;
        getDebugStream() << "bit band alias address = " << hexval(address) << " alias address = " << hexval(phaddr) << "\n";
        address = phaddr;
    }

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

    ss << hexval(address) << "@" << hexval(state->regs()->getPc());
    ss << "_" << hexval(size);

    uint32_t NLP_value = concreteValue;
    bool dr_flag = false;
    onSymbolicNLPRegisterReadEvent.emit(state, type, address, size, &NLP_value, &dr_flag);

    getInfoStream(g_s2e_state) << ss.str() << " size " << hexval(size) << "dr flag = " << dr_flag
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
    return hw->onReadPeripheral(g_s2e_state, SYMB_MMIO, physaddress, size, concreteValue);
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
        getInfoStream() << "writing mmio " << hexval(phaddr) << " concrete value: " << hexval(writeConcreteValue)
                         << "\n";
        // nlp regions
        for (auto nlpph : nlp_mmio) {
            if (phaddr >= nlpph.first && phaddr <= nlpph.second) {
                onSymbolicNLPRegisterWriteEvent.emit(g_s2e_state, SYMB_MMIO, phaddr, writeConcreteValue);
                return;
            }
        }
    } else {
        if (bit_alias) {
            getWarningsStream() << " bit band does not support symbolic value\n";
            exit(-1);
        }
        // evaluate symbolic regs
        klee::ref<klee::ConstantExpr> ce;
        ce = dyn_cast<klee::ConstantExpr>(g_s2e_state->concolics->evaluate(value));
        writeConcreteValue = ce->getZExtValue();
        getInfoStream() << "writing mmio " << hexval(phaddr) << " symbolic to concrete value: " << hexval(writeConcreteValue) << "\n";
        // nlp regions
        for (auto nlpph : nlp_mmio) {
            if (phaddr >= nlpph.first && phaddr <= nlpph.second) {
                onSymbolicNLPRegisterWriteEvent.emit(g_s2e_state, SYMB_MMIO, phaddr, writeConcreteValue);
                return;
            }
        }
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

bool comp(std::vector<uint32_t> &v1, std::vector<uint32_t> &v2) {
    for (int i = 0; i < v2.size(); i++) {
        if (std::find(v1.begin(), v1.end(), v2[i]) == v1.end())
            return false;
    }
    return true;
}

} // namespace hw
} // namespace plugins
} // namespace s2e
