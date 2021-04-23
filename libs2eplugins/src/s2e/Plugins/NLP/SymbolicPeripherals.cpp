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

#include <s2e/ConfigFile.h>
#include <s2e/S2E.h>
#include <s2e/SymbolicHardwareHook.h>
#include <s2e/Utils.h>

#include <llvm/Support/CommandLine.h>

#include "SymbolicPeripherals.h"

namespace {
llvm::cl::opt<bool> DebugSymbHw("debug-symbolic-hardware", llvm::cl::init(false));
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

S2E_DEFINE_PLUGIN(SymbolicPeripherals, "SymbolicPeripherals S2E plugin", "", );

void SymbolicPeripherals::initialize() {
    if (!configSymbolicMmioRange()) {
        getWarningsStream() << "Could not parse config\n";
        exit(-1);
    }

    g_symbolicMemoryHook = SymbolicMemoryHook(symbhw_is_mmio_symbolic, symbhw_symbread, symbhw_symbwrite, this);
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

klee::ref<klee::Expr> SymbolicPeripherals::createExpression(S2EExecutionState *state, SymbolicHardwareAccessType type,
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

    ss << hexval(address) << "@" << hexval(state->regs()->getPc());

    uint32_t NLP_value = concreteValue;
    onSymbolicNLPRegisterReadEvent.emit(state, type, address, size, &NLP_value);

    getDebugStream(g_s2e_state) << ss.str() << " size " << hexval(size)
                                << "NLP value =" << hexval(NLP_value) << "\n";

    ConcreteArray concolicValue;
    SymbHwGetConcolicVector(concreteValue, size, concolicValue);
    return state->createSymbolicValue(ss.str(), size * 8, concolicValue);
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
    return hw->createExpression(g_s2e_state, SYMB_MMIO, physaddress, size, concreteValue);
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

    uint32_t writeConcreteValue;
    if (isa<klee::ConstantExpr>(value)) {
        klee::ref<klee::ConstantExpr> ce = dyn_cast<klee::ConstantExpr>(value);
        writeConcreteValue = ce->getZExtValue();
        getDebugStream() << "writing mmio " << hexval(phaddr) << " concrete value: " << hexval(writeConcreteValue)
                         << "\n";
    } else {
        // evaluate symbolic regs
        klee::ref<klee::ConstantExpr> ce;
        ce = dyn_cast<klee::ConstantExpr>(g_s2e_state->concolics->evaluate(value));
        writeConcreteValue = ce->getZExtValue();
        getDebugStream() << "writing mmio " << hexval(phaddr) << " symbolic to concrete value: " << hexval(writeConcreteValue) << "\n";
    }

    onSymbolicNLPRegisterWriteEvent.emit(g_s2e_state, SYMB_MMIO, phaddr, writeConcreteValue);

}

} // namespace hw
} // namespace plugins
} // namespace s2e
