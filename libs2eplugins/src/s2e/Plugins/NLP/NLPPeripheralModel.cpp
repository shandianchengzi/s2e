///
/// Copyright (C) 2010-2015, Dependable Systems Laboratory, EPFL
/// All rights reserved.
///
/// Licensed under the Cyberhaven Research License Agreement.
///

#include <s2e/ConfigFile.h>
#include <s2e/S2E.h>
#include <s2e/SymbolicHardwareHook.h>
#include <s2e/Utils.h>
#include <s2e/cpu.h>
#include <sys/shm.h>

#include "NLPPeripheralModel.h"

namespace s2e {
namespace plugins {

S2E_DEFINE_PLUGIN(NLPPeripheralModel, "NLP Peripheral Model", "NLPPeripheralModel");

class NLPPeripheralModelState : public PluginState {
private:
PeripheralRegs peripheral_regs_value_map;
public:
    NLPPeripheralModelState() {
    }

    virtual ~NLPPeripheralModelState() {
    }

    static PluginState *factory(Plugin *, S2EExecutionState *) {
        return new NLPPeripheralModelState();
    }

    NLPPeripheralModelState *clone() const {
        return new NLPPeripheralModelState(*this);
    }

    void write_ph_value(uint32_t phaddr, uint32_t value) {
        peripheral_regs_value_map[phaddr] = value;
    }

    uint32_t get_ph_value(uint32_t phaddr) {
        return  peripheral_regs_value_map[phaddr];
    }

};

void NLPPeripheralModel::initialize() {

    // bool ok;
    // ConfigFile *cfg = s2e()->getConfig();

    hw::SymbolicPeripherals *symbolicPeripheralConnection = s2e()->getPlugin<hw::SymbolicPeripherals>();
    symbolicPeripheralConnection->onSymbolicNLPRegisterReadEvent.connect(sigc::mem_fun(*this, &NLPPeripheralModel::onPeripheralRead));
    symbolicPeripheralConnection->onSymbolicNLPRegisterWriteEvent.connect(sigc::mem_fun(*this, &NLPPeripheralModel::onPeripheralWrite));

}

void NLPPeripheralModel::onPeripheralRead(S2EExecutionState *state, SymbolicHardwareAccessType type,
                            uint32_t phaddr, unsigned size, uint32_t *NLPsymbolicvalue) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    *NLPsymbolicvalue = plgState->get_ph_value(phaddr);
    // onExternalInterruptEvent.emit(state, irq_no);
}

void NLPPeripheralModel::onPeripheralWrite(S2EExecutionState *state, SymbolicHardwareAccessType type,
                            uint32_t phaddr, uint32_t writeconcretevalue) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    plgState->write_ph_value(phaddr, writeconcretevalue);
}


} // namespace plugins
} // namespace s2e
