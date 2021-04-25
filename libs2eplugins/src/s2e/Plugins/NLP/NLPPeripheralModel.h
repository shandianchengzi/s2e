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

namespace s2e {
namespace plugins {
typedef std::map<uint32_t /* peripheraladdress */, uint32_t /* value */> PeripheralRegs;

class NLPPeripheralModel : public Plugin {
    S2E_PLUGIN
public:
    NLPPeripheralModel(S2E *s2e) : Plugin(s2e) {
    }
    void initialize();
    sigc::signal<void, S2EExecutionState *, uint32_t /* irq_no */> onExternalInterruptEvent;

private:
    std::map<uint32_t, PeripheralReg> peripheral_regs_value_map;
    std::vector<pari<std::vector<Equation>, std::vector<Equation>>> allTAs;
    uint32_t data_register;
    std::string data_register_type = 'R';
    bool ReadMemofromFile(std::string fileName);
    bool ReadTAfromFile(std::string fileName);
    void SplitString(const std::string &s, std::vector<std::string> &v, const std::string &c);
    bool getMemo(std::string peripheralcache, PeripheralReg &reg);
    void getTApairs(std::string peripheralcache, std::vector<Equation> &trigger, std::vector<Equation> &action);
    void extractEqu(std::string peripheralcache, std::vector<Equation> &vec, bool rel);


    sigc::connection symbolicPeripheralConnection;
    void onPeripheralRead(S2EExecutionState *state, SymbolicHardwareAccessType type, uint32_t phaddr,
                     unsigned size, uint32_t *NLPsymbolicvalue);

    void onPeripheralWrite(S2EExecutionState *state, SymbolicHardwareAccessType type, uint32_t phaddr,
                     uint32_t  writeconcretevalue);
};

} // namespace plugins
} // namespace s2e

#endif // S2E_PLUGINS_NLPPeripheralModel_H
