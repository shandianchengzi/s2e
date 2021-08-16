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

#include "ExternalInterrupt.h"

namespace s2e {
namespace plugins {

S2E_DEFINE_PLUGIN(ExternalInterrupt, "trigger and record external interrupts", "ExternalInterrupt", "NLPPeripheralModel");

class ExternalInterruptState : public PluginState {
private:
    typedef llvm::DenseMap<uint32_t, uint32_t> TBCounts;

    uint64_t tb_count;
    uint64_t re_tb_count;
    uint64_t new_tb_count;
    std::vector<uint32_t> last_irqs_bitmap;
    std::map<uint32_t /*external irq no*/, uint32_t /* count */> pirqs_map;
    std::map<uint32_t /* external irq no */, bool /*enable*/> active_irqs;
    bool disable_systick; // per state
    bool enable_interrupt;
    TBCounts new_tb_map;

public:
    ExternalInterruptState() {
        tb_count = 0;
        re_tb_count = 0;
        new_tb_map.clear();
        pirqs_map.clear();
        disable_systick = true;
        enable_interrupt = false;
        last_irqs_bitmap.push_back(0);
        last_irqs_bitmap.push_back(0);
    }

    virtual ~ExternalInterruptState() {
    }

    static PluginState *factory(Plugin *, S2EExecutionState *) {
        return new ExternalInterruptState();
    }

    ExternalInterruptState *clone() const {
        return new ExternalInterruptState(*this);
    }

    void inc_tb_num(uint32_t cur_pc) {
        ++tb_count;
        if (new_tb_map[cur_pc] < 1) {
            ++new_tb_map[cur_pc];
            ++new_tb_count;
            re_tb_count = 0;
        } else {
            ++re_tb_count;
        }
    }

    uint64_t get_tb_num() {
        return tb_count;
    }

    uint64_t get_newtb_num() {
        return new_tb_count;
    }

    uint64_t get_rettb_num() {
        return re_tb_count;
    }

    void set_systick_flag(bool systick_cmp_flag) {
        disable_systick = systick_cmp_flag;
    }

    bool get_systick_flag() {
        return disable_systick;
    }

    void set_enableinterrupt_flag(bool enableinterrupt) {
        enable_interrupt = enableinterrupt;
    }

    bool get_enableinterrupt_flag() {
        return enable_interrupt;
    }

    void insert_lastirqs_bitmap(std::vector<uint32_t> irqsbitmap) {
        last_irqs_bitmap = irqsbitmap;
    }

    std::vector<uint32_t> get_lastirqs_bitmap() {
        return last_irqs_bitmap;
    }

    void update_activeirqs(std::map<uint32_t, bool> activeirqs) {
        active_irqs = activeirqs;
    }

    void insert_activeirqs(uint32_t irq_no) {
        active_irqs[irq_no] = true;
    }

    std::map<uint32_t, bool> get_activeirqs() {
        return active_irqs;
    }
};

void ExternalInterrupt::initialize() {
    // tb_interval = s2e()->getConfig()->getInt(getConfigKey() + ".tbInterval", 2000, &ok);
    // tb_scale = s2e()->getConfig()->getInt(getConfigKey() + ".BBScale", 30000, &ok);
    /*s2e()->getCorePlugin()->onTranslateBlockStart.connect(*/
        //sigc::mem_fun(*this, &ExternalInterrupt::onTranslateBlockStart));
    bool ok;
    /*if (!ok) {*/
        //getWarningsStream()
            //<< "Could not set correct limit repeat conditions count, count should be greater than two\n";
        //return;
    //} else {
        //getDebugStream() << "trigger each external irq every " << tb_interval << " translation blocks"
                         //<< " total bb scale is " << tb_scale << "\n";
    /*}*/
    onNLPPeripheralModelConnection = s2e()->getPlugin<NLPPeripheralModel>();
    onNLPPeripheralModelConnection->onExternalInterruptEvent.connect(
        sigc::mem_fun(*this, &ExternalInterrupt::onExternelInterruptTrigger));

    ConfigFile *cfg = s2e()->getConfig();
    auto disableirqs = cfg->getIntegerList(getConfigKey() + ".disableIrqs");
    foreach2 (it, disableirqs.begin(), disableirqs.end()) {
        getDebugStream() << "Add disable irqs = " << hexval(*it) << "\n";
        disable_irqs.push_back(*it);
    }

    systick_disable_flag = s2e()->getConfig()->getBool(getConfigKey() + ".disableSystickInterrupt", false);
    if (systick_disable_flag) {
        systick_begin_point = s2e()->getConfig()->getInt(getConfigKey() + ".systickBeginPoint", 0x0, &ok);
        if (!ok) {
            getWarningsStream() << " systick begin point should be set!\n";
            return;
        } else {
            getInfoStream() << "systick begin point = " << hexval(systick_begin_point) << "\n";
        }
    }
}

/*void ExternalInterrupt::onTranslateBlockStart(ExecutionSignal *signal, S2EExecutionState *state, TranslationBlock *tb,*/
                                              //uint64_t pc) {
    //signal->connect(sigc::mem_fun(*this, &ExternalInterrupt::onBlockStart));
/*}*/

std::map<uint32_t, bool> setActiveIrqs(std::vector<uint32_t> irqs_bitmap) {
    std::map<uint32_t /* external irq no */, bool /*enable*/> active_irqs;

    for (int i = 0; i < irqs_bitmap.size(); i++) {
        for (int j = 0; j < 32; j++) {
            if (irqs_bitmap[i] & (1 << j)) {
                active_irqs[i * 32 + j] = true;
            }
        }
    }
    return active_irqs;
}

void ExternalInterrupt::onExternelInterruptTrigger(S2EExecutionState *state, uint32_t irq_no) {
    DECLARE_PLUGINSTATE(ExternalInterruptState, state);

    if (plgState->get_systick_flag() == true && systick_disable_flag) {
        s2e()->getExecutor()->disableSystickInterrupt(0);
        if (state->regs()->getPc() == systick_begin_point) {
            getDebugStream() << "enable systick at " << hexval(state->regs()->getPc()) << "\n";
            plgState->set_systick_flag(false);
            s2e()->getExecutor()->disableSystickInterrupt(7);
        }
    }

    std::vector<uint32_t> irqs_bitmap;
    std::vector<uint32_t> last_irqs_bitmap;
    irqs_bitmap.push_back(s2e()->getExecutor()->getActiveExternalInterrupt(0));
    getDebugStream() << "external bit map = " << hexval(irqs_bitmap[0]) << "\n";
    irqs_bitmap.push_back(s2e()->getExecutor()->getActiveExternalInterrupt(4));
    getDebugStream() << "external bit map 2 = " << hexval(irqs_bitmap[1]) << "\n";
    irqs_bitmap.push_back(s2e()->getExecutor()->getActiveExternalInterrupt(8));
    getDebugStream() << "external bit map 3 = " << hexval(irqs_bitmap[2]) << "\n";

    last_irqs_bitmap = plgState->get_lastirqs_bitmap();
    for (int k = 0; k < irqs_bitmap.size(); k++) {
        if (last_irqs_bitmap[k] != irqs_bitmap[k]) {
            getInfoStream() << "active irq has changed\n";
            getInfoStream() << "external irq bit map = " << hexval(irqs_bitmap[0]) << "\n";
            getInfoStream() << "external irq bit map = " << hexval(irqs_bitmap[1]) << "\n";
            getInfoStream() << "external irq bit map = " << hexval(irqs_bitmap[2]) << "\n";
            plgState->insert_lastirqs_bitmap(irqs_bitmap);
            plgState->update_activeirqs(setActiveIrqs(irqs_bitmap));
            break;
        }
    }

    std::vector<uint32_t> active_irqs;
    for (auto it : plgState->get_activeirqs()) {
        if (it.second == true) {
            active_irqs.push_back(it.first);
        }
    }

    if (std::find(active_irqs.begin(), active_irqs.end(), irq_no) != active_irqs.end()) {
        if (std::find(disable_irqs.begin(), disable_irqs.end(), irq_no - 16) == disable_irqs.end()) {
            getDebugStream() << " trigger external irq " << irq_no << "\n";
            s2e()->getExecutor()->setExternalInterrupt(irq_no);
        }
    } else {
        getWarningsStream() << "cannot trigger nlp interrupt no = " << irq_no << ", since it is not active\n";
    }

    // each exteranl irq trigger
}

} // namespace plugins
} // namespace s2e
