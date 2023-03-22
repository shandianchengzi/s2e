///
/// Copyright (C) 2020-2025, Hust IoTS&P
/// All rights reserved.
///
/// Licensed under the Cyberhaven Research License Agreement.
///

#ifndef S2E_PLUGINS_EXAMPLE_H
#define S2E_PLUGINS_EXAMPLE_H

#include <s2e/CorePlugin.h>
#include <s2e/Plugin.h>
#include <s2e/Plugins/uEmu/PeripheralModelLearning.h>
#include <s2e/Plugins/SEmu/NLPPeripheralModel.h>
#include <s2e/S2EExecutionState.h>

namespace s2e {
namespace plugins {

/* Execution status fault codes */
enum {
    /* 00 */ FAULT_NONE,
    /* 01 */ FAULT_TMOUT,
    /* 02 */ FAULT_CRASH,
    /* 03 */ FAULT_ERROR,
    /* 04 */ FAULT_NOINST,
    /* 05 */ FAULT_NOBITS,
    /* 06 */ END_uEmu
};
/* crash/hang codes */
enum {
    /* 00 */ TMOUT,
    /* 01 */ INVALIDPC,
    /* 02 */ OBREAD,
    /* 03 */ OBWRITE,
    /* 04 */ HARDFAULT,
    /* 05 */ UDPC,
    /* 06 */ INVALIDPH
};


class Pipe : public Plugin {
    S2E_PLUGIN
public:
    Pipe(S2E *s2e) : Plugin(s2e) {
    }

    void initialize();

    struct MEM {
        uint32_t baseaddr;
        uint32_t size;
    };

private:
    //sigc::connection concreteDataMemoryAccessConnection;
    NLPPeripheralModel *onSEmuConnection;
    PeripheralModelLearning *onuEmuConnection;

    bool enable_fuzzing;
    std::vector<uint8_t> testcase;
    std::map<uint32_t /* phaddr */, uint32_t /* size */> input_peripherals;
    std::map<uint32_t /* phaddr */, uint32_t /* size */> additional_writeable_ranges;
    std::map<uint32_t /* phaddr */, uint32_t /* cur_loc */> cur_read;

    void onConcreteDataMemoryAccess(S2EExecutionState *state, uint64_t vaddr, uint64_t value, uint8_t size,
                                    unsigned flags);
    void onInvalidPHs(S2EExecutionState *state, uint64_t addr);
    void onModeSwitch(S2EExecutionState *state, bool fuzzing_to_learning, bool *fork_point_flag);
    void onDataInput(S2EExecutionState *state, uint32_t phaddr, uint32_t t3_count,
                        uint32_t size, uint32_t *value, bool *doFuzz);

};

} // namespace plugins
} // namespace s2e

#endif // S2E_PLUGINS_EXAMPLE_H
