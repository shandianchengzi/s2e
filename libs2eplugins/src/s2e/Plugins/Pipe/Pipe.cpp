///
/// Copyright (C) 2020-2025, HUST IoT S&P
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

#include "Pipe.h"

namespace s2e {
namespace plugins {

S2E_DEFINE_PLUGIN(Pipe, "data pipe for emulator ", "Pipe", "Pipe");

void Pipe::initialize() {

    bool ok;
    std::string testcaseName = s2e()->getConfig()->getString(getConfigKey() + ".testcaseName", "NULL", &ok);
    if (ok) {
        std::ifstream fT;
        fT.open(testcaseName, std::ios::in | std::ios::binary);
        if (!fT) {
            getWarningsStream() << "Testcase file cannot be opened\n";
        }

        char tcB;
        while (fT.read(&tcB, sizeof(tcB))) {
            testcase.push_back(tcB);
            getInfoStream() << "input testcase " << hexval(tcB) << "\n";
        }
        fT.close();
        if (testcase.size() == 0) {
            getWarningsStream() << " The length of testcase should greater than zero\n";
            exit(-1);
        }
    } else {
        getWarningsStream() << "No Testcase file Provided\n";
    }

    std::string modelName = s2e()->getConfig()->getString(getConfigKey() + ".modelName", "NULL", &ok);
    if (modelName == "uEmu") {
        onuEmuConnection = s2e()->getPlugin<PeripheralModelLearning>();
        onuEmuConnection->onDataInput.connect(
            sigc::mem_fun(*this, &Pipe::onDataInput));
    } else if (modelName == "SEmu"){
        onSEmuConnection = s2e()->getPlugin<NLPPeripheralModel>();
        onSEmuConnection->onDataInput.connect(
            sigc::mem_fun(*this, &Pipe::onDataInput));
    } else {
        getWarningsStream() << "Unsupported peripheral model is given!\n";
    }

    //concreteDataMemoryAccessConnection = s2e()->getCorePlugin()->onConcreteDataMemoryAccess.connect(
    //    sigc::mem_fun(*this, &Pipe::onConcreteDataMemoryAccess));

}

void Pipe::onDataInput(S2EExecutionState *state, uint32_t phaddr, uint32_t t3_count,
                       uint32_t size, uint32_t *value, bool *doFuzz) {

    memset(value, 0, 4 * sizeof(char));

    if (*doFuzz && g_s2e_cache_mode && t3_count == 0) {
        if (cur_read[phaddr] > testcase.size()) {
            getInfoStream() << "The whole testcase has been read by firmware, specific testcase analysis finish\n";
            g_s2e->getCorePlugin()->onEngineShutdown.emit();
            // Flush here just in case ~S2E() is not called (e.g., if atexit()
            // shutdown handler was not called properly).
            g_s2e->flushOutputStreams();
            exit(0);
        }
        if (cur_read[phaddr] + size > testcase.size()) {
            size = testcase.size() - cur_read[phaddr];
        }

        if (size == 4) {
            *value = testcase[cur_read[phaddr]] | ((uint32_t) testcase[cur_read[phaddr]+1] << 8)
                | ((uint32_t) testcase[cur_read[phaddr]+2] << 16) | ((uint32_t) testcase[cur_read[phaddr]+3] << 24);
        } else if (size == 3) {
            *value = testcase[cur_read[phaddr]] | ((uint32_t) testcase[cur_read[phaddr]+1] << 8)
                | ((uint32_t) testcase[cur_read[phaddr]+2] << 16);
        } else if (size == 2) {
            *value = testcase[cur_read[phaddr]] | ((uint32_t) testcase[cur_read[phaddr]+1] << 8);
        } else {
            *value = testcase[cur_read[phaddr]];
        }
        cur_read[phaddr] += size;
        getInfoStream() << " read the " << cur_read[phaddr] << " Bytes from whole testcase :" << hexval(*value) << "\n";
    }
}

void Pipe::onConcreteDataMemoryAccess(S2EExecutionState *state, uint64_t address, uint64_t value, uint8_t size,
                                           unsigned flags) {


    //uint32_t pc = state->regs()->getPc();
    bool is_write = false;
    if (flags & MEM_TRACE_FLAG_WRITE) {
        is_write = true;
    }

}

} // namespace plugins
} // namespace s2e
