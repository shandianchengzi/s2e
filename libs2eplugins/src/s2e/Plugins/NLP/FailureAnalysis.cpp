///
/// Copyright (C) 2014-2015, Cyberhaven
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
#include <s2e/Utils.h>

#include "FailureAnalysis.h"

namespace s2e {
namespace plugins {

S2E_DEFINE_PLUGIN(FailureAnalysis, "Limits how many times each instruction in a module can fork", "");

void FailureAnalysis::initialize() {
    // Limit of forks per program counter, -1 means don't care
    bool ok;
    m_limit = s2e()->getConfig()->getInt(getConfigKey() + ".maxForkCount", 10, &ok);
    if ((int) m_limit != -1) {
        s2e()->getCorePlugin()->onStateForkDecide.connect(sigc::mem_fun(*this, &FailureAnalysis::onStateForkDecide));

        s2e()->getCorePlugin()->onStateFork.connect(sigc::mem_fun(*this, &FailureAnalysis::onFork));
    }

    // Wait 5 seconds before allowing an S2E instance to fork
    m_processForkDelay = s2e()->getConfig()->getInt(getConfigKey() + ".processForkDelay", 5);

    m_timerTicks = 0;
}

void FailureAnalysis::onStateForkDecide(S2EExecutionState *state, bool *doFork,
                                    const klee::ref<klee::Expr> &condition, bool *conditionFork) {
    // uint32_t curPc = state->regs()->getPc();
}

void FailureAnalysis::onFork(S2EExecutionState *state, const std::vector<S2EExecutionState *> &newStates,
                         const std::vector<klee::ref<klee::Expr>> &newConditions) {
    // uint32_t curPc = state->regs()->getPc();
}

} // namespace plugins
} // namespace s2e
