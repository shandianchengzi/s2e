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

#include "AFLFuzzer.h"

namespace s2e {
namespace plugins {

S2E_DEFINE_PLUGIN(AFLFuzzer, "trigger and record external interrupts", "AFLFuzzer", "NLPPeripheralModel");

class AFLFuzzerState : public PluginState {
private:
    uint64_t hit_count;

public:
    AFLFuzzerState() {
        hit_count = 0;
    }

    virtual ~AFLFuzzerState() {
    }

    static PluginState *factory(Plugin *, S2EExecutionState *) {
        return new AFLFuzzerState();
    }

    AFLFuzzerState *clone() const {
        return new AFLFuzzerState(*this);
    }

    void inc_hit_count() {
        hit_count++;
    }

    uint64_t get_hit_count() {
        return hit_count;
    }

    void clear_hit_count() {
        hit_count = 0;
    }
};

/* Set up SHM region and initialize other stuff. */

static void afl_setup(void) {

    AFL_shm_id = shmget((key_t) AFL_IoT_S2E_KEY, sizeof(struct AFL_data), IPC_CREAT | 0660);
    bitmap_shm_id = shmget((key_t) AFL_BITMAP_KEY, MAP_SIZE, IPC_CREAT | 0660);
    testcase_shm_id = shmget((key_t) AFL_TESTCASE_KEY, TESTCASE_SIZE, IPC_CREAT | 0660);

    if (AFL_shm_id < 0 || bitmap_shm_id < 0) {
        printf("shmget error\n");
        exit(-1);
    } else {
        printf("AFL_shm_id = %d bitmap_shm_id = %d\n", AFL_shm_id, bitmap_shm_id);
    }

    afl_shm = shmat(AFL_shm_id, NULL, 0);
    bitmap_shm = shmat(bitmap_shm_id, NULL, 0);
    testcase_shm = shmat(testcase_shm_id, NULL, 0);
    afl_con = (struct AFL_data *) afl_shm;
    afl_area_ptr = (unsigned char *) bitmap_shm;
    testcase = (uint8_t *) testcase_shm;
    if (!afl_area_ptr || !afl_con) {
        printf("shmat error\n");
        exit(-1);
    }
}

void AFLFuzzer::initialize() {

    bool ok;
    ConfigFile *cfg = s2e()->getConfig();

    enable_fuzzing = s2e()->getConfig()->getBool(getConfigKey() + ".useAFLFuzzer", false);
    if (!enable_fuzzing) {
        getWarningsStream()
            << "Please ensure 'enable_fuzz' is true in your .cfg file! AFLFuzzer can be only used in cache mode\n";
        return;
    }

    int disable_input_peripheral_size = g_s2e->getConfig()->getListSize(getConfigKey() + ".disableInputPeripherals", &ok);
    firmwareName = s2e()->getConfig()->getString(getConfigKey() + ".firmwareName2", "x.elf");

    for (unsigned i = 0; i < disable_input_peripheral_size; i++) {
        uint32_t phaddr, size;
        std::stringstream ssphs;
        ssphs << getConfigKey() << ".disableInputPeripherals"
              << "[" << (i + 1) << "]";

        phaddr = cfg->getInt(ssphs.str() + "[1]", 0, &ok);
        size = cfg->getInt(ssphs.str() + "[2]", 0, &ok);
        if (size > 4) {
            Ethernet.addr = phaddr;
            Ethernet.size = size;
            Ethernet.pos = 0;
        } else {
            disable_input_peripherals[phaddr] = size;
        }

        getDebugStream() << "Add fuzzing target ph address = " << hexval(phaddr) << " size = " << hexval(size) << "\n";
    }

    if (!ok) {
        getWarningsStream() << " input peripherals is not vaild\n";
        exit(-1);
    }

    int additional_range_size = g_s2e->getConfig()->getListSize(getConfigKey() + ".writeRanges", &ok);
    for (unsigned i = 0; i < additional_range_size; i++) {
        uint32_t baseaddr, size;
        std::stringstream ssranges;
        ssranges << getConfigKey() << ".writeRanges"
                 << "[" << (i + 1) << "]";

        baseaddr = cfg->getInt(ssranges.str() + "[1]", 0, &ok);
        size = cfg->getInt(ssranges.str() + "[2]", 0, &ok);
        additional_writeable_ranges[baseaddr] = size;

        getDebugStream() << "Add additional writeable address = " << hexval(baseaddr) << " size = " << hexval(size)
                         << "\n";
    }

    int rom_num = g_s2e->getConfig()->getListSize("mem.rom");
    int ram_num = g_s2e->getConfig()->getListSize("mem.ram");
    std::stringstream ssrom;
    std::stringstream ssram;

    for (int i = 0; i < rom_num; ++i) {
        ssrom << "mem.rom"
              << "[" << (i + 1) << "]";
        MEM rom;
        rom.baseaddr = cfg->getInt(ssrom.str() + "[1]", 0, &ok);
        if (!ok) {
            getWarningsStream() << "Could not parse " << ssrom.str() + "baseaddr"
                                << "\n";
            return;
        }
        rom.size = cfg->getInt(ssrom.str() + "[2]", 0, &ok);
        if (!ok) {
            getWarningsStream() << "Could not parse " << ssrom.str() + "size"
                                << "\n";
            return;
        }
        roms.push_back(rom);
        getDebugStream() << "valid rom " << i + 1 << " baseaddr:" << hexval(roms[i].baseaddr)
                         << " size:" << hexval(roms[i].size) << "\n";
    }

    for (int i = 0; i < ram_num; ++i) {
        ssram << "mem.ram"
              << "[" << (i + 1) << "]";
        MEM ram;
        ram.baseaddr = cfg->getInt(ssram.str() + "[1]", 0, &ok);
        if (!ok) {
            getWarningsStream() << "Could not parse " << ssram.str() + "baseaddr"
                                << "\n";
            return;
        }
        ram.size = cfg->getInt(ssram.str() + "[2]", 0, &ok);
        if (!ok) {
            getWarningsStream() << "Could not parse " << ssram.str() + "size"
                                << "\n";
            return;
        }
        rams.push_back(ram);
        getDebugStream() << "valid ram " << i + 1 << " baseaddr:" << hexval(rams[i].baseaddr)
                         << " size:" << hexval(rams[i].size) << "\n";
    }

    blockStartConnection = s2e()->getCorePlugin()->onTranslateBlockStart.connect(
        sigc::mem_fun(*this, &AFLFuzzer::onTranslateBlockStart));
    blockEndConnection = s2e()->getCorePlugin()->onTranslateBlockEnd.connect(
        sigc::mem_fun(*this, &AFLFuzzer::onTranslateBlockEnd));
    concreteDataMemoryAccessConnection = s2e()->getCorePlugin()->onConcreteDataMemoryAccess.connect(
        sigc::mem_fun(*this, &AFLFuzzer::onConcreteDataMemoryAccess));
    invalidPCAccessConnection =
        s2e()->getCorePlugin()->onInvalidPCAccess.connect(sigc::mem_fun(*this, &AFLFuzzer::onInvalidPCAccess));

    min_input_length = s2e()->getConfig()->getInt(getConfigKey() + ".minInputLength", 64);
    if (min_input_length < 4) {
        getWarningsStream() << " testcase length should at least 32B!\n";
        exit(-1);
    }
    fork_point = s2e()->getConfig()->getInt(getConfigKey() + ".forkPoint", 0x0, &ok);
    if (!ok || fork_point == 0x0) {
        getWarningsStream() << " fork point should be set at the beginning of one basic block!\n";
        exit(-1);
    } else {
        getInfoStream() << "fork point = " << hexval(fork_point) << "\n";
    }

    begin_point = s2e()->getConfig()->getInt(getConfigKey() + ".beginPoint", 0x0, &ok);
    if (!ok || begin_point == 0x0) {
        getWarningsStream() << " begin_point will be same as fork_point!\n";
        begin_point = fork_point;
    } else {
        getInfoStream() << "begin point = " << hexval(begin_point) << "\n";
    }

    fork_flag = false;
    total_time = 0;
    disable_interrupt_count = 0;
    hang_timeout = s2e()->getConfig()->getInt(getConfigKey() + ".hangTimeout", 10);
    init_time = time(NULL);
    start_time = time(NULL);

    auto crash_keys = cfg->getIntegerList(getConfigKey() + ".crashPoints");
    foreach2 (it, crash_keys.begin(), crash_keys.end()) {
        getWarningsStream() << "Add kill point address = " << hexval(*it) << "\n";
        crash_points.push_back(*it);
    }

    NLPPeripheralModel *PeripheralConnection = s2e()->getPlugin<NLPPeripheralModel>();
    PeripheralConnection->onBufferInput.connect(sigc::mem_fun(*this, &AFLFuzzer::onBufferInput));

    afl_setup();

    bitmap = (uint8_t *) malloc(MAP_SIZE);
    afl_start_code = 0;
    afl_end_code = 0xffffffff;
    unique_tb_num = 0;
}

template <typename T> static bool getConcolicValue(S2EExecutionState *state, unsigned offset, T *value) {
    auto size = sizeof(T);

    klee::ref<klee::Expr> expr = state->regs()->read(offset, size * 8);
    if (isa<klee::ConstantExpr>(expr)) {
        klee::ref<klee::ConstantExpr> ce = dyn_cast<klee::ConstantExpr>(expr);
        *value = ce->getZExtValue();
        return true;
    } else {
        // evaluate symobolic regs
        klee::ref<klee::ConstantExpr> ce;
        ce = dyn_cast<klee::ConstantExpr>(state->concolics->evaluate(expr));
        *value = ce->getZExtValue();
        return false;
    }
}

static void PrintRegs(S2EExecutionState *state) {
    for (unsigned i = 0; i < 15; ++i) {
        unsigned offset = offsetof(CPUARMState, regs[i]);
        target_ulong concreteData;

        if (getConcolicValue(state, offset, &concreteData)) {
            g_s2e->getWarningsStream() << "Regs " << i << " = " << hexval(concreteData) << "\n";
        } else {
            g_s2e->getWarningsStream() << "Sym Regs " << i << " = " << hexval(concreteData) << "\n";
        }
    }
}

void AFLFuzzer::onCrashHang(S2EExecutionState *state, uint32_t flag, uint32_t pc) {
    PrintRegs(state);
    memcpy(afl_area_ptr, bitmap, MAP_SIZE);
    if (flag == 1) {
        afl_con->AFL_return = FAULT_CRASH;
    } else {
        afl_con->AFL_return = FAULT_TMOUT;
    }
    fork_flag = false;
    std::string s;
    llvm::raw_string_ostream ss(s);
    ss << "Kill path due to Crash/Hang\n";
    ss.flush();
    s2e()->getExecutor()->terminateState(*state, s);
}

void AFLFuzzer::onBufferInput(S2EExecutionState *state, uint32_t phaddr, uint32_t *testcase_size,
                            std::queue<uint8_t> *value) {
    bool doFuzz;
    doFuzz = true;
    if (doFuzz) {
        *testcase_size = afl_con->AFL_size;
        if (afl_con->AFL_input) {
            getInfoStream() << "AFL_input = " << afl_con->AFL_input
                            << " AFL_size = " << afl_con->AFL_size << "\n";
            start_time = time(NULL);
            for (uint32_t cur_read = 0; cur_read < afl_con->AFL_size; cur_read++) {
                uint8_t fuzz_value;
                memcpy(&fuzz_value, testcase + cur_read, 1);
                value->push(fuzz_value);
                getInfoStream() << " " << hexval(fuzz_value);
            }
            if (afl_con->AFL_size < min_input_length) {
                for (uint32_t i = 0; i < min_input_length - afl_con->AFL_size; i++) {
                    value->push(0x0);
                }
            }
            getInfoStream() << "\n";
        } else {
            getWarningsStream() << "AFL testcase is not ready!! return 0 "<< afl_con->AFL_size << "\n";
            for (uint32_t i = 0; i < min_input_length; i++) {
                value->push(0x0);
            }
        }
    }
}


void AFLFuzzer::onInvalidPCAccess(S2EExecutionState *state, uint64_t addr) {
    uint32_t pc = state->regs()->getPc();
    getWarningsStream() << "Kill path due to invaild pc  = " << hexval(pc) << " addr = " << hexval(addr) << "\n";
    onCrashHang(state, 1, pc);
}

void AFLFuzzer::onConcreteDataMemoryAccess(S2EExecutionState *state, uint64_t address, uint64_t value, uint8_t size,
                                           unsigned flags) {

    bool is_write = false;
    if (flags & MEM_TRACE_FLAG_WRITE) {
        is_write = true;
    }
    uint32_t pc = state->regs()->getPc();

    // ram regions
    for (auto ram : rams) {
        if (address >= ram.baseaddr && address <= (ram.baseaddr + ram.size)) {
            return;
        }
    }

    // peripheral regions
    if (address >= 0x40000000 && address < 0x60000000) {
        return;
    }

    // external peripheral regions
    if (address >= 0xe0000000 && address < 0xe0100000) {
        return;
    }

    // additional user-defined available rw regions
    for (auto writeable_range : additional_writeable_ranges) {
        if (address >= writeable_range.first && address < writeable_range.first + writeable_range.second) {
            return;
        }
    }

    if (!is_write) {
        // only allow read from rom regions
        for (auto rom : roms) {
            if (address >= rom.baseaddr && address < (rom.baseaddr + rom.size)) {
                return;
            }
        }
        getWarningsStream() << "Kill Fuzz State due to out of bound read, access address = " << hexval(address)
                            << " pc = " << hexval(pc) << "\n";
    } else {
        getWarningsStream() << "Kill Fuzz State due to out of bound write, access address = " << hexval(address)
                            << " pc = " << hexval(pc) << "\n";
    }

    onCrashHang(state, 1, pc);
}

void AFLFuzzer::recordTBMap() {
    std::string fileName;
    std::size_t index = firmwareName.find_last_of("/\\");
    fileName = s2e()->getOutputDirectory() + "/" + firmwareName.substr(index + 1) + "fuzz_tb_map.txt";
    std::ofstream fTBmap;
    fTBmap.open(fileName, std::ios::out | std::ios::trunc);

    for (auto ittb : all_tb_map) {
        if (ittb.second > 0)
            fTBmap << hexval(ittb.first) << " " << ittb.second << std::endl;;
    }

    fTBmap.close();
}

void AFLFuzzer::recordTBNum() {
    std::string fileName;
    std::size_t index = firmwareName.find_last_of("/\\");
    fileName = s2e()->getOutputDirectory() + "/" + firmwareName.substr(index + 1) + "fuzz_tb_num.csv";
    std::ofstream fTBNum;
    fTBNum.open(fileName, std::ios::out | std::ios::trunc);

    for (auto ittbnum : total_time_tbnum) {
        fTBNum << ittbnum.first * 15 << "," << ittbnum.second << std::endl;;
    }

    fTBNum.close();
}

void AFLFuzzer::onTranslateBlockEnd(ExecutionSignal *signal, S2EExecutionState *state,
                                                 TranslationBlock *tb, uint64_t pc, bool staticTarget,
                                                 uint64_t staticTargetPc) {
    signal->connect(
        sigc::bind(sigc::mem_fun(*this, &AFLFuzzer::onBlockEnd), (unsigned) tb->se_tb_type));
}

static __thread uint64_t prev_loc;
void AFLFuzzer::onBlockEnd(S2EExecutionState *state, uint64_t cur_loc, unsigned source_type) {

    getInfoStream(state) << state->regs()->getInterruptFlag() << " current pc = " << hexval(cur_loc) <<  "\n";
    ++tb_num;
    end_time = time(NULL);
    if (fork_flag && ((end_time - init_time) / 60 > 14)) {
        init_time = time(NULL);
        total_time++;
        total_time_tbnum[total_time] = unique_tb_num;
        getWarningsStream() << 15 * total_time << "min: " << unique_tb_num << "unique tb number\n";
    }

    // uEmu ends up with fuzzer
    if (unlikely(afl_con->AFL_return == END_uEmu)) {
        recordTBMap();
        recordTBNum();
        getInfoStream() << "The total number of unique executed tb is " << unique_tb_num << "\n";
        getWarningsStream() << "==== Testing aborted by user via Fuzzer ====\n";
        g_s2e->getCorePlugin()->onEngineShutdown.emit();
        // Flush here just in case ~S2E() is not called (e.g., if atexit()
        // shutdown handler was not called properly).
        g_s2e->flushOutputStreams();
        exit(0);
    }

    if (state->regs()->getInterruptFlag() && state->regs()->getExceptionIndex() < 16) {
        disable_interrupt_count = 100;
    } else {
        if (disable_interrupt_count > 0) {
            disable_interrupt_count--;
        }
    }

    if (disable_interrupt_count == 0) {
        g_s2e_allow_interrupt = 1;
    } else {
        g_s2e_allow_interrupt = 0;
    }

    if (!state->regs()->getInterruptFlag()) {
        if (fork_flag && tb_num % 500 == 0) {
            getDebugStream() << " force exit every max loop tb num " << tb_num << "\n";
            g_s2e_allow_interrupt = 1;
            s2e()->getExecutor()->setCpuExitRequest();
        }
    }

    if (state->regs()->getInterruptFlag() && state->regs()->getExceptionIndex() < 10) {
        getWarningsStream() << "Kill Fuzz State due to Fault interrupt = " << state->regs()->getExceptionIndex()
                            << " pc = " << hexval(cur_loc) << "\n";
        onCrashHang(state, 1, cur_loc);
    }

    // user-defined crash points
    for (auto crash_point : crash_points) {
        if (fork_flag && crash_point == cur_loc) {
            getWarningsStream() << "Kill Fuzz state due to user-defined crash points\n";
            onCrashHang(state, 1, cur_loc);
        }
    }

    // crash/hang
    if (unlikely((end_time - start_time) > hang_timeout - 1)) {
        getWarningsStream() << "Kill Fuzz state due to timeout at pc = " << hexval(cur_loc) << "\n";
        if (unlikely((end_time - start_time) > hang_timeout)) {
            onCrashHang(state, 0, cur_loc);
        }
    }

    // path bitmap
    if (cur_loc > afl_end_code || cur_loc < afl_start_code || !bitmap)
        return;

    /* Looks like QEMU always maps to fixed locations, so ASAN is not a
     concern. Phew. But instruction addresses may be aligned. Let's mangle
     the value to get something quasi-uniform. */

    // Do not map external interrupt
    if (state->regs()->getInterruptFlag()) {
        if (state->regs()->getExceptionIndex() > 15) {
            return;
        } else if (state->regs()->getExceptionIndex() == 15) {
            cur_loc = (cur_loc >> 8) ^ (cur_loc << 4);
            cur_loc &= MAP_SIZE - 1;
            cur_loc |= 0x8000;
            if (cur_loc >= afl_inst_rms)
                return;
            if (bitmap[cur_loc]) // only count once for systick irq
                return;
            bitmap[cur_loc]++;
            return;
        }
    }


    cur_loc = (cur_loc >> 8) ^ (cur_loc << 4);
    cur_loc &= MAP_SIZE/2 - 1;

    /* Implement probabilistic instrumentation by looking at scrambled block
     address. This keeps the instrumented locations stable across runs. */

    if (cur_loc >= afl_inst_rms)
        return;
    bitmap[cur_loc ^ prev_loc]++;
    prev_loc = cur_loc >> 1;
}

void AFLFuzzer::onTranslateBlockStart(ExecutionSignal *signal, S2EExecutionState *state,
                                                   TranslationBlock *tb, uint64_t pc) {
    signal->connect(sigc::mem_fun(*this, &AFLFuzzer::onForkPoints));
}

void AFLFuzzer::onForkPoints(S2EExecutionState *state, uint64_t pc) {
    DECLARE_PLUGINSTATE(AFLFuzzerState, state);
    // record total bb number
    if (all_tb_map[pc] < 1) {
        getWarningsStream() << "The unqiue number of the executed basic blocks in current state is "
                            << unique_tb_num << " pc = " << hexval(pc) << "\n";
    }
    ++unique_tb_num;
    ++all_tb_map[pc];

    if (pc == fork_point) {
    //if (pc == fork_point || pc == 0x80003d4 || pc == 0x8000424) {
        //if (fork_flag == false) {
        if (fork_flag) {
            getInfoStream() << "3 fork state at pc = " << hexval(state->regs()->getPc()) << "\n";
            memcpy(afl_area_ptr, bitmap, MAP_SIZE);
            afl_con->AFL_input = 0;
            Ethernet.pos = 0;
            afl_con->AFL_return = 0;
            start_time = time(NULL);
            plgState->inc_hit_count();
            prev_loc = 0;
            uint8_t content = 0;
            for (int i = 0; i < 256; i++) {
                state->mem()->write(0x200000A0 + i, &content, sizeof(content));
            }
            if (plgState->get_hit_count() == 1000) {
                fork_flag = false;
                std::string s;
                llvm::raw_string_ostream ss(s);
                ss << "Kill path due to one thousand\n";
                ss.flush();
                s2e()->getExecutor()->terminateState(*state, s);
            }
            usleep(10000);
        } else {
            prev_loc = 0;
            memset(afl_area_ptr, 0, MAP_SIZE);
            if (begin_point == fork_point) {
                forkPoint(state);
                //PrintRegs(state);
            }
            getWarningsStream() << " 4 fork state at pc = " << hexval(state->regs()->getPc()) << "\n";
            start_time = time(NULL);
            fork_flag = true;
        }
    } else if (pc == begin_point) {
        if (fork_flag) {
            getInfoStream() << " 1 fork state at pc = " << hexval(state->regs()->getPc()) << "\n";
            memcpy(afl_area_ptr, bitmap, MAP_SIZE);
            afl_con->AFL_input = 0;
            Ethernet.pos = 0;
            afl_con->AFL_return = 0;
            prev_loc = 0;
            start_time = time(NULL);
            plgState->inc_hit_count();
            if (plgState->get_hit_count() == 1000) {
                fork_flag = false;
                std::string s;
                llvm::raw_string_ostream ss(s);
                ss << "Kill path due to one thousand\n";
                ss.flush();
                s2e()->getExecutor()->terminateState(*state, s);
            }
            usleep(10000);
        } else {
            prev_loc = 0;
            memset(afl_area_ptr, 0, MAP_SIZE);
            start_time = time(NULL);
            forkPoint(state);
            getWarningsStream() << " 0 fork state at pc = " << hexval(state->regs()->getPc()) << "\n";
            //PrintRegs(state);
            fork_flag = true;
        }
    }
}

void AFLFuzzer::forkPoint(S2EExecutionState *state) {
    //target_ulong count;
    //target_ulong nameptr;

    state->jumpToSymbolicCpp();


    std::string name = "fork_point";

    // add a meaningless symbol for a cond to fork
    klee::ref<klee::Expr> var = state->createSymbolicValue<uint32_t>(name, 0);

    for (unsigned i = 1; i < 2; ++i) {
        klee::ref<klee::Expr> val = klee::ConstantExpr::create(i, var->getWidth());
        klee::ref<klee::Expr> cond = klee::NeExpr::create(var, val);

        klee::Executor::StatePair sp = s2e()->getExecutor()->forkCondition(state, cond, true);
        assert(sp.first == state);
        assert(sp.second && sp.second != sp.first);
        if (sp.second) {
            // Re-execute the plugin invocation in the other state
            sp.second->pc = sp.second->prevPC;
        }
    }

    /*klee::ref<klee::Expr> cond = klee::EqExpr::create(var, klee::ConstantExpr::create(0, var->getWidth()));*/
    //if (!state->addConstraint(cond)) {
        //s2e()->getExecutor()->terminateState(*state, "Could not add condition");
    /*}*/
}


} // namespace plugins
} // namespace s2e
