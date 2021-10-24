///
/// Copyright (C) 2010-2015, Dependable Systems Laboratory, EPFL
/// All rights reserved.
///
/// Licensed under the Cyberhaven Research License Agreement.
///

#include "NLPPeripheralModel.h"
#include <s2e/ConfigFile.h>
#include <s2e/S2E.h>
#include <s2e/SymbolicHardwareHook.h>
#include <s2e/Utils.h>
#include <s2e/cpu.h>
#include <sys/shm.h>
#include <time.h>

namespace s2e {
namespace plugins {

S2E_DEFINE_PLUGIN(NLPPeripheralModel, "NLP Peripheral Model With Auto Timer", "NLPPeripheralModel");

class NLPPeripheralModelState : public PluginState {
private:
    RegMap state_map;
    std::map<int, int> exit_interrupt; // interrupt id, num
    std::map<uint32_t, uint32_t> interrupt_freq;
    uint32_t fork_point_count;
    // std::map<int, bool> exit_interrupt;
public:
    NLPPeripheralModelState() {
        interrupt_freq.clear();
        fork_point_count = 0;
    }

    virtual ~NLPPeripheralModelState() {
    }

    static PluginState *factory(Plugin *, S2EExecutionState *) {
        return new NLPPeripheralModelState();
    }

    NLPPeripheralModelState *clone() const {
        return new NLPPeripheralModelState(*this);
    }

    bool get_exit_interrupt(uint32_t num) {
        return exit_interrupt[num] > 0;
        // return exit_interrupt[num];
    }

    void set_exit_interrupt(uint32_t num, int cur) {
        exit_interrupt[num] += cur;
        // exit_interrupt[num] = cur;
    }

    RegMap get_state_map() {
        return state_map;
    }

    void insert_reg_map(uint32_t phaddr, PeripheralReg reg) {
        state_map[phaddr] = reg;
    }

    void write_ph_value(uint32_t phaddr, uint32_t value) {
        state_map[phaddr].cur_value = value;
    }

    uint32_t get_ph_value(uint32_t phaddr) {
        return state_map[phaddr].cur_value;
    }

    void write_dr_value(uint32_t phaddr, uint32_t value, uint32_t width) {
        state_map[phaddr].t_value = value;
        state_map[phaddr].t_size = 0; // width;
    }

    uint8_t get_dr_value(uint32_t phaddr, uint32_t width) {
        width *= 8;
        state_map[phaddr].r_size -= width;
        if (state_map[phaddr].r_value.empty()) {
            state_map[phaddr].r_size = 0;
            return 0;
        }
        uint8_t cur_value = state_map[phaddr].r_value.front();
        state_map[phaddr].r_value.pop();
        return cur_value;
        //int res = 0;
        //uint8_t cur_value = state_map[phaddr].r_value.front();
        //for (int i = 0; i < width; ++i) {
        //    res = (res << 1) & (cur_value & 1);
        //    cur_value >>= 1;
        //}
        //state_map[phaddr].front_left -= width;
        //if (state_map[phaddr].front_left <= 0) {
        //    state_map[phaddr].r_value.pop();
        //    state_map[phaddr].front_left = 8;
        //}
        //int length = 0;
        //for (int i = 0; i < width; ++i) {
        //    length |= (1 << i);
        //}
        //return state_map[phaddr].r_value & length;
    }

    void hardware_write_to_receive_buffer(uint32_t phaddr, std::queue<uint8_t> value, uint32_t width) {
        state_map[phaddr].r_size = width * 8; //TODO: width is all data in the queue.
        //state_map[phaddr].front_left = 8;     //front size
        state_map[phaddr].r_value = value;
    }

    void inc_irq_freq(uint32_t irq_no) {
        interrupt_freq[irq_no]++;
    }

    uint32_t get_irq_freq(uint32_t irq_no) {
        return interrupt_freq[irq_no];
    }

    std::map<uint32_t, uint32_t> get_irqs_freq() {
        return interrupt_freq;
    }

    void inc_fork_count() {
        fork_point_count++;
    }

    uint32_t get_fork_point_count() {
        return fork_point_count;
    }
};

bool NLPPeripheralModel::parseConfig(void) {
    ConfigFile *cfg = s2e()->getConfig();
    hw::PeripheralMmioRanges nlpphs;
    std::stringstream ss;
    ss << getConfigKey();
    getDebugStream() << "config " << ss.str() << "\n";
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

template <typename T>
bool NLPPeripheralModel::parseRangeList(ConfigFile *cfg, const std::string &key, T &result) {
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

void NLPPeripheralModel::initialize() {
    NLPfileName = s2e()->getConfig()->getString(getConfigKey() + ".NLPfileName", "all.txt");
    getDebugStream() << "NLP firmware name is " << NLPfileName << "\n";

    if (!parseConfig()) {
        getWarningsStream() << "Could not parse NLP range config\n";
        exit(-1);
    }

    hw::SymbolicPeripherals *symbolicPeripheralConnection = s2e()->getPlugin<hw::SymbolicPeripherals>();
    symbolicPeripheralConnection->onSymbolicNLPRegisterReadEvent.connect(
        sigc::mem_fun(*this, &NLPPeripheralModel::onPeripheralRead));
    symbolicPeripheralConnection->onSymbolicNLPRegisterWriteEvent.connect(
        sigc::mem_fun(*this, &NLPPeripheralModel::onPeripheralWrite));
    onInvalidStateDectionConnection = s2e()->getPlugin<InvalidStatesDetection>();
    onInvalidStateDectionConnection->onReceiveExternalDataEvent.connect(
        sigc::mem_fun(*this, &NLPPeripheralModel::onEnableReceive));
    s2e()->getCorePlugin()->onEngineShutdown.connect(sigc::mem_fun(*this, &NLPPeripheralModel::onStatistics));

    enable_fuzzing = s2e()->getConfig()->getBool(getConfigKey() + ".useFuzzer", false);
    if (enable_fuzzing) {
        init_dr_flag = false;
        s2e()->getCorePlugin()->onTranslateBlockEnd.connect(
            sigc::mem_fun(*this, &NLPPeripheralModel::onTranslateBlockEnd));
    }

    bool ok;
    fork_point = s2e()->getConfig()->getInt(getConfigKey() + ".forkPoint", 0x0, &ok);
    getWarningsStream() << "set fork_point phaddr = " << hexval(fork_point) << "\n";
    s2e()->getCorePlugin()->onTranslateBlockStart.connect(
        sigc::mem_fun(*this, &NLPPeripheralModel::onTranslateBlockStart));
    s2e()->getCorePlugin()->onExceptionExit.connect(sigc::mem_fun(*this, &NLPPeripheralModel::onExceptionExit));
    rw_count = 0;
    srand(time(NULL));
}

uint32_t NLPPeripheralModel::get_reg_value(RegMap &state_map, Field a) {
    uint32_t res;
    if (a.type == "T") {
        return state_map[a.phaddr].t_size;
    } else if (a.type == "R") {
        return state_map[a.phaddr].r_size;
    } else if (a.bits[0] == -1) {
        res = state_map[a.phaddr].cur_value;
    } else {
        res = 0;
        for (int i = 0; i < a.bits.size(); ++i) {
            int tmp = a.bits[i];
            res = (res << 1) + (state_map[a.phaddr].cur_value >> tmp & 1);
        }
    }
    return res;
}

void NLPPeripheralModel::set_reg_value(RegMap &state_map, Field a, uint32_t value) {
    if (a.bits[0] == -1) {
        state_map[a.phaddr].cur_value = value;
    } else {
        for (int i = 0; i < a.bits.size(); ++i) {
            int tmp = a.bits[i];
            int a2 = (value >> (a.bits.size() - 1 - i)) & 1;
            if (a2 == 1) {
                state_map[a.phaddr].cur_value |= (1 << tmp);
            } else {
                state_map[a.phaddr].cur_value &= ~(1 << tmp);
            }
        }
    }
}

void NLPPeripheralModel::onExceptionExit(S2EExecutionState *state, uint32_t irq_no) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    // interrupt vector+16
    // if (irq_no > 15)
    // plgState->set_exit_interrupt(irq_no - 16, false);
    if (irq_no > 15)
        irq_no -= 16;
    plgState->set_exit_interrupt(irq_no, -1);

    getInfoStream() << "EXIT Interrupt IRQ" << irq_no << " exit_inter = " << plgState->get_exit_interrupt(irq_no)
                    << "\n";
    // fuzzing mode, if exit irq, check out if the rx is still empty
    if (enable_fuzzing) {
        UpdateGraph(g_s2e_state, Write, 0);
    }
    /*
    if (plgState->get_exit_interrupt(irq_no) && enable_fuzzing) {
        getWarningsStream() << "IRQ Action restart = " << irq_no << "\n";
        bool irq_triggered = false;
        onExternalInterruptEvent.emit(state, equ.interrupt, &irq_triggered);
        if (irq_triggered) {
            getWarningsStream() << " DATA IRQ Action trigger interrupt equ.interrupt = " << interrupt_freq[equ.interrupt] << "\n";
            interrupt_freq[equ.interrupt]++;
        }
    }
    */
}

// void NLPPeripheralModel::onInvalidStatesDetection(S2EExecutionState *state, uint32_t pc, InvalidStatesType type,
//                                                       uint64_t tb_num) {
// UpdateFlag();
//}

void NLPPeripheralModel::onEnableReceive(S2EExecutionState *state, uint32_t pc, uint64_t tb_num) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    // Write a value to DR
    if (!enable_fuzzing) {
        getWarningsStream() << " write init dr value 0x2D! phaddr =  \n";

        for (auto phaddr : data_register) {
            std::queue<uint8_t> tmp;
            tmp.push(0x2D);
            plgState->hardware_write_to_receive_buffer(phaddr, tmp, 1);
        }
    }
    UpdateGraph(g_s2e_state, Rx, 0);
}

void NLPPeripheralModel::UpdateFlag(uint32_t phaddr) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, g_s2e_state);
    RegMap state_map = plgState->get_state_map();
    if (rw_count > 1) {
        timer += 1;
        //getDebugStream() << "start UpdateFlag" << rw_count << " " << timer << " " << allFlags.size() << "\n";

        int _idx = ta_numbers;
        FlagList allFlags;
        for (auto loc : Flags_range) {
            auto range = loc.first;
            auto flags = loc.second;
            if (phaddr >= range.first && phaddr <= range.second) {
                allFlags = flags;
                break;
            }
            _idx += flags.size();
        }
        if (phaddr == 0) {
            int total_sz = flags_numbers;
            _idx = ta_numbers+1;
            allFlags.reserve(total_sz); // preallocate memory
            for (auto loc : Flags_range) {
                allFlags.insert(allFlags.end(), loc.second.begin(), loc.second.end());
            }
        }

        for (auto c : allFlags) {
            ++_idx;
            if (c.a.type == "O") {
                getDebugStream() << "old Flag" << state_map[c.a.phaddr].cur_value << " bits " << c.a.bits[0]
                                 << "\n";
                int tmp = c.value[std::rand() % c.value.size()];
                auto old_value = state_map[c.a.phaddr].cur_value;
                set_reg_value(state_map, c.a, tmp);
                if (state_map[c.a.phaddr].cur_value == old_value)
                    continue;
                statistics[_idx] += 1;
                // set_reg_value(state_map, c.a, c.value);
                //getWarningsStream() <<_idx<< " Flag " << hexval(c.a.phaddr) <<" bit "<<c.a.bits[0]<< " value " << tmp << " size " << c.value.size()
                //                 << " " << std::rand() << "\n";
                // getDebugStream() << "Flag "<< hexval(c.a.phaddr)<<" value "<<c.value << "\n";
            } else {
                auto old_value = get_reg_value(state_map, c.a);
                uint32_t tmp = 0;
                tmp = (old_value << 1) + 1;
                if (tmp > c.value[0])
                    tmp = 0;
                statistics[_idx] += 1;
                //if (c.freq > 0) {
                //    tmp = (old_value << 1) + 1;
                //    if (tmp > c.value[0])
                //        tmp = 0;
                //} else {
                //    tmp = old_value >> 1;
                //    if (tmp < c.value[0])
                //        tmp = c.value[0];
                //}
                set_reg_value(state_map, c.a, tmp);
            }

            plgState->insert_reg_map(c.a.phaddr, state_map[c.a.phaddr]);
            auto tmp = plgState->get_state_map();
            getDebugStream() << "new Flag" << tmp[c.a.phaddr].cur_value << "\n";
        }
    }
}

/*void NLPPeripheralModel::onForceIRQCheck(S2EExecutionState *state, uint32_t pc, uint64_t re_tb_num) {*/
// DECLARE_PLUGINSTATE(NLPPeripheralModelState, g_s2e_state);
// getDebugStream() << "Force IRQ Check "<< hexval(re_tb_num) << "\n";
// for (auto phaddr: data_register) {
// if (disable_init_dr_value_flag[phaddr] != 1) {
// getWarningsStream() << " write init dr value 0xA! phaddr = "<< hexval(phaddr) << "\n";
// plgState->hardware_write_to_receive_buffer(phaddr, 0xA, 4);
//}
//}
// UpdateGraph(state, Write, 0);
/*}*/

bool NLPPeripheralModel::readNLPModelfromFile(S2EExecutionState *state, std::string fileName) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    std::ifstream fNLP;
    std::string line;
    fNLP.open(fileName, std::ios::in);
    if (!fNLP) {
        getWarningsStream() << "Could not open cache nlp file: " << fileName << "\n";
        return false;
    }

    std::string peripheralcache;
    while (getline(fNLP, peripheralcache)) {
        if (peripheralcache == "==")
            break;
        PeripheralReg reg;
        if (getMemo(peripheralcache, reg)) {
            if ((reg.type == "R" || reg.type == "T") && std::find(data_register.begin(), data_register.end(), reg.phaddr) == data_register.end()) {
                data_register.push_back(reg.phaddr);
                disable_init_dr_value_flag[reg.phaddr] = 0;
            }
            plgState->insert_reg_map(reg.phaddr, reg);
        } else {
            return false;
        }
    }

    uint32_t start = 0xFFFFFFFF, end = 0;
    TAMap allTAs;
    int _idx = 0;
    while (getline(fNLP, peripheralcache)) {
        if (peripheralcache == "==")
            break;
        if (peripheralcache == "--") {
            TA_range[std::make_pair(start, end)] = allTAs;
            allTAs.clear();
            start = 0xFFFFFFFF;
            end = 0;
            continue;
        }
        EquList trigger;
        EquList action;
        if (getTApairs(peripheralcache, trigger, action)) {
            allTAs.push_back(std::make_pair(trigger, action));
            for (auto equ : trigger) {
                if (equ.eq == "*")
                    continue;
                start = std::min(start, equ.a1.phaddr);
                end = std::max(end, equ.a1.phaddr);
            }
            for (auto equ : action) {
                start = std::min(start, equ.a1.phaddr);
                end = std::max(end, equ.a1.phaddr);
            }
            statistics[++_idx] = 0;
        } else {
            return false;
        }
    }

    ta_numbers = _idx;
    start = 0xFFFFFFFF;
    end = 0;
    FlagList allFlags;
    while (getline(fNLP, peripheralcache)) {
        if (peripheralcache == "--") {
            Flags_range.push_back(std::make_pair(std::make_pair(start, end), allFlags));
            allFlags.clear();
            start = 0xFFFFFFFF;
            end = 0;
            continue;
        }
        Flag count;
        if (extractFlag(peripheralcache, count)) {
            allFlags.push_back(count);
            for (auto equ : allFlags) {
                start = std::min(start, equ.a.phaddr);
                end = std::max(end, equ.a.phaddr);
            }
            statistics[++_idx] = 0;
        } else {
            return false;
        }
    }
    flags_numbers = _idx - ta_numbers;
    return true;
}

void NLPPeripheralModel::SplitString(const std::string &s, std::vector<std::string> &v, const std::string &c) {
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

void NLPPeripheralModel::SplitStringToInt(const std::string &s, std::vector<int> &v, const std::string &c, int dtype) {
    std::string::size_type pos1, pos2;
    getDebugStream() << s << "\n";
    pos2 = s.find(c);
    pos1 = 0;
    while (std::string::npos != pos2) {
        v.push_back(std::strtol(s.substr(pos1, pos2 - pos1).c_str(), NULL, dtype));
        getDebugStream() << s.substr(pos1, pos2 - pos1) << " "
                         << std::strtol(s.substr(pos1, pos2 - pos1).c_str(), NULL, dtype) << "\n";
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length()) {
        v.push_back(std::strtol(s.substr(pos1).c_str(), NULL, dtype));
        getDebugStream() << s.substr(pos1) << " " << std::strtol(s.substr(pos1).c_str(), NULL, dtype) << "\n";
    }
}

bool NLPPeripheralModel::getMemo(std::string peripheralcache, PeripheralReg &reg) {
    boost::smatch what;
    getDebugStream() << peripheralcache << "\n";
    if (!boost::regex_match(peripheralcache, what, MemoRegEx)) {
        getWarningsStream() << "getMemo match false\n";
        exit(0);
        return false;
    }

    if (what.size() != 2) {
        getWarningsStream() << "getMemo wrong size = " << what.size() << "\n";
        exit(0);
        return false;
    }

    std::vector<std::string> v;
    SplitString(what[1], v, "_");
    reg.type = v[0];
    reg.phaddr = std::stoull(v[1].c_str(), NULL, 16);
    reg.reset = std::stoull(v[2].c_str(), NULL, 16);
    reg.cur_value = reg.reset;
    reg.t_size = 0;
    reg.r_size = 0;
    reg.t_value = 0;
    //reg.r_value = new std::queue<uint8_t>();
    getDebugStream() << "type = " << reg.type << " phaddr = " << reg.phaddr << " reset value = " << reg.reset << "\n";
    return true;
}

bool NLPPeripheralModel::getTApairs(std::string peripheralcache, EquList &trigger, EquList &action) {
    std::vector<std::string> v;
    SplitString(peripheralcache, v, ":");
    std::string trigger_str = v[0];
    std::string action_str = v[1];
    bool trigger_rel = true, action_rel = true;
    if (trigger_str.find('|', 0) != std::string::npos) {
        trigger_rel = false;
    }

    if (action_str.find('|', 0) != std::string::npos) {
        action_rel = false;
    }
    getDebugStream() << " trigger = " << trigger_str << " action = " << action_str << "\n";

    bool res = extractEqu(trigger_str, trigger, trigger_rel) && extractEqu(action_str, action, action_rel);
    if (v.size() == 3) {
        action.back().interrupt = std::stoi(v[2].c_str(), NULL, 10);
        getDebugStream() << " trigger = " << trigger_str << " action = " << action_str
                         << " interrupt = " << action.back().interrupt << "\n";
    }
    return res;
}

bool NLPPeripheralModel::extractEqu(std::string peripheralcache, EquList &vec, bool rel) {
    boost::smatch what;
    getDebugStream() << peripheralcache << "\n";

    while (boost::regex_search(peripheralcache, what, TARegEx)) {
        std::string equ_str = what[0];
        std::vector<std::string> v;
        getDebugStream() << equ_str << "\n";
        SplitString(equ_str, v, ",");
        Equation equ;
        equ.rel = rel;
        equ.interrupt = -1;
        if (v[0] == "*") {
            equ.a1.type = v[0];
            equ.a1.bits = {-1};
            equ.a2.bits = {-1};
            equ.eq = "*";
            equ.type_a2 = "*";
        } else {
            equ.a1.type = v[0];
            equ.a1.phaddr = std::stoull(v[1].c_str(), NULL, 16);
            if (v[2] == "*")
                equ.a1.bits = {-1};
            else {
                SplitStringToInt(v[2], equ.a1.bits, "/", 10);
            }
            equ.eq = v[3];
            if (v[4] == "O" || v[4] == "C") {
                equ.type_a2 = "F";
                equ.value = 0;
                equ.a2.type = v[4];
                equ.a2.phaddr = std::stoull(v[5].c_str(), NULL, 16);
                if (v[6] == "*")
                    equ.a2.bits = {-1};
                else {
                    SplitStringToInt(v[6], equ.a2.bits, "/", 10);
                }
            } else if (v[4][0] != '*') {
                equ.type_a2 = "V";
                equ.value = std::stoull(v[4].c_str(), NULL, 2);
            } else {
                equ.value = 0;
                if (v[4] == "*") {
                    equ.type_a2 = v[4];
                } else {
                    equ.type_a2 = v[4][1];
                    equ.a2.type = v[4][1];
                    equ.a2.phaddr = std::stoull(v[4].substr(2, v[4].size() - 2).c_str(), NULL, 16);
                }
            }
        }
        getDebugStream() << "equ type = " << equ.a1.type << " equ phaddr = " << equ.a1.phaddr
                         << " equ bits = " << equ.a1.bits[0] << " equ = " << equ.eq << " type_a2 = " << equ.type_a2
                         << " value = " << equ.value << "\n";
        vec.push_back(equ);
        peripheralcache = what.suffix();
    }
    return true;
}

bool NLPPeripheralModel::extractFlag(std::string peripheralcache, Flag &flag) {
    boost::smatch what;
    getDebugStream() << peripheralcache << "\n";
    if (!boost::regex_match(peripheralcache, what, FlagEx)) {
        getWarningsStream() << "extractFlag match false" << peripheralcache << "\n";
        exit(0);
        return false;
    }
    if (what.size() != 2) {
        getWarningsStream() << "extractFlag wrong size = " << what.size() << "\n";
        exit(0);
        return false;
    }
    std::vector<std::string> v;
    SplitString(what[1], v, ",");
    flag.a.type = v[0];
    flag.a.phaddr = std::stoull(v[1].c_str(), NULL, 16);
    if (v[2] == "*")
        flag.a.bits = {-1};
    else {
        SplitStringToInt(v[2], flag.a.bits, "/", 10);
    }
    flag.freq = std::stoull(v[3].c_str(), NULL, 10);
    // flag.value = std::stoi(v[4].c_str(), NULL, 16);
    SplitStringToInt(v[4], flag.value, "/", 16);
    getDebugStream() << "extractFlag  " << hexval(flag.a.phaddr) << " " << flag.a.bits[0] << "\n";
    return true;
}

bool NLPPeripheralModel::compare(uint32_t a1, std::string sym, uint32_t a2) {
    // 1:= ; 2:>; 3:<; 4:>=; 5:<=
    if (sym == "*")
        return false;
    if (sym == "=")
        return a1 == a2;
    if (sym == ">")
        return a1 > a2;
    if (sym == "<")
        return a1 < a2;
    if (sym == ">=")
        return a1 >= a2;
    if (sym == "<=")
        return a1 <= a2;
    return false;
}

void NLPPeripheralModel::UpdateGraph(S2EExecutionState *state, RWType type, uint32_t phaddr) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    RegMap state_map = plgState->get_state_map();
    TAMap allTAs;
    int _idx = 0;
    for (auto loc : TA_range) {
        auto range = loc.first;
        auto TA = loc.second;
        if (phaddr >= range.first && phaddr <= range.second) {
            allTAs = TA;
            break;
        }
        _idx += TA.size();
    }
    if (phaddr == 0) {
        int total_sz = ta_numbers;
        _idx = 0;

        allTAs.reserve(total_sz); // preallocate memory
        for (auto loc : TA_range) {
            allTAs.insert(allTAs.end(), loc.second.begin(), loc.second.end());
        }
    }
    std::map<std::pair<uint32_t, int>, uint32_t> prev_action;
    for (auto ta : allTAs) {
        _idx += 1;
        EquList trigger = ta.first;
        bool rel;
        std::vector<bool> trigger_res;
        for (auto equ : trigger) {
            rel = equ.rel;
            if (equ.a1.type == "*") {
                trigger_res.push_back(true);
            } else if (equ.type_a2 == "*" && equ.a1.type == "R") {
                if (type == Read && std::find(data_register.begin(), data_register.end(), phaddr) != data_register.end()) {
                    trigger_res.push_back(true);
                } else
                    trigger_res.push_back(false);
            } else if (equ.type_a2 == "*" && equ.a1.type == "T") {
                if (type == Write && std::find(data_register.begin(), data_register.end(), phaddr) != data_register.end()) {
                    trigger_res.push_back(true);
                } else
                    trigger_res.push_back(false);
            } else if (equ.a1.type == "F" && (type != Write || phaddr != equ.a1.phaddr)) {
                trigger_res.push_back(false);
            } else {
                uint32_t a1, a2;
                a1 = get_reg_value(state_map, equ.a1);
                if (equ.type_a2 == "T") {
                    a2 = state_map[equ.a2.phaddr].t_size;
                } else if (equ.type_a2 == "R") {
                    a2 = state_map[equ.a2.phaddr].r_size;
                } else if (equ.type_a2 == "F") {
                    a2 = get_reg_value(state_map, equ.a2);
                } else if (equ.type_a2 == "V") {
                    a2 = equ.value;
                } else {
                    a2 = 0;
                    getDebugStream() << "ERROR " << a1 << " eq " << equ.eq << " \n";
                }
                if (equ.a1.type == "F") {
                    if (type == Write && a1 == a2 && phaddr == equ.a1.phaddr) {
                        getDebugStream() << "Write 1 to " << hexval(equ.a1.phaddr) << " eq " << equ.eq << " \n";
                        trigger_res.push_back(true);
                    } else
                        trigger_res.push_back(false);

                } else {
                    getDebugStream() << "intermediate trigger a1 " << hexval(equ.a1.phaddr)
                                     << " bit: " << equ.a1.bits[0] << " a1 " << a1 << " eq " << equ.eq << " a2 "
                                     << equ.value << " \n";
                    trigger_res.push_back(compare(a1, equ.eq, a2));
                }
            }
        }
        bool check = rel;
        if (rel == true) {
            for (bool idx : trigger_res) {
                // getDebugStream() << "idx = " << idx <<"\n";
                if (idx == false) {
                    check = false;
                    break;
                }
            }
        } else {
            for (auto idx : trigger_res) {
                if (idx == true) {
                    check = true;
                    break;
                }
            }
        }
        if (!check)
            continue;
        statistics[_idx] += 1;
        for (auto equ : trigger) {
            //auto _tmp = std::make_pair(equ.a1.phaddr, equ.a1.bits[0]);
            /*if (prev_action.find(_tmp) != prev_action.end()) {*/
            //chain_freq[{prev_action[_tmp], _idx}] += 1;
            //getDebugStream() << "chain a1 " << prev_action[_tmp] << " a2 " << _idx << " \n";
            /*}*/
            getDebugStream() << "trigger a1 " << hexval(equ.a1.phaddr) << " bit: " << equ.a1.bits[0] << " eq " << equ.eq
                             << " a2 " << equ.value << "statistics:" << _idx << " " << statistics[_idx] << " \n";
        }

        EquList action = ta.second;
        for (auto equ : action) {
            uint32_t a2;
            if (equ.type_a2 == "T") {
                a2 = state_map[equ.a2.phaddr].cur_value;
            } else if (equ.type_a2 == "R") {
                a2 = state_map[equ.a2.phaddr].cur_value;
            } else if (equ.type_a2 == "F") {
                a2 = get_reg_value(state_map, equ.a2);
                getDebugStream() << "get by address, phaddr" << hexval(equ.a2.phaddr) << " " << a2 << "\n";
            } else {
                a2 = equ.value;
            }

            auto _tmp = std::make_pair(equ.a1.phaddr, equ.a1.bits[0]);
            prev_action[_tmp] = _idx;

            if (equ.a1.type == "R") {
                state_map[equ.a1.phaddr].cur_value = a2;
                plgState->insert_reg_map(equ.a1.phaddr, state_map[equ.a1.phaddr]);
            } else if (equ.a1.type == "T") {
                state_map[equ.a1.phaddr].cur_value = a2;
                plgState->insert_reg_map(equ.a1.phaddr, state_map[equ.a1.phaddr]);
            } else {
                set_reg_value(state_map, equ.a1, a2);
                if (type == Read) {
                    getDebugStream() << "Read Action: phaddr =  " << hexval(equ.a1.phaddr)
                                     << " updated bit = " << equ.a1.bits[0]
                                     << " value = " << hexval(state_map[equ.a1.phaddr].cur_value) << " a2 = " << a2
                                     << "\n";
                } else {
                    getDebugStream() << "Write Action: phaddr =  " << hexval(equ.a1.phaddr)
                                     << " updated bit = " << equ.a1.bits[0]
                                     << " value = " << hexval(state_map[equ.a1.phaddr].cur_value) << " a2 = " << a2
                                     << "\n";
                }
                // update to state
                plgState->insert_reg_map(equ.a1.phaddr, state_map[equ.a1.phaddr]);
            }
            getDebugStream() << "equ.interrupt = " << equ.interrupt
                             << " exit_inter = " << plgState->get_exit_interrupt(equ.interrupt) << "\n";

            if (equ.interrupt == -1)
                continue;
            //no fuzzing mode, skip if the irq is triggered by writing to rx & interrupt_freq is more than once
            //if (!enable_fuzzing) {
            /*if (plgState->get_irq_freq(equ.interrupt) > 2) {*/
            //getWarningsStream() << " 0 DATA IRQ Action trigger interrupt equ.interrupt = " << plgState->get_irq_freq(equ.interrupt) << "\n";
            //continue;
            /*}*/
            //} else {
            /*if (plgState->get_irq_freq(equ.interrupt) > 10) {*/
            //getInfoStream() << " only trigger at most ten times DATA IRQ Action interrupt in fuzzing mode equ.interrupt = " << equ.interrupt << "\n";
            //continue;
            /*}*/
            //}
            //no fuzzing mode, skip if the irq is triggered by the phaddr that is in nlp_mmio
            if (!enable_fuzzing) {
                uint32_t tmp = equ.a1.phaddr;
                bool check = false;
                for (auto nlpph : nlp_mmio) {
                    if (tmp >= nlpph.first && tmp <= nlpph.second) {
                        check = true;
                        break;
                    }
                }
                if (!check)
                    continue;
            }

            //all mode, if the irq is not triggered, emit the irq. if successful emited, record the irq and wait for exiting
            if (!plgState->get_exit_interrupt(equ.interrupt)) {
                //interrupt_freq[equ.interrupt] += 1;
                //getWarningsStream() << "IRQ Action trigger interrupt equ.interrupt = " << equ.interrupt << "\n";

                bool irq_triggered = false;
                onExternalInterruptEvent.emit(state, equ.interrupt, &irq_triggered);
                getInfoStream() << " DATA IRQ Action trigger interrupt equ.interrupt = " << plgState->get_irq_freq(equ.interrupt) << " exit_interrupt = " << plgState->get_exit_interrupt(equ.interrupt) << " irq = " << equ.interrupt << "\n";
                if (irq_triggered) {
                    plgState->inc_irq_freq(equ.interrupt);
                    plgState->set_exit_interrupt(equ.interrupt, true);
                }
            }
            //else if (enable_fuzzing) {
            //    plgState->set_exit_interrupt(equ.interrupt, true);
            //}
        }
    }
}

void NLPPeripheralModel::onStatistics() {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, g_s2e_state);
    getInfoStream() << "write NLP files\n";
    std::string NLPstafileName = s2e()->getOutputDirectory() + "/" + "NLPStatistics.dat";
    std::ofstream fPHNLP;

    fPHNLP.open(NLPstafileName, std::ios::out | std::ios::trunc);

    fPHNLP << "nlp write: " << write_numbers << " nlp read: " << read_numbers << " ta_num: " << ta_numbers << "\n";

    uint32_t sum_ta = 0, sum_flag = 0, unique_ta = 0, unique_flag = 0, uncertain_flag = 0, unique_uncertain_flag = 0;
    int idx = 0;
    for (auto loc : TA_range) {
        for (auto ta : loc.second) {
            idx += 1;
            if (statistics[idx] == 0) continue;
            fPHNLP << "TA : " << idx << " " << hexval(ta.first[0].a1.phaddr) << " " << ta.first[0].a1.bits[0] << " " << ta.first[0].eq << " ";
            //fPHNLP << "TA : "<<idx<<" "<< hexval(ta.first[0].a1.phaddr) <<" "<<ta.first[0].a1.bits[0]<<" "<<ta.first[0].eq<<" "<<hexval(ta.first[0].a2.phaddr)<<" "<<ta.first[0].a2.bits[0]<<" "<<ta.first[0].value;
            if (ta.first.size() > 1) {
                fPHNLP << " " << hexval(ta.first.back().a1.phaddr) << " " << ta.first.back().a1.bits[0];
            }
            fPHNLP << "\n";
        }
    }
    int _tmp1 = 0, _tmp2 = 0;
    for (auto ta : statistics) {
        if (ta.first <= ta_numbers) {
            sum_ta += ta.second;
            unique_ta += ta.second > 0;
            fPHNLP << "TA id: " << ta.first << " cnt: " << ta.second << "\n";
        } else {
	    if (_tmp2 >= Flags_range[_tmp1].second.size()) {
	        _tmp1++;
	        _tmp2 = 0;
	    }
	    auto tmp = Flags_range[_tmp1].second[_tmp2].a.phaddr;
            bool uncertain = Flags_range[_tmp1].second[_tmp2].value.size() > 1;
	    if (uncertain) {
                unique_uncertain_flag += ta.second > 0;
                uncertain_flag += ta.second;
	    }
            sum_flag += ta.second;
            unique_flag += ta.second > 0;
            fPHNLP << "Flag uncertain? " << uncertain << " id: " << ta.first << " reg: " << hexval(tmp) << " bit: " << Flags_range[_tmp1].second[_tmp2].a.bits[0] << " cnt: " << ta.second << "\n";
	    _tmp2++;
        }
    }
    for (auto interrupt : plgState->get_irqs_freq()) {
        fPHNLP << "interrupt id:" << interrupt.first << " freq: " << interrupt.second << "\n";
    }
    for (auto chain : chain_freq) {
        fPHNLP << "chain id1:" << chain.first.first << " id2: " << chain.first.second << " freq: " << chain.second << "\n";
    }
    fPHNLP << "ta: " << sum_ta << "\\" << unique_ta << " flag: " << sum_flag << "\\" << unique_flag << " uncertain flag: " << uncertain_flag << "\\" << unique_uncertain_flag << "\n";
    fPHNLP.close();
}

std::pair<uint32_t, uint32_t> NLPPeripheralModel::AddressCorrection(S2EExecutionState *state, uint32_t phaddr) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    RegMap state_map = plgState->get_state_map();
    if (state_map.find(phaddr) != state_map.end())
        return {phaddr, 0};
    auto uppper_node = state_map.upper_bound(phaddr);
    // uint32_t new_phaddr = phaddr & 0xFFFFFFFC;
    if (uppper_node != state_map.begin())
        uppper_node--;
    uint32_t new_phaddr = uppper_node->first;
    uint32_t offset = (phaddr - new_phaddr) * 8;
    if (offset != 0)
        getInfoStream() << "correction " << hexval(phaddr) << " new correction " << hexval(new_phaddr) << " \n";
    return {new_phaddr, offset};
}

void NLPPeripheralModel::onPeripheralRead(S2EExecutionState *state, SymbolicHardwareAccessType type, uint32_t phaddr,
                                          unsigned size, uint32_t *NLPsymbolicvalue, bool *flag) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    rw_count++;
    if (rw_count == 1) {
        readNLPModelfromFile(state, NLPfileName);
        if (!enable_fuzzing) {
            // Write a value to DR
            getWarningsStream() << " write init dr value 0xA!  \n";
            for (auto _phaddr : data_register) {
                std::queue<uint8_t> tmp;
                tmp.push(0xA);
                plgState->hardware_write_to_receive_buffer(_phaddr, tmp, 1);
            }
            UpdateGraph(g_s2e_state, Rx, 0);
        }
    }
    read_numbers += 1;
    UpdateFlag(phaddr);
    auto correction = AddressCorrection(state, phaddr);
    phaddr = correction.first;
    *flag = false;
    if (std::find(data_register.begin(), data_register.end(), phaddr) != data_register.end()) {
        *flag = true;
        disable_init_dr_value_flag[phaddr] = 1;
        std::vector<unsigned char> data;
        for (uint32_t i = 0; i < size; i++) {
            data.push_back(plgState->get_dr_value(phaddr, 1));
        }
        if (size == 4) {
            *NLPsymbolicvalue = data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
        } else if (size == 2) {
            *NLPsymbolicvalue = data[0] | ((uint32_t)data[1] << 8);
        } else {
            *NLPsymbolicvalue = data[0];
        }
        getInfoStream() << "Read data register " << hexval(phaddr) << " width " << size << " value " << hexval(*NLPsymbolicvalue) << "\n";
    } else {
        *NLPsymbolicvalue = plgState->get_ph_value(phaddr);
    }
    UpdateGraph(g_s2e_state, Read, phaddr);
    getDebugStream() << "correction " << hexval(phaddr) << " value " << *NLPsymbolicvalue << " \n";
    if (correction.second != 0) {
        *NLPsymbolicvalue = *NLPsymbolicvalue >> correction.second;
    }
    getDebugStream() << "Read phaddr " << hexval(phaddr) << " value " << *NLPsymbolicvalue << " \n";
}

void NLPPeripheralModel::onPeripheralWrite(S2EExecutionState *state, SymbolicHardwareAccessType type, uint32_t phaddr,
                                           uint32_t writeconcretevalue) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    rw_count++;
    if (rw_count == 1) {
        readNLPModelfromFile(state, NLPfileName);
        if (!enable_fuzzing) {
            // Write a value to DR
            getWarningsStream() << " write init dr value 0xA! \n";
            for (auto _phaddr : data_register) {
                std::queue<uint8_t> tmp;
                tmp.push(0xA);
                plgState->hardware_write_to_receive_buffer(_phaddr, tmp, 1);
            }
            UpdateGraph(g_s2e_state, Rx, 0);
        }
    }
    write_numbers += 1;
    auto correction = AddressCorrection(state, phaddr);
    phaddr = correction.first;
    if (correction.second != 0) {
        writeconcretevalue = writeconcretevalue << correction.second;
    }
    if (std::find(data_register.begin(), data_register.end(), phaddr) != data_register.end()) {
        plgState->write_dr_value(phaddr, writeconcretevalue, 32);
        getDebugStream() << "Write to data register " << phaddr << " " << hexval(phaddr)
                         << " value: " << writeconcretevalue << " \n";
    } else {
        plgState->write_ph_value(phaddr, writeconcretevalue);
        getDebugStream() << "Write to phaddr " << hexval(phaddr) << " value: " << writeconcretevalue << " \n";
    }
    UpdateGraph(g_s2e_state, Write, phaddr);
}

void NLPPeripheralModel::onTranslateBlockStart(ExecutionSignal *signal, S2EExecutionState *state, TranslationBlock *tb,
                                               uint64_t pc) {
    signal->connect(sigc::mem_fun(*this, &NLPPeripheralModel::onForkPoints));
}

void NLPPeripheralModel::onForkPoints(S2EExecutionState *state, uint64_t pc) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    if (pc == fork_point) {
        init_dr_flag = true;
        plgState->inc_fork_count();
        if (!enable_fuzzing) {
            getWarningsStream() << "already go though Main Loop Point Count = " << plgState->get_fork_point_count() << "\n";
            getWarningsStream() << "===========unit test pass============\n";
            g_s2e->getCorePlugin()->onEngineShutdown.emit();
            // Flush here just in case ~S2E() is not called (e.g., if atexit()
            // shutdown handler was not called properly).
            g_s2e->flushOutputStreams();
            exit(0);
        }
    }
}

void NLPPeripheralModel::onTranslateBlockEnd(ExecutionSignal *signal, S2EExecutionState *state, TranslationBlock *tb,
                                             uint64_t pc, bool staticTarget, uint64_t staticTargetPc) {
    signal->connect(sigc::bind(sigc::mem_fun(*this, &NLPPeripheralModel::onBlockEnd), (unsigned)tb->se_tb_type));
}

void NLPPeripheralModel::onBlockEnd(S2EExecutionState *state, uint64_t cur_loc, unsigned source_type) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    if (init_dr_flag == true) {
        std::queue<uint8_t> return_value;
        uint32_t AFL_size = 0;
        // Write a value to DR
        for (uint32_t i = 0; i < data_register.size(); ++i) {
            if (i == 0) {
                onBufferInput.emit(state, data_register[i], &AFL_size, &return_value);
                plgState->hardware_write_to_receive_buffer(data_register[i], return_value, return_value.size());
            } else {
                plgState->hardware_write_to_receive_buffer(data_register[i], return_value, AFL_size);
            }
            getInfoStream() << "write to receiver buffer " << hexval(data_register[i])
                            << " return value: " << hexval(return_value.front()) << "\n";
        }
        UpdateGraph(state, Rx, 0);
        init_dr_flag = false;
    }
}
}
} // namespace s2e::plugins
