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
    RegMap state_map;
    TAMap allTAs;
    uint32_t data_register;
    //std::map<std::string, uint32_t> symbol_list = {
    //    {"*",0},{"=",1},{">":2},{"<",3},{">=",4},{"<=",5}
    //};
    //0:= ; 1:>; 2: <; 3: >=; 4: <=

    void UpdateGraph(RWType type, uint32_t phaddr) {
        for (auto ta: allTAs) {
            EquList trigger = ta.first;
            bool rel;
            std::vector<bool> trigger_res;
            for (auto equ: trigger) {
                rel = equ.rel;
                if (equ.type == "R") {
                    if (type == Read && phaddr == data_register)
                        trigger_res.push_back(true);
                    else
                        trigger_res.push_back(false);
                } else {
                    uint32_t a1, a2;
                    if (equ.bits == "*") {
                        a1 = state_map[equ.phaddr].cur_value;
                    } else {
                        uint32_t tmp = std::stoull(equ.bits, NULL, 10);
                        a1 = state_map[equ.phaddr].cur_value >> tmp & 1;
                    }
                    if (equ.type_a2 == "T") {
                        a2 = state_map[data_register].t_size;
                    } else if(equ.type_a2 == "R") {
                        a2 = state_map[data_register].r_size;
                    }
                    else a2 = equ.value;
                    trigger_res.push_back(compare(a1, equ.eq, a2));
                }
            }
            bool check = rel;
            if (rel == true){
                for (auto idx: trigger_res) {
                    if (idx == false) {
                        check = false;
                        break;
                    }
                }
            } else {
                for (auto idx: trigger_res) {
                    if (idx == true) {
                        check = true;
                        break;
                    }
                }
            }
            if (!check) continue;

            EquList action = ta.second;
            for (auto equ: action) {
                uint32_t a2;
                if (equ.type_a2 == "T") {
                    a2 = state_map[data_register].t_size;
                } else if (equ.type_a2 == "R") {
                    a2 = state_map[data_register].r_size;
                } else {
                    a2 = equ.value;
                }

                if (equ.type == "R") {
                    state_map[equ.phaddr].r_size = a2;
                } else if (equ.type == "T") {
                    state_map[equ.phaddr].t_size = a2;
                } else {
                    uint32_t tmp = std::stoull(equ.bits, NULL, 10);
                    if (a2 == 1)
                        state_map[equ.phaddr].cur_value |= (1 << tmp);
                    else
                        state_map[equ.phaddr].cur_value &= ~(1 << tmp);
                }
            }
        }
    }


    bool compare(uint32_t a1, std::string sym, uint32_t a2) {
        //1:= ; 2:>; 3:<; 4:>=; 5:<=
        if (sym == "*")
            return false;
        if (sym == "=")
            return a1 == a2;
        if (sym ==  ">")
            return a1 > a2;
        if (sym ==  "<")
            return a1 < a2;
        if (sym ==  ">=")
            return a1 >= a2;
        if (sym ==  "<=")
            return a1 <= a2;
        return false;
    }

public:
    NLPPeripheralModelState() {
    }

    virtual ~NLPPeripheralModelState() {
    }

    void initialize_graph(RegMap m, TAMap ta, uint32_t dr) {
	printf("address %u regs length %lu ta length %lu\n",dr, m.size(), ta.size());
        //state_map = m;
	printf("ori map:\n");
	for (auto iter: m) {
                printf("address: %u,  reg cur_value: %u\n",iter.first,  iter.second.cur_value);
		PeripheralReg reg;
		reg.type = iter.second.type;
		reg.phaddr  = iter.second.phaddr;
		reg.reset = iter.second.reset;
		reg.cur_value = iter.second.cur_value;
		printf("new reg phaddr%u, value %u\n", reg.phaddr, reg.cur_value);
		state_map[reg.phaddr] = reg;
        }
	//std::copy(m.begin(),m.end(),std::inserter(state_map,state_map.begin()));
	//state_map.insert(m.begin(), m.end());
	printf("test copy RegMap: \n");
	for (auto iter: state_map) {
		printf("address: %u,  reg phaddr: %u\n",iter.first,  iter.second.phaddr);
	}
	printf("test copy TAMap: \n");
	std::copy(ta.begin(),ta.end(),std::inserter(allTAs, allTAs.begin()));
        //allTAs = ta;
	for (auto iter: allTAs) {
		printf("first trigger type: %u, first action type: %u\n", iter.first[0].phaddr, iter.second[0].phaddr);
	}
        data_register = dr;
    }

    static PluginState *factory(Plugin *, S2EExecutionState *) {
        return new NLPPeripheralModelState();
    }

    NLPPeripheralModelState *clone() const {
        return new NLPPeripheralModelState(*this);
    }

    void write_ph_value(uint32_t phaddr, uint32_t value) {
        if (data_register == phaddr) {
            state_map[phaddr].t_value = value;
            state_map[phaddr].t_size = 1;
            return;
        }
        state_map[phaddr].cur_value = value;
        UpdateGraph(Write, phaddr);
    }

    uint32_t get_ph_value(uint32_t phaddr) {
        if (data_register == phaddr) {
            return state_map[phaddr].r_value;
        }
        UpdateGraph(Read, phaddr);
        return  state_map[phaddr].cur_value;
    }

    void hardware_write_to_receive_buffer(uint32_t value) {
        state_map[data_register].r_size = 1;
        state_map[data_register].r_value = value;
        UpdateGraph(Write, data_register);
    }
};

void NLPPeripheralModel::initialize() {
    std::string NLPfileName = s2e()->getConfig()->getString(getConfigKey() + ".NLPfileName", "all.txt");
    getDebugStream() << "NLP firmware name is " << NLPfileName << "\n";
    readNLPModelfromFile(g_s2e_state, NLPfileName);
    hw::SymbolicPeripherals *symbolicPeripheralConnection = s2e()->getPlugin<hw::SymbolicPeripherals>();
    symbolicPeripheralConnection->onSymbolicNLPRegisterReadEvent.connect(
                            sigc::mem_fun(*this, &NLPPeripheralModel::onPeripheralRead));
    symbolicPeripheralConnection->onSymbolicNLPRegisterWriteEvent.connect(
                            sigc::mem_fun(*this, &NLPPeripheralModel::onPeripheralWrite));
    s2e()->getCorePlugin()->onTimer.connect(sigc::mem_fun(*this, &NLPPeripheralModel::onTimer));
    srand(0);
}

void NLPPeripheralModel::onTimer() {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, g_s2e_state);
    uint32_t rand_org = rand();
    uint32_t rand_value = int(((rand_org % 0x7ffe) * 1.0 / 0x7fff) * 4294967295);
    getDebugStream() << " input rand value = " << rand_value << "\n";
    plgState->hardware_write_to_receive_buffer(rand_value);
}

bool NLPPeripheralModel::readNLPModelfromFile(S2EExecutionState *state, std::string fileName) {
    std::ifstream fNLP;
    std::string line;
    fNLP.open(fileName, std::ios::in);
    if (!fNLP) {
        getWarningsStream() << "Could not open cache nlp file: "
                            << fileName << "\n";
        return false;
    }

    RegMap peripheral_regs_value_map;
    TAMap allTAs;
    uint32_t data_register;

    std::string peripheralcache;
    while (getline(fNLP, peripheralcache)) {
        if (peripheralcache == "==") break;
        PeripheralReg reg;
        if (getMemo(peripheralcache, reg)) {
            peripheral_regs_value_map[reg.phaddr] = reg;
            if (reg.type == "R")
                data_register = reg.phaddr;
        }
    }

    while (getline(fNLP, peripheralcache)) {
        EquList trigger;
        EquList action;
        if (getTApairs(peripheralcache, trigger, action)) {
            allTAs.push_back(make_pair(trigger, action));
        }
    }

    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    getDebugStream() << "start to initialize graph\n";
    plgState->initialize_graph(peripheral_regs_value_map, allTAs, data_register);
    getDebugStream() << "end initialize graph\n";
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

bool NLPPeripheralModel::getMemo(std::string peripheralcache, PeripheralReg &reg) {
    boost::smatch what;
    getDebugStream() << peripheralcache << "\n";
    if (!boost::regex_match(peripheralcache, what, MemoRegEx)) {
        getWarningsStream() << "getMemo match false\n";
        exit(0);
        return false;
    }

    if (what.size() != 2) {
        getWarningsStream() << "wrong size = " << what.size() << "\n";
        exit(0);
        return false;
    }

    std::vector<std::string> v;
    SplitString(what[1], v, "_");
    reg.type = v[0];
    reg.phaddr = std::stoull(v[1].c_str(), NULL, 10);
    reg.reset = std::stoull(v[2].c_str(), NULL, 10);
    getDebugStream() << "type = " << reg.type << " phaddr = " << reg.phaddr << " reset value = " << reg.reset << "\n";
    reg.cur_value = reg.reset;
    reg.t_size = 0;
    reg.r_size = 0;
    reg.t_value = 0;
    reg.r_value = 0;
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

    return extractEqu(trigger_str, trigger, trigger_rel) && extractEqu(action_str, action, action_rel);
}

bool NLPPeripheralModel::extractEqu(std::string peripheralcache, EquList &vec, bool rel){
    boost::smatch what;
    getDebugStream() << peripheralcache << "\n";

    while (boost::regex_search(peripheralcache, what, TARegEx)) {
        std::string equ_str = what[0];
        std::vector<std::string> v;
        getDebugStream() << equ_str << "\n";
        SplitString(equ_str, v, ",");
        getDebugStream() << v[0] << " " << v[1] << " " << v[2] << " " << v[3] << " " << v[4] << "\n";
        Equation equ;
        equ.rel = rel;
        if (v[0] == "*") {
            equ.type = v[0];
        } else {
            equ.type = v[0];
            equ.phaddr = std::stoull(v[1].c_str(), NULL, 10);
            equ.bits = v[2];
            equ.eq = v[3];
            if (v[4][0] != '*') {
                equ.type_a2 = "V";
                equ.value = std::stoull(v[4].c_str(), NULL, 10);
            } else {
                equ.value = 0;
                equ.type_a2 = v[4][1];
            }
        }
        getDebugStream() << "equ type = " << equ.type << " equ phaddr = " << equ.phaddr
                        << " equ bits = " << equ.bits << " equ = " << equ.eq << " type_a2 = " << equ.type_a2 << " value = " << equ.value << "\n";
        vec.push_back(equ);
	peripheralcache = what.suffix();
    }
    return true;
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
