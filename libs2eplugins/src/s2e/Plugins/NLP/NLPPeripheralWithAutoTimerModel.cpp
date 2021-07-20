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
    std::map<int, bool> exit_interrupt;

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

    bool get_exit_interrupt(uint32_t num) {
	    return exit_interrupt[num];
    }

    void set_exit_interrupt(uint32_t num, bool cur) {
	    exit_interrupt[num] = cur;
    }

    RegMap get_state_map() {
        return state_map;
    }
    void insert_reg_map (uint32_t phaddr, PeripheralReg reg) {
        state_map[phaddr] = reg;
    }

    void write_ph_value(uint32_t phaddr, uint32_t value) {
        state_map[phaddr].cur_value = value;
        // UpdateGraph(Write, phaddr);
    }

    void write_dr_value(uint32_t phaddr, uint32_t value) {
        state_map[phaddr].t_value = value;
        state_map[phaddr].t_size = 0; // ignore
    }

    uint32_t get_ph_value(uint32_t phaddr) {
        // UpdateGraph(Read, phaddr);
        return  state_map[phaddr].cur_value;
    }

    uint32_t get_dr_value(uint32_t phaddr) {
        state_map[phaddr].r_size = 0;
        return state_map[phaddr].r_value;
    }

    /*
    void hardware_write_to_receive_buffer(uint32_t phaddr,uint32_t value) {
        state_map[phaddr].r_size = 1;
        state_map[phaddr].r_value = value;
    }

    void auto_countdown(std::vector<uint32_t> phaddr,uint32_t value) {
        for (auto p: phaddr) {
	    if (state_map[p].cur_value <= value) 
                state_map[p].cur_value = 0;
	    else
                state_map[p].cur_value -= value;
        }
    }
    */
};

void NLPPeripheralModel::initialize() {
    NLPfileName = s2e()->getConfig()->getString(getConfigKey() + ".NLPfileName", "all.txt");
    getDebugStream() << "NLP firmware name is " << NLPfileName << "\n";
    hw::SymbolicPeripherals *symbolicPeripheralConnection = s2e()->getPlugin<hw::SymbolicPeripherals>();
    symbolicPeripheralConnection->onSymbolicNLPRegisterReadEvent.connect(
                            sigc::mem_fun(*this, &NLPPeripheralModel::onPeripheralRead));
    symbolicPeripheralConnection->onSymbolicNLPRegisterWriteEvent.connect(
                            sigc::mem_fun(*this, &NLPPeripheralModel::onPeripheralWrite));
    //s2e()->getCorePlugin()->onTimer.connect(sigc::mem_fun(*this, &NLPPeripheralModel::onTimer));
    s2e()->getCorePlugin()->onTimer.connect(sigc::mem_fun(*this, &NLPPeripheralModel::CountDown));
    s2e()->getCorePlugin()->onExceptionExit.connect(
        sigc::mem_fun(*this, &NLPPeripheralModel::onExceptionExit));
    rw_count = 0;
    srand(0);
}

void NLPPeripheralModel::onExceptionExit(S2EExecutionState *state, uint32_t irq_no) {
	DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
	//interrupt vector+16
	plgState->set_exit_interrupt(irq_no+16, false);
        getDebugStream() << "EXIT Interrupt IRQ" << irq_no << "\n";
}

void NLPPeripheralModel::CountDown() {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, g_s2e_state);
    RegMap state_map = plgState->get_state_map();
    if (rw_count > 1) {
        uint32_t freq = 0x80000000;
        getDebugStream(g_s2e_state) << " countdown value = " << hexval(freq) << "\n";
	for (auto p: countdown_register) {
            getDebugStream(g_s2e_state) << "Interrupt reg: "<<hexval(p)<<" cur: "<<state_map[p].cur_value<<"\n";
        }
        plgState->auto_countdown(countdown_register, freq);
        UpdateGraph(g_s2e_state, Write, 0);
    }
}

void NLPPeripheralModel::onTimer() {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, g_s2e_state);
    if (rw_count > 1) {
        uint32_t rand_org = rand();
        uint32_t rand_value = int(((rand_org % 0x7ffe) * 1.0 / 0x7fff) * 4294967295);
        getDebugStream(g_s2e_state) << " input rand value = " << hexval(rand_value) << "\n";
        plgState->hardware_write_to_receive_buffer(data_register, rand_value);
        UpdateGraph(g_s2e_state, Write, data_register);
    }
}

bool NLPPeripheralModel::readNLPModelfromFile(S2EExecutionState *state, std::string fileName) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    std::ifstream fNLP;
    std::string line;
    fNLP.open(fileName, std::ios::in);
    if (!fNLP) {
        getWarningsStream() << "Could not open cache nlp file: "
                            << fileName << "\n";
        return false;
    }

    std::string peripheralcache;
    while (getline(fNLP, peripheralcache)) {
        if (peripheralcache == "==") break;
        PeripheralReg reg;
        if (getMemo(peripheralcache, reg)) {
            if (reg.type == "R") {
                data_register = reg.phaddr;
            } else if (reg.type == "P") {
                countdown_register.push_back(reg.phaddr);
            }
            plgState->insert_reg_map(reg.phaddr, reg);
        } else {
            return false;
        }
    }

    while (getline(fNLP, peripheralcache)) {
        EquList trigger;
        EquList action;
        if (getTApairs(peripheralcache, trigger, action)) {
            allTAs.push_back(std::make_pair(trigger, action));
        } else {
            return false;
        }
    }

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

void NLPPeripheralModel::SplitStringToInt(const std::string &s, std::vector<int> &v, const std::string &c) {
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (std::string::npos != pos2) {
        v.push_back(std::atoi(s.substr(pos1, pos2 - pos1).c_str()));
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v.push_back(std::atoi(s.substr(pos1).c_str()));
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
    reg.phaddr = std::stoull(v[1].c_str(), NULL, 16);
    reg.reset = std::stoull(v[2].c_str(), NULL, 16);
    reg.cur_value = reg.reset;
    reg.t_size = 0;
    reg.r_size = 0;
    reg.t_value = 0;
    reg.r_value = 0;
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
        getDebugStream() << " trigger = " << trigger_str << " action = " << action_str <<" interrupt = "<< action.back().interrupt << "\n";
    }
    return res;
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
        equ.interrupt = -1;
        if (v[0] == "*") {
            equ.a1.type = v[0];
            equ.a1.bits = {-1};
            equ.eq = "*";
            equ.type_a2 = "*";
        } else {
            equ.a1.type = v[0];
            equ.a1.phaddr = std::stoull(v[1].c_str(), NULL, 16);
	    std::vector<int> bits;
	    if (v[2] == "*")
		equ.a1.bits = {-1};
	    else {
	    	SplitStringToInt(v[2], bits, "/");
            	equ.a1.bits = bits;
	    }
            equ.eq = v[3];
	    getDebugStream() << v[0]<<v[1]<<v[2]<<v[3]<<v[4]<<"\n";
            if (v[4] == "O" || v[4] == "C") {
                equ.type_a2 = "F";
                equ.value = 0;
                equ.a2.type = v[4];
                equ.a2.phaddr = std::stoull(v[5].c_str(), NULL, 16);
		if (v[5] == "*")
                	equ.a1.bits = {-1};
            	else {
			bits.clear();
               	 	SplitStringToInt(v[5], bits, "/");
        	        equ.a1.bits = bits;
	        }
            } else if (v[4][0] != '*') {
                equ.type_a2 = "V";
                equ.value = std::stoull(v[4].c_str(), NULL, 2);
            } else {
                equ.value = 0;
                equ.type_a2 = v[4][1];
            }
        }
        getDebugStream() << "equ type = " << equ.a1.type << " equ phaddr = " << equ.a1.phaddr
                        << " equ bits = " << equ.a1.bits[0] << " equ = " << equ.eq << " type_a2 = " << equ.type_a2 << " value = " << equ.value << "\n";
        vec.push_back(equ);
        peripheralcache = what.suffix();
    }
    return true;
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

uint32_t get_reg_value(RegMap &state_map, Field a) {
    uint32_t res;
    if (a.bits[0] == -1) {
        res = state_map[a.phaddr].cur_value;
    } else {
        res = 0;
        for (int i = 0; i < a.bits.size(); ++i) {
            int tmp = a.bits[i]-0;
            res = res*i*2 + (state_map[a.phaddr].cur_value >> tmp & 1);
        }
    }
    return res;  
}

void set_reg_value(RegMap &state_map, Field a, uint32_t value) {
    if (a.bits[0] == -1) {
        state_map[a.phaddr].cur_value = value;
    } else {
        for (int i = 0; i < a.bits.size(); ++i) {
            int tmp = a.bits[i]-0;
            int a2 = value >> (a.bits.size()-1-i);
            if (a2 == 1) {
                state_map[a.phaddr].cur_value |= (1 << tmp);
            } else {
                state_map[a.phaddr].cur_value &= ~(1 << tmp);
            }
        }
    }
}

void NLPPeripheralModel::UpdateGraph(S2EExecutionState *state, RWType type, uint32_t phaddr) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    RegMap state_map = plgState->get_state_map();
    for (auto ta: allTAs) {
        EquList trigger = ta.first;
        bool rel;
        std::vector<bool> trigger_res;
        for (auto equ: trigger) {
            rel = equ.rel;
            if (equ.a1.type == "*") {
                trigger_res.push_back(true);
            } else if (equ.a1.type == "R") {
                if (type == Read && phaddr == data_register)
                    trigger_res.push_back(true);
                else
                    trigger_res.push_back(false);
            } else if (equ.a1.type == "T") {
                if (type == Write && phaddr == data_register)
                    trigger_res.push_back(true);
                else
                    trigger_res.push_back(false);
            } else {
                uint32_t a1, a2;
                a1 = get_reg_value(state_map, equ.a1);
                if (equ.type_a2 == "T") {
                    a2 = state_map[data_register].t_size;
                } else if(equ.type_a2 == "R") {
                    a2 = state_map[data_register].r_size;
                } else if(equ.type_a2 == "F") {
                    a2 = get_reg_value(state_map, equ.a2);
                } else if(equ.type_a2 == "V") {
                    a2 = equ.value;
                } else {
			        a2 = 0;
                    getDebugStream() << "ERROR "<<a1<<" eq "<<equ.eq<<" \n";
                }
		if (equ.a1.type == "F") {
                    if (type == Write && a1 == a2 && phaddr == equ.a1.phaddr) {
			    getDebugStream() << "Write 1 to "<<hexval(equ.a1.phaddr)<<" eq "<<equ.eq<<" \n";
			    trigger_res.push_back(true);
		    } else
                            trigger_res.push_back(false);

		} else
                    trigger_res.push_back(compare(a1, equ.eq, a2));
                getDebugStream() << "compare a1 " <<hexval(equ.a1.phaddr)<<" :"<<a1<<" eq "<<equ.eq<<" a2 "<<a2<<" result "<<trigger_res.back()<<" \n";
            }
        }
        bool check = rel;
        if (rel == true) {
            for (bool idx: trigger_res) {
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
        for (auto equ: trigger) {
            getDebugStream() << "trigger a1 "<<hexval(equ.a1.phaddr)<<" bit: "<<equ.a1.bits[0]<<" eq "<<equ.eq<<" a2 "<<equ.value<<" \n";
        }

        EquList action = ta.second;
        for (auto equ: action) {
            uint32_t a2;
            if (equ.type_a2 == "T") {
                a2 = state_map[data_register].t_size;
            } else if (equ.type_a2 == "R") {
                a2 = state_map[data_register].r_size;
            } else if (equ.type_a2 == "F"){
                a2 = get_reg_value(state_map, equ.a2);
            } else {
                a2 = equ.value;
            }

            if (equ.a1.type == "R") {
                state_map[data_register].r_size = a2;
                plgState->insert_reg_map(data_register, state_map[data_register]);
            } else if (equ.a1.type == "T") {
                state_map[data_register].t_size = a2;
                plgState->insert_reg_map(data_register, state_map[data_register]);
            } else {
                set_reg_value(state_map, equ.a1, a2);
                if (type == Read) {
                    getDebugStream() << "Read Action: phaddr =  "<<  hexval(equ.a1.phaddr) << " updated bit = " <<equ.a1.bits[0]
                        << " value = " << hexval(state_map[equ.a1.phaddr].cur_value) << " a2 = " << a2 << "\n";
                } else {
                    getDebugStream() << "Write Action: phaddr =  "<<  hexval(equ.a1.phaddr) << " updated bit = " <<equ.a1.bits[0]
                        << " value = " << hexval(state_map[equ.a1.phaddr].cur_value) << " a2 = " << a2 << "\n";
                }
                // update to state
                plgState->insert_reg_map(equ.a1.phaddr, state_map[equ.a1.phaddr]);
            }
	    getDebugStream() << "equ.interrupt = " <<equ.interrupt<< "\n";
	    for (auto p: countdown_register) {
                getDebugStream(g_s2e_state) << "update graph Interrupt reg: "<< hexval(p) <<" cur: "<< hexval(state_map[p].cur_value) <<"\n";
            }
            if (equ.interrupt != -1 && !plgState->get_exit_interrupt(equ.interrupt)) {
                getDebugStream() << "IRQ Action trigger interrupt equ.interrupt = " << equ.interrupt << "\n";
                onExternalInterruptEvent.emit(state, equ.interrupt);
		plgState->set_exit_interrupt(equ.interrupt, true);
            }
        }
    }
}

void NLPPeripheralModel::onPeripheralRead(S2EExecutionState *state, SymbolicHardwareAccessType type,
                            uint32_t phaddr, unsigned size, uint32_t *NLPsymbolicvalue) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    rw_count++;
    if (rw_count == 1) {
        readNLPModelfromFile(state, NLPfileName);
    }

    UpdateGraph(state, Read, phaddr);
    if (phaddr == data_register) {
        *NLPsymbolicvalue = plgState->get_dr_value(phaddr);
    } else {
        *NLPsymbolicvalue = plgState->get_ph_value(phaddr);
    }
}

void NLPPeripheralModel::onPeripheralWrite(S2EExecutionState *state, SymbolicHardwareAccessType type,
                            uint32_t phaddr, uint32_t writeconcretevalue) {
    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    rw_count++;
    if (rw_count == 1) {
        readNLPModelfromFile(state, NLPfileName);
    }
    if (phaddr == data_register) {
        plgState->write_dr_value(phaddr, writeconcretevalue);
        getDebugStream() << "Write to data register "<<data_register<<" "<<hexval(phaddr)<<" value: "<<writeconcretevalue<<" \n";
    } else {
        plgState->write_ph_value(phaddr, writeconcretevalue);
	getDebugStream() << "Write to phaddr "<<hexval(phaddr)<<" value: "<<writeconcretevalue<<" \n";
    }
    UpdateGraph(state, Write, phaddr);
}


} // namespace plugins
} // namespace s2e
