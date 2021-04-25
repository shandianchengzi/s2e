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
    std::map<uint32_t, PeripheralReg> peripheral_regs_value_map;
    std::vector<pair<std::vector<Equation>, std::vector<Equation>>> allTAs;
    uint32_t data_register;
    std::string data_register_type = 'R';
    //std::map<std::string, uint32_t> symbol_list = {
    //    {"*",0},{"=",1},{">":2},{"<",3},{">=",4},{"<=",5}
    //};
    //0:= ; 1:>; 2: <; 3: >=; 4: <=

    void UpdateGraph(uint32_t type, uint32_t phaddr) {
        for (auto ta: allTAs) {
            std::vector<Equation> trigger = ta.first;
            bool rel = true;
            std::vector<bool> trigger_res;
            for (auto equ: trigger) {
                if (equ.type == "R") {
                    if (type == 1 && phaddr == data_register)
                        trigger_res.push_back(true);
                    else
                        trigger_res.push_back(false);
                } else {
                    uint32_t a1, a2;
                    if (equ.bits == "*") {
                        a1 = peripheral_regs_value_map[equ.phaddr].cur;
                    } else {
                        uint32_t tmp = std::stoull(equ.bits, NULL, 10);
                        a1 = peripheral_regs_value_map[equ.phaddr].cur>>tmp&1;
                    }
                    if (equ.linkaddr != NULL) a2 = *equ.linkaddr;
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

            std::vector<Equation> action = ta.second;
            for (auto equ: action) {
                uint32_t a2;
                if (equ.linkaddr != NULL) a2 = *equ.linkaddr;
                else a2 = equ.value;
                if (equ.type == "R") {
                    peripheral_regs_value_map[equ.phaddr].r_size = a2;
                } else if (equ.type == "T") {
                    peripheral_regs_value_map[equ.phaddr].t_size = a2;
                } else {
                    uint32_t tmp = std::stoull(equ.bits, NULL, 10);
                    if (a2 == 1)
                        peripheral_regs_value_map[equ.phaddr].cur |= (1<<tmp);
                    else
                        peripheral_regs_value_map[equ.phaddr].cur &= ~(1<<tmp);
                }
            }
        }
    }


    bool compare(uint32_t a1, std::string sym, uint32_t a2) {
        //1:= ; 2:>; 3: <; 4: >=; 5: <=
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
    
    void initialize_graph(std::map<uint32_t, PeripheralReg>& m, std::vector<pair<std::vector<Equation>, std::vector<Equation>>> &ta, uint32_t dr) {
        peripheral_regs_value_map = m;
        allTAs = ta;
        data_register = dr;
    }

    static PluginState *factory(Plugin *, S2EExecutionState *) {
        return new NLPPeripheralModelState();
    }

    NLPPeripheralModelState *clone() const {
        return new NLPPeripheralModelState(*this);
    }

    void write_ph_value(uint32_t phaddr, uint32_t value) {
        if (data_register == phaddr){
            //getWarningsStream() << "write to transmit buffer"
            //                    << "\n";
            peripheral_regs_value_map[phaddr].t_value = value;
            peripheral_regs_value_map[phaddr].t_size = 1;
            return;                    
        }
        peripheral_regs_value_map[phaddr].cur = value;
        UpdateGraph(0, phaddr);//0,
    }

    uint32_t get_ph_value(uint32_t phaddr) {
        if (data_register == phaddr){
            //getWarningsStream() << "read from receive buffer"
            //                    << "\n";
            return peripheral_regs_value_map[phaddr].r_value;                    
        }
        UpdateGraph(1, phaddr);
        return  peripheral_regs_value_map[phaddr].cur;
    }

    void hardware_write_to_receive_buffer(uint32_t value) {
        peripheral_regs_value_map[data_register].r_size = 1;
        peripheral_regs_value_map[data_register].r_value = 1;
        UpdateGraph(0, data_register);
    }
};

void NLPPeripheralModel::initialize() {
    ReadKBfromFile("all.txt");
    //ReadMemofromFile("memory.txt");
    //ReadTAfromFile("register.txt");

    // bool ok;
    // ConfigFile *cfg = s2e()->getConfig();
        
    hw::SymbolicPeripherals *symbolicPeripheralConnection = s2e()->getPlugin<hw::SymbolicPeripherals>();
    symbolicPeripheralConnection->onSymbolicNLPRegisterReadEvent.connect(sigc::mem_fun(*this, &NLPPeripheralModel::onPeripheralRead));
    symbolicPeripheralConnection->onSymbolicNLPRegisterWriteEvent.connect(sigc::mem_fun(*this, &NLPPeripheralModel::onPeripheralWrite));


    DECLARE_PLUGINSTATE(NLPPeripheralModelState, state);
    plgState->initialize_graph(peripheral_regs_value_map, allTAs, data_register);
}
bool NLPPeripheralModel::ReadKBfromFile(std::string fileName) {
    std::ifstream fPHKB;
    std::string line;
    fPHKB.open(fileName, std::ios::in);
    if (!fPHKB) {
        getWarningsStream() << "Could not open cache peripheral knowledge base file: " 
                            << fileName << " \n";
        return false;
    }

    std::string peripheralcache;
    while (getline(fPHKB, peripheralcache)) {
        if (peripheralcache == "==") break;
        PeripheralReg reg;
        if (getMemo(peripheralcache, reg)) {
            peripheral_regs_value_map[reg.phaddr] = reg;
            if (reg.type == data_register_type)
                data_register = reg.phaddr;
        }
    }

    while (getline(fPHKB, peripheralcache)) {
        std::vector<Equation> trigger;
        std::vector<Equation> action;
        if (getTApairs(peripheralcache, trigger, action)) {
            allTAs.push_back(make_pair(trigger, action));
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

bool NLPPeripheralModel::getMemo(std::string peripheralcache, PeripheralReg &reg) {
    boost::smatch what;
    if (!boost::regex_match(peripheralcache, what, MemoRegEx)) {
        getWarningsStream() << "match false"
                            << "\n";
        exit(0);
        return false;
    }

    if (what.size() != 1) {
        getWarningsStream() << "wrong size = " << what.size() << "\n";
        exit(0);
        return false;
    }

    std::vector<std::string> v;
    SplitString(what[0], v, "_");
    reg.type = v[0];
    reg.phaddr = std::stoull(v[1].c_str(), NULL, 10);
    reg.reset = std::stoull(v[2].c_str(), NULL, 10);
    reg.cur = reg.reset;
    reg.t_size = 0;
    reg.r_size = 0;
    reg.t_value = 0;
    reg.r_value = 0;
    return true;
}

bool NLPPeripheralModel::getTApairs(std::string peripheralcache, std::vector<Equation> &trigger, std::vector<Equation> &action) {
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
    
    return extractEqu(trigger_str, trigger, trigger_rel) && extractEqu(action_str, action, action_rel);
}

bool NLPPeripheralModel::extractEqu(std::string peripheralcache, std::vector<Equation> &vec, bool rel){
    boost::smatch what;
    if (!boost::regex_match(peripheralcache, what, TARegEx)) {
        getWarningsStream() << "match false"
                            << "\n";
        exit(0);
        return false;
    }

    if (what.size() == 0) {
        getWarningsStream() << "wrong size = " << what.size() << "\n";
        exit(0);
        return false;
    }
    for (auto equ_str: what) {
        std::vector<std::string> v;
        SplitString(equ_str, v, ",");
        Equation equ;
        equ.rel = rel;
        if (v[0] == "*") {
            equ.type = v[0];
        } else {
            Equation equ;
            equ.type = v[0];
            equ.phaddr = std::stoull(v[1].c_str(), NULL, 10);
            equ.bits = v[2];
            equ.eq = v[3];
            if (v[4][0] != '*') {
                equ.linkaddr = NULL;
                equ.value = std::stoull(v[4].c_str(), NULL, 10);
            } else {
                equ.value = 0;
                //std::stoull(v[4].substr(2, v[4].length()-2).c_str(), NULL, 10)
                if (v[4][1] == 'T')
                    equ.linkaddr = &peripheral_regs_value_map[data_register].t_size;
                else if (v[4][1] == 'R')
                    equ.linkaddr = &peripheral_regs_value_map[data_register].r_size;
            }
        }
        vec.push_back(equ);
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
