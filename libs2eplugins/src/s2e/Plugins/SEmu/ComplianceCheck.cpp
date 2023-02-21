//
/// Copyright (C) 2010-2015, Dependable Systems Laboratory, EPFL
/// All rights reserved.
///
/// Licensed under the Cyberhaven Research License Agreement.
///

#include "ComplianceCheck.h"
#include <algorithm>
#include <s2e/ConfigFile.h>
#include <s2e/S2E.h>
#include <s2e/SymbolicHardwareHook.h>
#include <s2e/Utils.h>
#include <s2e/cpu.h>
#include <sys/shm.h>
#include <time.h>

namespace s2e {
namespace plugins {

S2E_DEFINE_PLUGIN(ComplianceCheck, "Complaince Check Model", "ComplianceCheckModel");

void ComplianceCheck::initialize() {
    onNLPPeripheralModelConnection = s2e()->getPlugin<NLPPeripheralModel>();
    onNLPPeripheralModelConnection->onHardwareWrite.connect(sigc::mem_fun(*this, &ComplianceCheck::onHardwareWrite));
    onNLPPeripheralModelConnection->onFirmwareWrite.connect(sigc::mem_fun(*this, &ComplianceCheck::onPeripheralWrite));
    onNLPPeripheralModelConnection->onFirmwareRead.connect(sigc::mem_fun(*this, &ComplianceCheck::onPeripheralRead));
    onNLPPeripheralModelConnection->onFirmwareCondition.connect(
        sigc::mem_fun(*this, &ComplianceCheck::onPeripheralCondition));
    s2e()->getCorePlugin()->onEngineShutdown.connect(sigc::mem_fun(*this, &ComplianceCheck::onComplianceCheck));

    CCfileName = s2e()->getConfig()->getString(getConfigKey() + ".CCfileName", "all.txt");
    if (!readCCModelfromFile(CCfileName)) {
        getWarningsStream() << "Could not open cache CC file: " << CCfileName << "\n";
        exit(-1);
    } else {
        getInfoStream() << "CC peripheral model file name is " << CCfileName << "\n";
    }
}

bool ComplianceCheck::readCCModelfromFile(std::string &fileName) {
    std::ifstream fNLP;
    std::string line;
    fNLP.open(fileName, std::ios::in);
    if (!fNLP) {
        return false;
    }

    std::string peripheralcache;
    while (getline(fNLP, peripheralcache)) {
        if (peripheralcache == "==")
            break;
        if (!getSequences(peripheralcache))
            return false;
    }
    return true;
}

bool ComplianceCheck::getSequences(std::string &peripheralcache) {
    getDebugStream() << peripheralcache << "\n";
    std::vector<std::string> actions;
    SplitString(peripheralcache, actions, "->");
    std::vector<FieldList> ans;
    for (auto &action : actions) {
        std::vector<std::string> seqs;
        SplitString(action, seqs, "&");
        FieldList tmp;
        for (auto &seq : seqs) {
            Field field;
            ReadField(seq, field);
            tmp.push_back(field);
        }
        ans.push_back(tmp);
    }
    sequences.push_back(ans);
    return true;
}

void ComplianceCheck::ReadField(std::string &expressions, Field &field) {
    getDebugStream() << expressions << "\n";
    std::vector<std::string> v;
    SplitString(expressions, v, ",");
    field.type = v[0];
    field.phaddr = std::stoull(v[1].c_str(), NULL, 16);
    field.bits = getBits(v[2]);
    if (v.size() == 3)
        field.value = 1;
    if (v.size() == 5 && v[4] != "*") {
        field.value = std::stoull(v[4].c_str(), NULL, 2);
    }
}

std::vector<long> ComplianceCheck::getBits(std::string &bits) {
    std::vector<long> res;
    if (bits == "*")
        return {-1};
    else {
        SplitStringToInt(bits, res, "/", 10);
        return res;
    }
}

void ComplianceCheck::SplitStringToInt(const std::string &s, std::vector<long> &v, const std::string &c, int dtype) {
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (std::string::npos != pos2) {
        v.push_back(std::strtol(s.substr(pos1, pos2 - pos1).c_str(), NULL, dtype));
        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length()) {
        v.push_back(std::strtol(s.substr(pos1).c_str(), NULL, dtype));
    }
}

void ComplianceCheck::SplitString(const std::string &s, std::vector<std::string> &v, const std::string &c) {
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

void ComplianceCheck::onHardwareWrite(S2EExecutionState *state, uint32_t phaddr, uint32_t cur_val) {
    int32_t irq = -1;
    if (state->regs()->getInterruptFlag())
        irq = state->regs()->getExceptionIndex() - 16;
    if (prev_access != Write || prev_irq != irq)
        cur_time++;
    prev_access = Write;
    prev_irq = irq;
    getDebugStream() << "ComplianceCheck hardware write!! time: " << cur_time << " irq: " << irq
                     << " phaddr: " << hexval(phaddr) << " cur_val: " << hexval(cur_val) << "\n";
    recording_write[phaddr].push_back(Access("HW", cur_time, irq, phaddr, cur_val, state->regs()->getPc()));
}

void ComplianceCheck::onPeripheralRead(S2EExecutionState *state, uint32_t phaddr, uint32_t cur_val) {
    int32_t irq = -1;
    if (state->regs()->getInterruptFlag())
        irq = state->regs()->getExceptionIndex() - 16;
    if (prev_access != Read || prev_irq != irq)
        cur_time++;
    prev_access = Read;
    prev_irq = irq;
    recording_read[phaddr].push_back(Access("FR", cur_time, irq, phaddr, cur_val, state->regs()->getPc()));
    getDebugStream() << "ComplianceCheck READ  time: " << cur_time << " irq: " << irq << " phaddr: " << hexval(phaddr)
                     << " cur_val: " << cur_val << "\n";
}

void ComplianceCheck::onPeripheralWrite(S2EExecutionState *state, uint32_t phaddr, uint32_t cur_val) {
    int32_t irq = -1;
    if (state->regs()->getInterruptFlag())
        irq = state->regs()->getExceptionIndex() - 16;
    if (prev_access != Write || prev_irq != irq)
        cur_time++;
    prev_access = Write;
    prev_irq = irq;
    if (cur_time % 5 == 0) {
        onComplianceCheck();
    }
    if (cur_time > 150) {
        onComplianceCheck();
        exit(-1);
    }
    recording_write[phaddr].push_back(Access("FW", cur_time, irq, phaddr, cur_val, state->regs()->getPc()));
    getDebugStream() << "ComplianceCheck WRITE  time: " << cur_time << " irq: " << irq << " phaddr: " << hexval(phaddr)
                     << " cur_val: " << cur_val << " cur_time: " << cur_time << "\n";
}

void ComplianceCheck::onPeripheralCondition(S2EExecutionState *state, uint32_t phaddr, uint32_t cur_val) {
    int32_t irq = -1;
    if (state->regs()->getInterruptFlag())
        irq = state->regs()->getExceptionIndex() - 16;
    if (prev_access != Check || prev_irq != irq)
        cur_time++;
    prev_access = Check;
    prev_irq = irq;
    getDebugStream() << "ComplianceCheck condition time: " << cur_time << " irq: " << irq
                     << " phaddr: " << hexval(phaddr) << " cur_val: " << cur_val << "\n";
    recording_check[phaddr].push_back(Access("FC", cur_time, irq, phaddr, cur_val, state->regs()->getPc()));
}

bool ComplianceCheck::checkField(Field &field, uint32_t cur_value) {
    if (field.bits[0] == -1)
        return true;

    uint32_t res = 0;
    for (int i = 0; i < field.bits.size(); ++i) {
        int tmp = field.bits[i];
        res = (res << 1) + (cur_value >> tmp & 1);
    }
    getDebugStream() << "checkField" << hexval(field.phaddr) << " access cur_value: " << hexval(cur_value)
                     << " at bit: " << field.bits[0] << "  value: " << res << " field.value: " << field.value << "\n";
    return res == field.value;
}

void ComplianceCheck::getExsitence(std::vector<Access> &accesses, Field &rule, AccessPair &new_existence) {
    for (auto &access : accesses) {
        if (!checkField(rule, access.cur_value))
            continue;
        new_existence[access.irq].push_back(access.time);
    }
}

void ComplianceCheck::checkAtomic(std::vector<AccessPair> &existence_seq, Race &races) {
    for (auto idx = existence_seq.size() - 1; idx > 0; --idx) {
        for (auto &rule : existence_seq[idx]) {
            for (auto &t : rule.second) {
                auto prev_time = t - 1;
                std::vector<uint32_t> &tmp = existence_seq[idx - 1][rule.first];
                if (find(tmp.begin(), tmp.end(), prev_time) == tmp.end()) {
                    getDebugStream() << "existence cur_time: " << t << "prevtime: " << t - 1 << "\n";
                    races.push_back({prev_time, t});
                }
            }
        }
    }
}

void ComplianceCheck::type1Check(Race &races) {
    for (auto &seq : sequences) {
        std::vector<AccessPair> existence_seq;
        bool rule_checker = true;
        for (auto &rule : seq) {
            std::vector<AccessPair> rules_existence;
            for (Field &f : rule) {
                AccessPair access;
                if (f.type == "CC") {
                    if (recording_check.find(f.phaddr) == recording_check.end())
                        continue;
                    getExsitence(recording_check[f.phaddr], f, access);
                } else if (f.type == "CW") {
                    if (recording_write.find(f.phaddr) == recording_write.end())
                        continue;
                    getExsitence(recording_write[f.phaddr], f, access);
                } else if (f.type == "CR") {
                    if (recording_read.find(f.phaddr) == recording_read.end())
                        continue;
                    getExsitence(recording_read[f.phaddr], f, access);
                }
                if (access.size() == 0) {
                    rule_checker = false;
                    break;
                }
                getDebugStream() << "type 1 recording match rule phaddr:" << hexval(f.phaddr) << " " << access.size()
                                 << " rule: " << f.bits[0] << "\n";
                rules_existence.push_back(access);
            }
            if (!rule_checker || rules_existence.size() == 0) {
                existence_seq.clear();
                break;
            } else if (rules_existence.size() == 1) {
                existence_seq.push_back(rules_existence[0]);
            } else {
                existence_seq.clear();
                getWarningsStream() << "cannot handle two rules now!"
                                    << "\n";
                break;
            }
        }
        if (existence_seq.size() != 0)
            checkAtomic(existence_seq, races);
    }
}

void ComplianceCheck::checkClear(std::vector<AccessPair> &existence_seq, Race &races) {
    for (auto idx = 0; idx < existence_seq.size() - 1; ++idx) {
        for (auto &rule : existence_seq[idx]) {
            for (auto &t : rule.second) {
                std::vector<uint32_t> &tmp = existence_seq[idx + 1][rule.first];
                // getDebugStream() << "cur time: " << t << " tmp size:" << tmp.size() << "\n";
                auto idx = upper_bound(tmp.begin(), tmp.end(), t);
                if (idx == tmp.end()) {
                    races.push_back({t, 0});
                    getDebugStream() << "cur time: " << t << " race_size: " << races.size() << "\n";
                } else {
                    getDebugStream() << "non CE existence cur_time: " << t << "nexttime: " << idx - tmp.begin() << "\n";
                }
            }
        }
    }
}

void ComplianceCheck::type4Check(Race &races) {
    for (auto &seq : sequences) {
        std::vector<AccessPair> existence_seq;
        bool rule_checker = true;
        for (auto &rule : seq) {
            std::vector<AccessPair> rules_existence;
            for (Field &f : rule) {
                AccessPair access;
                if (f.type == "CE") {
                    getDebugStream() << "CE phaddr: " << hexval(f.phaddr) << "\n";
                    if (recording_write.find(f.phaddr) == recording_write.end())
                        continue;
                    getExsitence(recording_write[f.phaddr], f, access);
                } else
                    break;

                if (access.size() == 0) {
                    rule_checker = false;
                    break;
                }
                getDebugStream() << "type 4 recording match rule phaddr:" << hexval(f.phaddr)
                                 << " access_size: " << access.size() << " rule bit: " << f.bits[0] << "\n";
                rules_existence.push_back(access);
            }
            if (!rule_checker || rules_existence.size() == 0) {
                existence_seq.clear();
                break;
            } else if (rules_existence.size() == 1) {
                existence_seq.push_back(rules_existence[0]);
            } else {
                existence_seq.clear();
                getWarningsStream() << "cannot handle two rules now!"
                                    << "\n";
                break;
            }
        }
        getDebugStream() << "existence_seq size:" << existence_seq.size() << "\n";
        if (existence_seq.size() != 0)
            checkClear(existence_seq, races);
        getDebugStream() << "get total races events =" << races.size() << "\n";
    }
}

void ComplianceCheck::onComplianceCheck() {
    Race races;
    getInfoStream() << "verify race events \n";
    getInfoStream() << "type 1 check\n";
    type1Check(races);
    getInfoStream() << "get type 1 races events =" << races.size() << "\n";
    type4Check(races);
    getInfoStream() << "get type 2 races events =" << races.size() << "\n";
    if (races.size() == 0)
        return;

    getInfoStream() << "write Compliance Check files\n";
    std::string NLPstafileName = s2e()->getOutputDirectory() + "/" + "ComplianceCheck.dat";
    std::ofstream fPHNLP;
    fPHNLP.open(NLPstafileName, std::ios::out | std::ios::trunc);

    std::map<int32_t, std::vector<access>> all_recordings;
    for (auto &record : recording_read) {
        for (auto &access : record.second) {
            all_recordings[access.time].push_back(access);
        }
    }
    for (auto &record : recording_write) {
        for (auto &access : record.second) {
            all_recordings[access.time].push_back(access);
        }
    }
    for (auto &record : recording_check) {
        for (auto &access : record.second) {
            all_recordings[access.time].push_back(access);
        }
    }
    fPHNLP << "-------Compliance Check Results-------\n";
    for (auto &race : races) {
        for (auto &time : race) {
            for (auto &access : all_recordings[time])
                fPHNLP << "time: " << access.time << " type: " << access.type << " irq: " << access.irq
                       << " phaddr: " << hexval(access.phaddr) << " pc: " << hexval(access.pc) << "\n";
        }
        fPHNLP << "==================\n";
    }

    fPHNLP.close();
    exit(-1);
}

} // namespace plugins
} // namespace s2e
