/**
 * @file rvss_vm.cpp
 * @brief RVSS VM implementation
 * @author Vishank Singh, https://github.com/VishankSingh
 */

#include "vm/rvss/dynamic_branch_pipelined_rvss_vm.h"

#include "utils.h"
#include "globals.h"
#include "common/instructions.h"
#include "config.h"
#include "vm/pipelineRegisters.h"

#include <cctype>
#include <cstdint>
#include <iostream>
#include <tuple>
#include <map>
#include <vector>
#include <stack>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

using instruction_set::Instruction;
using instruction_set::get_instr_encoding;

RVSSVM_DYNAMIC::RVSSVM_DYNAMIC() : VmBase() {
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

static uint ForwardA;
static uint ForwardB;
static uint ForwardC;
static uint Branch;
static int64_t execResult1;
static int64_t execResult2;
static int64_t execResult3;
static int64_t memResult1;
static int64_t memResult2;
static int64_t memResult3;
static bool pickResult1;
static bool pickResult2;
static bool pickResult3;
static bool stall;
static int NumStalls = 0;
static bool DStall;
static uint DForwardA;
static uint DForwardB;
static int64_t DResultA;
static int64_t DResultB;
static uint branchTaken;
static int64_t targetImm;
static bool flush;
static std::map<uint64_t,std::vector<uint64_t>> branchHistory;
static uint64_t correctPC;

static void initializeForwardControlSignals(){
    ForwardA = 0;
    ForwardB = 0;
    ForwardC = 0;
    Branch = 0;
    execResult1 = 0;
    execResult2 = 0;
    execResult3 = 0;
    memResult1 = 0;
    memResult2 = 0;
    memResult3 = 0;
    stall = false;
    pickResult1 = true;
    pickResult2 = true;
    pickResult3 = true;
    DStall=false;
    DForwardA = 0;
    DForwardB = 0;
    DResultA = 0;
    DResultB = 0;
    branchTaken = 0;
    targetImm = 0;
    flush = false;
    correctPC = 0;
}

static bool checkProcessOver(){
    bool yes=true;
    if(IF_ID.readInstruction()!=0)yes=false;
    if(ID_EX.readOpcode()!=0)yes=false;
    if(EX_MEM.readOpcode()!=0)yes=false;
    if(MEM_WB.readOpcode()!=0)yes=false;
    return yes;
}

// case get_instr_encoding(Instruction::kfle_s).funct7: // f(eq|lt|le).s
//         case get_instr_encoding(Instruction::kfcvt_w_s).funct7: // fcvt.(w|wu|l|lu).s
//         case get_instr_encoding(Instruction::kfmv_x_w).funct7: // fmv.x.w , fclass.s

static void HazardDetectionUnit(){
    
    if(!EX_MEM.readIsDouble() && !EX_MEM.readIsFloat() && !ID_EX.readIsFloat() && !ID_EX.readIsDouble()){
        if(EX_MEM.WriteBackSignal() && (EX_MEM.readRd()!=0) && EX_MEM.readRd()==ID_EX.readRs1())ForwardA=10;
        if(EX_MEM.WriteBackSignal() && (EX_MEM.readRd()!=0) && EX_MEM.readRd()==ID_EX.readRs2() && (!ID_EX.ExecuteSignal() || ID_EX.MemWrite()))ForwardB=10;
    }

    else if((EX_MEM.readIsDouble() || EX_MEM.readIsFloat()) && (ID_EX.readIsFloat() || ID_EX.readIsDouble())){
        if(EX_MEM.readFunct7()==get_instr_encoding(Instruction::kfle_s).funct7 || EX_MEM.readFunct7()==get_instr_encoding(Instruction::kfcvt_w_s).funct7 || EX_MEM.readFunct7()==get_instr_encoding(Instruction::kfmv_x_w).funct7){
            if(ID_EX.readFunct7()==0b1101000 || ID_EX.readFunct7()==0b1111000 || ID_EX.readOpcode()==0b0000111 || ID_EX.readOpcode()==0b0100111){
                if(EX_MEM.WriteBackSignal() && EX_MEM.readRd()!=0 && EX_MEM.readRd()==ID_EX.readRs1())ForwardA=10;
            }
        }
        else{
            if(ID_EX.readFunct7()==0b1101000 || ID_EX.readFunct7()==0b1111000 || ID_EX.readOpcode()==0b0000111 || ID_EX.readOpcode()==0b0100111){
                if(ID_EX.MemWrite()){
                    if(EX_MEM.WriteBackSignal() && EX_MEM.readRd()==ID_EX.readRs2())ForwardB=10;
                }
            }
            else{
                if(EX_MEM.WriteBackSignal() && EX_MEM.readRd()==ID_EX.readRs1())ForwardA=10;
                if(EX_MEM.WriteBackSignal() && EX_MEM.readRd()==ID_EX.readRs2() && (!ID_EX.ExecuteSignal() || ID_EX.MemWrite()))ForwardB=10;
                if(EX_MEM.WriteBackSignal() && EX_MEM.readRd()==ID_EX.readRs3())ForwardC=10;
            }
        }
    }
    else if((!EX_MEM.readIsDouble() && !EX_MEM.readIsFloat()) && (ID_EX.readIsDouble() || ID_EX.readIsFloat())){
        if(ID_EX.readFunct7()==0b1101000 || ID_EX.readFunct7()==0b1111000 || ID_EX.readOpcode()==0b0000111 || ID_EX.readOpcode()==0b0100111){
            if(EX_MEM.WriteBackSignal() && EX_MEM.readRd()!=0 && EX_MEM.readRd()==ID_EX.readRs1())ForwardA=10;
        }
    }
    else {
        if(EX_MEM.readFunct7()==get_instr_encoding(Instruction::kfle_s).funct7 || EX_MEM.readFunct7()==get_instr_encoding(Instruction::kfcvt_w_s).funct7 || EX_MEM.readFunct7()==get_instr_encoding(Instruction::kfmv_x_w).funct7){
            if(EX_MEM.WriteBackSignal() && EX_MEM.readRd()!=0 && EX_MEM.readRd()==ID_EX.readRs1())ForwardA=10;
            if(EX_MEM.WriteBackSignal() && EX_MEM.readRd()!=0 && EX_MEM.readRd()==ID_EX.readRs2() && (!ID_EX.ExecuteSignal() || ID_EX.MemWrite()))ForwardB=10;
        }
    }

    if(!MEM_WB.readIsDouble() && !MEM_WB.readIsFloat() && !ID_EX.readIsFloat() && !ID_EX.readIsDouble()){
        if(MEM_WB.WriteBackSignal() && (MEM_WB.readRd()!=0) && MEM_WB.readRd()==ID_EX.readRs1() && ForwardA!=10)ForwardA=1;
        if(MEM_WB.WriteBackSignal() && (MEM_WB.readRd()!=0) && MEM_WB.readRd()==ID_EX.readRs2() && (!ID_EX.ExecuteSignal() || ID_EX.MemWrite()) && ForwardB!=10)ForwardB=1;
    }
    else if((MEM_WB.readIsDouble() || MEM_WB.readIsFloat()) && (ID_EX.readIsFloat() || ID_EX.readIsDouble())){

        if(MEM_WB.readFunct7()==get_instr_encoding(Instruction::kfle_s).funct7 || MEM_WB.readFunct7()==get_instr_encoding(Instruction::kfcvt_w_s).funct7 || MEM_WB.readFunct7()==get_instr_encoding(Instruction::kfmv_x_w).funct7){
            if(ID_EX.readFunct7()==0b1101000 || ID_EX.readFunct7()==0b1111000 || ID_EX.readOpcode()==0b0000111 || ID_EX.readOpcode()==0b0100111){
                if(MEM_WB.WriteBackSignal() && MEM_WB.readRd()!=0 && MEM_WB.readRd()==ID_EX.readRs1() && ForwardA!=10)ForwardA=1;
            }
        }
        else{
            if(ID_EX.readFunct7()==0b1101000 || ID_EX.readFunct7()==0b1111000 || ID_EX.readOpcode()==0b0000111 || ID_EX.readOpcode()==0b0100111){
                if(ID_EX.MemWrite()){
                    if(MEM_WB.WriteBackSignal() && MEM_WB.readRd()==ID_EX.readRs2() && (!ID_EX.ExecuteSignal() || ID_EX.MemWrite()) && ForwardB!= 10)ForwardB=1;
                }
            }
            else{
                if(MEM_WB.WriteBackSignal() && MEM_WB.readRd()==ID_EX.readRs1() && ForwardA!= 10)ForwardA=1;
                if(MEM_WB.WriteBackSignal() && MEM_WB.readRd()==ID_EX.readRs2() && (!ID_EX.ExecuteSignal() || ID_EX.MemWrite()) && ForwardB!=10)ForwardB=1;
                if(MEM_WB.WriteBackSignal() && MEM_WB.readRd()==ID_EX.readRs3() && ForwardC!= 10)ForwardC=1;
            }
        }
    }
    else if((!MEM_WB.readIsDouble() && !MEM_WB.readIsFloat()) && (ID_EX.readIsDouble() || ID_EX.readIsFloat())){
        if(ID_EX.readFunct7()==0b1101000 || ID_EX.readFunct7()==0b1111000 || ID_EX.readOpcode()==0b0000111 || ID_EX.readOpcode()==0b0100111){
            if(MEM_WB.WriteBackSignal() && MEM_WB.readRd()!=0 && MEM_WB.readRd()==ID_EX.readRs1() && ForwardA!=10)ForwardA=1;
        }
    }
    else{
        if(MEM_WB.readFunct7()==get_instr_encoding(Instruction::kfle_s).funct7 || MEM_WB.readFunct7()==get_instr_encoding(Instruction::kfcvt_w_s).funct7 || MEM_WB.readFunct7()==get_instr_encoding(Instruction::kfmv_x_w).funct7){
            if(MEM_WB.WriteBackSignal() && MEM_WB.readRd()!=0 && MEM_WB.readRd()==ID_EX.readRs1() && ForwardA!=10)ForwardA=1;
            if(MEM_WB.WriteBackSignal() && MEM_WB.readRd()!=0 && MEM_WB.readRd()==ID_EX.readRs2() && ForwardB!=10 && (!ID_EX.ExecuteSignal() || ID_EX.MemWrite()))ForwardB=1;
        }
    }

    if(EX_MEM.readIsBranch())Branch=1;
}

static void ForwardUnit(){
    if(ForwardA==10){
        if(EX_MEM.MemRead())stall=true;
        else execResult1=EX_MEM.readExecutionResult();
    }
    if(ForwardA==1){
        if(MEM_WB.MemRead()){memResult1=MEM_WB.readMemoryResult();pickResult1=false;}
        else execResult1=MEM_WB.readExecutionResult();
    }
    if(ForwardB==10){
        if(EX_MEM.MemRead())stall=true;
        else execResult2=EX_MEM.readExecutionResult();
    }
    if(ForwardB==1){
        if(MEM_WB.MemRead()){memResult2=MEM_WB.readMemoryResult();pickResult2=false;}
        else execResult2=MEM_WB.readExecutionResult();
    }
    if(ForwardC==10){
        if(MEM_WB.MemRead()){memResult3=MEM_WB.readMemoryResult();pickResult3=false;}
        else execResult3=MEM_WB.readExecutionResult();
    }
    if(ForwardC==1){
        if(MEM_WB.MemRead()){memResult3=MEM_WB.readMemoryResult();pickResult3=false;}
        else execResult3=MEM_WB.readExecutionResult();
    }
}

void RVSSVM_DYNAMIC::DecodeHazard(){
    control_unit_.SetControlSignals(IF_ID.readInstruction());
    uint32_t currentInstruction = IF_ID.readInstruction();
    uint8_t rs1 = (currentInstruction >> 15) & 0b11111;
    uint8_t rs2 = (currentInstruction >> 20) & 0b11111;
    if(!control_unit_.GetBranch())return ;
    if(ID_EX.WriteBackSignal() && ID_EX.readRd() != 0 && rs1 == ID_EX.readRd()){
        if(ID_EX.readIsFloat() || ID_EX.readIsDouble()){
            uint8_t funct7 = ID_EX.readFunct7();
            if(get_instr_encoding(Instruction::kfle_s).funct7 == funct7 || get_instr_encoding(Instruction::kfcvt_w_s).funct7 == funct7 || get_instr_encoding(Instruction::kfmv_x_w).funct7 == funct7){
                DStall = true;
            }
        }
        else DStall=true;
    }
    if(EX_MEM.MemRead() && EX_MEM.readRd() != 0 && EX_MEM.readRd()==rs1){
        if(!(EX_MEM.readIsFloat() || EX_MEM.readIsDouble()))DStall=true;
    }
    bool changed1 = false;
    if(EX_MEM.WriteBackSignal() && EX_MEM.readRd() != 0 && EX_MEM.readRd() == rs1 && !DStall){
        if(EX_MEM.readIsFloat() || EX_MEM.readIsDouble()){
            uint8_t funct7 = EX_MEM.readFunct7();
            if(get_instr_encoding(Instruction::kfle_s).funct7 == funct7 || get_instr_encoding(Instruction::kfcvt_w_s).funct7 == funct7 || get_instr_encoding(Instruction::kfmv_x_w).funct7 == funct7){
                DForwardA = 10;DResultA=EX_MEM.readExecutionResult();changed1=true;
            }
        }
        else {DForwardA=10;DResultA=EX_MEM.readExecutionResult();changed1=true;}
    }
    if(MEM_WB.WriteBackSignal() && MEM_WB.readRd() != 0 && MEM_WB.readRd() == rs1 && !DStall && !changed1){
        if(MEM_WB.readIsFloat() || MEM_WB.readIsDouble() ){
            uint8_t funct7 = MEM_WB.readFunct7();
            if(get_instr_encoding(Instruction::kfle_s).funct7 == funct7 || get_instr_encoding(Instruction::kfcvt_w_s).funct7 == funct7 || get_instr_encoding(Instruction::kfmv_x_w).funct7 == funct7){
                DForwardA=1;DResultA=MEM_WB.readExecutionResult();
            }
        }
        else{
            DForwardA=1;
            if(MEM_WB.MemRead())DResultA=MEM_WB.readMemoryResult();
            else DResultA=MEM_WB.readExecutionResult();
        }
    }
    bool changed2 = false;
    if(ID_EX.WriteBackSignal() && ID_EX.readRd() != 0 && rs2 == ID_EX.readRd() && !control_unit_.GetAluSrc()){
        if(ID_EX.readIsFloat() || ID_EX.readIsDouble()){
            uint8_t funct7 = ID_EX.readFunct7();
            if(get_instr_encoding(Instruction::kfle_s).funct7 == funct7 || get_instr_encoding(Instruction::kfcvt_w_s).funct7 == funct7 || get_instr_encoding(Instruction::kfmv_x_w).funct7 == funct7){
                DStall = true;
            }
        }
        else DStall=true;
    }
    if(EX_MEM.MemRead() && EX_MEM.readRd() != 0 && EX_MEM.readRd()==rs2 && !control_unit_.GetAluSrc()){
        if(!(EX_MEM.readIsFloat() || EX_MEM.readIsDouble()))DStall=true;
    }
    if(EX_MEM.WriteBackSignal() && EX_MEM.readRd() != 0 && EX_MEM.readRd() == rs2 && !control_unit_.GetAluSrc() && !DStall){
        if(EX_MEM.readIsFloat() || EX_MEM.readIsDouble()){
            uint8_t funct7 = EX_MEM.readFunct7();
            if(get_instr_encoding(Instruction::kfle_s).funct7 == funct7 || get_instr_encoding(Instruction::kfcvt_w_s).funct7 == funct7 || get_instr_encoding(Instruction::kfmv_x_w).funct7 == funct7){
                DForwardB=10;DResultB=EX_MEM.readExecutionResult();changed2=true;
            }
        }
        else {DForwardB=10;DResultB=EX_MEM.readExecutionResult();changed2=true;}
    }
    if(MEM_WB.WriteBackSignal() && MEM_WB.readRd() != 0 && MEM_WB.readRd() == rs2 && !control_unit_.GetAluSrc() && !DStall && !changed2){
        if(MEM_WB.readIsFloat() || MEM_WB.readIsDouble()){
            uint8_t funct7 = MEM_WB.readFunct7();
            if(get_instr_encoding(Instruction::kfle_s).funct7 == funct7 || get_instr_encoding(Instruction::kfcvt_w_s).funct7 == funct7 || get_instr_encoding(Instruction::kfmv_x_w).funct7 == funct7){
                DForwardB=1;DResultB=MEM_WB.readMemoryResult();
            }
        }
        else{
            DForwardB=1;
            if(MEM_WB.MemRead())DResultB=MEM_WB.readMemoryResult();
            else DResultB=MEM_WB.readExecutionResult();
        }
    }
    control_unit_.SetControlSignals(0);
}

RVSSVM_DYNAMIC::~RVSSVM_DYNAMIC() = default;

void RVSSVM_DYNAMIC::Fetch() {
    current_instruction_ = memory_controller_.ReadWord(program_counter_);
    IF_ID.fetchInstruction(current_instruction_);
    IF_ID.modifyProgramCounter(program_counter_);
    uint8_t opcode = current_instruction_ & 0b1111111;
    if(branchHistory.find(program_counter_)!=branchHistory.end()){
        if(branchHistory[program_counter_][0] ){
            if(opcode == get_instr_encoding(Instruction::kjalr).opcode){
                UpdateProgramCounter(4);
            }
            else{
                uint64_t value = branchHistory[program_counter_][1];
                UpdateProgramCounter(-program_counter_);
                UpdateProgramCounter(value);
            }
            // uint64_t value = branchHistory[program_counter_][1];
            // UpdateProgramCounter(-program_counter_);
            // UpdateProgramCounter(value);
        }
        else {
            UpdateProgramCounter(4);
        }
    }
    else UpdateProgramCounter(4);
}

void RVSSVM_DYNAMIC::Decode() {
    control_unit_.SetControlSignals(IF_ID.readInstruction());
    uint32_t currentInstruction = IF_ID.readInstruction();
    uint8_t rs1 = (currentInstruction >> 15) & 0b11111;
    uint8_t rs2 = (currentInstruction >> 20) & 0b11111;
    uint8_t opcode = currentInstruction & 0b1111111;
    uint8_t funct3 = (currentInstruction >> 12) & 0b111;
    uint8_t funct7 = (currentInstruction >> 25) & 0b1111111;
    int32_t imm = ImmGenerator(currentInstruction);
    uint64_t reg1_value = registers_.ReadGpr(rs1);
    uint64_t reg2_value = registers_.ReadGpr(rs2);
    alu::AluOp aluOperation = control_unit_.GetAluSignal(current_instruction_, control_unit_.GetAluOp());

    if(control_unit_.GetBranch()){
        if(DForwardA > 0)reg1_value=DResultA;
        if(DForwardB > 0)reg2_value=DResultB;
        if (control_unit_.GetAluSrc()) {
            reg2_value = static_cast<uint64_t>(static_cast<int64_t>(imm));
        }
        bool overflow = false;
        std::tie(execution_result_, overflow) = alu_.execute(aluOperation, reg1_value, reg2_value);
        if (opcode==get_instr_encoding(Instruction::kjalr).opcode || opcode==get_instr_encoding(Instruction::kjal).opcode){
            return_address_ = IF_ID.readProgramCounter()+4;
            next_pc_ = IF_ID.readProgramCounter()+4;
            if(branchHistory.find(IF_ID.readProgramCounter()) != branchHistory.end()){
                if(opcode == get_instr_encoding(Instruction::kjalr).opcode){
                    correctPC = (execution_result_);
                    branchHistory[IF_ID.readProgramCounter()]= {1,correctPC};
                    flush = true;
                }
                else{
                    flush = false;
                }
                // flush = false;
            }
            else {
                if(opcode==get_instr_encoding(Instruction::kjalr).opcode){
                    correctPC = (execution_result_);
                    branchHistory[IF_ID.readProgramCounter()]= {1,correctPC};
                }
                else{
                    branchHistory[IF_ID.readProgramCounter()]={1, IF_ID.readProgramCounter()+imm};
                    correctPC = imm+IF_ID.readProgramCounter();
                }
                flush=true;
            }
            execution_result_ = next_pc_;
        }
        else if (opcode==get_instr_encoding(Instruction::kbeq).opcode ||
                    opcode==get_instr_encoding(Instruction::kbne).opcode ||
                    opcode==get_instr_encoding(Instruction::kblt).opcode ||
                    opcode==get_instr_encoding(Instruction::kbge).opcode ||
                    opcode==get_instr_encoding(Instruction::kbltu).opcode ||
                    opcode==get_instr_encoding(Instruction::kbgeu).opcode) {
            switch (funct3) {
                case 0b000: {// BEQ
                branch_flag_ = (execution_result_==0);
                break;
                }
                case 0b001: {// BNE
                branch_flag_ = (execution_result_!=0);
                break;
                }
                case 0b100: {// BLT
                branch_flag_ = (execution_result_==1);
                break;
                }
                case 0b101: {// BGE
                branch_flag_ = (execution_result_==0);
                break;
                }
                case 0b110: {// BLTU
                branch_flag_ = (execution_result_==1);
                break;
                }
                case 0b111: {// BGEU
                branch_flag_ = (execution_result_==0);
                break;
                }
            }
            if(branchHistory.find(IF_ID.readProgramCounter()) != branchHistory.end() && opcode == 0b1100011){
                if(branchHistory[IF_ID.readProgramCounter()][0] == branch_flag_)flush=false;
                else{
                    flush=true;
                    if(branch_flag_ == 1){
                        branchHistory[IF_ID.readProgramCounter()] = {1,IF_ID.readProgramCounter() + imm};
                        correctPC = imm+IF_ID.readProgramCounter();
                    }
                    else{
                        branchHistory[IF_ID.readProgramCounter()] = {0,IF_ID.readProgramCounter()+4};
                        correctPC = IF_ID.readProgramCounter()+4;
                    }
                }
            }
            else if(opcode == 0b1100011){
                if(branch_flag_ == 1){
                    branchHistory[IF_ID.readProgramCounter()] = {1,imm+IF_ID.readProgramCounter()};
                    correctPC = imm+IF_ID.readProgramCounter();
                    flush = true;
                }
                else{
                    branchHistory[IF_ID.readProgramCounter()] = {0,IF_ID.readProgramCounter()+imm};
                    correctPC = IF_ID.readProgramCounter()+4;
                    flush = false;
                }
            }
        }
    }

    ID_EX.modifyReadData1(reg1_value);
    ID_EX.modifyReadData2(reg2_value);
    ID_EX.modifyImmediate(imm);
    ID_EX.modifyexecuteSignal(control_unit_.GetAluSrc());
    ID_EX.modifyOpcode(opcode);
    ID_EX.modifyFunct3(funct3);
    ID_EX.modifyFunct7(funct7);
    ID_EX.modifyAluOp(aluOperation);
    ID_EX.modifyMemRead(control_unit_.GetMemRead());
    ID_EX.modifyWriteBackSignal(control_unit_.GetRegWrite());
    ID_EX.modifyMemWrite(control_unit_.GetMemWrite());
    ID_EX.modifyIsBranch(control_unit_.GetBranch());
    ID_EX.modifyIsFloat(instruction_set::isFInstruction(currentInstruction));
    ID_EX.modifyIsDouble(instruction_set::isDInstruction(currentInstruction));
    ID_EX.modifyIsCSR(opcode==0b1110011);
    ID_EX.modifyRd((currentInstruction >> 7) & 0b11111);
    ID_EX.modifyRs1(rs1);
    ID_EX.modifyRs2(rs2);
    ID_EX.modifyRs3((currentInstruction >> 27) & 0b11111);
    ID_EX.modifyProgramCounter(IF_ID.readProgramCounter());
    ID_EX.modifyNextPC(next_pc_);
    ID_EX.modifyExecutionResult(execution_result_);
}

void RVSSVM_DYNAMIC::Execute() {
    uint8_t opcode = ID_EX.readOpcode();
    uint8_t funct3 = ID_EX.readFunct3();

    if (opcode == get_instr_encoding(Instruction::kecall).opcode &&
        funct3 == get_instr_encoding(Instruction::kecall).funct3) {
        HandleSyscall();
        return;
    }

    if (ID_EX.readIsFloat()) { // RV64 F
        ExecuteFloat();
        return;
    } else if (ID_EX.readIsDouble()) {
        ExecuteDouble();
        return;
    } else if (ID_EX.readIsCSR()) {
        ExecuteCsr();
        return;
    }
    ID_EX.modifyReadData1(registers_.ReadGpr(ID_EX.readRs1()));
    ID_EX.modifyReadData2(registers_.ReadGpr(ID_EX.readRs2()));
    uint64_t reg1_value = ID_EX.ReadData1();
    uint64_t reg2_value = ID_EX.ReadData2();
    int32_t imm = ID_EX.readImmediate();

    if(ForwardA==10 || ForwardA==1){
        if(pickResult1)reg1_value=execResult1;
        else reg1_value=memResult1;
    }

    if(ForwardB==10 || ForwardB==1){
        if(pickResult2)reg2_value=execResult2;
        else reg2_value=memResult2;
    }

    bool overflow = false;

    if (ID_EX.ExecuteSignal()) {
        reg2_value = static_cast<uint64_t>(static_cast<int64_t>(imm));
    }

    alu::AluOp aluOperation = ID_EX.readAluOp();
    std::tie(execution_result_, overflow) = alu_.execute(aluOperation, reg1_value, reg2_value);
    if(get_instr_encoding(Instruction::klui).opcode == opcode && ID_EX.WriteBackSignal()){
        execution_result_ = (ID_EX.readImmediate() << 12 );
    }
    EX_MEM.modifyExecutionResult(execution_result_);
    EX_MEM.modifyIsBranch(ID_EX.readIsBranch());
    EX_MEM.modifyMemRead(ID_EX.MemRead());
    EX_MEM.modifyMemWrite(ID_EX.MemWrite());
    EX_MEM.modifyWriteBackSignal(ID_EX.WriteBackSignal());
    EX_MEM.modifyOpcode(ID_EX.readOpcode());
    EX_MEM.modifyFunct3(ID_EX.readFunct3());
    EX_MEM.modifyImmediate(ID_EX.readImmediate());
    EX_MEM.modifyIsFloat(ID_EX.readIsFloat());
    EX_MEM.modifyIsDouble(ID_EX.readIsDouble());
    EX_MEM.modifyIsCSR(ID_EX.readIsCSR());
    EX_MEM.modifyRd(ID_EX.readRd());
    EX_MEM.modifyFunct7(ID_EX.readFunct7());
    EX_MEM.modifyRs1(ID_EX.readRs1());ForwardA=10;
    EX_MEM.modifyRs2(ID_EX.readRs2());
    EX_MEM.modifyRs3(ID_EX.readRs3());

    // if (ID_EX.readIsBranch()) {
    //     if (opcode==get_instr_encoding(Instruction::kjalr).opcode ||
    //         opcode==get_instr_encoding(Instruction::kjal).opcode) {
    //     next_pc_ = static_cast<int64_t>(program_counter_); // PC was already updated in Fetch()
    //     UpdateProgramCounter(-4);
    //     return_address_ = program_counter_ + 4;
    //     if (opcode==get_instr_encoding(Instruction::kjalr).opcode) {
    //         UpdateProgramCounter(-program_counter_ + (execution_result_));
    //     } else if (opcode==get_instr_encoding(Instruction::kjal).opcode) {
    //         UpdateProgramCounter(imm);
    //     }
    //     } else if (opcode==get_instr_encoding(Instruction::kbeq).opcode ||
    //                 opcode==get_instr_encoding(Instruction::kbne).opcode ||
    //                 opcode==get_instr_encoding(Instruction::kblt).opcode ||
    //                 opcode==get_instr_encoding(Instruction::kbge).opcode ||
    //                 opcode==get_instr_encoding(Instruction::kbltu).opcode ||
    //                 opcode==get_instr_encoding(Instruction::kbgeu).opcode) {
    //     switch (funct3) {
    //         case 0b000: {// BEQ
    //         branch_flag_ = (execution_result_==0);
    //         break;
    //         }
    //         case 0b001: {// BNE
    //         branch_flag_ = (execution_result_!=0);
    //         break;
    //         }
    //         case 0b100: {// BLT
    //         branch_flag_ = (execution_result_==1);
    //         break;
    //         }
    //         case 0b101: {// BGE
    //         branch_flag_ = (execution_result_==0);
    //         break;
    //         }
    //         case 0b110: {// BLTU
    //         branch_flag_ = (execution_result_==1);
    //         break;
    //         }
    //         case 0b111: {// BGEU
    //         branch_flag_ = (execution_result_==0);
    //         break;
    //         }
    //     }

    //     }



    // }

    // if (branch_flag_ && opcode==0b1100011) {
    //     UpdateProgramCounter(-4);
    //     UpdateProgramCounter(imm);
    // }


    if (opcode==get_instr_encoding(Instruction::kauipc).opcode) { // AUIPC
        execution_result_ = static_cast<int64_t>(program_counter_) - 4 + (imm << 12);
    }
    EX_MEM.modifyExecutionResult(execution_result_);
    EX_MEM.modifyNextPC(ID_EX.readNextPC());
    if(EX_MEM.readIsBranch())EX_MEM.modifyExecutionResult(ID_EX.readExecutionResult());
}

void RVSSVM_DYNAMIC::ExecuteFloat() {
    uint8_t opcode = ID_EX.readOpcode();
    uint8_t funct3 = ID_EX.readFunct3();
    uint8_t funct7 = ID_EX.readFunct7();
    uint8_t rm = funct3;
    uint8_t rs1 = ID_EX.readRs1();
    uint8_t rs2 = ID_EX.readRs2();
    uint8_t rs3 = ID_EX.readRs3();

    uint8_t fcsr_status = 0;

    int32_t imm = ID_EX.readImmediate();

    if (rm==0b111) {
        rm = registers_.ReadCsr(0x002);
    }

    uint64_t reg1_value = registers_.ReadFpr(rs1);
    uint64_t reg2_value = registers_.ReadFpr(rs2);
    uint64_t reg3_value = registers_.ReadFpr(rs3);

    if (funct7==0b1101000 || funct7==0b1111000 || opcode==0b0000111 || opcode==0b0100111) {
        reg1_value = registers_.ReadGpr(rs1);
    }

    if(ForwardA==10 || ForwardA==1){
        if(pickResult1)reg1_value=execResult1;
        else reg1_value=memResult1;
    }

    if(ForwardB==10 || ForwardB==1){
        if(pickResult2)reg2_value=execResult2;
        else reg2_value=memResult2;
    }

    if(ForwardC==10 || ForwardC==1){
        if(pickResult3)reg3_value=execResult3;
        else reg3_value=memResult3;
    }

    if (ID_EX.ExecuteSignal()) {
        reg2_value = static_cast<uint64_t>(static_cast<int64_t>(imm));
    }

    alu::AluOp aluOperation = ID_EX.readAluOp();
    std::tie(execution_result_, fcsr_status) = alu::Alu::fpexecute(aluOperation, reg1_value, reg2_value, reg3_value, rm);

    EX_MEM.modifyExecutionResult(execution_result_);
    EX_MEM.modifyIsBranch(ID_EX.readIsBranch());
    EX_MEM.modifyMemRead(ID_EX.MemRead());
    EX_MEM.modifyMemWrite(ID_EX.MemWrite());
    EX_MEM.modifyWriteBackSignal(ID_EX.WriteBackSignal());
    EX_MEM.modifyOpcode(ID_EX.readOpcode());
    EX_MEM.modifyFunct3(ID_EX.readFunct3());
    EX_MEM.modifyImmediate(ID_EX.readImmediate());
    EX_MEM.modifyIsFloat(ID_EX.readIsFloat());
    EX_MEM.modifyIsDouble(ID_EX.readIsDouble());
    EX_MEM.modifyIsCSR(ID_EX.readIsCSR());
    EX_MEM.modifyRd(ID_EX.readRd());
    EX_MEM.modifyFunct7(ID_EX.readFunct7());
    EX_MEM.modifyRs1(ID_EX.readRs1());
    EX_MEM.modifyRs2(ID_EX.readRs2());
    EX_MEM.modifyRs3(ID_EX.readRs3());
    EX_MEM.modifyNextPC(ID_EX.readNextPC());
    registers_.WriteCsr(0x003, fcsr_status);
}

void RVSSVM_DYNAMIC::ExecuteDouble() {
    uint8_t opcode = ID_EX.readOpcode();
    uint8_t funct3 = ID_EX.readFunct3();
    uint8_t funct7 = ID_EX.readFunct7();
    uint8_t rm = funct3;
    uint8_t rs1 = ID_EX.readRs1();
    uint8_t rs2 = ID_EX.readRs2();
    uint8_t rs3 = ID_EX.readRs3();

    uint8_t fcsr_status = 0;

    int32_t imm = ImmGenerator(ID_EX.readImmediate());

    uint64_t reg1_value = registers_.ReadFpr(rs1);
    uint64_t reg2_value = registers_.ReadFpr(rs2);
    uint64_t reg3_value = registers_.ReadFpr(rs3);

    if (funct7==0b1101001 || funct7==0b1111001 || opcode==0b0000111 || opcode==0b0100111) {
        reg1_value = registers_.ReadGpr(rs1);
    }

    if(ForwardA==10 || ForwardA==1){
        if(pickResult1)reg1_value=execResult1;
        else reg1_value=memResult1;
    }

    if(ForwardB==10 || ForwardB==1){
        if(pickResult2)reg2_value=execResult2;
        else reg2_value=memResult2;
    }

    if(ForwardC==10 || ForwardC==1){
        if(pickResult3)reg3_value=execResult3;
        else reg3_value=memResult3;
    }

    if (ID_EX.ExecuteSignal()) {
        reg2_value = static_cast<uint64_t>(static_cast<int64_t>(imm));
    }

    alu::AluOp aluOperation = ID_EX.readAluOp();
    std::tie(execution_result_, fcsr_status) = alu::Alu::dfpexecute(aluOperation, reg1_value, reg2_value, reg3_value, rm);

    EX_MEM.modifyExecutionResult(execution_result_);
    EX_MEM.modifyIsBranch(ID_EX.readIsBranch());
    EX_MEM.modifyMemRead(ID_EX.MemRead());
    EX_MEM.modifyMemWrite(ID_EX.MemWrite());
    EX_MEM.modifyWriteBackSignal(ID_EX.WriteBackSignal());
    EX_MEM.modifyOpcode(ID_EX.readOpcode());
    EX_MEM.modifyFunct3(ID_EX.readFunct3());
    EX_MEM.modifyImmediate(ID_EX.readImmediate());
    EX_MEM.modifyIsFloat(ID_EX.readIsFloat());
    EX_MEM.modifyIsDouble(ID_EX.readIsDouble());
    EX_MEM.modifyIsCSR(ID_EX.readIsCSR());
    EX_MEM.modifyRd(ID_EX.readRd());
    EX_MEM.modifyFunct7(ID_EX.readFunct7());
    EX_MEM.modifyRs1(ID_EX.readRs1());
    EX_MEM.modifyRs2(ID_EX.readRs2());
    EX_MEM.modifyRs3(ID_EX.readRs3());
    EX_MEM.modifyNextPC(ID_EX.readNextPC());
}

void RVSSVM_DYNAMIC::ExecuteCsr() {
    uint8_t rs1 = (current_instruction_ >> 15) & 0b11111;
    uint16_t csr = (current_instruction_ >> 20) & 0xFFF;
    uint64_t csr_val = registers_.ReadCsr(csr);

    csr_target_address_ = csr;
    csr_old_value_ = csr_val;
    csr_write_val_ = registers_.ReadGpr(rs1);
    csr_uimm_ = rs1;
}

// TODO: implement writeback for syscalls
void RVSSVM_DYNAMIC::HandleSyscall() {
    uint64_t syscall_number = registers_.ReadGpr(17);
    switch (syscall_number) {
        case SYSCALL_PRINT_INT: {
            if (!globals::vm_as_backend) {
                std::cout << "[Syscall output: ";
            } else {
            std::cout << "VM_STDOUT_START";
            }
            std::cout << static_cast<int64_t>(registers_.ReadGpr(10)); // Print signed integer
            if (!globals::vm_as_backend) {
                std::cout << "]" << std::endl;
            } else {
            std::cout << "VM_STDOUT_END" << std::endl;
            }
            break;
        }
        case SYSCALL_PRINT_FLOAT: { // print float
            if (!globals::vm_as_backend) {
                std::cout << "[Syscall output: ";
            } else {
            std::cout << "VM_STDOUT_START";
            }
            float float_value;
            uint64_t raw = registers_.ReadGpr(10);
            std::memcpy(&float_value, &raw, sizeof(float_value));
            std::cout << std::setprecision(std::numeric_limits<float>::max_digits10) << float_value;
            if (!globals::vm_as_backend) {
                std::cout << "]" << std::endl;
            } else {
            std::cout << "VM_STDOUT_END" << std::endl;
            }
            break;
        }
        case SYSCALL_PRINT_DOUBLE: { // print double
            if (!globals::vm_as_backend) {
                std::cout << "[Syscall output: ";
            } else {
            std::cout << "VM_STDOUT_START";
            }
            double double_value;
            uint64_t raw = registers_.ReadGpr(10);
            std::memcpy(&double_value, &raw, sizeof(double_value));
            std::cout << std::setprecision(std::numeric_limits<double>::max_digits10) << double_value;
            if (!globals::vm_as_backend) {
                std::cout << "]" << std::endl;
            } else {
            std::cout << "VM_STDOUT_END" << std::endl;
            }
            break;
        }
        case SYSCALL_PRINT_STRING: {
            if (!globals::vm_as_backend) {
                std::cout << "[Syscall output: ";
            }
            PrintString(registers_.ReadGpr(10)); // Print string
            if (!globals::vm_as_backend) {
                std::cout << "]" << std::endl;
            }
            break;
        }
        case SYSCALL_EXIT: {
            stop_requested_ = true; // Stop the VM
            if (!globals::vm_as_backend) {
                std::cout << "VM_EXIT" << std::endl;
            }
            output_status_ = "VM_EXIT";
            std::cout << "Exited with exit code: " << registers_.ReadGpr(10) << std::endl;
            exit(0); // Exit the program
            break;
        }
        case SYSCALL_READ: { // Read
        uint64_t file_descriptor = registers_.ReadGpr(10);
        uint64_t buffer_address = registers_.ReadGpr(11);
        uint64_t length = registers_.ReadGpr(12);

        if (file_descriptor == 0) {
            // Read from stdin
            std::string input;
            {
            std::cout << "VM_STDIN_START" << std::endl;
            output_status_ = "VM_STDIN_START";
            std::unique_lock<std::mutex> lock(input_mutex_);
            input_cv_.wait(lock, [this]() {
                return !input_queue_.empty();
            });
            output_status_ = "VM_STDIN_END";
            std::cout << "VM_STDIN_END" << std::endl;

            input = input_queue_.front();
            input_queue_.pop();
            }


            std::vector<uint8_t> old_bytes_vec(length, 0);
            std::vector<uint8_t> new_bytes_vec(length, 0);

            for (size_t i = 0; i < length; ++i) {
            old_bytes_vec[i] = memory_controller_.ReadByte(buffer_address + i);
            }
            
            for (size_t i = 0; i < input.size() && i < length; ++i) {
            memory_controller_.WriteByte(buffer_address + i, static_cast<uint8_t>(input[i]));
            }
            if (input.size() < length) {
            memory_controller_.WriteByte(buffer_address + input.size(), '\0');
            }

            for (size_t i = 0; i < length; ++i) {
            new_bytes_vec[i] = memory_controller_.ReadByte(buffer_address + i);
            }

            current_delta_.memory_changes.push_back({
            buffer_address,
            old_bytes_vec,
            new_bytes_vec
            });

            uint64_t old_reg = registers_.ReadGpr(10);
            unsigned int reg_index = 10;
            unsigned int reg_type = 0; // 0 for GPR, 1 for CSR, 2 for FPR
            uint64_t new_reg = std::min(static_cast<uint64_t>(length), static_cast<uint64_t>(input.size()));
            registers_.WriteGpr(10, new_reg);
            if (old_reg != new_reg) {
            current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
            }

        } else {
            std::cerr << "Unsupported file descriptor: " << file_descriptor << std::endl;
        }
        break;
        }
        case SYSCALL_WRITE: { // Write
            uint64_t file_descriptor = registers_.ReadGpr(10);
            uint64_t buffer_address = registers_.ReadGpr(11);
            uint64_t length = registers_.ReadGpr(12);

            if (file_descriptor == 1) { // stdout
            std::cout << "VM_STDOUT_START";
            output_status_ = "VM_STDOUT_START";
            uint64_t bytes_printed = 0;
            for (uint64_t i = 0; i < length; ++i) {
                char c = memory_controller_.ReadByte(buffer_address + i);
                // if (c == '\0') {
                //     break;
                // }
                std::cout << c;
                bytes_printed++;
            }
            std::cout << std::flush;
            output_status_ = "VM_STDOUT_END";
            std::cout << "VM_STDOUT_END" << std::endl;

            uint64_t old_reg = registers_.ReadGpr(10);
            unsigned int reg_index = 10;
            unsigned int reg_type = 0; // 0 for GPR, 1 for CSR, 2 for FPR
            uint64_t new_reg = std::min(static_cast<uint64_t>(length), bytes_printed);
            registers_.WriteGpr(10, new_reg);
            if (old_reg != new_reg) {
                current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
            }
            } else {
                std::cerr << "Unsupported file descriptor: " << file_descriptor << std::endl;
            }
            break;
        }
        default: {
        std::cerr << "Unknown syscall number: " << syscall_number << std::endl;
        break;
        }
    }
}

void RVSSVM_DYNAMIC::WriteMemory() {
    uint8_t opcode = EX_MEM.readOpcode();
    uint8_t rs2 = EX_MEM.readRs2();
    uint8_t funct3 = EX_MEM.readFunct3();

    if (opcode == 0b1110011 && funct3 == 0b000) {
        return;
    }

    if (EX_MEM.readIsFloat()) { // RV64 F
        WriteMemoryFloat();
        return;
    } else if (EX_MEM.readIsDouble()) {
        WriteMemoryDouble();
        return;
    }

    int64_t executionResult = EX_MEM.readExecutionResult();
    if (EX_MEM.MemRead()) {
        switch (funct3) {
        case 0b000: {// LB
            memory_result_ = static_cast<int8_t>(memory_controller_.ReadByte(executionResult));
            break;
        }
        case 0b001: {// LH
            memory_result_ = static_cast<int16_t>(memory_controller_.ReadHalfWord(executionResult));
            break;
        }
        case 0b010: {// LW
            memory_result_ = static_cast<int32_t>(memory_controller_.ReadWord(executionResult));
            break;
        }
        case 0b011: {// LD
            memory_result_ = memory_controller_.ReadDoubleWord(executionResult);
            break;
        }
        case 0b100: {// LBU
            memory_result_ = static_cast<uint8_t>(memory_controller_.ReadByte(executionResult));
            break;
        }
        case 0b101: {// LHU
            memory_result_ = static_cast<uint16_t>(memory_controller_.ReadHalfWord(executionResult));
            break;
        }
        case 0b110: {// LWU
            memory_result_ = static_cast<uint32_t>(memory_controller_.ReadWord(executionResult));
            break;
        }
        }
    }
    MEM_WB.modifyMemoryResult(memory_result_);
    MEM_WB.modifyExecutionResult(executionResult);
    MEM_WB.modifyMemRead(EX_MEM.MemRead());
    MEM_WB.modifyMemWrite(EX_MEM.MemWrite());
    MEM_WB.modifyWriteBackSignal(EX_MEM.WriteBackSignal());
    MEM_WB.modifyOpcode(EX_MEM.readOpcode());
    MEM_WB.modifyRd(EX_MEM.readRd());
    MEM_WB.modifyRs2(EX_MEM.readRs2());
    MEM_WB.modifyFunct3(EX_MEM.readFunct3());
    MEM_WB.modifyImmediate(EX_MEM.readImmediate());
    MEM_WB.modifyIsFloat(EX_MEM.readIsFloat());
    MEM_WB.modifyIsDouble(EX_MEM.readIsDouble());
    MEM_WB.modifyIsCSR(EX_MEM.readIsCSR());
    MEM_WB.modifyIsBranch(EX_MEM.readIsBranch());
    MEM_WB.modifyFunct7(EX_MEM.readFunct7());
    MEM_WB.modifyRs1(EX_MEM.readRs1());
    MEM_WB.modifyRs3(EX_MEM.readRs3());
    MEM_WB.modifyNextPC(EX_MEM.readNextPC());
    uint64_t addr = 0;
    std::vector<uint8_t> old_bytes_vec;
    std::vector<uint8_t> new_bytes_vec;

    // TODO: use direct read to read memory for undo/redo functionality, i.e. ReadByte -> ReadByte_d


    if (EX_MEM.MemWrite()) {
        switch (funct3) {
        case 0b000: {// SB
            addr = executionResult;
            old_bytes_vec.push_back(memory_controller_.ReadByte(addr));
            memory_controller_.WriteByte(execution_result_, registers_.ReadGpr(rs2) & 0xFF);
            new_bytes_vec.push_back(memory_controller_.ReadByte(addr));
            break;
        }
        case 0b001: {// SH
            addr = executionResult;
            for (size_t i = 0; i < 2; ++i) {
            old_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
            }
            memory_controller_.WriteHalfWord(executionResult, registers_.ReadGpr(rs2) & 0xFFFF);
            for (size_t i = 0; i < 2; ++i) {
            new_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
            }
            break;
        }
        case 0b010: {// SW
            addr = executionResult;
            for (size_t i = 0; i < 4; ++i) {
            old_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
            }
            memory_controller_.WriteWord(executionResult, registers_.ReadGpr(rs2) & 0xFFFFFFFF);
            for (size_t i = 0; i < 4; ++i) {
            new_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
            }
            break;
        }
        case 0b011: {// SD
            addr = executionResult;
            for (size_t i = 0; i < 8; ++i) {
            old_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
            }
            memory_controller_.WriteDoubleWord(executionResult, registers_.ReadGpr(rs2) & 0xFFFFFFFFFFFFFFFF);
            for (size_t i = 0; i < 8; ++i) {
            new_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
            }
            break;
        }
        }
    }

    if (old_bytes_vec != new_bytes_vec) {
        current_delta_.memory_changes.push_back({
        addr,
        old_bytes_vec,
        new_bytes_vec
        });
    }
}

void RVSSVM_DYNAMIC::WriteMemoryFloat() {
    uint8_t rs2 = EX_MEM.readRs2();

    if (EX_MEM.MemRead()) { // FLW
        memory_result_ = memory_controller_.ReadWord(EX_MEM.readExecutionResult());
    }

    uint64_t addr = 0;
    std::vector<uint8_t> old_bytes_vec;
    std::vector<uint8_t> new_bytes_vec;

    if (EX_MEM.MemWrite()) { // FSW
        addr = execution_result_;
        for (size_t i = 0; i < 4; ++i) {
        old_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
        }
        uint32_t val = registers_.ReadFpr(rs2) & 0xFFFFFFFF;
        memory_controller_.WriteWord(execution_result_, val);
        // new_bytes_vec.push_back(memory_controller_.ReadByte(addr));
        for (size_t i = 0; i < 4; ++i) {
        new_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
        }
    }

    if (old_bytes_vec!=new_bytes_vec) {
        current_delta_.memory_changes.push_back({addr, old_bytes_vec, new_bytes_vec});
    }

    MEM_WB.modifyMemoryResult(memory_result_);
    MEM_WB.modifyExecutionResult(EX_MEM.readExecutionResult());
    MEM_WB.modifyMemRead(EX_MEM.MemRead());
    MEM_WB.modifyMemWrite(EX_MEM.MemWrite());
    MEM_WB.modifyWriteBackSignal(EX_MEM.WriteBackSignal());
    MEM_WB.modifyOpcode(EX_MEM.readOpcode());
    MEM_WB.modifyRd(EX_MEM.readRd());
    MEM_WB.modifyRs2(EX_MEM.readRs2());
    MEM_WB.modifyFunct3(EX_MEM.readFunct3());
    MEM_WB.modifyImmediate(EX_MEM.readImmediate());
    MEM_WB.modifyIsFloat(EX_MEM.readIsFloat());
    MEM_WB.modifyIsDouble(EX_MEM.readIsDouble());
    MEM_WB.modifyIsCSR(EX_MEM.readIsCSR());
    MEM_WB.modifyIsBranch(EX_MEM.readIsBranch());
    MEM_WB.modifyFunct7(EX_MEM.readFunct7());
    MEM_WB.modifyRs1(EX_MEM.readRs1());
    MEM_WB.modifyRs3(EX_MEM.readRs3());
    MEM_WB.modifyNextPC(EX_MEM.readNextPC());
}

void RVSSVM_DYNAMIC::WriteMemoryDouble() {
    uint8_t rs2 = EX_MEM.readRs2();

    if (EX_MEM.MemRead()) {// FLD
        memory_result_ = memory_controller_.ReadDoubleWord(EX_MEM.readExecutionResult());
    }

    uint64_t addr = 0;
    std::vector<uint8_t> old_bytes_vec;
    std::vector<uint8_t> new_bytes_vec;

    if (EX_MEM.MemWrite()) {// FSD
        addr = EX_MEM.readExecutionResult();
        for (size_t i = 0; i < 8; ++i) {
        old_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
        }
        memory_controller_.WriteDoubleWord(execution_result_, registers_.ReadFpr(rs2));
        for (size_t i = 0; i < 8; ++i) {
        new_bytes_vec.push_back(memory_controller_.ReadByte(addr + i));
        }
    }

    if (old_bytes_vec!=new_bytes_vec) {
        current_delta_.memory_changes.push_back({addr, old_bytes_vec, new_bytes_vec});
    }

    MEM_WB.modifyMemoryResult(memory_result_);
    MEM_WB.modifyExecutionResult(EX_MEM.readExecutionResult());
    MEM_WB.modifyMemRead(EX_MEM.MemRead());
    MEM_WB.modifyMemWrite(EX_MEM.MemWrite());
    MEM_WB.modifyWriteBackSignal(EX_MEM.WriteBackSignal());
    MEM_WB.modifyOpcode(EX_MEM.readOpcode());
    MEM_WB.modifyRd(EX_MEM.readRd());
    MEM_WB.modifyRs2(EX_MEM.readRs2());
    MEM_WB.modifyFunct3(EX_MEM.readFunct3());
    MEM_WB.modifyImmediate(EX_MEM.readImmediate());
    MEM_WB.modifyIsFloat(EX_MEM.readIsFloat());
    MEM_WB.modifyIsDouble(EX_MEM.readIsDouble());
    MEM_WB.modifyIsCSR(EX_MEM.readIsCSR());
    MEM_WB.modifyIsBranch(EX_MEM.readIsBranch());
    MEM_WB.modifyFunct7(EX_MEM.readFunct7());
    MEM_WB.modifyRs1(EX_MEM.readRs1());
    MEM_WB.modifyRs3(EX_MEM.readRs3());
    MEM_WB.modifyNextPC(EX_MEM.readNextPC());
}

void RVSSVM_DYNAMIC::WriteBack() {
    uint8_t opcode = MEM_WB.readOpcode();
    uint8_t funct3 = MEM_WB.readOpcode();
    uint8_t rd = MEM_WB.readRd();
    int32_t imm = MEM_WB.readImmediate();
    int64_t memoryResult = MEM_WB.readMemoryResult();
    int64_t executionResult = MEM_WB.readExecutionResult();

    if (opcode == get_instr_encoding(Instruction::kecall).opcode &&
        funct3 == get_instr_encoding(Instruction::kecall).funct3) { // ecall
        return;
    }

    if (MEM_WB.readIsFloat()) { // RV64 F
        WriteBackFloat();
        return;
    } else if (MEM_WB.readIsDouble()) {
        WriteBackDouble();
        return;
    } else if (opcode==0b1110011) { // CSR opcode
        WriteBackCsr();
        return;
    }

    uint64_t old_reg = registers_.ReadGpr(rd);
    unsigned int reg_index = rd;
    unsigned int reg_type = 0; // 0 for GPR, 1 for CSR, 2 for FPR


    if (MEM_WB.WriteBackSignal()) {
        switch (opcode) {
        case get_instr_encoding(Instruction::kRtype).opcode: /* R-Type */
        case get_instr_encoding(Instruction::kItype).opcode: /* I-Type */
        case get_instr_encoding(Instruction::kauipc).opcode: /* AUIPC */ {
            registers_.WriteGpr(rd, executionResult);
            break;
        }
        case get_instr_encoding(Instruction::kLoadType).opcode: /* Load */ {
            registers_.WriteGpr(rd, memoryResult);
            break;
        }
        case get_instr_encoding(Instruction::kjalr).opcode: /* JALR */
        case get_instr_encoding(Instruction::kjal).opcode: /* JAL */ {
            registers_.WriteGpr(rd, MEM_WB.readNextPC());
            break;
        }
        case get_instr_encoding(Instruction::klui).opcode: /* LUI */ {
            registers_.WriteGpr(rd, (imm << 12));
            break;
        }
        default: break;
        }
    }

    if (opcode==get_instr_encoding(Instruction::kjal).opcode) /* JAL */ {
        // Updated in Execute()
    }
    if (opcode==get_instr_encoding(Instruction::kjalr).opcode) /* JALR */ {
        // registers_.WriteGpr(rd, return_address_); // Write back to rs1
        // Updated in Execute()
    }

    uint64_t new_reg = registers_.ReadGpr(rd);
    if (old_reg!=new_reg) {
        current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
    }

}

void RVSSVM_DYNAMIC::WriteBackFloat() {
    uint8_t opcode = MEM_WB.readOpcode();
    uint8_t funct7 = MEM_WB.readFunct7();
    uint8_t rd = MEM_WB.readRd();

    uint64_t old_reg = 0;
    unsigned int reg_index = rd;
    unsigned int reg_type = 2; // 0 for GPR, 1 for CSR, 2 for FPR
    uint64_t new_reg = 0;

    if (MEM_WB.WriteBackSignal()) {
        switch(funct7) {
        // write to GPR
        case get_instr_encoding(Instruction::kfle_s).funct7: // f(eq|lt|le).s
        case get_instr_encoding(Instruction::kfcvt_w_s).funct7: // fcvt.(w|wu|l|lu).s
        case get_instr_encoding(Instruction::kfmv_x_w).funct7: // fmv.x.w , fclass.s
        {
            old_reg = registers_.ReadGpr(rd);
            registers_.WriteGpr(rd, MEM_WB.readExecutionResult());
            new_reg = MEM_WB.readExecutionResult();
            reg_type = 0; // GPR
            break;
        }

        // write to FPR
        default: {
            switch (opcode) {
            case get_instr_encoding(Instruction::kflw).opcode: {
                old_reg = registers_.ReadFpr(rd);
                registers_.WriteFpr(rd, MEM_WB.readMemoryResult());
                new_reg = MEM_WB.readMemoryResult();
                reg_type = 2; // FPR
                break;
            }

            default: {
                old_reg = registers_.ReadFpr(rd);
                registers_.WriteFpr(rd, MEM_WB.readExecutionResult());
                new_reg = MEM_WB.readExecutionResult();
                reg_type = 2; // FPR
                break;
            }
            }
        }
        }

        // // write to GPR
        // if (funct7==0b1010000
        //     || funct7==0b1100000
        //     || funct7==0b1110000) { // f(eq|lt|le).s, fcvt.(w|wu|l|lu).s
        //   old_reg = registers_.ReadGpr(rd);
        //   registers_.WriteGpr(rd, execution_result_);
        //   new_reg = execution_result_;
        //   reg_type = 0; // GPR

        // }
        // // write to FPR
        // else if (opcode==get_instr_encoding(Instruction::kflw).opcode) {
        //   old_reg = registers_.ReadFpr(rd);
        //   registers_.WriteFpr(rd, memory_result_);
        //   new_reg = memory_result_;
        //   reg_type = 2; // FPR
        // } else {
        //   old_reg = registers_.ReadFpr(rd);
        //   registers_.WriteFpr(rd, execution_result_);
        //   new_reg = execution_result_;
        //   reg_type = 2; // FPR
        // }
    }

    if (old_reg!=new_reg) {
        current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
    }
}

void RVSSVM_DYNAMIC::WriteBackDouble() {
    uint8_t opcode = MEM_WB.readOpcode();
    uint8_t funct7 = MEM_WB.readFunct7();
    uint8_t rd = MEM_WB.readRd();

    uint64_t old_reg = 0;
    unsigned int reg_index = rd;
    unsigned int reg_type = 2; // 0 for GPR, 1 for CSR, 2 for FPR
    uint64_t new_reg = 0;

    if (MEM_WB.WriteBackSignal()) {
        // write to GPR
        if (funct7==0b1010001
            || funct7==0b1100001
            || funct7==0b1110001) { // f(eq|lt|le).d, fcvt.(w|wu|l|lu).d
        old_reg = registers_.ReadGpr(rd);
        registers_.WriteGpr(rd, MEM_WB.readExecutionResult());
        new_reg = MEM_WB.readExecutionResult();
        reg_type = 0; // GPR
        }
        // write to FPR
        else if (opcode==0b0000111) {
        old_reg = registers_.ReadFpr(rd);
        registers_.WriteFpr(rd, MEM_WB.readMemoryResult());
        new_reg = MEM_WB.readMemoryResult();
        reg_type = 2; // FPR
        } else {
        old_reg = registers_.ReadFpr(rd);
        registers_.WriteFpr(rd, MEM_WB.readExecutionResult());
        new_reg = MEM_WB.readExecutionResult();
        reg_type = 2; // FPR
        }
    }

    if (old_reg!=new_reg) {
        current_delta_.register_changes.push_back({reg_index, reg_type, old_reg, new_reg});
    }

    
    return;
}

void RVSSVM_DYNAMIC::WriteBackCsr() {
    uint8_t rd = (current_instruction_ >> 7) & 0b11111;
    uint8_t funct3 = (current_instruction_ >> 12) & 0b111;

    switch (funct3) {
        case get_instr_encoding(Instruction::kcsrrw).funct3: { // CSRRW
        registers_.WriteGpr(rd, csr_old_value_);
        registers_.WriteCsr(csr_target_address_, csr_write_val_);
        break;
        }
        case get_instr_encoding(Instruction::kcsrrs).funct3: { // CSRRS
        registers_.WriteGpr(rd, csr_old_value_);
        if (csr_write_val_!=0) {
            registers_.WriteCsr(csr_target_address_, csr_old_value_ | csr_write_val_);
        }
        break;
        }
        case get_instr_encoding(Instruction::kcsrrc).funct3: { // CSRRC
        registers_.WriteGpr(rd, csr_old_value_);
        if (csr_write_val_!=0) {
            registers_.WriteCsr(csr_target_address_, csr_old_value_ & ~csr_write_val_);
        }
        break;
        }
        case get_instr_encoding(Instruction::kcsrrwi).funct3: { // CSRRWI
        registers_.WriteGpr(rd, csr_old_value_);
        registers_.WriteCsr(csr_target_address_, csr_uimm_);
        break;
        }
        case get_instr_encoding(Instruction::kcsrrsi).funct3: { // CSRRSI
        registers_.WriteGpr(rd, csr_old_value_);
        if (csr_uimm_!=0) {
            registers_.WriteCsr(csr_target_address_, csr_old_value_ | csr_uimm_);
        }
        break;
        }
        case get_instr_encoding(Instruction::kcsrrci).funct3: { // CSRRCI
        registers_.WriteGpr(rd, csr_old_value_);
        if (csr_uimm_!=0) {
            registers_.WriteCsr(csr_target_address_, csr_old_value_ & ~csr_uimm_);
        }
        break;
        }
    }

}

void RVSSVM_DYNAMIC::Run() {
    ClearStop();
    uint64_t instruction_executed = 0;
    while (!stop_requested_ && program_counter_ < program_size_) {
        if (instruction_executed > vm_config::config.getInstructionExecutionLimit())break;
        initializeForwardControlSignals();
        HazardDetectionUnit();
        ForwardUnit();
        DecodeHazard();

        WriteBack();
        WriteMemory();
        if(!stall){
            Execute();
            if(!DStall){
                Decode();
                Fetch();
                if(flush){
                    UpdateProgramCounter(-program_counter_);
                    UpdateProgramCounter(correctPC);
                    IF_ID.fetchInstruction(0);NumStalls++;
                }
            }
            else{
                NumStalls++;
                ID_EX.modifyWriteBackSignal(0);
                ID_EX.modifyMemRead(0);
                ID_EX.modifyMemWrite(0);
            }
        }
        else{
            NumStalls++;
            EX_MEM.modifyWriteBackSignal(0);
            EX_MEM.modifyMemRead(0);
            EX_MEM.modifyMemWrite(0);
        }
        instructions_retired_++;
        instruction_executed++;
        cycle_s_++;
    }
    while(!checkProcessOver()){
        if (instruction_executed > vm_config::config.getInstructionExecutionLimit())break;
        initializeForwardControlSignals();
        HazardDetectionUnit();
        ForwardUnit();
        DecodeHazard();

        WriteBack();
        WriteMemory();
        if(!stall){
            Execute();
            if(!DStall){
                Decode();
                Fetch();
                if(flush){
                    UpdateProgramCounter(-program_counter_);
                    UpdateProgramCounter(correctPC);
                    IF_ID.fetchInstruction(0);NumStalls++;
                }
            }
            else{
                NumStalls++;
                ID_EX.modifyWriteBackSignal(0);
                ID_EX.modifyMemRead(0);
                ID_EX.modifyMemWrite(0);
            }
        }
        else{
            NumStalls++;
            EX_MEM.modifyWriteBackSignal(0);
            EX_MEM.modifyMemRead(0);
            EX_MEM.modifyMemWrite(0);
        }
        instructions_retired_++;
        instruction_executed++;
        cycle_s_++;
    }
    std::cout << "General Purpose registers \n";
    for (int i = 0; i < 32; i++) {
        std::cout << "r" << i << ": "
                << static_cast<int64_t>(registers_.ReadGpr(i)) << "  ";
        if ((i + 1) % 8 == 0) {
            std::cout << "\n";
        }
    }
    std::cout << "\nFloating point registers \n";
    for (int i = 0; i < 32; i++) {
        float a;
        uint64_t b = registers_.ReadFpr(i);
        std::memcpy(&a,&b,sizeof(float));
        std::cout << "f" << i << ": "
                << a << "  ";
        if ((i + 1) % 8 == 0) {
            std::cout << "\n";
        }
    }
    std::cout<<"Total Cycles: "<<cycle_s_<<std::endl;
    std::cout<<"Stalls required: "<<NumStalls<<std::endl;
    if (program_counter_ >= program_size_) {
        std::cout << "VM_PROGRAM_END" << std::endl;
        output_status_ = "VM_PROGRAM_END";
    }
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

void RVSSVM_DYNAMIC::DebugRun() {
    ClearStop();
    uint64_t instruction_executed = 0;
    while (!stop_requested_ && program_counter_ < program_size_) {
        if (instruction_executed > vm_config::config.getInstructionExecutionLimit())
        break;
        current_delta_.old_pc = program_counter_;
        if (std::find(breakpoints_.begin(), breakpoints_.end(), program_counter_) == breakpoints_.end()) {
        WriteBack();
        WriteMemory();
        Execute();
        Decode();
        Fetch();
        instructions_retired_++;
        instruction_executed++;
        cycle_s_++;

        current_delta_.new_pc = program_counter_;
        // history_.push(current_delta_);
        undo_stack_.push(current_delta_);
        while (!redo_stack_.empty()) {
            redo_stack_.pop();
        }
        current_delta_ = StepDelta();
        if (program_counter_ < program_size_) {
            std::cout << "VM_STEP_COMPLETED" << std::endl;
            output_status_ = "VM_STEP_COMPLETED";
        } else if (program_counter_ >= program_size_) {
            std::cout << "VM_LAST_INSTRUCTION_STEPPED" << std::endl;
            output_status_ = "VM_LAST_INSTRUCTION_STEPPED";
        }
        DumpRegisters(globals::registers_dump_file_path, registers_);
        DumpState(globals::vm_state_dump_file_path);

        unsigned int delay_ms = vm_config::config.getRunStepDelay();
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        
        } else {
        std::cout << "VM_BREAKPOINT_HIT " << program_counter_ << std::endl;
        output_status_ = "VM_BREAKPOINT_HIT";
        break;
        }
    }
    if (program_counter_ >= program_size_) {
        std::cout << "VM_PROGRAM_END" << std::endl;
        output_status_ = "VM_PROGRAM_END";
    }
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

void RVSSVM_DYNAMIC::Step() {
    current_delta_.old_pc = program_counter_;
    current_delta_.pipeLineSnapShot.old_IF_ID = IF_ID;
    current_delta_.pipeLineSnapShot.old_ID_EX = ID_EX;
    current_delta_.pipeLineSnapShot.old_EX_MEM = EX_MEM;
    current_delta_.pipeLineSnapShot.old_MEM_WB = MEM_WB;
    current_delta_.NumStalls.oldStalls = NumStalls;
    if (!checkProcessOver() || (program_size_ > 0 && program_counter_ == 0)) {
        initializeForwardControlSignals();
        HazardDetectionUnit();
        ForwardUnit();
        DecodeHazard();

        WriteBack();
        WriteMemory();
        if(!stall){
            Execute();
            if(!DStall){
                Decode();
                Fetch();
                if(flush){
                    UpdateProgramCounter(-program_counter_);
                    UpdateProgramCounter(correctPC);
                    IF_ID.fetchInstruction(0);NumStalls++;
                }
            }
            else{
                NumStalls++;
                ID_EX.modifyWriteBackSignal(0);
                ID_EX.modifyMemRead(0);
                ID_EX.modifyMemWrite(0);
            }
        }
        else{
            NumStalls++;
            EX_MEM.modifyWriteBackSignal(0);
            EX_MEM.modifyMemRead(0);
            EX_MEM.modifyMemWrite(0);
        }
        instructions_retired_++;
        cycle_s_++;
        current_delta_.new_pc = program_counter_;
        current_delta_.pipeLineSnapShot.new_IF_ID = IF_ID;
        current_delta_.pipeLineSnapShot.new_ID_EX = ID_EX;
        current_delta_.pipeLineSnapShot.new_EX_MEM = EX_MEM;
        current_delta_.pipeLineSnapShot.new_MEM_WB = MEM_WB;
        current_delta_.NumStalls.newStalls = NumStalls;

        std::cout << "General Purpose registers \n";
        for (int i = 0; i < 32; i++) {
            std::cout << "r" << i << ": "
                    << static_cast<int64_t>(registers_.ReadGpr(i)) << "  ";
            if ((i + 1) % 8 == 0) {
                std::cout << "\n";
            }
        }
        std::cout << "\nFloating point registers \n";
        for (int i = 0; i < 32; i++) {
            std::cout << "f" << i << ": "
                    << (registers_.ReadFpr(i) & 0xFFFFFFFF) << "  ";
            if ((i + 1) % 8 == 0) {
                std::cout << "\n";
            }
        }

        undo_stack_.push(current_delta_);
        while (!redo_stack_.empty()) {
        redo_stack_.pop();
        }

        current_delta_ = StepDelta();


        if (!checkProcessOver()) {
        std::cout << "VM_STEP_COMPLETED" << std::endl;
        output_status_ = "VM_STEP_COMPLETED";
        } else if (checkProcessOver()) {
        std::cout << "VM_LAST_INSTRUCTION_STEPPED" << std::endl;
        output_status_ = "VM_LAST_INSTRUCTION_STEPPED";
        }

    } else if (checkProcessOver()) {
        std::cout << "VM_PROGRAM_END" << std::endl;
        output_status_ = "VM_PROGRAM_END";
    }
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

void RVSSVM_DYNAMIC::Undo() {
    if (undo_stack_.empty()) {
        std::cout << "VM_NO_MORE_UNDO" << std::endl;
        output_status_ = "VM_NO_MORE_UNDO";
        return;
    }

    StepDelta last = undo_stack_.top();
    undo_stack_.pop();

    // if (!history_.can_undo()) {
    //     std::cout << "Nothing to undo.\n";
    //     return;
    // }

    // StepDelta last = history_.undo();

    for (const auto &change : last.register_changes) {
        switch (change.reg_type) {
        case 0: { // GPR
            registers_.WriteGpr(change.reg_index, change.old_value);
            break;
        }
        case 1: { // CSR
            registers_.WriteCsr(change.reg_index, change.old_value);
            break;
        }
        case 2: { // FPR
            registers_.WriteFpr(change.reg_index, change.old_value);
            break;
        }
        default:std::cerr << "Invalid register type: " << change.reg_type << std::endl;
            break;
        }
    }

    for (const auto &change : last.memory_changes) {
        for (size_t i = 0; i < change.old_bytes_vec.size(); ++i) {
        memory_controller_.WriteByte(change.address + i, change.old_bytes_vec[i]);
        }
    }

    program_counter_ = last.old_pc;
    program_counter_ = last.old_pc;
    IF_ID = last.pipeLineSnapShot.old_IF_ID;
    ID_EX = last.pipeLineSnapShot.old_ID_EX;
    EX_MEM =  last.pipeLineSnapShot.old_EX_MEM;
    MEM_WB = last.pipeLineSnapShot.old_MEM_WB;
    NumStalls = last.NumStalls.oldStalls;
    instructions_retired_--;
    cycle_s_--;
    std::cout << "General Purpose registers \n";
    for (int i = 0; i < 32; i++) {
        std::cout << "r" << i << ": "
                << static_cast<int64_t>(registers_.ReadGpr(i)) << "  ";
        if ((i + 1) % 8 == 0) {
            std::cout << "\n";
        }
    }
    std::cout << "\nFloating point registers \n";
    for (int i = 0; i < 32; i++) {
        std::cout << "f" << i << ": "
                << (registers_.ReadFpr(i) & 0xFFFFFFFF) << "  ";
        if ((i + 1) % 8 == 0) {
            std::cout << "\n";
        }
    }

    redo_stack_.push(last);

    output_status_ = "VM_UNDO_COMPLETED";
    std::cout << "VM_UNDO_COMPLETED" << std::endl;

    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
}

void RVSSVM_DYNAMIC::Redo() {
    if (redo_stack_.empty()) {
        std::cout << "VM_NO_MORE_REDO" << std::endl;
        return;
    }

    StepDelta next = redo_stack_.top();
    redo_stack_.pop();

    // if (!history_.can_redo()) {
    //       std::cout << "Nothing to redo.\n";
    //       return;
    //   }

    //   StepDelta next = history_.redo();

    for (const auto &change : next.register_changes) {
        switch (change.reg_type) {
        case 0: { // GPR
            registers_.WriteGpr(change.reg_index, change.new_value);
            break;
        }
        case 1: { // CSR
            registers_.WriteCsr(change.reg_index, change.new_value);
            break;
        }
        case 2: { // FPR
            registers_.WriteFpr(change.reg_index, change.new_value);
            break;
        }
        default:std::cerr << "Invalid register type: " << change.reg_type << std::endl;
            break;
        }
    }

    for (const auto &change : next.memory_changes) {
        for (size_t i = 0; i < change.new_bytes_vec.size(); ++i) {
        memory_controller_.WriteByte(change.address + i, change.new_bytes_vec[i]);
        }
    }

    program_counter_ = next.new_pc;
    IF_ID = next.pipeLineSnapShot.new_IF_ID;
    ID_EX = next.pipeLineSnapShot.new_ID_EX;
    EX_MEM = next.pipeLineSnapShot.new_EX_MEM;
    MEM_WB = next.pipeLineSnapShot.new_MEM_WB;
    NumStalls = next.NumStalls.newStalls;
    instructions_retired_++;
    cycle_s_++;
    std::cout << "General Purpose registers \n";
    for (int i = 0; i < 32; i++) {
        std::cout << "r" << i << ": "
                << static_cast<int64_t>(registers_.ReadGpr(i)) << "  ";
        if ((i + 1) % 8 == 0) {
            std::cout << "\n";
        }
    }
    std::cout << "\nFloating point registers \n";
    for (int i = 0; i < 32; i++) {
        std::cout << "f" << i << ": "
                << (registers_.ReadFpr(i) & 0xFFFFFFFF) << "  ";
        if ((i + 1) % 8 == 0) {
            std::cout << "\n";
        }
    }
    DumpRegisters(globals::registers_dump_file_path, registers_);
    DumpState(globals::vm_state_dump_file_path);
    undo_stack_.push(next);

}

void RVSSVM_DYNAMIC::Reset() {
    program_counter_ = 0;
    instructions_retired_ = 0;
    cycle_s_ = 0;
    registers_.Reset();
    memory_controller_.Reset();
    control_unit_.Reset();
    branch_flag_ = false;
    next_pc_ = 0;
    execution_result_ = 0;
    memory_result_ = 0;

    return_address_ = 0;
    csr_target_address_ = 0;
    csr_old_value_ = 0;
    csr_write_val_ = 0;
    csr_uimm_ = 0;
    current_delta_.register_changes.clear();
    current_delta_.memory_changes.clear();
    current_delta_.old_pc = 0;
    current_delta_.new_pc = 0;
    undo_stack_ = std::stack<StepDelta>();
    redo_stack_ = std::stack<StepDelta>();
}