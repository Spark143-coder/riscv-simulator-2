#include<bits/stdc++.h>
#include "vm/rvss/hazard_detection_pipelined_rvss_vm.h"
#include "vm/rvss/forwarding_pipelined_rvss_vm.h"
#include "vm/rvss/hazard_detection_pipelined_rvss_vm_2.h"
#include "vm/rvss/pipelined_rvss_vm.h"
#include "vm/rvss/rvss_vm.h"
#include "vm/rvss/static_branch_pipelined_rvss_vm.h"

class pipelineRegister{
    private:
        uint32_t instruction;
        uint64_t readData1;
        uint64_t readData2;
        uint8_t rd;
        int64_t immediate;
        uint executeSignal;
        uint memRead;
        uint memWrite;
        uint writeBack;
        uint isBranch;
        uint8_t opcode;
        uint8_t funct3;
        alu::AluOp aluOperation;
        int64_t executionResult;
        int64_t memoryResult;
        uint isFloat;
        uint isDouble;
        uint isCSR;
        uint8_t rs1;
        uint8_t rs2;
        uint8_t rs3;
        uint8_t funct7;
    public:
        pipelineRegister(uint32_t instruction,uint64_t readData1,uint64_t readData2,int64_t immediate,uint memRead,uint memWrite,uint executeSignal,uint writeBack,uint8_t opcode,uint8_t funct3,uint8_t funct7,uint isFloat,uint isDouble,uint isCSR,uint isBranch,uint8_t rd);
        void Reset();
        uint32_t readInstruction() const;
        uint64_t ReadData1() const;
        uint64_t ReadData2() const;
        int64_t readImmediate() const;
        uint ExecuteSignal() const;
        uint MemRead() const;
        uint MemWrite() const;
        uint WriteBackSignal() const;
        alu::AluOp readAluOp() const;
        uint8_t readOpcode() const;
        uint8_t readFunct3() const;
        int64_t readExecutionResult() const;
        int64_t readMemoryResult() const;
        uint readIsBranch() const;
        uint readIsFloat() const;
        uint readIsDouble() const;
        uint readIsCSR() const;
        uint8_t readRd() const;
        uint8_t readRs1() const;
        uint8_t readRs2() const;
        uint8_t readRs3() const;
        uint8_t readFunct7() const;

        void fetchInstruction(uint32_t instruction);
        void modifyReadData1(uint64_t readData1);
        void modifyReadData2(uint64_t readData2);
        void modifyImmediate(int64_t immediate);
        void modifyMemRead(uint MemRead);
        void modifyMemWrite(uint MemWrite);
        void modifyexecuteSignal(uint executeSignal);
        void modifyWriteBackSignal(uint writeBack);
        void modifyAluOp(alu::AluOp aluOp);
        void modifyOpcode(uint8_t opcode);
        void modifyFunct3(uint8_t funct3);
        void modifyExecutionResult(int64_t executionResult);
        void modifyMemoryResult(int64_t memoryResult);
        void modifyIsBranch(uint isBranch);
        void modifyIsFloat(uint isFloat);
        void modifyIsDouble(uint isDouble);
        void modifyIsCSR(uint isCSR);
        void modifyRd(uint8_t Rd);
        void modifyRs1(uint8_t rs1);
        void modifyRs2(uint8_t rs2);
        void modifyRs3(uint8_t rs3);
        void modifyFunct7(uint8_t funct7);
};

extern pipelineRegister IF_ID;
extern pipelineRegister ID_EX;
extern pipelineRegister EX_MEM;
extern pipelineRegister MEM_WB;

