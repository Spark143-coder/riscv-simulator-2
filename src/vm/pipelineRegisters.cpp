#include <bits/stdc++.h>
#include "vm/pipelineRegisters.h"

pipelineRegister::pipelineRegister(){
    this-> instruction = 0;
    this-> readData1 = 0;
    this-> readData2 = 0;
    this-> rd = 0;
    this-> immediate = 0;
    this-> executeSignal = 0;
    this-> memRead = 0;
    this-> memWrite = 0;
    this-> writeBack = 0;
    this-> isBranch = 0;
    this-> opcode = 0;
    this-> funct3 = 0;
    this-> executionResult = 0;
    this-> memoryResult = 0;
    this-> isFloat = 0;
    this-> isDouble = 0;
    this-> isCSR = 0;
    this-> rs1 = 0;
    this-> rs2 = 0;
    this-> rs3 = 0;
    this-> funct7 = 0;
    this-> programCounter = 0;
}

pipelineRegister::pipelineRegister(uint32_t instruction,uint64_t readData1,uint64_t readData2,int64_t immediate,uint memRead,uint memWrite,uint executeSignal,uint writeBack,uint8_t opcode,uint8_t funct3,uint8_t funct7,uint isFloat,uint isDouble,uint isCSR,uint isBranch,uint8_t rd){
    this->instruction = instruction;
    this->readData1 = readData1;
    this->readData2 = readData2;
    this->immediate = immediate;
    this->memRead = memRead;
    this->memWrite = memWrite;
    this->executeSignal = executeSignal;
    this->writeBack = writeBack;
    this->opcode = opcode;
    this->funct3 = funct3;
    this->funct7 = funct7;
    this->isFloat = isFloat;
    this->isDouble = isDouble;
    this->isCSR = isCSR;
    this->isBranch = isBranch;
    this->rd = rd;
}

void pipelineRegister::Reset(){
    instruction = 0;
    readData1 = 0;
    readData2 = 0;
    immediate = 0;
    memWrite = 0;
    memRead = 0;
    executeSignal = 0;
    writeBack = 0;
}

uint32_t pipelineRegister::readInstruction() const{
    return (this->instruction);
}

uint64_t pipelineRegister::ReadData1() const{
    return (this->readData1);
}

uint64_t pipelineRegister::ReadData2() const{
    return (this->readData2);
}

int64_t pipelineRegister::readImmediate() const{
    return (this->immediate);
}

uint pipelineRegister::ExecuteSignal() const{
    return (this->executeSignal);
}

uint pipelineRegister::MemRead() const{
    return (this->memRead);
}

uint pipelineRegister::MemWrite() const{
    return (this->memWrite);
}

uint pipelineRegister::WriteBackSignal() const{
    return (this->writeBack);
}

alu::AluOp pipelineRegister::readAluOp() const{
    return (this->aluOperation);
}

uint8_t pipelineRegister::readOpcode() const{
    return (this->opcode);
}

uint8_t pipelineRegister::readFunct3() const{
    return (this->funct3);
}

int64_t pipelineRegister::readExecutionResult() const{
    return (this->executionResult );
}

int64_t pipelineRegister::readMemoryResult() const{
    return (this->memoryResult);
}

uint pipelineRegister::readIsBranch() const{
    return (this->isBranch);
}

uint8_t pipelineRegister::readRd() const{
    return (this->rd);
}

uint pipelineRegister::readIsFloat() const{
    return (this->isFloat);
}

uint pipelineRegister::readIsDouble() const{
    return (this->isDouble);
}

uint8_t pipelineRegister::readRs1() const{
    return (this->rs1);
}

uint8_t pipelineRegister::readRs2() const{
    return (this->rs2);
}

uint8_t pipelineRegister::readRs3() const{
    return (this->rs3);
}

uint pipelineRegister::readIsCSR() const{
    return (this->isCSR);
}

uint8_t pipelineRegister::readFunct7() const{
    return (this->funct7);
}

uint64_t pipelineRegister::readProgramCounter() const{
    return (this->programCounter);
}

int64_t pipelineRegister::readNextPC() const{
    return (this->nextPC);
}

void pipelineRegister::fetchInstruction(uint32_t instruction) {
    this->instruction = instruction;
}

void pipelineRegister::modifyReadData1(uint64_t readData1){
    this->readData1 = readData1;
}

void pipelineRegister::modifyReadData2(uint64_t readData2){
    this->readData2 = readData2;
}

void pipelineRegister::modifyImmediate(int64_t immediate){
    this->immediate = immediate;
}

void pipelineRegister::modifyMemRead(uint MemRead){
    this->memRead = MemRead;
}

void pipelineRegister::modifyMemWrite(uint MemWrite){
    this->memWrite = MemWrite;
}

void pipelineRegister::modifyexecuteSignal(uint executeSignal){
    this->executeSignal = executeSignal;
}

void pipelineRegister::modifyWriteBackSignal(uint writeBack){
    this->writeBack = writeBack;
}

void pipelineRegister::modifyAluOp(alu::AluOp aluOp){
    this->aluOperation = aluOp;
}

void pipelineRegister::modifyOpcode(uint8_t opcode){
    this->opcode = opcode;
}

void pipelineRegister::modifyFunct3(uint8_t funct3){
    this->funct3 = funct3;
}

void pipelineRegister::modifyExecutionResult(int64_t executionResult){
    this->executionResult = executionResult;
}

void pipelineRegister::modifyMemoryResult(int64_t memoryResult){
    this->memoryResult = memoryResult;
}

void pipelineRegister::modifyIsBranch(uint isBranch){
    this->isBranch = isBranch;
}

void pipelineRegister::modifyIsFloat(uint isFloat){
    this->isFloat = isFloat;
}

void pipelineRegister::modifyIsDouble(uint isDouble){
    this->isDouble = isDouble;
}

void pipelineRegister::modifyIsCSR(uint isCSR){
    this->isCSR = isCSR;
}

void pipelineRegister::modifyRd(uint8_t rd){
    this->rd = rd;
}

void pipelineRegister::modifyRs1(uint8_t rs1){
    this->rs1 = rs1;
}

void pipelineRegister::modifyRs3(uint8_t rs3){
    this->rs3 = rs3;
}

void pipelineRegister::modifyRs2(uint8_t rs2){
    this->rs2 = rs2;
}

void pipelineRegister::modifyFunct7(uint8_t funct7){
    this->funct7 = funct7;
}

void pipelineRegister::modifyProgramCounter(uint64_t pc){
    this->programCounter = pc;
}

void pipelineRegister::modifyNextPC(int64_t nextPC){
    this->nextPC = nextPC;
}

pipelineRegister IF_ID(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
pipelineRegister ID_EX(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
pipelineRegister EX_MEM(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);
pipelineRegister MEM_WB(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);