#include<bits/stdc++.h>
#include<vm/pipelineRegisters.h>

pipelineRegister::pipelineRegister(std::string instruction,std::string readData1,std::string readData2,std::string immediate,std::string memory,std::string executeSignal,std::string writeBack){
    this->instruction = instruction;
    this->readData1 = readData1;
    this->readData2 = readData2;
    this->immediate = immediate;
    this->memory = memory;
    this->executeSignal = executeSignal;
    this->writeBack = writeBack;
}

void pipelineRegister::Reset(){
    instruction[0] = '\0';
    readData1[0] = '\0';
    readData2[0] = '\0';
    immediate[0] = '\0';
    memory[0] = '\0';
    executeSignal[0]='\0';
    writeBack[0]='\0';
}

std::string pipelineRegister::readInstruction() const{
    return (this->instruction);
}

std::string pipelineRegister::ReadData1() const{
    return (this->readData1);
}

std::string pipelineRegister::ReadData2() const{
    return (this->readData2);
}

std::string pipelineRegister::readImmediate() const{
    return (this->immediate);
}

std::string pipelineRegister::ExecuteSignal() const{
    return (this->executeSignal);
}

std::string pipelineRegister::MemorySignal() const{
    return (this->memory);
}

std::string pipelineRegister::WriteBackSignal() const{
    return (this->writeBack);
}

void pipelineRegister::fetchInstruction(std::string instruction) {
    this->instruction = instruction;
}

void pipelineRegister::modifyReadData1(std::string readData1){
    this->readData1 = readData1;
}

void pipelineRegister::modifyReadData2(std::string readData2){
    this->readData2 = readData2;
}

void pipelineRegister::modifyImmediate(std::string immediate){
    this->immediate = immediate;
}

void pipelineRegister::modifyMemorySignal(std::string memory){
    this->memory = memory;
}

void pipelineRegister::modifyexecuteSignal(std::string executeSignal){
    this->executeSignal = executeSignal;
}

void pipelineRegister::modifyWriteBackSignal(std::string writeBack){
    this->writeBack = writeBack;
}