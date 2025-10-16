#include<bits/stdc++.h>

class pipelineRegister{
    private:
        std::string instruction;
        std::string readData1;
        std::string readData2;
        std::string immediate;
        std::string executeSignal;
        std::string memory;
        std::string writeBack;
    public:
        pipelineRegister(std::string instruction,std::string readData1,std::string readData2,std::string immediate,std::string memory,std::string executeSignal,std::string writeBack);
        void Reset();
        std::string readInstruction() const;
        std::string ReadData1() const;
        std::string ReadData2() const;
        std::string readImmediate() const;
        std::string ExecuteSignal() const;
        std::string MemorySignal() const;
        std::string WriteBackSignal() const;

        void fetchInstruction(std::string instruction);
        void modifyReadData1(std::string readData1);
        void modifyReadData2(std::string readData2);
        void modifyImmediate(std::string immediate);
        void modifyMemorySignal(std::string memory);
        void modifyexecuteSignal(std::string executeSignal);
        void modifyWriteBackSignal(std::string writeBack);
        
};

