#include<bits/stdc++.h>

class pipelineRegister{
    private:
        char instruction[100];
        char readData1[100];
        char readData2[100];
        char immediate[100];
        char executeSignal[100];
        char memory[100];
        char writeBack[100];
    public:
        pipelineRegister(char instruction[],char readData1[],char readData2[],char immediate[],char memory[],char executeSignal[],char writeBack[]);
        void Reset();
        char* readInstruction() const;
        char* ReadData1() const;
        char* ReadData2() const;
        char* readImmediate() const;
        char* ExecuteSignal() const;
        char* MemorySignal() const;
        char* WriteBackSignal() const;

        void fetchInstruction(char instruction[]);
        void modifyReadData1(char readData1[]);
        void modifyReadData2(char readData2[]);
        void modifyImmediate(char immediate[]);
        void modifyMemorySignal(char memory[]);
        void modifyexecuteSignal(char executeSignal[]);
        void modifyWriteBackSignal(char writeBack[]);
        
};

