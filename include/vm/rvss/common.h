#ifndef VM_RVSS_RVSS_COMMON_H
#define VM_RVSS_RVSS_COMMON_H

#include <vector>
#include <cstdint>
#include "vm/pipelineRegisters.h"

struct RegisterChange {
    unsigned int reg_index;
    unsigned int reg_type; // 0 for GPR, 1 for CSR, 2 for FPR
    uint64_t old_value;
    uint64_t new_value;
};

struct MemoryChange {
    uint64_t address;
    std::vector<uint8_t> old_bytes_vec;
    std::vector<uint8_t> new_bytes_vec;
};

struct StallChange{
    uint64_t oldStalls;
    uint64_t newStalls;
};

typedef struct PipelineStateSnapshot PipelineStateSnapshot;

struct PipelineStateSnapshot{
    pipelineRegister old_IF_ID;
    pipelineRegister old_ID_EX;
    pipelineRegister old_EX_MEM;
    pipelineRegister old_MEM_WB;
    pipelineRegister new_IF_ID;
    pipelineRegister new_ID_EX;
    pipelineRegister new_EX_MEM;
    pipelineRegister new_MEM_WB;
};

struct StepDelta {
    uint64_t old_pc;
    uint64_t new_pc;
    std::vector<RegisterChange> register_changes;
    std::vector<MemoryChange> memory_changes;
    PipelineStateSnapshot pipeLineSnapShot;
    StallChange NumStalls;
};

#endif