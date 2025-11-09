#ifndef VM_RVSS_RVSS_COMMON_H
#define VM_RVSS_RVSS_COMMON_H

#include <vector>
#include <cstdint>

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

struct StepDelta {
    uint64_t old_pc;
    uint64_t new_pc;
    std::vector<RegisterChange> register_changes;
    std::vector<MemoryChange> memory_changes;
};

#endif