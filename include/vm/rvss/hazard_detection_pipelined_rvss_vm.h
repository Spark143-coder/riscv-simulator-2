#ifndef HAZARD_RVSS_VM_H
#define HAZARD_RVSS_VM_H


#include "vm/vm_base.h"
#include "vm/rvss/common.h"

#include "rvss_control_unit.h"

#include <stack>
#include <vector>
#include <iostream>
#include<string>
#include <cstdint>


//Hazard detection class and its member function definitions
class RVSSVM_HAZARD : public VmBase {
public:
    RVSSControlUnit control_unit_;
    std::atomic<bool> stop_requested_ = false;


    std::stack<StepDelta> undo_stack_;
    std::stack<StepDelta> redo_stack_;

    StepDelta current_delta_;

    int64_t execution_result_{};
    int64_t memory_result_{};
    uint64_t return_address_{};

    bool branch_flag_ = false;
  int64_t next_pc_{}; // for jal, jalr,

  // CSR intermediate variables
    uint16_t csr_target_address_{};
    uint64_t csr_old_value_{};
    uint64_t csr_write_val_{};
    uint8_t csr_uimm_{};

    void Fetch();

    void Decode();

    void Execute();
    void ExecuteFloat();
    void ExecuteDouble();
    void ExecuteCsr();
    void HandleSyscall();

    void WriteMemory();
    void WriteMemoryFloat();
    void WriteMemoryDouble();

    void WriteBack();
    void WriteBackFloat();
    void WriteBackDouble();
    void WriteBackCsr();

    RVSSVM_HAZARD();
    ~RVSSVM_HAZARD();

    void Run() override;
    void DebugRun() override;
    void Step() override;
    void Undo() override;
    void Redo() override;
    void Reset() override;

    void RequestStop() override {
        stop_requested_ = true;
    }

    bool IsStopRequested() const {
        return stop_requested_;
    }
    
    void ClearStop() {
        stop_requested_ = false;
    }

    void PrintType() {
    std::cout << "rvssvm" << std::endl;
    }
};

#endif
