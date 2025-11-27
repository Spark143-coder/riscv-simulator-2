#ifndef RVSS_CONTROL_UNIT_H
#define RVSS_CONTROL_UNIT_H

#include "../control_unit_base.h"


class RVSSControlUnit : public ControlUnit {
  public:
  void SetControlSignals(uint32_t instruction) override;

  alu::AluOp GetAluSignal(uint32_t instruction, bool ALUOp) override;

};

#endif