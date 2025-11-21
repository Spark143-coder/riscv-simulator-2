/**
 * File Name: command_handler.h
 * Author: Vishank Singh
 * Github: https://github.com/VishankSingh
 */
#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "vm/rvss/hazard_detection_pipelined_rvss_vm.h"
#include "vm/rvss/forwarding_pipelined_rvss_vm.h"
#include "vm/rvss/hazard_detection_pipelined_rvss_vm_2.h"
#include "vm/rvss/pipelined_rvss_vm.h"
#include "vm/rvss/rvss_vm.h"
#include "vm/rvss/static_branch_pipelined_rvss_vm.h"
#include "vm/rvss/dynamic_branch_pipelined_rvss_vm.h"

#include <vector>
#include<string>

namespace command_handler {
enum class CommandType {
  INVALID,
  MODIFY_CONFIG,
  LOAD,
  RUN,
  STOP,
  DEBUG_RUN,
  STEP,
  UNDO,
  REDO,
  RESET,
  MODIFY_REGISTER,
  GET_REGISTER,
  MODIFY_MEMORY,
  DUMP_MEMORY,
  PRINT_MEMORY,
  GET_MEMORY_POINT,
  DUMP_CACHE,
  ADD_BREAKPOINT,
  REMOVE_BREAKPOINT,
  VM_STDIN,
  EXIT
};

enum class CommandArgumentType {
  NONE,
  FILE,
  ADDRESS,
  REGISTER,
  VALUE
};

struct Command {
  CommandType type;
  std::vector<std::string> args;

  Command(CommandType type, const std::vector<std::string> &args)
      : type(type), args(args) {}

};

Command ParseCommand(const std::string &input);

void ExecuteCommand(const Command& command, VmBase* vm);

} // namespace CommandParser





#endif // COMMAND_HANDLER_H