#pragma once
#include <string>
namespace transmuter::controller
{
/** Run a registered workflow without exposing program implementations to the launcher. */
void runProgram(const std::string& name);
}  // namespace transmuter::controller
