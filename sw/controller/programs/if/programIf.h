#ifndef TRANSMUTER_CONTROLLER_PROGRAMS_PROGRAM_IF_H
#define TRANSMUTER_CONTROLLER_PROGRAMS_PROGRAM_IF_H
#include <3rdParty/magic_enum.hpp>

namespace transmuter::controller::programs
{

/**
 * @brief The ProgramIf is the interface that must be implemented by all programs
 * It defines the needed execution and stop methods for the program as well as a
 * state the program can be in.
 */
class ProgramIf
{
   public:
    virtual ~ProgramIf() = default;
    virtual void execute() = 0;

    void stop()
    {
        setState(ProgramState::FINISHED);
    }

   protected:
    enum class ProgramState
    {
        IDLE,
        SETUP,
        PATH_GENERATION,
        MILLING_SIMULATION,
        PAUSED,
        FINISHED,
    };

    void setState(ProgramState state)
    {
        INFO("Program entered state: " << magic_enum::enum_name(state).data());
        m_state = state;
    }

    ProgramState getState() const
    {
        return m_state;
    }

   private:
    ProgramState m_state = ProgramState::SETUP;
};
}  // namespace transmuter::controller::programs
#endif