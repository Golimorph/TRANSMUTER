#ifndef TRANSMUTER_CONTROLLER_PROGRAMS_PROGRAM_IF_H
#define TRANSMUTER_CONTROLLER_PROGRAMS_PROGRAM_IF_H
#include <3rdParty/magic_enum.hpp>

namespace transmuter::controller::programs
{

/**
 * @brief The ProgramIf is the interface that must be implemented by all programs
 * It defines the needed execution and stop methods for the program as well as the
 * states the program can be in.
 */
class ProgramIf
{
   public:
    enum class ProgramState
    {
        IDLE,
        RUNNING,
        PAUSED,
        FINISHED,
        STOPPED,
        FAILED,
    };

    virtual ~ProgramIf() = default;
    virtual void execute() = 0;

    ProgramState getState() const
    {
        return m_state;
    }

    void stop()
    {
        setState(ProgramState::STOPPED);
    }

   protected:
    void setState(ProgramState state)
    {
        INFO("Program entered state: " << magic_enum::enum_name(state).data());
        m_state = state;
    }

   private:
    ProgramState m_state = ProgramState::IDLE;
};
}  // namespace transmuter::controller::programs
#endif
