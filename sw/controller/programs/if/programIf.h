#ifndef TRANSMUTER_CONTROLLER_PROGRAMS_PROGRAM_IF_H
#define TRANSMUTER_CONTROLLER_PROGRAMS_PROGRAM_IF_H
#include <3rdParty/magic_enum.hpp>
#include <atomic>

#include "trace.h"

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
        ROUGH_CUTTING,
        FINE_CUTTING,
        FINISHED,
        STOPPED,
    };

    virtual ~ProgramIf() = default;
    virtual void execute() = 0;

    ProgramState getState() const
    {
        return m_state.load();
    }

    /** Request a stop; execute() records the STOPPED transition in its event loop. */
    void stop()
    {
        m_stopRequested.store(true);
    }

   protected:
    bool isStopRequested() const
    {
        return m_stopRequested.load();
    }

    std::atomic<bool>& stopSignal()
    {
        return m_stopRequested;
    }

    void setState(ProgramState state)
    {
        INFO("Program entered state: " << magic_enum::enum_name(state).data());
        m_state.store(state);
    }

   private:
    std::atomic<ProgramState> m_state{ProgramState::IDLE};
    std::atomic<bool> m_stopRequested{false};
};
}  // namespace transmuter::controller::programs
#endif
