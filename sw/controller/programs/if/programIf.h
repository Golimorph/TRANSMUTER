#ifndef TRANSMUTER_CONTROLLER_PROGRAMS_PROGRAM_IF_H
#define TRANSMUTER_CONTROLLER_PROGRAMS_PROGRAM_IF_H

namespace transmuter::controller::programs
{

class ProgramIf
{
   public:
    virtual ~ProgramIf() = default;
    virtual void execute() = 0;
    void stop()
    {
        m_state = ProgramState::FINISHED;
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