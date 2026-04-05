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
        m_keepRunning.store(false);
    }

   protected:
    std::atomic<bool> m_keepRunning{true};
};
}  // namespace transmuter::controller::programs
#endif