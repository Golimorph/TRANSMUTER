#ifndef TRANSMUTER_CONTROLLER_H
#define TRANSMUTER_CONTROLLER_H

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "programIf.h"
#include "trace.h"

namespace transmuter::controller
{

/**
 * @brief The Controller class manages the registration and execution of programs.
 *
 * It maintains a catalog of program factories, allowing for dynamic instantiation
 * and execution of registered programs based on their string identifiers.
 * The reason for using a catalog of creator functions is to be resource effective
 * so that we only create instances of the programs when they are actually needed, rather than at
 * startup.
 */
class Controller
{
   public:
    using ProgramFactory = std::function<std::unique_ptr<programs::ProgramIf>()>;

    /**
     * @brief Adds a program type to the available catalog.
     * @tparam T The class type (must inherit from ProgramIf).
     * @param name The unique string identifier for this program.
     * @note Crashes the application if the name is already taken.
     */
    template <typename T>
    void add(const std::string& name)
    {
        if (m_catalog.count(name) > 0)
        {
            SHUTDOWN_ERROR("Controller: Program '" + name +
                           "' is already registered. Duplicate forbidden.");
        }

        m_catalog[name] = []() { return std::make_unique<T>(); };

        INFO("Controller: Registered program '" + name + "'.");
    }

    /**
     * @brief Instantiates and runs a registered program by name.
     * @param name The identifier provided during registration.
     * @note This is a blocking call. Crashes if the name is not found.
     */
    void run(const std::string& name)
    {
        if (m_catalog.find(name) == m_catalog.end())
        {
            SHUTDOWN_ERROR("Program '" + name + "' not found in catalog.");
        }
        auto program = m_catalog[name]();
        program->execute();
    }

   private:
    std::map<std::string, ProgramFactory> m_catalog;
};

}  // namespace transmuter::controller
#endif