#ifndef TRANSMUTER_UTILS_TRACE_INTERNAL_H
#define TRANSMUTER_UTILS_TRACE_INTERNAL_H

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace transmuter::utils
{

inline uint32_t currentTraceMask = 1;
inline auto lastPrintTime = std::chrono::steady_clock::now();

inline std::ofstream& getLogStream()
{
    static std::ofstream logFile("log.txt", std::ios::out | std::ios::trunc);
    return logFile;
}

inline std::string getTimestamp()
{
    auto now = std::chrono::system_clock::now();
    std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
    std::tm localTime = *std::localtime(&nowTime);

    auto delta = std::chrono::steady_clock::now() - lastPrintTime;
    lastPrintTime = std::chrono::steady_clock::now();

    std::ostringstream oss;
    oss << "[" << std::put_time(&localTime, "%H:%M:%S") << "] " << "[Δ" << std::fixed
        << std::setprecision(4) << std::chrono::duration<double>(delta).count() << "s] ";
    //<< "[Thread ID:" << std::this_thread::get_id() << "]";
    return oss.str();
}

inline void writeToLog(const std::string& output, bool isError)
{
    if (isError)
        std::cerr << output << std::endl;
    else
        std::cout << output << std::endl;

    auto& logFile = getLogStream();
    if (logFile.is_open())
    {
        logFile << output << std::endl;
        logFile.flush();
    }
}

}  // namespace transmuter::utils

#endif