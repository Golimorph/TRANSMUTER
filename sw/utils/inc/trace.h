#ifndef TRANSMUTER_UTILS_TRACE_H
#define TRANSMUTER_UTILS_TRACE_H

#include <cstring>  // For strrchr
#include <iostream>
#include <sstream>
#include <thread>

#include "traceInternal.h"

// Helper to get only the filename from the full path
#define __FILENAME__                                     \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 \
                            : (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__))

#define SET_TRACE_MASK(mask) (transmuter::utils::currentTraceMask = (mask))

// Level 0: SHUTDOWN_ERROR (Always prints, then exits)
#define SHUTDOWN_ERROR(msg)                                                                    \
    {                                                                                          \
        std::stringstream ss;                                                                  \
        ss << " [SHUTDOWN_ERROR] " << transmuter::utils::getTimestamp() << "[" << __FILENAME__ \
           << ":" << __LINE__ << "] " << msg;                                                  \
        transmuter::utils::writeToLog(ss.str(), true);                                         \
        std::exit(EXIT_FAILURE);                                                               \
    }

// Level 1: INFO (Always prints)
#define INFO(msg)                                                                           \
    {                                                                                       \
        std::stringstream ss;                                                               \
        ss << " [INFO] " << transmuter::utils::getTimestamp() << "[" << __FILENAME__ << ":" \
           << __LINE__ << "] " << msg;                                                      \
        transmuter::utils::writeToLog(ss.str(), false);                                     \
    }

// Level 2+: TRACE_INFO (Prints only if bit is set in mask)
#define TRACE_INFO(bit, msg)                                                             \
    {                                                                                    \
        if (transmuter::utils::currentTraceMask & (bit))                                 \
        {                                                                                \
            std::stringstream ss;                                                        \
            ss << " [TRACE:" << bit << "] " << transmuter::utils::getTimestamp() << msg; \
            transmuter::utils::writeToLog(ss.str(), false);                              \
        }                                                                                \
    }

#endif