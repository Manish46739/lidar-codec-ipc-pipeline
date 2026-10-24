#pragma once

#include <iostream>
#include <mutex>
#include <string>

namespace lidar::util {

enum class LogLevel { info, warning, error };

inline void log(LogLevel level, const std::string& message) {
    static std::mutex output_mutex;
    const char* label = level == LogLevel::error ? "ERROR" :
                        level == LogLevel::warning ? "WARN" : "INFO";
    std::lock_guard<std::mutex> lock(output_mutex);
    std::ostream& output = level == LogLevel::error ? std::cerr : std::cout;
    output << '[' << label << "] " << message << '\n';
}

} // namespace lidar::util
