#pragma once

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace lidar::util {

class CliOptions {
public:
    CliOptions(int argc, char** argv, const std::vector<std::string>& allowed) {
        for (int i = 1; i < argc; ++i) {
            const std::string key = argv[i];
            if (key == "--help" || key == "-h") {
                help_ = true;
                continue;
            }
            if (key.rfind("--", 0) != 0) {
                throw std::invalid_argument("Expected an option beginning with --: " + key);
            }
            if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
                throw std::invalid_argument("Unknown option: " + key);
            }
            if (i + 1 >= argc) {
                throw std::invalid_argument("Missing value for option: " + key);
            }
            if (!values_.emplace(key, argv[++i]).second) {
                throw std::invalid_argument("Option specified more than once: " + key);
            }
        }
    }

    bool helpRequested() const noexcept { return help_; }

    std::string get(const std::string& key, const std::string& fallback) const {
        const auto it = values_.find(key);
        return it == values_.end() ? fallback : it->second;
    }

    uint32_t getUint32(const std::string& key, uint32_t fallback,
                       uint32_t minimum, uint32_t maximum) const {
        const auto it = values_.find(key);
        if (it == values_.end()) {
            return fallback;
        }
        char* end = nullptr;
        errno = 0;
        const unsigned long value = std::strtoul(it->second.c_str(), &end, 10);
        if (errno != 0 || end == it->second.c_str() || *end != '\0' ||
            value < minimum || value > maximum) {
            throw std::invalid_argument("Invalid value for " + key + ": " + it->second);
        }
        return static_cast<uint32_t>(value);
    }

private:
    std::map<std::string, std::string> values_;
    bool help_{false};
};

} // namespace lidar::util
