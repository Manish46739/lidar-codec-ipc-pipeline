#pragma once

#include "protocol/vlp16_packet.hpp"

#include <istream>
#include <ostream>
#include <stdexcept>

namespace lidar::network {

inline bool readPacket(std::istream& input, protocol::Vlp16Packet& packet) {
    input.read(reinterpret_cast<char*>(&packet), sizeof(packet));
    const std::streamsize bytes_read = input.gcount();
    if (bytes_read == 0 && input.eof()) {
        return false;
    }
    if (bytes_read != static_cast<std::streamsize>(sizeof(packet))) {
        throw std::runtime_error("Packet capture ends with an incomplete VLP-16 packet");
    }
    return true;
}

inline void writePacket(std::ostream& output, const protocol::Vlp16Packet& packet) {
    output.write(reinterpret_cast<const char*>(&packet), sizeof(packet));
    if (!output) {
        throw std::runtime_error("Failed to write VLP-16 packet capture");
    }
}

} // namespace lidar::network
