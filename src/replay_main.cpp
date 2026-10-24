#include "network/packet_file.hpp"
#include "util/cli.hpp"
#include "util/log.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t g_running = 1;
void sigHandler(int) { g_running = 0; }
}

int main(int argc, char** argv) {
    std::signal(SIGINT, sigHandler);
    int socket_fd = -1;
    try {
        lidar::util::CliOptions options(argc, argv, {"--input", "--port", "--rate-hz"});
        if (options.helpRequested()) {
            std::cout << "Usage: lidar_replay --input capture.vlp16 [--port 1..65535] "
                         "[--rate-hz 1..10000]\n";
            return 0;
        }

        const std::string input_path = options.get("--input", "");
        if (input_path.empty()) {
            throw std::invalid_argument("Required option missing: --input");
        }
        const uint16_t port = static_cast<uint16_t>(
            options.getUint32("--port", lidar::protocol::VLP16_UDP_PORT, 1, 65535));
        const uint32_t rate_hz = options.getUint32("--rate-hz", 754, 1, 10000);
        std::ifstream input(input_path, std::ios::binary);
        if (!input) {
            throw std::runtime_error("Unable to open packet capture: " + input_path);
        }

        socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0) {
            throw std::runtime_error("Failed to create replay UDP socket");
        }
        sockaddr_in destination{};
        destination.sin_family = AF_INET;
        destination.sin_port = htons(port);
        if (inet_pton(AF_INET, "127.0.0.1", &destination.sin_addr) != 1) {
            throw std::runtime_error("Invalid loopback destination address");
        }

        lidar::util::log(lidar::util::LogLevel::info,
                         "Replaying " + input_path + " to 127.0.0.1:" +
                             std::to_string(port) + " at " + std::to_string(rate_hz) + " Hz");

        uint64_t sent_packets = 0;
        lidar::protocol::Vlp16Packet packet{};
        while (g_running && lidar::network::readPacket(input, packet)) {
            const ssize_t sent = sendto(socket_fd, &packet, sizeof(packet), 0,
                                        reinterpret_cast<sockaddr*>(&destination),
                                        sizeof(destination));
            if (sent != static_cast<ssize_t>(sizeof(packet))) {
                throw std::runtime_error("Failed to send replay UDP packet");
            }
            ++sent_packets;
            std::this_thread::sleep_for(std::chrono::microseconds(1000000 / rate_hz));
        }
        close(socket_fd);
        socket_fd = -1;
        lidar::util::log(lidar::util::LogLevel::info,
                         "replay_complete packets_sent=" + std::to_string(sent_packets));
    } catch (const std::exception& ex) {
        if (socket_fd >= 0) {
            close(socket_fd);
        }
        lidar::util::log(lidar::util::LogLevel::error, ex.what());
        return 1;
    }
    return 0;
}
