#include "protocol/vlp16_packet.hpp"
#include "network/packet_file.hpp"
#include "util/cli.hpp"
#include "util/log.hpp"
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
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
} // namespace

int runSimulator(int argc, char** argv) {
    lidar::util::CliOptions options(argc, argv, {"--port", "--rate-hz", "--capture"});
    if (options.helpRequested()) {
        std::cout << "Usage: lidar_simulator [--port 1..65535] [--rate-hz 1..10000] "
                     "[--capture file.vlp16]\n";
        return 0;
    }
    const uint16_t port = static_cast<uint16_t>(options.getUint32(
        "--port", lidar::protocol::VLP16_UDP_PORT, 1, 65535));
    const uint32_t rate_hz = options.getUint32("--rate-hz", 754, 1, 10000);
    const std::string capture_path = options.get("--capture", "");
    std::ofstream capture;
    if (!capture_path.empty()) {
        capture.open(capture_path, std::ios::binary);
        if (!capture) {
            throw std::runtime_error("Unable to open packet capture for writing: " + capture_path);
        }
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        lidar::util::log(lidar::util::LogLevel::error, "Failed to create simulator UDP socket");
        return 1;
    }

    sockaddr_in dest{};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);
    if (inet_pton(AF_INET, "127.0.0.1", &dest.sin_addr) != 1) {
        close(sock);
        lidar::util::log(lidar::util::LogLevel::error, "Invalid loopback destination address");
        return 1;
    }

    lidar::util::log(lidar::util::LogLevel::info,
                     "Streaming 1206-byte VLP-16 packets to 127.0.0.1:" +
                         std::to_string(port) + " at " + std::to_string(rate_hz) + " Hz");

    uint16_t current_azimuth = 0;
    uint64_t packets_sent = 0;

    while (g_running) {
        lidar::protocol::Vlp16Packet pkt{};
        auto now = std::chrono::steady_clock::now().time_since_epoch();
        pkt.timestamp_us = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(now).count() & 0xFFFFFFFF);
        pkt.return_mode = 0x37;
        pkt.product_id = 0x22;

        for (size_t b = 0; b < lidar::protocol::BLOCKS_PER_PACKET; ++b) {
            pkt.blocks[b].flag = lidar::protocol::VLP16_BLOCK_FLAG;
            pkt.blocks[b].azimuth = current_azimuth;
            current_azimuth = static_cast<uint16_t>((current_azimuth + 40) % 36000);

            for (size_t c = 0; c < lidar::protocol::CHANNELS_PER_BLOCK; ++c) {
                double angle_rad = (pkt.blocks[b].azimuth / 100.0) * (M_PI / 180.0);
                double radius_m = 10.0 + 2.0 * std::sin(4.0 * angle_rad);
                pkt.blocks[b].channels[c].distance = static_cast<uint16_t>(radius_m / 0.002);
                pkt.blocks[b].channels[c].intensity = static_cast<uint8_t>((c * 8) % 255);
            }
        }

        const ssize_t sent = sendto(sock, &pkt, sizeof(pkt), 0,
                                    reinterpret_cast<sockaddr*>(&dest), sizeof(dest));
        if (sent != static_cast<ssize_t>(sizeof(pkt))) {
            close(sock);
            lidar::util::log(lidar::util::LogLevel::error, "UDP packet send failed");
            return 1;
        }
        if (capture.is_open()) {
            lidar::network::writePacket(capture, pkt);
        }
        ++packets_sent;

        if (packets_sent % 750 == 0) {
            lidar::util::log(lidar::util::LogLevel::info,
                             "packets_sent=" + std::to_string(packets_sent));
        }

        std::this_thread::sleep_for(std::chrono::microseconds(1000000 / rate_hz));
    }

    close(sock);
    if (capture.is_open()) {
        capture.close();
        if (!capture) {
            lidar::util::log(lidar::util::LogLevel::error, "Failed to finalize packet capture");
            return 1;
        }
    }
    return 0;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, sigHandler);
    try {
        return runSimulator(argc, argv);
    } catch (const std::exception& ex) {
        lidar::util::log(lidar::util::LogLevel::error, ex.what());
        return 1;
    }
}
