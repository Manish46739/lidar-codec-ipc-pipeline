#include "codec/vlp16_codec.hpp"
#include "concurrency/spsc_ring_buffer.hpp"
#include "ipc/shm_arena.hpp"
#include "network/udp_epoll_receiver.hpp"
#include "protocol/vlp16_packet.hpp"
#include "util/cli.hpp"
#include "util/log.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <exception>
#include <iostream>
#include <sstream>
#include <thread>

namespace {
std::atomic<bool> g_running{true};
void sigHandler(int) { g_running.store(false); }
} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, sigHandler);

    try {
        lidar::util::CliOptions options(argc, argv, {"--port", "--shm-name", "--stats-interval-ms"});
        if (options.helpRequested()) {
            std::cout << "Usage: lidar_driver [--port 1..65535] [--shm-name /name] "
                         "[--stats-interval-ms 250..60000]\n";
            return 0;
        }
        const auto port = static_cast<uint16_t>(options.getUint32(
            "--port", lidar::protocol::VLP16_UDP_PORT, 1, 65535));
        const auto stats_interval_ms = options.getUint32("--stats-interval-ms", 5000, 250, 60000);
        const auto shm_name = options.get("--shm-name", lidar::ipc::SHM_OBJECT_NAME);

        lidar::ipc::ShmManager shm(true, shm_name);
        auto* arena = shm.arena();

        lidar::network::UdpEpollReceiver receiver(port);
        lidar::concurrency::SpscRingBuffer<lidar::protocol::Vlp16Packet, 1024> packet_queue;
        lidar::codec::Vlp16Codec codec;
        std::atomic<uint64_t> received_packets{0};
        std::atomic<uint64_t> malformed_packets{0};
        std::atomic<uint64_t> dropped_packets{0};
        std::atomic<uint64_t> processed_packets{0};
        std::atomic<uint64_t> published_frames{0};
        std::exception_ptr network_error;

        lidar::util::log(lidar::util::LogLevel::info,
                         "UDP listener on port " + std::to_string(port));
        lidar::util::log(lidar::util::LogLevel::info, "Shared memory: " + shm_name);

        std::thread net_thread([&]() {
            try {
                while (g_running.load(std::memory_order_relaxed)) {
                    receiver.pollOnce(10, [&](const uint8_t* data, size_t len) {
                        received_packets.fetch_add(1, std::memory_order_relaxed);
                        if (len != sizeof(lidar::protocol::Vlp16Packet)) {
                            malformed_packets.fetch_add(1, std::memory_order_relaxed);
                            return;
                        }
                        lidar::protocol::Vlp16Packet pkt{};
                        std::memcpy(&pkt, data, sizeof(pkt));
                        if (!packet_queue.push(pkt)) {
                            dropped_packets.fetch_add(1, std::memory_order_relaxed);
                        }
                    });
                }
            } catch (...) {
                network_error = std::current_exception();
                g_running.store(false, std::memory_order_relaxed);
            }
        });

        uint64_t current_seq = 0;
        size_t active_slot = 0;
        arena->slots[active_slot].num_points = 0;
        auto last_stats = std::chrono::steady_clock::now();
        uint64_t previous_received = 0;

        while (g_running.load(std::memory_order_relaxed)) {
            lidar::protocol::Vlp16Packet pkt;
            if (packet_queue.pop(pkt)) {
                processed_packets.fetch_add(1, std::memory_order_relaxed);
                auto& current_frame = arena->slots[active_slot];
                bool sweep_done = codec.decodePacket(pkt, current_frame);

                if (sweep_done && current_frame.num_points > 0) {
                    auto now = std::chrono::steady_clock::now().time_since_epoch();
                    current_frame.timestamp_us = static_cast<uint64_t>(
                        std::chrono::duration_cast<std::chrono::microseconds>(now).count());
                    current_frame.frame_id = ++current_seq;

                    arena->published_seq.store(current_seq, std::memory_order_release);
                    published_frames.fetch_add(1, std::memory_order_relaxed);

                    active_slot = (active_slot + 1) % lidar::ipc::SHM_RING_SLOTS;
                    arena->slots[active_slot].num_points = 0;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - last_stats >= std::chrono::milliseconds(stats_interval_ms)) {
                const uint64_t received = received_packets.load(std::memory_order_relaxed);
                const double elapsed_seconds = std::chrono::duration<double>(now - last_stats).count();
                const double receive_rate = (received - previous_received) / elapsed_seconds;
                std::ostringstream stats;
                stats << "metrics rx_packets=" << received
                      << " rx_packets_per_sec=" << static_cast<uint64_t>(receive_rate)
                      << " malformed=" << malformed_packets.load(std::memory_order_relaxed)
                      << " queue_drops=" << dropped_packets.load(std::memory_order_relaxed)
                      << " decoded=" << processed_packets.load(std::memory_order_relaxed)
                      << " frames=" << published_frames.load(std::memory_order_relaxed)
                      << " queue_depth~=" << packet_queue.approximateSize()
                      << " corrupted_blocks=" << codec.corruptedBlocks();
                lidar::util::log(lidar::util::LogLevel::info, stats.str());
                last_stats = now;
                previous_received = received;
            }
        }

        net_thread.join();
        if (network_error) {
            std::rethrow_exception(network_error);
        }
    } catch (const std::exception& ex) {
        lidar::util::log(lidar::util::LogLevel::error, ex.what());
        return 1;
    }

    return 0;
}
