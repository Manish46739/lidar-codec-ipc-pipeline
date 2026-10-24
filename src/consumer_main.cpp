#include "ipc/shm_arena.hpp"
#include "util/cli.hpp"
#include "util/log.hpp"
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

namespace {
volatile std::sig_atomic_t g_running = 1;
void sigHandler(int) { g_running = 0; }

void exportToPly(const lidar::protocol::PointCloudFrame& frame, const std::string& filename) {
    std::ofstream out(filename);
    if (!out) {
        throw std::runtime_error("Unable to open PLY output file: " + filename);
    }
    out << "ply\nformat ascii 1.0\n";
    out << "comment frame_id " << frame.frame_id << "\n";
    out << "comment timestamp_us_steady_clock " << frame.timestamp_us << "\n";
    out << "comment coordinates sensor_frame\n";
    out << "element vertex " << frame.num_points << "\n";
    out << "property float x\nproperty float y\nproperty float z\nproperty float intensity\n";
    out << "end_header\n";
    for (uint32_t i = 0; i < frame.num_points; ++i) {
        const auto& p = frame.points[i];
        out << p.x << " " << p.y << " " << p.z << " " << p.intensity << "\n";
    }
    if (!out) {
        throw std::runtime_error("Failed while writing PLY output file: " + filename);
    }
}
} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, sigHandler);

    try {
        lidar::util::CliOptions options(argc, argv, {"--shm-name", "--output", "--min-points"});
        if (options.helpRequested()) {
            std::cout << "Usage: perception_consumer [--shm-name /name] [--output file.ply] "
                         "[--min-points 1..36000]\n";
            return 0;
        }
        const auto shm_name = options.get("--shm-name", lidar::ipc::SHM_OBJECT_NAME);
        const auto output_path = options.get("--output", "sample_scan.ply");
        const auto min_points = options.getUint32("--min-points", 1000, 1,
                                                  static_cast<uint32_t>(lidar::protocol::MAX_POINTS_PER_FRAME));
        lidar::ipc::ShmManager shm(false, shm_name);
        auto* arena = shm.arena();

        lidar::util::log(lidar::util::LogLevel::info, "Attached to shared memory " + shm_name);

        uint64_t last_seen_seq = 0;
        bool exported_sample = false;

        while (g_running) {
            uint64_t published = arena->published_seq.load(std::memory_order_acquire);
            if (published > last_seen_seq) {
                size_t slot_idx = (published - 1) % lidar::ipc::SHM_RING_SLOTS;
                const auto& frame = arena->slots[slot_idx];
                if (frame.num_points > lidar::protocol::MAX_POINTS_PER_FRAME) {
                    throw std::runtime_error("Shared-memory frame has an invalid point count");
                }

                auto now = std::chrono::steady_clock::now().time_since_epoch();
                uint64_t now_us = static_cast<uint64_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(now).count());
                uint64_t ipc_latency_us = (now_us >= frame.timestamp_us) ? (now_us - frame.timestamp_us) : 0;

                std::ostringstream message;
                message << "frame=" << frame.frame_id << " points=" << frame.num_points
                        << " latency_us=" << ipc_latency_us;
                lidar::util::log(lidar::util::LogLevel::info, message.str());

                if (!exported_sample && frame.num_points >= min_points) {
                    exportToPly(frame, output_path);
                    lidar::util::log(lidar::util::LogLevel::info, "Exported PLY: " + output_path);
                    exported_sample = true;
                }

                last_seen_seq = published;
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
    } catch (const std::exception& ex) {
        lidar::util::log(lidar::util::LogLevel::error, ex.what());
        return 1;
    }

    return 0;
}
