#pragma once
#include "protocol/vlp16_packet.hpp"
#include <array>
#include <cmath>

namespace lidar::codec {

class Vlp16Codec {
public:
    Vlp16Codec() {
        constexpr double DEG_TO_RAD = M_PI / 180.0;

        for (size_t i = 0; i < 36000; ++i) {
            double rad = (static_cast<double>(i) / 100.0) * DEG_TO_RAD;
            sin_azimuth_[i] = static_cast<float>(std::sin(rad));
            cos_azimuth_[i] = static_cast<float>(std::cos(rad));
        }

        constexpr std::array<double, 16> vertical_angles_deg = {
            -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0,
            -7.0,  9.0, -5.0,  11.0, -3.0, 13.0, -1.0, 15.0
        };

        for (size_t i = 0; i < 16; ++i) {
            double rad = vertical_angles_deg[i] * DEG_TO_RAD;
            sin_vert_[i] = static_cast<float>(std::sin(rad));
            cos_vert_[i] = static_cast<float>(std::cos(rad));
        }
    }

    bool decodePacket(const protocol::Vlp16Packet& pkt, protocol::PointCloudFrame& out_frame) {
        bool frame_completed = false;

        for (size_t b = 0; b < protocol::BLOCKS_PER_PACKET; ++b) {
            const auto& block = pkt.blocks[b];
            if (block.flag != protocol::VLP16_BLOCK_FLAG) {
                ++corrupted_blocks_;
                continue;
            }

            uint16_t azimuth = block.azimuth;
            if (azimuth >= 36000) {
                ++corrupted_blocks_;
                continue;
            }

            if (has_last_azimuth_ && azimuth < last_azimuth_) {
                frame_completed = true;
            }
            last_azimuth_ = azimuth;
            has_last_azimuth_ = true;

            const float sin_az = sin_azimuth_[azimuth];
            const float cos_az = cos_azimuth_[azimuth];

            for (size_t c = 0; c < protocol::CHANNELS_PER_BLOCK; ++c) {
                const auto& ret = block.channels[c];
                if (ret.distance == 0) {
                    continue;
                }

                if (out_frame.num_points >= protocol::MAX_POINTS_PER_FRAME) {
                    break;
                }

                const size_t laser_id = c % protocol::LASERS_PER_FIRING;
                const float dist_m = static_cast<float>(ret.distance) * protocol::DISTANCE_RESOLUTION_M;
                const float xy_proj = dist_m * cos_vert_[laser_id];

                auto& pt = out_frame.points[out_frame.num_points++];
                pt.x = xy_proj * sin_az;
                pt.y = xy_proj * cos_az;
                pt.z = dist_m * sin_vert_[laser_id];
                pt.intensity = static_cast<float>(ret.intensity);
            }
        }

        out_frame.timestamp_us = pkt.timestamp_us;
        return frame_completed;
    }

    uint64_t corruptedBlocks() const noexcept { return corrupted_blocks_; }
    void resetState() noexcept { has_last_azimuth_ = false; last_azimuth_ = 0; }

private:
    std::array<float, 36000> sin_azimuth_{};
    std::array<float, 36000> cos_azimuth_{};
    std::array<float, 16> sin_vert_{};
    std::array<float, 16> cos_vert_{};

    uint16_t last_azimuth_{0};
    bool has_last_azimuth_{false};
    uint64_t corrupted_blocks_{0};
};

} // namespace lidar::codec
