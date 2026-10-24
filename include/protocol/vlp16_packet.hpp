#pragma once
#include <cstdint>
#include <cstddef>

namespace lidar::protocol {

constexpr uint16_t VLP16_UDP_PORT = 2368;
constexpr uint16_t VLP16_BLOCK_FLAG = 0xEEFF; // Little-endian representation of 0xFFEE
constexpr size_t BLOCKS_PER_PACKET = 12;
constexpr size_t CHANNELS_PER_BLOCK = 32;
constexpr size_t LASERS_PER_FIRING = 16;
constexpr float DISTANCE_RESOLUTION_M = 0.002f; // 2 mm increments
constexpr size_t MAX_POINTS_PER_FRAME = 36000;

#pragma pack(push, 1)
struct LaserReturn {
    uint16_t distance;   // 2mm resolution
    uint8_t intensity;   // 0-255 reflectivity
};

struct DataBlock {
    uint16_t flag;       // 0xEEFF
    uint16_t azimuth;    // 0 to 35999 (hundredths of a degree: 0.00 to 359.99)
    LaserReturn channels[CHANNELS_PER_BLOCK];
};

struct Vlp16Packet {
    DataBlock blocks[BLOCKS_PER_PACKET]; // 1200 bytes
    uint32_t timestamp_us;               // 4 bytes
    uint8_t return_mode;                 // 1 byte (0x37 = Strongest)
    uint8_t product_id;                  // 1 byte (0x22 = VLP-16)
};
#pragma pack(pop)

static_assert(sizeof(LaserReturn) == 3, "LaserReturn must be 3 bytes");
static_assert(sizeof(DataBlock) == 100, "DataBlock must be 100 bytes");
static_assert(sizeof(Vlp16Packet) == 1206, "Vlp16Packet must be exactly 1206 bytes");

struct alignas(16) PointXYZI {
    float x;
    float y;
    float z;
    float intensity;
};

struct PointCloudFrame {
    uint64_t frame_id{0};
    uint64_t timestamp_us{0};
    uint32_t num_points{0};
    PointXYZI points[MAX_POINTS_PER_FRAME];
};

} // namespace lidar::protocol
