#include "codec/vlp16_codec.hpp"
#include "concurrency/spsc_ring_buffer.hpp"
#include "network/packet_file.hpp"
#include "protocol/vlp16_packet.hpp"
#include "util/cli.hpp"
#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <sstream>

using namespace lidar::protocol;
using namespace lidar::codec;

TEST(Vlp16CodecTest, StructPackingSize) {
    EXPECT_EQ(sizeof(LaserReturn), 3u);
    EXPECT_EQ(sizeof(DataBlock), 100u);
    EXPECT_EQ(sizeof(Vlp16Packet), 1206u);
    EXPECT_EQ(alignof(PointXYZI), 16u);
}

TEST(Vlp16CodecTest, RejectsCorruptedBlockFlag) {
    Vlp16Codec codec;
    Vlp16Packet pkt{};
    PointCloudFrame frame{};

    codec.decodePacket(pkt, frame);
    EXPECT_EQ(frame.num_points, 0u);
    EXPECT_EQ(codec.corruptedBlocks(), 12u);
}

TEST(Vlp16CodecTest, AccurateSphericalToCartesianConversion) {
    Vlp16Codec codec;
    Vlp16Packet pkt{};
    PointCloudFrame frame{};

    pkt.blocks[0].flag = VLP16_BLOCK_FLAG;
    pkt.blocks[0].azimuth = 0;
    pkt.blocks[0].channels[1].distance = 5000;
    pkt.blocks[0].channels[1].intensity = 200;

    codec.decodePacket(pkt, frame);

    ASSERT_EQ(frame.num_points, 1u);
    const double rad_1deg = 1.0 * M_PI / 180.0;
    EXPECT_NEAR(frame.points[0].x, 0.0f, 1e-4f);
    EXPECT_NEAR(frame.points[0].y, static_cast<float>(10.0 * std::cos(rad_1deg)), 1e-4f);
    EXPECT_NEAR(frame.points[0].z, static_cast<float>(10.0 * std::sin(rad_1deg)), 1e-4f);
    EXPECT_FLOAT_EQ(frame.points[0].intensity, 200.0f);
}

TEST(Vlp16CodecTest, Detects360DegreeAzimuthWrapAround) {
    Vlp16Codec codec;
    Vlp16Packet pkt{};
    PointCloudFrame frame{};

    pkt.blocks[0].flag = VLP16_BLOCK_FLAG;
    pkt.blocks[0].azimuth = 35900;
    pkt.blocks[0].channels[0].distance = 1000;

    pkt.blocks[1].flag = VLP16_BLOCK_FLAG;
    pkt.blocks[1].azimuth = 10;
    pkt.blocks[1].channels[0].distance = 1000;

    bool wrapped = codec.decodePacket(pkt, frame);
    EXPECT_TRUE(wrapped);
    EXPECT_EQ(frame.num_points, 2u);
}

TEST(Vlp16CodecTest, HighThroughputBenchmark) {
    Vlp16Codec codec;
    Vlp16Packet pkt{};
    PointCloudFrame frame{};

    for (size_t b = 0; b < BLOCKS_PER_PACKET; ++b) {
        pkt.blocks[b].flag = VLP16_BLOCK_FLAG;
        pkt.blocks[b].azimuth = static_cast<uint16_t>(b * 100);
        for (size_t c = 0; c < CHANNELS_PER_BLOCK; ++c) {
            pkt.blocks[b].channels[c].distance = 4000;
            pkt.blocks[b].channels[c].intensity = 128;
        }
    }

    constexpr size_t ITERATIONS = 10000;
    volatile float checksum = 0.0f;
    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < ITERATIONS; ++i) {
        frame.num_points = 0;
        codec.decodePacket(pkt, frame);
        checksum += frame.points[0].y;
    }
    auto end = std::chrono::high_resolution_clock::now();
    double total_us = std::chrono::duration<double, std::micro>(end - start).count();
    double us_per_pkt = total_us / ITERATIONS;

    std::cout << "[ BENCHMARK ] Decoded " << ITERATIONS << " VLP-16 packets in "
              << (total_us / 1000.0) << " ms (" << us_per_pkt << " us/packet)\n";
    EXPECT_NE(checksum, 0.0f);
    EXPECT_LT(us_per_pkt, 5000.0);
}

TEST(SpscRingBufferTest, PreservesOrderAndReportsFullQueue) {
    lidar::concurrency::SpscRingBuffer<int, 4> queue;
    EXPECT_TRUE(queue.push(10));
    EXPECT_TRUE(queue.push(20));
    EXPECT_TRUE(queue.push(30));
    EXPECT_FALSE(queue.push(40));
    EXPECT_EQ(queue.approximateSize(), 3u);

    int value = 0;
    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 10);
    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 20);
    EXPECT_TRUE(queue.push(40));
    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 30);
    EXPECT_TRUE(queue.pop(value));
    EXPECT_EQ(value, 40);
}

TEST(CliOptionsTest, ParsesOptionsAndRejectsOutOfRangeValues) {
    char arg0[] = "driver";
    char arg1[] = "--port";
    char arg2[] = "2369";
    char* argv[] = {arg0, arg1, arg2};
    lidar::util::CliOptions options(3, argv, {"--port"});
    EXPECT_EQ(options.getUint32("--port", 2368, 1, 65535), 2369u);

    char bad_arg2[] = "70000";
    char* bad_argv[] = {arg0, arg1, bad_arg2};
    lidar::util::CliOptions invalid_options(3, bad_argv, {"--port"});
    EXPECT_THROW(invalid_options.getUint32("--port", 2368, 1, 65535), std::invalid_argument);
}

TEST(PacketFileTest, ReadsAndWritesFixedSizePacketRecords) {
    Vlp16Packet source{};
    source.blocks[3].azimuth = 12345;
    source.blocks[3].channels[4].distance = 6789;
    std::stringstream file(std::ios::in | std::ios::out | std::ios::binary);

    lidar::network::writePacket(file, source);
    file.seekg(0);

    Vlp16Packet restored{};
    ASSERT_TRUE(lidar::network::readPacket(file, restored));
    EXPECT_EQ(restored.blocks[3].azimuth, 12345);
    EXPECT_EQ(restored.blocks[3].channels[4].distance, 6789);
    EXPECT_FALSE(lidar::network::readPacket(file, restored));
}

TEST(PacketFileTest, RejectsTruncatedPacketRecord) {
    std::stringstream file(std::ios::in | std::ios::out | std::ios::binary);
    file.write("short", 5);
    file.seekg(0);
    Vlp16Packet packet{};
    EXPECT_THROW(lidar::network::readPacket(file, packet), std::runtime_error);
}
