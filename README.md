# LiDAR Codec IPC Pipeline

A Linux/C++17 demonstration of a sensor-data pipeline: a synthetic VLP-16-like UDP source feeds a driver that validates and decodes packets into 3D point clouds, then publishes completed frames through POSIX shared memory for a separate consumer.

> **Scope note:** This is an educational simulator, not a certified Velodyne driver or production ADAS component. The packet layout and angle model are intentionally simplified. Performance figures should be measured on the machine running the project; the benchmark is a smoke check, not a cross-machine guarantee.

## Architecture

```text
+-----------------------+       UDP datagrams       +-----------------------------+
| lidar_simulator       | ------------------------> | lidar_driver                |
| synthetic packet data |       default :2368       | epoll -> SPSC -> VLP decoder|
+-----------------------+                            +--------------+--------------+
                                                                      |
                                                          POSIX shared-memory ring
                                                          default /lidar_shm_ring_v1
                                                                      |
                                                                      v
                                                   +------------------------------+
                                                   | perception_consumer          |
                                                   | metrics + optional PLY export|
                                                   +------------------------------+
```

The network thread and decode loop are decoupled by a fixed-size SPSC queue. Queue capacity is 1024 entries, with 1023 usable entries because the ring reserves one slot to distinguish full from empty. If it fills, the receiver counts and drops the incoming packet rather than overwriting unread data. Completed frames are published in a four-slot shared-memory ring.

## Features

- C++17, CMake, Linux UDP sockets and `epoll`
- Fixed-size single-producer/single-consumer queue
- VLP-16-style packet validation and spherical-to-Cartesian conversion
- POSIX shared-memory frame transport
- Runtime options for UDP port, simulator rate, shared-memory name, stats interval, PLY output, and export threshold
- Optional raw packet capture and paced UDP replay for repeatable decoder experiments
- PLY export with frame, timestamp, and coordinate-frame comments
- Operational counters: packet receive rate, malformed datagrams, queue drops, decoded packets, published frames, approximate queue depth, and corrupted blocks
- Checked socket, shared-memory, and PLY file operations
- GoogleTest coverage for packet layout, decode behavior, queue behavior, CLI validation, and a decoder benchmark smoke check
- GitHub Actions build/test matrix with normal and ASan/UBSan configurations
- Docker development and repeatable Linux build environment

## Requirements

- Linux (native) or Docker Desktop with Linux containers
- Native build dependencies: CMake 3.14+, C++17 compiler, pthreads, GoogleTest development package

The receiver uses Linux `epoll` and POSIX shared memory; native Windows builds are not supported.

## Build and test with Docker

From the repository directory:

```bash
docker build -t lidar-adas-env .
docker run --rm -v "$PWD:/workspace" lidar-adas-env bash -lc \
  "cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && \
   cmake --build build --parallel 4 && \
   ctest --test-dir build --output-on-failure"
```

On PowerShell, use `-v "${PWD}:/workspace"` for the bind mount.

### Native Linux build

On Ubuntu:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libgtest-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

### Sanitizers

Run AddressSanitizer and UndefinedBehaviorSanitizer on supported Linux toolchains:

```bash
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DENABLE_SANITIZERS=ON
cmake --build build-asan --parallel
ctest --test-dir build-asan --output-on-failure
```

## Run the pipeline

Build the binaries first, then run the driver, consumer, and simulator in three terminals. The processes must share a Linux IPC namespace for POSIX shared memory.

For a simple Docker demo, start the driver container with a bind mount:

```bash
docker run --rm --shm-size=256m -v "$PWD:/workspace" \
  --name lidar_live lidar-adas-env ./build/lidar_driver
```

In another terminal, attach the consumer and simulator to that same container:

```bash
docker exec -it lidar_live ./build/perception_consumer
docker exec -it lidar_live ./build/lidar_simulator
```

Use Ctrl+C to stop the driver; then stop the other processes. The consumer writes `sample_scan.ply` in the mounted workspace once it reads a frame meeting its minimum point threshold.

PowerShell version:

```powershell
docker run --rm --shm-size=256m -v "${PWD}:/workspace" --name lidar_live lidar-adas-env ./build/lidar_driver
docker exec -it lidar_live ./build/perception_consumer
docker exec -it lidar_live ./build/lidar_simulator
```

## Runtime configuration

All options are validated; invalid, duplicated, unknown, or value-less options return an error instead of silently falling back.

### Driver

```text
lidar_driver [--port 1..65535] [--shm-name /name] [--stats-interval-ms 250..60000]
```

Defaults: UDP port `2368`, shared-memory object `/lidar_shm_ring_v1`, metrics every `5000` ms.

Example:

```bash
./build/lidar_driver --port 2370 --shm-name /lidar_demo --stats-interval-ms 1000
```

### Simulator

```text
lidar_simulator [--port 1..65535] [--rate-hz 1..10000] [--capture file.vlp16]
```

Defaults: destination port `2368`, rate `754` packets/second. The simulator sends only to loopback (`127.0.0.1`). When `--capture` is supplied, each generated 1206-byte packet is saved in a raw fixed-record file while it is streamed.

### Offline packet replay

```text
lidar_replay --input capture.vlp16 [--port 1..65535] [--rate-hz 1..10000]
```

Replay sends each recorded packet to loopback at the selected rate and stops at end-of-file. The capture format is headerless: consecutive records are exactly `sizeof(Vlp16Packet)` (1206) bytes. A partial final record is reported as an error. For example:

```bash
./build/lidar_simulator --capture demo.vlp16 --rate-hz 500
# Stop the simulator with Ctrl+C once enough sample data has been recorded.
./build/lidar_replay --input demo.vlp16 --rate-hz 500
```

### Consumer

```text
perception_consumer [--shm-name /name] [--output file.ply] [--min-points 1..36000]
```

Defaults: `/lidar_shm_ring_v1`, `sample_scan.ply`, and minimum of `1000` points.

Example configuration across all processes:

```bash
./build/lidar_driver --port 2370 --shm-name /lidar_demo --stats-interval-ms 1000
./build/perception_consumer --shm-name /lidar_demo --output demo.ply --min-points 500
./build/lidar_simulator --port 2370 --rate-hz 500
```

## Packet and point conversion

The simplified packet is exactly 1206 bytes: twelve 100-byte data blocks, a four-byte timestamp, and two metadata bytes. Compile-time assertions guard the expected structure sizes. A return's encoded distance uses a 2 mm step:

```text
distance_m = encoded_distance * 0.002
```

For azimuth `90°`, vertical angle `1°`, and encoded distance `5000`:

```text
range = 5000 * 0.002 = 10 m
x = 10 * cos(1°) * sin(90°)  ~= 9.9985 m
y = 10 * cos(1°) * cos(90°)  ~= 0 m
z = 10 * sin(1°)              ~= 0.1745 m
```

The implementation precomputes trigonometric lookup tables for azimuth and the fixed vertical laser angles to avoid repeated `sin`/`cos` calculations per return.

## Metrics and performance

The driver periodically logs cumulative operational counts and receive rate, for example:

```text
[INFO] metrics rx_packets=3770 rx_packets_per_sec=753 malformed=0 queue_drops=0 decoded=3770 frames=50 queue_depth~=0 corrupted_blocks=0
```

These values are illustrative only. Actual throughput, packet loss, queue pressure, frame size, and latency depend on the host and scheduling. The consumer's latency is measured between the driver's frame-publication timestamp and consumer read time using `steady_clock`; it is a local process-pipeline measurement, not sensor-to-world clock latency.

The GoogleTest benchmark consumes a decoded point in a checksum to prevent the compiler from removing the work. Its generous time bound is only a gross regression/smoke guard. For meaningful performance claims, run repeated measurements on the target machine under controlled load and report the compiler, CPU, build type, and test procedure.

## Reliability and limitations

- A malformed UDP datagram is counted and ignored; queue saturation increments `queue_drops`.
- System-call and output-file failures are surfaced as errors.
- The decoder checks block flags, azimuth bounds, zero ranges, and output capacity.
- UDP is best-effort: network-level loss is not retransmitted. The current counters identify queue drops and malformed packets but cannot distinguish kernel/network loss without additional sequence-number or socket-drop instrumentation.
- This simplified model does not include full sensor calibration, dual-return modes, packet sequence validation, time synchronization, or a production-safe shared-memory reader/writer ownership protocol.
- The shared-memory ring is optimized for the single local demo consumer; consumers must keep up with producer reuse of the ring.

## Project layout

```text
.devcontainer/       VS Code dev-container settings
.github/workflows/   CI build, tests, and sanitizer job matrix
include/             packet protocol, codec, IPC, networking, utilities
src/                 simulator, capture replay, driver, and consumer entry points
tests/               unit tests and benchmark smoke check
Dockerfile           Ubuntu build environment
CMakeLists.txt       targets, test registration, sanitizer option
```

## Resume bullet (edit to match your verified measurements)

> Built a C++17 Linux LiDAR streaming pipeline with UDP/epoll ingestion, a bounded SPSC queue, VLP-16-style point decoding, and POSIX shared-memory frame delivery; added CLI validation, operational metrics, GoogleTest coverage, Docker reproducibility, and ASan/UBSan CI.

Only add a measured throughput or latency number to a resume after benchmarking on a named target machine and preserving the test conditions.
