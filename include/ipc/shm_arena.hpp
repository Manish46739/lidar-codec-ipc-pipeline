#pragma once
#include "protocol/vlp16_packet.hpp"
#include <atomic>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>

namespace lidar::ipc {

constexpr const char* SHM_OBJECT_NAME = "/lidar_shm_ring_v1";
constexpr size_t SHM_RING_SLOTS = 4;

struct SharedMemoryArena {
    alignas(64) std::atomic<uint64_t> published_seq{0};
    protocol::PointCloudFrame slots[SHM_RING_SLOTS];
};

class ShmManager {
public:
    explicit ShmManager(bool is_creator, std::string object_name = SHM_OBJECT_NAME)
        : is_creator_(is_creator), object_name_(std::move(object_name)) {
        if (object_name_.empty() || object_name_.front() != '/') {
            throw std::invalid_argument("Shared-memory name must begin with /");
        }
        int flags = is_creator ? (O_CREAT | O_EXCL | O_RDWR) : O_RDWR;
        fd_ = shm_open(object_name_.c_str(), flags, 0600);
        if (fd_ < 0) {
            throw std::runtime_error("shm_open(" + object_name_ +
                                     ") failed (another driver may already own it): " +
                                     std::strerror(errno));
        }

        if (is_creator) {
            if (ftruncate(fd_, sizeof(SharedMemoryArena)) < 0) {
                const int error = errno;
                close(fd_);
                fd_ = -1;
                shm_unlink(object_name_.c_str());
                throw std::runtime_error("ftruncate failed: " + std::string(std::strerror(error)));
            }
        }

        struct stat object_status {};
        const int stat_result = fstat(fd_, &object_status);
        if (stat_result < 0) {
            const int error = errno;
            close(fd_);
            fd_ = -1;
            if (is_creator_) {
                shm_unlink(object_name_.c_str());
            }
            throw std::runtime_error("fstat shared memory failed: " +
                                     std::string(std::strerror(error)));
        }
        if (object_status.st_size < static_cast<off_t>(sizeof(SharedMemoryArena))) {
            close(fd_);
            fd_ = -1;
            if (is_creator_) {
                shm_unlink(object_name_.c_str());
            }
            throw std::runtime_error("Shared-memory object is smaller than the expected arena");
        }

        void* ptr = mmap(nullptr, sizeof(SharedMemoryArena),
                         PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (ptr == MAP_FAILED) {
            const int error = errno;
            close(fd_);
            fd_ = -1;
            throw std::runtime_error("mmap failed: " + std::string(std::strerror(error)));
        }

        arena_ = static_cast<SharedMemoryArena*>(ptr);
        if (is_creator) {
            new (arena_) SharedMemoryArena{};
        }
    }

    ShmManager(const ShmManager&) = delete;
    ShmManager& operator=(const ShmManager&) = delete;

    ~ShmManager() {
        if (arena_) {
            munmap(arena_, sizeof(SharedMemoryArena));
        }
        if (fd_ >= 0) {
            close(fd_);
        }
        if (is_creator_) {
            shm_unlink(object_name_.c_str());
        }
    }

    SharedMemoryArena* arena() noexcept { return arena_; }

private:
    bool is_creator_;
    std::string object_name_;
    int fd_{-1};
    SharedMemoryArena* arena_{nullptr};
};

} // namespace lidar::ipc
