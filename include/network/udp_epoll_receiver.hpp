#pragma once
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace lidar::network {

class UdpEpollReceiver {
public:
    explicit UdpEpollReceiver(uint16_t port) {
        sock_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_fd_ < 0) {
            fail("socket");
        }

        int opt = 1;
        if (setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            fail("setsockopt(SO_REUSEADDR)");
        }

        int rcvbuf = 4 * 1024 * 1024;
        if (setsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) < 0) {
            fail("setsockopt(SO_RCVBUF)");
        }

        int flags = fcntl(sock_fd_, F_GETFL, 0);
        if (flags < 0 || fcntl(sock_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
            fail("fcntl(O_NONBLOCK)");
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        if (bind(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            fail("bind");
        }

        epoll_fd_ = epoll_create1(0);
        if (epoll_fd_ < 0) {
            fail("epoll_create1");
        }

        epoll_event ev{};
        ev.events = EPOLLIN;
        ev.data.fd = sock_fd_;
        if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, sock_fd_, &ev) < 0) {
            fail("epoll_ctl");
        }
    }

    ~UdpEpollReceiver() {
        if (epoll_fd_ >= 0) close(epoll_fd_);
        if (sock_fd_ >= 0) close(sock_fd_);
    }

    template <typename PacketCallback>
    void pollOnce(int timeout_ms, PacketCallback&& cb) {
        epoll_event events[8];
        int nfds = epoll_wait(epoll_fd_, events, 8, timeout_ms);
        if (nfds < 0) {
            if (errno == EINTR) {
                return;
            }
            throw std::runtime_error(std::string("epoll_wait failed: ") + std::strerror(errno));
        }
        for (int i = 0; i < nfds; ++i) {
            if (events[i].data.fd == sock_fd_) {
                while (true) {
                    ssize_t n = recvfrom(sock_fd_, recv_buf_, sizeof(recv_buf_), 0, nullptr, nullptr);
                    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                        break;
                    }
                    if (n < 0) {
                        throw std::runtime_error(std::string("recvfrom failed: ") + std::strerror(errno));
                    }
                    cb(recv_buf_, static_cast<size_t>(n));
                }
            }
        }
    }

private:
    [[noreturn]] void fail(const char* operation) {
        const int error = errno;
        if (epoll_fd_ >= 0) {
            close(epoll_fd_);
            epoll_fd_ = -1;
        }
        if (sock_fd_ >= 0) {
            close(sock_fd_);
            sock_fd_ = -1;
        }
        throw std::runtime_error(std::string(operation) + " failed: " + std::strerror(error));
    }

    int sock_fd_{-1};
    int epoll_fd_{-1};
    uint8_t recv_buf_[2048]{};
};

} // namespace lidar::network
