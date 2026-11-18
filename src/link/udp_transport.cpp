#include "udp_transport.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace gcs::link {

namespace {

std::string sysError(const std::string& what) {
    return what + ": " + std::strerror(errno);
}

}  // namespace

static_assert(sizeof(sockaddr_in) <= 16, "peer_ storage holds a sockaddr_in");

UdpTransport::UdpTransport(std::string bind_host, uint16_t port, std::string remote_host, uint16_t remote_port)
    : bind_host_(std::move(bind_host)),
      port_(port),
      remote_host_(std::move(remote_host)),
      remote_port_(remote_port) {}

UdpTransport::~UdpTransport() {
    close();
}

bool UdpTransport::isOpen() const {
    return fd_ >= 0;
}

bool UdpTransport::open(std::string& error) {
    close();
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    if (::inet_pton(AF_INET, bind_host_.c_str(), &addr.sin_addr) != 1) {
        error = "not an IP address: " + bind_host_;
        return false;
    }
    sockaddr_in remote{};
    if (!remote_host_.empty()) {
        remote.sin_family = AF_INET;
        remote.sin_port = htons(remote_port_);
        if (remote_port_ == 0 || ::inet_pton(AF_INET, remote_host_.c_str(), &remote.sin_addr) != 1) {
            error = "not an IP address and port: " + remote_host_ + ":" + std::to_string(remote_port_);
            return false;
        }
    }

    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        error = sysError("socket");
        return false;
    }
    // no SO_REUSEADDR: with it a second GCS would silently share the port
    // (and the vehicle's packets) instead of being told it is taken
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0 || ::fcntl(fd, F_SETFD, FD_CLOEXEC) < 0) {
        error = sysError("fcntl");
        ::close(fd);
        return false;
    }
    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        error = errno == EADDRINUSE ? "port " + std::to_string(port_) + " is in use by another program"
                                    : sysError("bind " + bind_host_ + ":" + std::to_string(port_));
        ::close(fd);
        return false;
    }
    sockaddr_in bound{};
    socklen_t bound_len = sizeof(bound);
    bound_port_ = ::getsockname(fd, reinterpret_cast<sockaddr*>(&bound), &bound_len) == 0 ? ntohs(bound.sin_port) : port_;
    fd_ = fd;
    if (!remote_host_.empty()) {
        std::memcpy(peer_.data(), &remote, sizeof(remote));
        peer_known_ = true;
    }
    return true;
}

void UdpTransport::close() {
    if (fd_ >= 0) {
        ::close(static_cast<int>(fd_));
        fd_ = -1;
    }
    peer_known_ = false;
}

int UdpTransport::waitReadable(int timeout_ms, std::string& error) {
    if (fd_ < 0) {
        error = "socket closed";
        return -1;
    }
    pollfd p{};
    p.fd = static_cast<int>(fd_);
    p.events = POLLIN;
    const int ready = ::poll(&p, 1, timeout_ms);
    if (ready < 0) {
        if (errno == EINTR) {
            return 0;
        }
        error = sysError("poll");
        return -1;
    }
    if (ready > 0 && (p.revents & POLLNVAL) != 0) {
        error = "socket closed";
        return -1;
    }
    return ready > 0 ? 1 : 0;
}

long UdpTransport::read(uint8_t* buf, size_t max, std::string& error) {
    if (fd_ < 0) {
        error = "socket closed";
        return -1;
    }
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);
    const auto n = ::recvfrom(static_cast<int>(fd_), buf, max, 0, reinterpret_cast<sockaddr*>(&from), &from_len);
    if (n < 0) {
        // nothing more, or an ICMP "port unreachable" echo of our own send
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR || errno == ECONNREFUSED) {
            return 0;
        }
        error = sysError("recvfrom");
        return -1;
    }
    if (n > 0) {
        std::memcpy(peer_.data(), &from, sizeof(from));
        peer_known_ = true;
    }
    return static_cast<long>(n);
}

void UdpTransport::write(const uint8_t* data, size_t len) {
    if (fd_ < 0 || !peer_known_) {
        return;
    }
    sockaddr_in to{};
    std::memcpy(&to, peer_.data(), sizeof(to));
    ::sendto(static_cast<int>(fd_), data, len, 0, reinterpret_cast<const sockaddr*>(&to), sizeof(to));
}

std::string UdpTransport::describe() const {
    std::string s = "UDP " + bind_host_ + ":" + std::to_string(port_);
    if (!remote_host_.empty()) {
        s += " → " + remote_host_ + ":" + std::to_string(remote_port_);
    }
    return s;
}

}  // namespace gcs::link
