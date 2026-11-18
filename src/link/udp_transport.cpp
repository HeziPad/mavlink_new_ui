#include "udp_transport.hpp"

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#include <mstcpip.h>  // SIO_UDP_CONNRESET
#include <mutex>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#endif

#include <cstring>

namespace gcs::link {

namespace {

// ---- the few socket calls that differ between Windows and POSIX ------------------------

#if defined(_WIN32)
using Sock = SOCKET;
constexpr intptr_t kNoSocket = -1;

bool netInit(std::string& error) {
    static std::once_flag once;
    static int rc = 0;
    std::call_once(once, [] {
        WSADATA d{};
        rc = WSAStartup(MAKEWORD(2, 2), &d);
    });
    if (rc != 0) {
        error = "WSAStartup failed (" + std::to_string(rc) + ")";
        return false;
    }
    return true;
}
int lastError() { return WSAGetLastError(); }
std::string errorText(int err) {
    char buf[256] = {};
    const DWORD n = FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr,
                                   static_cast<DWORD>(err), 0, buf, sizeof(buf), nullptr);
    std::string s(buf, n);
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r' || s.back() == ' ' || s.back() == '.')) {
        s.pop_back();
    }
    return s.empty() ? "error " + std::to_string(err) : s;
}
bool isAddrInUse(int err) { return err == WSAEADDRINUSE || err == WSAEACCES; }
// nothing to read now, or an ICMP "port unreachable" echo of our own send
bool isTransient(int err) { return err == WSAEWOULDBLOCK || err == WSAECONNRESET || err == WSAEMSGSIZE || err == WSAEINTR; }
void closeSock(intptr_t s) { ::closesocket(static_cast<Sock>(s)); }
bool setupSocket(Sock s) {
    // a second program must not share the port silently
    BOOL excl = TRUE;
    ::setsockopt(s, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, reinterpret_cast<const char*>(&excl), sizeof(excl));
    // otherwise an ICMP "port unreachable" after a send makes the next recvfrom fail
    BOOL report = FALSE;
    DWORD bytes = 0;
    ::WSAIoctl(s, SIO_UDP_CONNRESET, &report, sizeof(report), nullptr, 0, &bytes, nullptr, nullptr);
    u_long nonblocking = 1;
    return ::ioctlsocket(s, FIONBIO, &nonblocking) == 0;
}
int pollIn(Sock s, int timeout_ms, bool& invalid) {
    WSAPOLLFD p{};
    p.fd = s;
    p.events = POLLRDNORM;
    const int r = ::WSAPoll(&p, 1, timeout_ms);
    invalid = r > 0 && (p.revents & POLLNVAL) != 0;
    return r;
}
#else
using Sock = int;
constexpr intptr_t kNoSocket = -1;

bool netInit(std::string&) { return true; }
int lastError() { return errno; }
std::string errorText(int err) { return std::strerror(err); }
bool isAddrInUse(int err) { return err == EADDRINUSE; }
bool isTransient(int err) { return err == EAGAIN || err == EWOULDBLOCK || err == EINTR || err == ECONNREFUSED; }
void closeSock(intptr_t s) { ::close(static_cast<Sock>(s)); }
bool setupSocket(Sock s) {
    // no SO_REUSEADDR: with it a second GCS would silently share the port
    // (and the vehicle's packets) instead of being told it is taken
    const int flags = ::fcntl(s, F_GETFL, 0);
    return flags >= 0 && ::fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0 && ::fcntl(s, F_SETFD, FD_CLOEXEC) == 0;
}
int pollIn(Sock s, int timeout_ms, bool& invalid) {
    pollfd p{};
    p.fd = s;
    p.events = POLLIN;
    const int r = ::poll(&p, 1, timeout_ms);
    invalid = r > 0 && (p.revents & POLLNVAL) != 0;
    if (r < 0 && errno == EINTR) {
        return 0;
    }
    return r;
}
#endif

Sock sock(intptr_t fd) { return static_cast<Sock>(fd); }

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
    return fd_ != kNoSocket;
}

bool UdpTransport::open(std::string& error) {
    close();
    if (!netInit(error)) {
        return false;
    }
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

    const Sock s = ::socket(AF_INET, SOCK_DGRAM, 0);
#if defined(_WIN32)
    if (s == INVALID_SOCKET) {
#else
    if (s < 0) {
#endif
        error = "socket: " + errorText(lastError());
        return false;
    }
    if (!setupSocket(s)) {
        error = "socket setup: " + errorText(lastError());
        closeSock(static_cast<intptr_t>(s));
        return false;
    }
    if (::bind(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        const int err = lastError();
        error = isAddrInUse(err) ? "port " + std::to_string(port_) + " is in use by another program"
                                 : "bind " + bind_host_ + ":" + std::to_string(port_) + ": " + errorText(err);
        closeSock(static_cast<intptr_t>(s));
        return false;
    }
    sockaddr_in bound{};
    socklen_t bound_len = sizeof(bound);
    bound_port_ = ::getsockname(s, reinterpret_cast<sockaddr*>(&bound), &bound_len) == 0 ? ntohs(bound.sin_port) : port_;
    fd_ = static_cast<intptr_t>(s);
    if (!remote_host_.empty()) {
        std::memcpy(peer_.data(), &remote, sizeof(remote));
        peer_known_ = true;
    }
    return true;
}

void UdpTransport::close() {
    if (fd_ != kNoSocket) {
        closeSock(fd_);
        fd_ = kNoSocket;
    }
    peer_known_ = false;
}

int UdpTransport::waitReadable(int timeout_ms, std::string& error) {
    if (fd_ == kNoSocket) {
        error = "socket closed";
        return -1;
    }
    bool invalid = false;
    const int ready = pollIn(sock(fd_), timeout_ms, invalid);
    if (ready < 0) {
        error = "poll: " + errorText(lastError());
        return -1;
    }
    if (invalid) {
        error = "socket closed";
        return -1;
    }
    return ready > 0 ? 1 : 0;
}

long UdpTransport::read(uint8_t* buf, size_t max, std::string& error) {
    if (fd_ == kNoSocket) {
        error = "socket closed";
        return -1;
    }
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);
    const auto n = ::recvfrom(sock(fd_), reinterpret_cast<char*>(buf), static_cast<int>(max), 0,
                              reinterpret_cast<sockaddr*>(&from), &from_len);
    if (n < 0) {
        const int err = lastError();
        if (isTransient(err)) {
            return 0;
        }
        error = "recvfrom: " + errorText(err);
        return -1;
    }
    if (n > 0) {
        std::memcpy(peer_.data(), &from, sizeof(from));
        peer_known_ = true;
    }
    return static_cast<long>(n);
}

void UdpTransport::write(const uint8_t* data, size_t len) {
    if (fd_ == kNoSocket || !peer_known_) {
        return;
    }
    sockaddr_in to{};
    std::memcpy(&to, peer_.data(), sizeof(to));
    ::sendto(sock(fd_), reinterpret_cast<const char*>(data), static_cast<int>(len), 0,
             reinterpret_cast<const sockaddr*>(&to), sizeof(to));
}

std::string UdpTransport::describe() const {
    std::string s = "UDP " + bind_host_ + ":" + std::to_string(port_);
    if (!remote_host_.empty()) {
        s += " → " + remote_host_ + ":" + std::to_string(remote_port_);
    }
    return s;
}

}  // namespace gcs::link
