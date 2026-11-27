#include "udp_link.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace tsim {
namespace {

/* A megabyte each way. The kernel default on loopback is a few hundred
 * kilobytes, which at 40 bytes a packet is tens of thousands of samples, more
 * than a delayed link needs and less than a receiver that stops reading will
 * eventually overflow. The overflow is left in: it is what a real socket does
 * when the far side stalls, and a link that never dropped would be a link with
 * a queue this project did not write. */
constexpr unsigned kSocketBufferBytes = 1u << 20u;

std::string errno_text() {
    return std::string(std::strerror(errno));
}

} // namespace

UdpSocket::~UdpSocket() {
    close();
}

bool UdpSocket::open(std::uint16_t local_port, std::string *error) {
    close();
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        if (error != nullptr) {
            *error = errno_text();
        }
        return false;
    }

    int buffer_bytes = static_cast<int>(kSocketBufferBytes);
    (void)::setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &buffer_bytes, sizeof buffer_bytes);
    (void)::setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &buffer_bytes, sizeof buffer_bytes);

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(local_port);
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::bind(fd_, reinterpret_cast<const sockaddr *>(&address), sizeof address) != 0) {
        if (error != nullptr) {
            *error = errno_text();
        }
        close();
        return false;
    }

    sockaddr_in bound{};
    socklen_t bound_length = sizeof bound;
    if (::getsockname(fd_, reinterpret_cast<sockaddr *>(&bound), &bound_length) == 0) {
        local_port_ = ntohs(bound.sin_port);
    }

    const int flags = ::fcntl(fd_, F_GETFL, 0);
    /* Both operands unsigned before the or, because a signed flag word is the
     * kind of thing that is right until a bit above 30 shows up. */
    const unsigned non_blocking = static_cast<unsigned>(O_NONBLOCK);
    if (flags < 0 ||
        ::fcntl(fd_, F_SETFL, static_cast<int>(static_cast<unsigned>(flags) | non_blocking)) != 0) {
        if (error != nullptr) {
            *error = errno_text();
        }
        close();
        return false;
    }
    return true;
}

void UdpSocket::close() {
    if (fd_ >= 0) {
        (void)::close(fd_);
        fd_ = -1;
    }
    local_port_ = 0u;
}

bool UdpSocket::is_open() const {
    return fd_ >= 0;
}

std::uint16_t UdpSocket::local_port() const {
    return local_port_;
}

bool UdpSocket::send_to(std::uint16_t peer_port, const std::uint8_t *data, std::size_t length,
                        std::string *error) {
    if (!is_open()) {
        if (error != nullptr) {
            *error = "socket is not open";
        }
        return false;
    }
    if (peer_port == 0u) {
        if (error != nullptr) {
            *error = "peer port 0 has no meaning";
        }
        return false;
    }
    sockaddr_in peer{};
    peer.sin_family = AF_INET;
    peer.sin_port = htons(peer_port);
    peer.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    const ssize_t sent =
        ::sendto(fd_, data, length, 0, reinterpret_cast<const sockaddr *>(&peer), sizeof peer);
    if (sent != static_cast<ssize_t>(length)) {
        if (error != nullptr) {
            *error = errno_text();
        }
        return false;
    }
    return true;
}

int UdpSocket::receive_waiting(std::uint8_t *buffer, std::size_t capacity, int timeout_ms) {
    if (!is_open()) {
        return -1;
    }
    pollfd waiting{};
    waiting.fd = fd_;
    waiting.events = POLLIN;
    const int ready = ::poll(&waiting, 1, timeout_ms);
    if (ready < 0) {
        if (errno == EINTR) {
            return 0;
        }
        return -1;
    }
    if (ready == 0) {
        return 0;
    }
    return receive(buffer, capacity);
}

int UdpSocket::receive(std::uint8_t *buffer, std::size_t capacity) {
    if (!is_open()) {
        return -1;
    }
    const ssize_t read = ::recv(fd_, buffer, capacity, 0);
    if (read < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        return -1;
    }
    return static_cast<int>(read);
}

} // namespace tsim
