/* udp_link.h, a real datagram socket between two processes.
 *
 * This is the part of day 6 that is not a simulation. Two drives are two
 * programs, and the only way they can be two programs exchanging a joint angle
 * at 20 kHz is a socket. A shared variable in one process is a link with no
 * transport, and the day 5 study already showed what a link costs; what it
 * could not show is what the operating system adds on top.
 *
 * One socket per side, bound to a chosen loopback port, non blocking. Sending
 * to a port that has not been bound yet is not an error on a datagram socket,
 * the packet is simply lost, which is what happens on a network and is the
 * reason the node counts what it receives rather than assuming.
 *
 * POSIX only, because the harness builds on the runner and on the sandbox and
 * both are Linux. A Windows build would need a second implementation of these
 * five calls, and there is no Windows in this project.
 */
#ifndef TSIM_HARNESS_UDP_LINK_H
#define TSIM_HARNESS_UDP_LINK_H

#include <cstddef>
#include <cstdint>
#include <string>

namespace tsim {

class UdpSocket {
  public:
    UdpSocket() = default;
    ~UdpSocket();
    UdpSocket(const UdpSocket &) = delete;
    UdpSocket &operator=(const UdpSocket &) = delete;

    /* Bind to loopback. Port 0 asks the kernel for a free one and local_port()
     * then reports what it chose. */
    bool open(std::uint16_t local_port, std::string *error);
    void close();
    bool is_open() const;

    /* The port actually bound, which is not the one asked for when it was 0. */
    std::uint16_t local_port() const;

    bool send_to(std::uint16_t peer_port, const std::uint8_t *data, std::size_t length,
                 std::string *error);

    /* Bytes read, 0 when nothing is waiting, -1 on error. Never blocks, because
     * a node that blocked on a link would stop being a control loop. */
    int receive(std::uint8_t *buffer, std::size_t capacity);

    /* Bytes read, 0 if nothing arrived inside the timeout, -1 on error. The
     * lockstep in tandem_node.cpp waits here for the peer, and it is the only
     * blocking call in the harness: a node that waits for a peer that has died
     * has to be able to say so instead of stopping for ever. */
    int receive_waiting(std::uint8_t *buffer, std::size_t capacity, int timeout_ms);

  private:
    int fd_ = -1;
    std::uint16_t local_port_ = 0;
};

} // namespace tsim

#endif /* TSIM_HARNESS_UDP_LINK_H */
