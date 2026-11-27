/* Socket tests, written before the socket wrapper existed.
 *
 * These run the real kernel stack on loopback, in one process, with no fork and
 * no timing assumption. What they check is the three things a datagram wrapper
 * gets wrong: it blocks, it returns stale bytes, or it reports a port it did not
 * get. The two process version is the day 6 study, which is a run and not a
 * test, because a test that spawns nodes and reads a wall clock fails when the
 * machine is busy and that is a property of the runner rather than of the code.
 */

#include <cstdint>

#include "tandem_wire.h"
#include "tsim_test.h"
#include "udp_link.h"

namespace {

std::uint16_t bind_and_report(tsim::UdpSocket *socket) {
    std::string error;
    if (!socket->open(0u, &error)) {
        std::fprintf(stderr, "open failed: %s\n", error.c_str());
        return 0u;
    }
    return socket->local_port();
}

} // namespace

TSIM_TEST(socket_binds_a_port_the_kernel_chose) {
    tsim::UdpSocket socket;
    const std::uint16_t port = bind_and_report(&socket);
    TSIM_CHECK(socket.is_open());
    /* A port of zero would mean the kernel refused and the wrapper did not
     * notice, which is the failure that turns a node into a silent listener. */
    TSIM_CHECK(port != 0u);
}

TSIM_TEST(socket_carries_a_packet_between_two_processes_worth_of_sockets) {
    tsim::UdpSocket sender;
    tsim::UdpSocket receiver;
    const std::uint16_t sender_port = bind_and_report(&sender);
    const std::uint16_t receiver_port = bind_and_report(&receiver);
    TSIM_CHECK(sender_port != 0u);
    TSIM_CHECK(receiver_port != 0u);
    TSIM_CHECK(sender_port != receiver_port);

    tsim::WireSample sent;
    sent.sequence = 7u;
    sent.release_window = 4242u;
    sent.stamp_ns = 123456789ull;
    sent.position_rad = 0.0042f;
    sent.velocity_rad_s = -3.5f;
    sent.torque_nm = 0.61f;
    sent.energy_j = -0.75f;

    std::uint8_t out[tsim::kWireSampleBytes]{};
    (void)tsim::encode_sample(sent, out, sizeof out);
    std::string error;
    TSIM_CHECK(sender.send_to(receiver_port, out, sizeof out, &error));

    std::uint8_t in[tsim::kWireSampleBytes]{};
    const int length = receiver.receive(in, sizeof in);
    TSIM_CHECK(length == static_cast<int>(tsim::kWireSampleBytes));

    tsim::WireSample received;
    TSIM_CHECK(tsim::decode_sample(in, static_cast<std::size_t>(length), &received));
    TSIM_CHECK(received.sequence == sent.sequence);
    TSIM_CHECK(received.release_window == sent.release_window);
    TSIM_CHECK(received.position_rad == sent.position_rad);
    TSIM_CHECK(received.energy_j == sent.energy_j);
}

TSIM_TEST(socket_says_so_when_nothing_is_waiting) {
    /* A node drains the socket every tick and most ticks have nothing in it.
     * Returning the last packet again, or blocking, would make the link look
     * perfect by repeating one sample for ever. */
    tsim::UdpSocket socket;
    (void)bind_and_report(&socket);
    std::uint8_t buffer[tsim::kWireSampleBytes]{};
    TSIM_CHECK(socket.receive(buffer, sizeof buffer) == 0);
    TSIM_CHECK(socket.receive(buffer, sizeof buffer) == 0);
}

TSIM_TEST(socket_drains_what_arrived_in_order_and_then_stops) {
    /* A queue longer than one packet is what the receiver sees when its loop is
     * slower than the sender's, and draining it is how the node finds the newest
     * sample instead of the oldest. */
    tsim::UdpSocket sender;
    tsim::UdpSocket receiver;
    (void)bind_and_report(&sender);
    const std::uint16_t receiver_port = bind_and_report(&receiver);

    for (std::uint32_t sequence = 1u; sequence <= 3u; ++sequence) {
        tsim::WireSample sent;
        sent.sequence = sequence;
        sent.position_rad = 0.001f * static_cast<float>(sequence);
        std::uint8_t out[tsim::kWireSampleBytes]{};
        (void)tsim::encode_sample(sent, out, sizeof out);
        std::string error;
        TSIM_CHECK(sender.send_to(receiver_port, out, sizeof out, &error));
    }

    std::uint8_t in[tsim::kWireSampleBytes]{};
    for (std::uint32_t sequence = 1u; sequence <= 3u; ++sequence) {
        const int length = receiver.receive(in, sizeof in);
        TSIM_CHECK(length == static_cast<int>(tsim::kWireSampleBytes));
        tsim::WireSample received;
        TSIM_CHECK(tsim::decode_sample(in, static_cast<std::size_t>(length), &received));
        TSIM_CHECK(received.sequence == sequence);
    }
    TSIM_CHECK(receiver.receive(in, sizeof in) == 0);
}

TSIM_TEST(socket_refuses_a_send_without_a_peer_and_says_which) {
    tsim::UdpSocket socket;
    (void)bind_and_report(&socket);
    std::uint8_t out[tsim::kWireSampleBytes]{};
    std::string error;
    /* Port zero has no meaning as a peer, so the call has to fail loudly rather
     * than send into the void and let a run look like a link with total loss. */
    TSIM_CHECK(!socket.send_to(0u, out, sizeof out, &error));
    TSIM_CHECK(!error.empty());
}
