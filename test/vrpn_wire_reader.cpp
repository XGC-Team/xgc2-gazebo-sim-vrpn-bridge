// Test-only raw VRPN connection reader: register both senders and inspect the
// received payloads before any tracker-specific ROS filtering.
#include <vrpn_Connection.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <sys/syscall.h>
#include <unistd.h>

static unsigned long udp_reads = 0, tcp_reads = 0;
extern "C" ssize_t recv(int fd, void* buf, size_t len, int flags) {
    const auto result = syscall(SYS_recvfrom, fd, buf, len, flags, nullptr, nullptr);
    if (result > 0) {
        int type = 0; socklen_t size = sizeof(type);
        if (getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &size) == 0) {
            if (type == SOCK_DGRAM) ++udp_reads;
            if (type == SOCK_STREAM) ++tcp_reads;
        }
    }
    return result;
}
extern "C" ssize_t read(int fd, void* buf, size_t len) {
    const auto result = syscall(SYS_read, fd, buf, len);
    if (result > 0) {
        int type = 0; socklen_t size = sizeof(type);
        if (getsockopt(fd, SOL_SOCKET, SO_TYPE, &type, &size) == 0 && type == SOCK_STREAM) ++tcp_reads;
    }
    return result;
}
struct Context { vrpn_Connection* connection; const char* kind; };
static int VRPN_CALLBACK receive(void* arg, vrpn_HANDLERPARAM p) {
    auto* ctx = static_cast<Context*>(arg);
    const char* sender = ctx->connection->sender_name(p.sender);
    std::printf("WIRE %s %s %ld %ld ", sender ? sender : "?", ctx->kind,
                static_cast<long>(p.msg_time.tv_sec), static_cast<long>(p.msg_time.tv_usec));
    for (int i = 0; i < p.payload_len; ++i) std::printf("%02x", static_cast<unsigned char>(p.buffer[i]));
    std::putchar('\n');
    return 0;
}
int main(int argc, char** argv) {
    if (argc != 3) return 2;
    auto* connection = vrpn_get_connection_by_name(argv[1]);
    if (!connection) return 3;
    connection->register_sender("uav1"); connection->register_sender("uav2");
    const char* types[] = {"vrpn_Tracker Pos_Quat", "vrpn_Tracker Velocity", "vrpn_Tracker Acceleration"};
    Context contexts[] = {{connection,"pose"}, {connection,"velocity"}, {connection,"acceleration"}};
    for (int i=0; i<3; ++i)
        if (connection->register_handler(connection->register_message_type(types[i]), receive, &contexts[i], vrpn_ANY_SENDER)) return 4;
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(std::atoi(argv[2]));
    while (std::chrono::steady_clock::now() < end) { connection->mainloop(); usleep(1000); }
    std::printf("SOCKET udp=%lu tcp=%lu\n", udp_reads, tcp_reads);
    connection->removeReference();
}
