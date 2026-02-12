#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>

class UDPSender {
public:
    UDPSender(const std::string& ip, int port) {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        addr.sin_addr.s_addr = inet_addr(ip.c_str());
    }

    void send(const std::string& message) {
        sendto(sock, message.c_str(), message.size(), 0, (struct sockaddr*)&addr, sizeof(addr));
    }

    ~UDPSender() { close(sock); }

private:
    int sock;
    struct sockaddr_in addr;
};