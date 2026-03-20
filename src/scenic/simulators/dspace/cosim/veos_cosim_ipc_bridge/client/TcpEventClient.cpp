#include "TcpEventClient.h"

#include <ws2tcpip.h>

TcpEventClient::TcpEventClient() : sock_(INVALID_SOCKET), wsa_started_(false) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) == 0) {
        wsa_started_ = true;
    }
}

TcpEventClient::~TcpEventClient() {
    Disconnect();
    if (wsa_started_) {
        WSACleanup();
    }
}

bool TcpEventClient::Connect(const std::string& host, unsigned short port) {
    Disconnect();
    if (!wsa_started_) return false;

    sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock_ == INVALID_SOCKET) return false;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if (InetPtonA(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        closesocket(sock_);
        sock_ = INVALID_SOCKET;
        return false;
    }

    if (connect(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        closesocket(sock_);
        sock_ = INVALID_SOCKET;
        return false;
    }

    return true;
}

void TcpEventClient::Disconnect() {
    if (sock_ != INVALID_SOCKET) {
        closesocket(sock_);
        sock_ = INVALID_SOCKET;
    }
}

bool TcpEventClient::IsConnected() const {
    return sock_ != INVALID_SOCKET;
}

bool TcpEventClient::SendLine(const std::string& line) {
    if (sock_ == INVALID_SOCKET) return false;
    const char* data = line.c_str();
    int remaining = static_cast<int>(line.size());
    while (remaining > 0) {
        int sent = send(sock_, data, remaining, 0);
        if (sent == SOCKET_ERROR || sent == 0) return false;
        data += sent;
        remaining -= sent;
    }
    return true;
}
