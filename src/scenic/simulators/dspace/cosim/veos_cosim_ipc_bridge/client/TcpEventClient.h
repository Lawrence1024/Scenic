#pragma once

#include <string>
#include <winsock2.h>

#pragma comment(lib, "Ws2_32.lib")

class TcpEventClient {
public:
    TcpEventClient();
    ~TcpEventClient();

    bool Connect(const std::string& host, unsigned short port);
    void Disconnect();
    bool IsConnected() const;

    bool SendLine(const std::string& line);
    bool ReceiveLine(std::string& outLine);
    bool SendAndWaitLine(const std::string& line, const std::string& expectedReply);

private:
    SOCKET sock_;
    bool wsa_started_;
};