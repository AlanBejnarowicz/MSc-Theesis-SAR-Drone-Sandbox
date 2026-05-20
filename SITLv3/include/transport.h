#pragma once

// ================================================================
// TRANSPORT.H  —  UDP send/receive, isolated from controller logic
//
// SITL:    POSIX sockets (Linux/Mac/Windows with Winsock)
// ESP-IDF: swap socket calls for lwIP equivalents
//          Function signatures stay identical
// ================================================================

#include <string>
#include <cstring>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    using socket_t = SOCKET;
    #define INVALID_SOCK INVALID_SOCKET
    #define CLOSE_SOCK(s) closesocket(s)
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <fcntl.h>
    using socket_t = int;
    #define INVALID_SOCK (-1)
    #define CLOSE_SOCK(s) close(s)
#endif

#include <stdexcept>

class UDPSocket
{
public:
    UDPSocket() = default;
    ~UDPSocket() { shutdown(); }   


    // Open a socket that listens on listenPort
    // and sends to remoteHost:remotePort
    bool open(const std::string& remoteHost,
              int                remotePort,
              int                listenPort,
              int                timeoutMs = 10)
    {
#ifdef _WIN32
        WSADATA wsa;
        WSAStartup(MAKEWORD(2,2), &wsa);
#endif
        _sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (_sock == INVALID_SOCK) return false;

        // Bind to listen port
        sockaddr_in local{};
        local.sin_family      = AF_INET;
        local.sin_port        = htons(listenPort);
        local.sin_addr.s_addr = INADDR_ANY;
        if (bind(_sock, (sockaddr*)&local, sizeof(local)) < 0)
            return false;

        // Increase socket receive buffer
        int bufSize = 2 * 1024 * 1024;  // 2MB
        setsockopt(_sock, SOL_SOCKET, SO_RCVBUF, &bufSize, sizeof(bufSize));

        // Set receive timeout
#ifdef _WIN32
        DWORD tv = timeoutMs;
        setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(tv));
#else
        timeval tv{ timeoutMs / 1000, (timeoutMs % 1000) * 1000 };
        setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

        // Store remote address for send
        _remote.sin_family = AF_INET;
        _remote.sin_port   = htons(remotePort);
        inet_pton(AF_INET, remoteHost.c_str(), &_remote.sin_addr);

        _open = true;
        return true;
    }

    // Send string to remote
    bool send(const std::string& data)
    {
        if (!_open) return false;
        return sendto(_sock, data.c_str(), (int)data.size(), 0,
                      (sockaddr*)&_remote, sizeof(_remote)) > 0;
    }

    // Receive into string, returns false on timeout/error
    bool receive(std::string& out, int maxBytes = 8192)
    {
        if (!_open) return false;
        _buf.resize(maxBytes);
        int n = recvfrom(_sock, _buf.data(), maxBytes, 0, nullptr, nullptr);
        if (n <= 0) return false;
        out.assign(_buf.data(), n);
        return true;
    }

    void shutdown()
    {
        if (_open) {
            CLOSE_SOCK(_sock);
            _open = false;
        }
    }

    bool isOpen() const { return _open; }

private:
    socket_t    _sock  = INVALID_SOCK;
    sockaddr_in _remote{};
    bool        _open  = false;
    std::string _buf;
};

// ================================================================
// ESP-IDF SWAP GUIDE
// Replace socket() / bind() / sendto() / recvfrom() with:
//   lwip_socket(), lwip_bind(), lwip_sendto(), lwip_recvfrom()
// Everything else stays the same — same class, same interface
// ================================================================
