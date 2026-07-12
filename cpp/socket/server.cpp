#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

int main() {
    //创建socket(AF_INET:IPv4, SOCK_STREAM:TCP, 0:默认协议)
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) {
        std::cerr << "socket 创建失败" << std::endl;
        return -1;
    }

    //绑定IP和端口
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY; // 绑定到所有可用的网络接口
    address.sin_port = htons(8080); // 绑定端口8080

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "bind 绑定失败" << std::endl;
        return -1;
    }

    //监听端口
    if (listen(server_fd, 3) < 0) {
        std::cerr << "listen 监听失败" << std::endl;
        return -1;
    }
    std::cout << "服务器已启动，等待客户端连接..." << std::endl;

    //接受客户端连接
    int addrlen = sizeof(address);
    int new_socket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
    if (new_socket < 0) {
        std::cerr << "accept 接受连接失败" << std::endl;
        return -1;
    }
    std::cout << "客户端已连接" << std::endl;

    //接受和发送客户端数据
    char buffer[1024] = {0};
    read(new_socket, buffer, 1024);
    std::cout << "收到客户端数据: " << buffer << std::endl;

    const char* hello = "Hello, client!";
    send(new_socket, hello, strlen(hello),0);
    std::cout << "发送数据给客户端: " << hello << std::endl;

    close(new_socket);
    close(server_fd);

    return 0;
}