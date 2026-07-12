#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

int main() {
    //创建socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "socket 创建失败" << std::endl;
        return -1;
    }

    //配置目标服务的IP和端口
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(8080); // 目标端口8080

    //将IP地址从文本转换为二进制形式
    if (inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr) <= 0) {
        std::cerr << "IP地址转换失败" << std::endl;
        return -1;
    }
    std::cout << "IP地址转换成功" << std::endl;
    
    //连接到服务器
    if (connect(sock, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "连接服务器失败" << std::endl;
        return -1;
    }
    std::cout << "连接服务器成功" << std::endl;

    //发送和接受数据
    const char* message = "Hello, Server!";
    send(sock, message, strlen(message), 0);
    std::cout << "消息已发送: " << message << std::endl;

    char buffer[1024] = {0};
    read(sock, buffer, 1024);
    std::cout << "收到服务器响应: " << buffer << std::endl;

    close(sock);
    return 0;
}