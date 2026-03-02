#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

int main() {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) {
        std::cerr << "Socket 常见失败..." <<std::endl;
        return -1;
    }
    std::cout << "服务器已启动，等待连接..." <<std::endl;
    close(server_fd);
    return 0;
}