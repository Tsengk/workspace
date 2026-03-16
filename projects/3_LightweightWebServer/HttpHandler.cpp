#include "HttpHandler.h"
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <sys/socket.h>

void HttpHandler::handlerConnection(int clientSocket) {
    char buffer[4096] = {0};

    //读取HTTP请求
    ssize_t bytesRead = read(clientSocket, buffer, sizeof(buffer) - 1);
    if(bytesRead <=0) {
        close(clientSocket);
        return;
    }

    //简单打印请求行
    std::string request(buffer);
    std::istringstream requestStream(request);
    std::string requestLine;
    std::getline(requestStream,requestLine);
    std::cout << "收到请求："<<requestLine << std::endl;

    //读取本地index.html文件
    std::ifstream file("webroot/index.html");
    std::string responseBody;
    if (file.is_open()) {
        std::stringstream buffer;
        buffer  << file.rdbuf();
        responseBody = buffer.str();
    }else {
        responseBody = "<h1>404 Not Found</h1>";
    }

    //构建HTTP响应报文
    std::string httpResponse =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type:text/html;charset=UTF-8\r\n"
        "Content-Length:" + std::to_string(responseBody.size()) + "\r\n"
        "Connection:close\r\n\r\n" + 
        responseBody;

    //发送响应并关闭连接
    write(clientSocket,httpResponse.c_str(),httpResponse.size());
    close(clientSocket);
}