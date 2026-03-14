#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include "ThreadPool.h"
#include "HttpHandler.h"


const int PORT = 8080;
const int THREAD_POOL_SIZE = 8;

int main(){
    //初始化线程池
    ThradPool pool(THREAD_POOL_SIZE);
    std::cout << "Web 服务器启动，使用"<< THREAD_POOL_SIZE <<"个线程"<<std::endl;

    //创建socket
    int serverSocket = socket(AF_INET,SOCK_STREAM,0);
    if (serverSocket == -1) {
        std::cerr << "创建Socket失败！" <<std::endl;
        return -1;
    }

    //允许端口复用
    int opt = 1;
    setsockopt(serverSocket,SOL_SOCKET,SO_REUSEADDR, &opt,sizeof(opt));

    //绑定端口
    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_addr = INADDR_ANY;
    serverAddress.sin_port = htons(Port);

    if(bind(serverSocket,(struct sockaddr*)&serverAddress,sizeof(serverAddress)) < 0) {
        std::cerr << "绑定端口"<<PORT << "失败！"<< std::endl;
        return -1;
    }

    //监听连接
    if(listen(serverSocket,10) < 0) {
        std::cerr << "监听失败！" <<std::endl;
        return -1;
    }

    std::cout << "服务器正在监听端口：" <<PORT <<"(http://localhost:"<<PORT<<")"<<std::endl;

    //事件循环：接收连接并放入线程池
    while (true) {
        sockaddr_in clientAddress{};
        socklen_t clineLen = sizeof(clientAddress);
        int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddress,&clientLen);
        
        if (clintSocket < 0) {
            std::cerr << "接收连接失败！" << std::endl;
            contiue;
        }
        
        pool.enqueue([clientSocket]() {
            HttpHandler::handleConnection(clientSocket);
        });
    }

    close(serverSocket);
    return 0;
}