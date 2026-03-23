#include <iostream>
#include <sys/socket.h>// 提供网络编程(Socket)最基础的函数
#include <netinet/in.h>// 提供网络地址格式的结构体 (比如 sockaddr_in)
#include <unistd.h>// 提供操作系统级别的系统调用，比如 close()
#include "ThreadPool.h"
#include "HttpHandler.h"


const int PORT = 8080;// 服务器监听的端口号
const int THREAD_POOL_SIZE = 8;// 线程池里预先创建 8 个工作线程

int main(){
    //初始化线程池
    ThreadPool pool(THREAD_POOL_SIZE);
    std::cout << "Web 服务器启动，使用"<< THREAD_POOL_SIZE <<"个线程"<<std::endl;

    //创建socket
    // socket() 返回一个整数(文件描述符)，你可以把它当成这个网络连接的“身份证号”。
    // AF_INET: 使用 IPv4 地址; SOCK_STREAM: 使用面向连接的 TCP 协议; 0: 默认协议
    int serverSocket = socket(AF_INET,SOCK_STREAM,0);
    if (serverSocket == -1) {
        std::cerr << "创建Socket失败！" <<std::endl;
        return -1;
    }

    //允许端口复用
    //设置端口复用：防止服务器刚关闭时，端口还处于 TIME_WAIT 状态导致无法立即重启
    int opt = 1;
    setsockopt(serverSocket,SOL_SOCKET,SO_REUSEADDR, &opt,sizeof(opt));

    //绑定端口
    // 准备一个结构体，用来存放服务器的地址信息
    sockaddr_in serverAddress{};
    serverAddress.sin_family = AF_INET;// 必须和前面一样是 IPv4
    serverAddress.sin_addr.s_addr = INADDR_ANY;// 允许接收来自任何机器的 IP 连接请求
    serverAddress.sin_port = htons(PORT);// htons() 是个转换函数：把我们电脑上的数字格式，转换成网络传输的标准格式 (网络字节序)

    //正式把 Socket 和上面配置的地址/端口绑定起来
    if(bind(serverSocket,(struct sockaddr*)&serverAddress,sizeof(serverAddress)) < 0) {
        std::cerr << "绑定端口"<<PORT << "失败！"<< std::endl;
        return -1;
    }

    //监听连接
    // 10 表示“全连接队列”的长度，即同时允许最多 10 个客户端在门口排队等候处理
    if(listen(serverSocket,10) < 0) {
        std::cerr << "监听失败！" <<std::endl;
        return -1;
    }

    std::cout << "服务器正在监听端口：" <<PORT <<"(http://localhost:"<<PORT<<")"<<std::endl;

    //事件循环：接收连接并放入线程池
    while (true) {
        // 这两个变量用来保存即将连进来的客户端的地址信息
        sockaddr_in clientAddress{};
        socklen_t clineLen = sizeof(clientAddress);

        // accept() 是一个“阻塞”函数：如果没有人访问，程序就卡在这里不动
        // 一旦有人在浏览器访问了，accept 就会返回一个新的 Socket (clientSocket)，专门用来和这个客户通信
        int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddress,&clineLen);
        
        if (clientSocket < 0) {
            std::cerr << "接收连接失败！" << std::endl;
            continue;
        }
        //分发任务给线程池
        // 主线程不负责具体的交流，它用 Lambda 表达式 [clientSocket]() {...} 把任务打包
        pool.enqueue([clientSocket]() {
            HttpHandler::handlerConnection(clientSocket);
        });
    }

    close(serverSocket);
    return 0;
}