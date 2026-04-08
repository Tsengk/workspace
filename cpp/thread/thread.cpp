#include <iostream>
#include <thread>


void worker(int id) {
    std::cout << "线程" << id << "正在运行。。。" << std::endl;
}

int main() {
    std::thread t1(worker,1);

    t1.join();

    return 0;


}