#include <iostream>
#include <deque>

int main() {
    std::deque<int> mydeque = {20,30,40};

    mydeque.push_front(10);
    mydeque.push_back(50);
    std::cout << "==========mydeque===========" << std::endl;
    for(int i : mydeque) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    std::cout << "front:" << mydeque.at(0) << std::endl;
    std::cout << "back:" << mydeque.at(4) << std::endl;

    mydeque.pop_front();
    mydeque.pop_back();
    std::cout << "==========erase_mydeque=======" << std::endl;
    for(int i : mydeque) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    return 0;
}