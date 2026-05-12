#include <iostream>
#include <stack>
#include <queue>

int main() {
    std::stack<int> mystack;
    mystack.push(10);
    mystack.push(20);
    mystack.push(30);

    std::cout << "======stack=====" << std::endl;
    while (!mystack.empty())
    {
        std::cout << mystack.top() << " ";
        mystack.pop();
    }
    std::cout << std::endl;


    std::queue<int> myqueue;
    myqueue.push(10);
    myqueue.push(20);
    myqueue.push(30);

    std::cout << "=======queue======" << std::endl;
    while (!myqueue.empty())
    {
        std::cout << myqueue.front() << " ";
        myqueue.pop();
    }
    std::cout << std::endl;
    
    std::priority_queue<int> mypq;
    mypq.push(10);
    mypq.push(30);
    mypq.push(20);

    std::cout << "========priority_queue======" << std::endl;
    while (!mypq.empty())
    {
        std::cout << mypq.top() << " ";
        mypq.pop();
    }
    std::cout << std::endl;

    return 0;
    
    
}