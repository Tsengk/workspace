#include <iostream>

void countCalls() {
    //static int count = 0;
    int count = 0;
    count++;
    std::cout << "函数被调用了 " << count <<std::endl;
}

int main() {
    countCalls();
    countCalls();
    countCalls();
    return 0;
}