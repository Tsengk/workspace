#include <iostream>
#include <set>

int main() {
    std::set<int> myset;

    myset.insert(10);
    myset.insert(50);
    myset.insert(40);
    myset.insert(10);

    for(auto& i : myset) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}