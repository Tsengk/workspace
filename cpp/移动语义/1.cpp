#include <iostream>
#include <vector>

class dynamicarray {
public:
    int* data;
    size_t size;

    dynamicarray(size_t s) : size(s), data(new int[s]) {}

    dynamicarray(const dynamicarray& other) : size(other.size),data(new int[other.size]) {
        for(int i=0;i<size;++i) {
            data[i] = other.data[i];
            std::cout << "Copy Constructor called\n";
        }
    }
    
    dynamicarray(dynamicarray&& other) noexcept : size(other.size), data(other.data) {
        other.data = nullptr;
        other.size = 0;
        std::cout << "Move Constructor called\n";
    }
};

int main() {
    std::vector<dynamicarray> vec;
    dynamicarray a(1);

    vec.push_back(a);

    vec.push_back(std::move(a));

    return 0;

}