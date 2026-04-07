#include <iostream>
#include <vector>
#include <cstring> // For strlen 用来计算字符串长度`

class StringProcessor {
    char* data;
    size_t length;

    // 默认构造函数
    StringProcessor(const char* s) {
        std::cout << "Construct: allocate " << strlen(s) << " bytes.\n";
        length = strlen(s);
        data = new char[length + 1]; // 1. 获取资源 (Heap Allocation)
        memcpy(data, s, length + 1);//把s的length + 1个字节（包括'\0'）复制到data中
    }

    // 1. 传统的拷贝构造函数 (Deep Copy)
    StringProcessor(const StringProcessor& other) {
        std::cout << "Copy Construct: DEEP COPY " << other.length << " bytes.\n";
        length = other.length;
        data = new char[length + 1];
        memcpy(data,other.data,length + 1);
    }

    // 2. 移动构造函数
    // 底层逻辑：参数是右值引用。将源对象的指针赋给新对象，然后将源对象置空。
    StringProcessor(StringProcessor&& other) noexcept : data(other.data), length(other.length) {
        std::cout << "Move Construct: TAKE OWNERSHIP of " << length << " bytes.\n";
        // 关键操作：将源对象置为空状态，防止它析构时释放了我们刚偷来的资源
        other.data = nullptr;
        other.length = 0;
    }
}