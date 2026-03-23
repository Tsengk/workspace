#include <iostream>
#include <memory>
#include <vector>

// 1. 模拟一个需要管理资源的类
class DataBuffer {
public:
    int* data;
    size_t size;

    // 传统构造函数：手动分配堆内存
    DataBuffer(size_t s) : size(s) {
        data = new int[size];// 1. 获取资源 (Heap Allocation)
        std::cout << "DataBuffer: Hand-allocated->" << size << "-<ints.\n";
     }

    // 析构函数：手动释放堆内存
    ~DataBuffer() {
        delete[] data; // 2. 释放资源 (Manual Deallocation)
        std::cout << "DataBuffer: Hand-deallocated\n";
    }
};

// 2. RAII 演示：使用对象管理资源
void demonstrate_raii() {
    std::cout << "--- Enter demonstrate_raii ---\n";

    DataBuffer buffer(100);

    std::cout << "--- Exit demonstrate_raii (Auto-析构) ---\n";
}

// 3. 智能指针底层逻辑
void demonstrate_smart_pointers() {
    std::cout << "\n--- Enter demonstrate_smart_pointers ---\n";


    // std::unique_ptr (C++11)
    // 保证同一时间只有一个 unique_ptr 拥有该资源。所有权转移必须用 std::move。
    //std::unique_ptr<DataBuffer> 管理一个堆对象
    //make_unique<DataBuffer>(200)
    //--在堆上创建一个 DataBuffer(200)对象
    //--用 unique_ptr 接管它，确保它在 unique_ptr 销毁时自动释放
    std::unique_ptr<DataBuffer> u_ptr = std::make_unique<DataBuffer>(200);


    // std::shared_ptr (C++11)
    // 底层逻辑：它包含两个指针：一个指向 DataBuffer 对象，另一个指向“控制块”。
    //std::shared_ptr<DataBuffer> 管理另一个堆对象并引入控制块
    //make_shared<DataBuffer>(300)
    //--在堆上创建一个 DataBuffer(300)对象
    //--用 shared_ptr 接管它，确保它在 shared_ptr 销毁时自动释放
    std::shared_ptr<DataBuffer> s_ptr1 = std::make_shared<DataBuffer>(300);

    //作用域内创建另一个 shared_ptr，指向同一对象，增加引用计数,初始引用计数为1
    {
        std::shared_ptr<DataBuffer> s_ptr2 = s_ptr1;
        std::cout << "s_ptr1 count:" << s_ptr1.use_count() << "\n";
        std::cout << "s_ptr1 count:" << s_ptr2.use_count() << "\n";
    }
    //离开作用域后，s_ptr2自动销毁，引用计数减少
    std::cout << "s_ptr1 count:" << s_ptr1.use_count() << "\n";
    std::cout << "--- Exit demonstrate_smart_pointers (Auto-析构) ---\n";   
}


int main() {
    demonstrate_raii();
    demonstrate_smart_pointers();
    return 0;
}