#include <iostream>
#include <stdexcept>
#include "FileHandler.h"
#include "ScopeGuard.h"

void test_file_handler() {
    std::cout << "Testing FileHandler..." << std::endl;
    try {
        FileHandler fh("test.txt", "w");
        std::fputs("Hello, RAII!", fh.get());//使用 get() 方法获取底层 FILE* 指针，直接调用 C 标准库函数写入数据

        std::cout << "准备抛出异常，测试析构函数是否会被调用...\n";
        throw std::runtime_error("Simulated exception");//在抛出异常后，FileHandler 的析构函数应该会被调用，确保文件被正确关闭
    } catch (const std::exception& e) {
        std::cout << "Caught exception: " << e.what() << std::endl;
    }
}

void test_scope_guard() {
    std::cout << "\n--- 测试 ScopeGuard (DEFER) ---\n";
    
    // 模拟一段需要手动释放的内存/资源
    int* raw_array = new int[100];
    std::cout << "动态数组已分配。\n";

    // 声明清理动作
    DEFER(
        delete[] raw_array;
        std::cout << "动态数组已被 DEFER 自动释放。\n";
    );

    // DEFER 支持在同一作用域内定义多个
    DEFER(
        std::cout << "这是另一个清理动作 (注意：析构顺序与构造顺序相反)。\n";
    );

    std::cout << "正在处理数据...\n";
    // 即使函数有多个 return 分支，或者发生异常，DEFER 保证资源一定会被释放
}
int main() {
    test_file_handler();
    test_scope_guard();
    return 0;
}