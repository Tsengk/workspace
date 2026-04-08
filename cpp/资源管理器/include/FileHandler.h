#ifndef FILE_HANDLER_H
#define FILE_HANDLER_H

#include <cstdio>
#include <string>

class FileHandler {
private:
    std::FILE* file_ptr;

public:
    FileHandler(const std::string& filename,const std::string& mode);

    ~FileHandler();

    //防止重复释放
    FileHandler(const FileHandler&) = delete;//禁止拷贝构造函数
    FileHandler& operator=(const FileHandler&) = delete;//禁止拷贝赋值运算符

    //允许移动构造和移动赋值（转移资源所有权）
    FileHandler(FileHandler&& other) noexcept;
    FileHandler& operator=(FileHandler&& other) noexcept;

    //提供对底层资源的访问，成员函数不允许修改成员变量，返回类型为FILE*，以便用户可以直接使用C标准库函数操作文件
    std::FILE* get() const;

    //检查资源是否有效,成员函数不允许修改成员变量，返回类型为bool，表示资源是否有效
    bool isValid() const;
};
#endif // FILE_HANDLER_H