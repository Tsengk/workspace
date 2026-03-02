#ifndef STUDENTMANAGER_H
#define STUDENTMANAGER_H

#include "Student.h"
#include <vector>
#include <string>

//StudentManager类：负责处理增删改查及文件I、O逻辑
class StudentManager {
private:
    std::vector<Student> students; //内存中的学生数据容器
    std::string dataFiles; //数据保持的文件路径
public:
    StudentManager(const std::string& filePath);

    void addStudent();

    void displayAll() const;

    void removeStudent();

    void loadFromFile();

    void saveToFile() const;
};

#endif