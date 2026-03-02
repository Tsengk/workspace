#include "StudentManager.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>

StudentManager::StudentManager(const std::string& filePath) : dataFiles(filePath) {
    loadFromFile();
}

void StudentManager::addStudent() {
    std::string id,name;
    double score;
    std::cout << "请输入学号：";
    std::cin >> id;
    std::cout << "请输入姓名：";
    std::cin >> name;
    std::cout << "请输入成绩：";
    std::cin >> score;

    students.emplace_back(id,name,score);
    std::cout << "添加成功！" << std::endl;
    saveToFile();
}

void StudentManager::displayAll() const {
    if(students.empty()) {
        std::cout << "当前没有学生记录。" << std::endl;
        return;
    }
    std::cout << "-----------------------------------" << std::endl;
    std::cout << std::left << std::setw(15) << "学号" << std::setw(20) << "姓名" <<std::setw(10) << "成绩" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    for(const auto& student : students) {
        student.printInfo();
    }
    std::cout << "-----------------------------------" << std::endl;
}

void StudentManager::removeStudent() {
    std::string targetId;
    std::cout << "请输入要删除的学生学号：";
    std::cin >> targetId;

    auto it = std::remove_if(students.begin(),students.end(),[&targetId](const Student& s) {return s.id == targetId;});

    if (it != students.end()) {
        students.erase(it,students.end());
        std::cout << "删除成功！";
        saveToFile();
    }else {
        std::cout << "未找到学号为" << targetId << "的学生。" << std::endl;
    }
}

void StudentManager::saveToFile() const {
    std::ofstream outfile(dataFiles);
    if (!outfile.is_open()) {
        std::cerr << "错误：无法打开文件保持数据！" << std::endl;
        return;
    }
    for (const auto& s : students) {
        outfile << s.id << " " << s.name << " " << s.score << std::endl;
    }
    outfile.close();
}

void StudentManager::loadFromFile() {
    std::ifstream infile(dataFiles);
    if (!infile.is_open()) {
        return;
    }

    students.clear();
    std::string id,name;
    double score;
    while (infile >> id >> name >> score) {
        students.emplace_back(id,name,score);
    }
    infile.close();
}