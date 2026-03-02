#include "Student.h"
#include <iostream>
#include <iomanip>

Student::Student(std::string id,std::string name,double score):id(id),name(name),score(score){}

void Student::printInfo() const {
    //使用iomanip库对齐输出
    std::cout << std::left << std::setw(13) << id << std::setw(18) << name << std::setw(20) << score << std::endl;
}