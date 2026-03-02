#ifndef STUDENT_H
#define STUDENT_H

#include <string>

class Student {
public:
    std::string id;
    std::string name;
    double score;

    Student() = default;

    Student(std::string id,std::string name,double score);

    void printInfo() const;
};

#endif