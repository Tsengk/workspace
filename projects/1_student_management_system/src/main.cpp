#include "StudentManager.h"
#include <iostream>

void showMenu() {
    std::cout << "\n===学生信息管理系统===\n";
    std::cout << "1.录入学生信息\n";
    std::cout << "2.显示所有学生\n";
    std::cout << "3.删除学生信息\n";
    std::cout << "0.退出系统\n";
    std::cout << "请选择操作：";
}

int main() {
    StudentManager manager("../data/students.txt");
    int choice;

    while(true) {
        showMenu();
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(1000,'\n');
            std::cout << "输入无效，请输入数字。\n";
            continue;
        }

        switch(choice) {
            case 1:
                manager.addStudent();
                break;
            case 2:
                manager.displayAll();
                break;
            case 3:
                manager.removeStudent();
                break;
            case 0:
                std::cout << "系统已退出。\n";
                return 0;
            default:
                std::cout << "无效的选项，请重新输入。\n";
                break;
        }
    }
    return 0;
}