#include "Calculator.h"
#include <iostream>
#include <string>

int main() {
    Calculator calc;
    std::string input;

    std::cout << "===表达式计算器===\n";
    std::cout << "输入数学表达式，支持 （+ - * /） 、括号以及小数。\n";
    std::cout << "输入 'q' 或 'exit' 退出程序。\n";

    while (true) {
        std::cout << "请输入表达式: ";
        std::getline(std::cin,input);

        if (input == "q" || input == "ecit") {
            std::cout << "退出程序。再见！\n";
            break;
        }

        try {
            double result = calc.evaluate(input);
            std::cout << "结果: " << result << "\n";
        } catch (const std::exception& e) {
            std::cout << "错误: " << e.what() << "\n";
        }
    }
    return 0;
}