#ifndef CALCULATOR_H
#define CALCULATOR_H

#include <string>

//Calculator 类：负责解析并计算数学表达式字符串
class Calculator {
public:
    //核心接口：传入字符串，返回计算结果
    double evaluate(const std::string& expression);

private:
    //辅助函数：解析表达式，处理运算符优先级等
    int getPrecedence(char op);

    //辅助函数：执行基本运算
    double applyOp(double a,double b, char op);
};

#endif // CALCULATOR_H