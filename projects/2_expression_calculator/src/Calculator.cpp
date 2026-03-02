#include "Calculator.h"
#include <stack>
#include <cctype>
#include <stdexcept>

int Calculator::getPrecedence(char op) {
    if (op == '+' || op == '-') return 1;
    if (op == '*' || op == '/') return 2;
    return 0;//括号或其他字符
}

double Calculator::applyOp(double a, double b, char op) {
    switch (op) {
        case '+': return a + b;
        case '-': return a - b;
        case '*': return a * b;
        case '/':
            if (b == 0) throw std::runtime_error("错误：除以零");
            return a / b;
    }
    throw std::runtime_error("错误：未知运算符");
}
 double Calculator::evaluate(const std::string& expression) {
    std::stack<double> values; // 存储数值的栈
    std::stack<char> ops; // 存储运算符的栈

    std::string processedExpr = "";
    for (size_t i = 0; i < expression.length(); i++) {
        //去除空格，如果遇到空格跳过剩余的代码量执行下一次循环
        if(isspace(expression[i])) continue;
        //补0逻辑
        if (expression[i] == '-' && (processedExpr.empty() || processedExpr.back() == '(')) {
            processedExpr += '0';
        }
        processedExpr += expression[i];
    }

    //遍历处理后的表达式
    for (size_t i=0; i < processedExpr.length(); i++) {
        char currentChar = processedExpr[i];

        if (currentChar == '(') {
            ops.push(currentChar);
        }

        //如果是数字或小数点，完整解析浮点数
        else if (isdigit(currentChar) || currentChar == '.') {
            double val = 0;
            bool isDecimal = false;
            double decimalMultiplier = 0.1;

            while (i < processedExpr.length() && (isdigit(processedExpr[i]) || processedExpr[i] == '.')) {
                if (processedExpr[i] == '.') {
                    if (isDecimal) throw std::runtime_error("语法错误：数字包含多个小数点");
                    isDecimal = true;
                }else {
                    if (!isDecimal) {
                        val = val * 10 + (processedExpr[i] - '0');
                    }else {
                        val = val + (processedExpr[i] - '0') * decimalMultiplier;
                        decimalMultiplier /= 10;
                    }
                }
                i++;
            }
            values.push(val);
            i--;// 因为外层 for 循环会 i++，这里需要退回一步
        }

        //如果遇到右括号，解决括号内的所有运算
        else if (currentChar == ')') {
            while (!ops.empty() && ops.top() != '(') {
                if (values.size() < 2) throw std::runtime_error("语法错误：操作数缺失");
                double val2 = values.top(); values.pop();
                double val1 = values.top(); values.pop();
                char op = ops.top(); ops.pop();
                values.push(applyOp(val1, val2, op));
            }
            if (!ops.empty()) ops.pop(); // 弹出 '('
        }
        //如果是运算符，处理运算优先级
        else if (currentChar == '+' || currentChar == '-' || currentChar == '*' || currentChar == '/') {
            while (!ops.empty() && getPrecedence(ops.top()) >= getPrecedence(currentChar)) {
                if (values.size() < 2) throw std::runtime_error("语法错误：操作数缺失");
                double val2 = values.top(); values.pop();
                double val1 = values.top(); values.pop();
                char op = ops.top(); ops.pop();
                values.push(applyOp(val1, val2, op));
            }
            ops.push(currentChar);
        }
    }
    // 处理剩余的运算符
    while (!ops.empty()) {
        if (values.size() < 2) throw std::runtime_error("语法错误：操作数缺失");
        double val2 = values.top(); values.pop();
        double val1 = values.top(); values.pop();
        char op = ops.top(); ops.pop();
        values.push(applyOp(val1, val2, op));
    }
    if (values.size() != 1) throw std::runtime_error("语法错误：表达式不完整");
    return values.top();
 }