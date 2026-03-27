#include <iostream>
using namespace std;

class BankAccount {
protected:
    double balance;

public:
    BankAccount (double init_balance) {
        balance = init_balance;
    }

    virtual void withdraw(double amount) {
        if(amount <= balance) {
            balance -= amount;
            cout << "【普通账户】取款成功！当前余额：" << balance << endl;
        }else {
            cout << "【普通账户】取款失败！余额不足！" << endl;
        }
    }
};


class CreditAccount : public BankAccount {
private:
    double overdraft_limit;

public:
    CreditAccount (double init_balance,double limit) : BankAccount(init_balance) {
        overdraft_limit = limit;
    }

    void withdraw(double amount) override {
        if(amount <= balance + overdraft_limit) {
            balance -= amount;
            cout << "【信用卡】取款成功。当前余额:" << balance << endl;
        }else {
            cout << "【信用卡】取款失败：超出透支额度！" << endl;
        }
        
    }
};

int main() {
    BankAccount normal_account(10);
    CreditAccount credit_account(10,500);

    BankAccount* account1 = &normal_account;
    CreditAccount* account2 = &credit_account;


    account1->withdraw(50);
    account2->withdraw(50);


    return 0;
}