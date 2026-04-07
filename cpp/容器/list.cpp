#include <iostream>
#include <list>

int main() {
    std::list<int> list = {1,2,3};
    list.push_front(5);
    list.push_back(5);

    std::cout << "front: " << list.front() << " " << "back: " << list.back() << std::endl;

    list.pop_front();
    std::cout << "after erease front: " << list.front() << std::endl;

    std::cout << "==========list==============" << std::endl;
    for(int i : list) {std::cout << i << std::endl;}

    auto it = list.begin();
    //等效于std::list<int>::iterator it = list.begin();
    for(int i=0 ; i<3 ; ++i) {
        ++it;
    }

    list.insert(it,4);

    std::cout << "==========inset_list=======" << std::endl;
    for(int i : list) {std::cout << i << std::endl;}

}