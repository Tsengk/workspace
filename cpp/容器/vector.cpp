#include <iostream>
#include <vector>

int main() {
    std::vector<int> vec1;
    std::vector<int> vec2 = {10,20,30};
    std::vector<int> vec3(5,100); //包含五个元素，每个元素都是100

    vec1.push_back(1);
    vec1.push_back(2);//尾部添加元素（常用）

    vec1.emplace_back(3);//尾部构造函数，性能好
    vec1.emplace_back(4);
    vec1.emplace_back(5);

    vec1.pop_back();//移除最后一个元素

    std::cout << vec1.at(3) << std::endl;

    //std::cout << vec1[0] << " " << vec1[1] << " " << vec1[2] << " " << vec1[3] << " " << vec1[4]<< std::endl;
    for(int i : vec1) {std::cout << i << " ";}
    std::cout << std::endl;
    for(int i : vec2) {std::cout << i << " ";}
    std::cout << std::endl;
    for(int i : vec3) {std::cout << i << " ";}
    std::cout << std::endl;

    return 0;
}