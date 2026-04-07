#include <iostream>
#include <map>
#include <string>

int main() {
    std::map<std::string,int> mymap;

    mymap["nsz"] = 23;
    mymap["lyy"] = 23;

    for(const auto& i : mymap) {
        std::cout << i.first << ":" << i.second << std::endl;
    }
}