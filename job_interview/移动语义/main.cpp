#include <iostream>
#include <vector>
#include <cstring>

class StringProcessor {
public:
    char* data;
    size_t length;

    StringProcessor(const char* s) {
        std::cout << "Construct:allocate  " << strlen(s) << " bytes.\n";
        length = strlen(s);
        data = new char[length + 1];
        memcpy(data,s,length + 1);
    }

    StringProcessor(const StringProcessor& other) {
        std::cout << "Copy Construct: DEEP COPY" << other.length << " bytes.\n";
        length = other.length;
        data = new char[length + 1];
        memcpy(data,other.data,length + 1);
    }
    StringProcessor(StringProcessor&& other) noexcept : data(other.data),length(other.length){
        std::cout << "Move Construct: TAKE OWERSHIP of " << length << " bytes.\n";
        other.data = nullptr;
        other.length = 0;
    }

    ~StringProcessor() {
        if(data) {
            std::cout << "Destruct: free " << length << " bytes.\n";
            delete[] data;
        }else {
            std::cout << "Destruct : No resource to free.\n";
        }
    }
};

void demostrate_move_semantics() {
    StringProcessor sp1("Holle_World!");
    StringProcessor sp2 = sp1;
    StringProcessor sp3 = std::move(sp1);
}

void demostrate_lambda_closure() {

}


int main() {
    demostrate_move_semantics();
    return 0;
}