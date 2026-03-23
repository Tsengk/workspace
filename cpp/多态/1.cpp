#include <iostream>
#include <vector>
using namespace std;

class Sensor {
public:
    virtual void readData() = 0;//纯虚函数
    virtual ~Sensor() {
        cout << "sensor 析构" << endl;
    }
};

class lidar : public Sensor {
public:
    void readData() override {
        cout << "Lidar:读取点云数据..." << endl;
    }
    ~lidar() {
        cout << "lidar 析构" << endl;
    }
};

class camera : public Sensor {
public:
    void readData() override {
        cout << "camera:读取图像数据..." << endl;
    }
    ~camera() {
        cout << "camera 析构"<< endl;
    }
};
//Sensor* sen 父类指针
void collectData(Sensor* sen){
    sen->readData();//调用指针指向的子类重写函数
}


int main() {
    vector<Sensor*> sensors;//创建父类指针容器

    sensors.push_back(new lidar());
    sensors.push_back(new camera());

    for(Sensor* sensor : sensors){
        collectData(sensor);
    }
    return 0;
}