#include <string>
#include <unistd.h>
#include <signal.h>
#include "radar_driver.h"
#include "runnable/global/data_pool.h"

int main(int argc, char* argv[])
{
    std::string node_name = "lidar_driver";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    node_name = ros::this_node::getName();
    printf("---------------------------main node_name is %s---------------------------\n", node_name.c_str());
    std::shared_ptr<avos::driver::RadarDriver> driver_ptr = std::make_shared<avos::driver::RadarDriver>();
    if (driver_ptr->Init(nh, private_nh) == -1) {
        std::cout <<  node_name << " Init() failed!";
    }
    if (driver_ptr->Start() == -1) {
        std::cout << node_name << " Start() failed!";
    }

    ros::spin();

    return 0;
}
