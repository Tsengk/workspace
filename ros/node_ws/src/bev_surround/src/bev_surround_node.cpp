#include "bev_surround/bev_surround.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "bev_surround_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    bev_surround::BevSurround bev_surround(nh, pnh);

    ros::spin();
    return 0;
}