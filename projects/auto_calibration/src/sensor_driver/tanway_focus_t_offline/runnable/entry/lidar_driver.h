#ifndef __AVOS_DRIVER_INNO_ROBINW_DRIVER_H__
#define __AVOS_DRIVER_INNO_ROBINW_DRIVER_H__

#include <ros/ros.h>
#include <memory>
#include "runnable/global/data_pool.h"
#include "gassd_msgs_gsds/SensorLidarPacket.h"
#include "FocusTDecode.hpp"
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>



#define  TMP_TIME_SYNC_ERROR_CODE	0x0400
#define  TMP_DATA_ERROR_CODE		0x0401
namespace avos {
namespace driver {

class LidarDriver
{
public:
	LidarDriver();
	~LidarDriver();


	int Init(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
	int Start();
	int SetThreadCorePri(pthread_t thread_handler,  const std::vector<int> &core, int pri);

	void LidarPacketsCallBack(const gassd_msgs_gsds::SensorLidarPacket::ConstPtr & packet);

private:
	void ReadConfig(ros::NodeHandle& private_nh);

private:
    ros::Publisher pub_point_cloud;
	ros::Subscriber sub_packet;

	HWLidarDriverConfig 		  	driver_cfg;

	VPointCloud::Ptr 				point_cloud_data;
	PacketData 						data_buf;

	int debug_ = 0;
	int time_position_ = 0;
	int select_cpu_ = -1;
	bool call_back_init = false;
	std::string node_name_;
	uint64_t seq_ = 0;
	bool 	 get_shm_cfg_ = true;
	bool 	 set_shm_cfg_ = false;

public:
    avos::driver::LidarCaliConfig lidar_cali_config;
    std::string   lidar_name_;
private:
	bool 		  init =  false;
	int			  pub_logic;
	uint64_t      first_pkg_ts;
    FocusTDecoder decoder;
    bool          is_single = false;
};

}
}
#endif  // __AVOS_DRIVER_INNO_ROBINW_DRIVER_H__