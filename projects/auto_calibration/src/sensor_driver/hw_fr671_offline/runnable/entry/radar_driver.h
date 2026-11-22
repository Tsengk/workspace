#ifndef __AVOS_DRIVER_INNO_ROBINW_DRIVER_H__
#define __AVOS_DRIVER_INNO_ROBINW_DRIVER_H__

#include <ros/ros.h>
#include <memory>
#include "runnable/global/data_pool.h"
#include "gassd_msgs/SensorRadarPacket.h"
#include "huawei_decode.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>



#define  TMP_TIME_SYNC_ERROR_CODE	0x0400
#define  TMP_DATA_ERROR_CODE		0x0401
namespace avos {
namespace driver {

class RadarDriver
{
public:
	RadarDriver();
	~RadarDriver();


	int Init(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
	int Start();
	int SetThreadCorePri(pthread_t thread_handler,  const std::vector<int> &core, int pri);

	void RadarPacketsCallBack(const gassd_msgs::SensorLidarPacket::ConstPtr & packet);

private:
	void ReadConfig(ros::NodeHandle& private_nh);

private:
    ros::Publisher pub_point_cloud;
	ros::Subscriber sub_packet;

	HWRadarDriverConfig 		  	driver_cfg;
	avos::driver::DecoderConfig		radar_cali_config;

	VPointCloud::Ptr 				point_cloud_data;
	PacketData 						data_buf;

	std::shared_ptr<avos::driver::HWDecoder> decoder;
	int debug_ = 0;
	int time_position_ = 0;
	int select_cpu_ = -1;
	bool call_back_init = false;
	std::string node_name_;
	uint64_t seq_ = 0;
	bool 	 get_shm_cfg_ = true;
	bool 	 set_shm_cfg_ = false;

	int			  pub_logic;
	uint64_t      first_pkg_ts;
};

}
}
#endif  // __AVOS_DRIVER_INNO_ROBINW_DRIVER_H__