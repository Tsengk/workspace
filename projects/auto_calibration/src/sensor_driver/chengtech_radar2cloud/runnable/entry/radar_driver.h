#ifndef __AVOS_DRIVER_INNO_ROBINW_DRIVER_H__
#define __AVOS_DRIVER_INNO_ROBINW_DRIVER_H__

#include <ros/ros.h>
#include <memory>
#include "runnable/global/data_pool.h"
#include "gassd_msgs_gsds/ChengTechRadarObjs.h"
// #include "ChengTechRadarObjs.h"
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

	void RadarPacketsCallBack(const gassd_msgs_gsds::ChengTechRadarObjs::ConstPtr & packet);

private:
	void ReadConfig(ros::NodeHandle& private_nh);

	void ConvertPoint(VPoint &point, const DecoderConfig &config) {

		float x_coord = point.x;
		float y_coord = point.y;
		float z_coord = point.z;
		point.x = config.transform[0] * x_coord + config.transform[1] * y_coord + config.transform[2] * z_coord + config.transform[3];
		point.y = config.transform[4] * x_coord + config.transform[5] * y_coord + config.transform[6] * z_coord + config.transform[7];
		point.z = config.transform[8] * x_coord + config.transform[9] * y_coord + config.transform[10] * z_coord + config.transform[11];
		// point.angle = point.angle + config.tf_yaw;
		// point.angle = point.angle >= 0.0 ? point.angle : point.angle + 360.0;
		// point.angle = point.angle <= 360.0 ? point.angle : point.angle - 360.0;
	}

private:
    ros::Publisher pub_point_cloud;
	ros::Subscriber sub_packet;

	HWRadarDriverConfig 		  	driver_cfg;
	avos::driver::DecoderConfig		radar_cali_config;

	VPointCloud::Ptr 				point_cloud_data;
	PacketData 						data_buf;

	int debug_ = 0;
	int time_position_ = 0;
	int select_cpu_ = -1;
	bool call_back_init = false;
	std::string node_name_;
	uint64_t seq_ = 0;

	int			  pub_logic;
	uint64_t      first_pkg_ts;
};

}
}
#endif  // __AVOS_DRIVER_INNO_ROBINW_DRIVER_H__