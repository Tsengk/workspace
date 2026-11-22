#ifndef __AVOS_DRIVER_HUAWEI_LIDAR_DECODER_H__
#define __AVOS_DRIVER_HUAWEI_LIDAR_DECODER_H__

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <string.h>
#include <atomic>

#include "pointcloud_ctrl.h"
#include "pointcloud_data.h"

#include "../global/data_pool.h"

#define DEG2ARC 3.14159265 / 180.0

namespace avos {
namespace driver {

class HWDecoder
{
public:
	HWDecoder();
	~HWDecoder();

public:
	int Init(const DecoderConfig &config) ;
	void ConvertHuaweiPacktsToPoint(const gassd_msgs::CommonUdpPacket &packet, PacketPointData &point_cloud);

	int SetSupplementaryData(const gassd_msgs::SensorLidarPacket::ConstPtr &packets_ptr, int debug=0);

	double GetActCosValue(int act_angle) {
		while (act_angle < 0) {
			act_angle += CULCULATE_TABLE_SIZE;
		}
		while (act_angle > CULCULATE_TABLE_SIZE) {
			act_angle -= CULCULATE_TABLE_SIZE;
		}
		if (act_angle >= CULCULATE_TABLE_SIZE || act_angle < 0) {
			return 0;
		}
		return cos_table[act_angle];
	}

	double GetActSinValue(int act_angle) {
		while (act_angle < 0) {
			act_angle += CULCULATE_TABLE_SIZE;
		}
		while (act_angle > CULCULATE_TABLE_SIZE) {
			act_angle -= CULCULATE_TABLE_SIZE;
		}
		if (act_angle >= CULCULATE_TABLE_SIZE || act_angle < 0) {
			return 0;
		}
		return sin_table[act_angle];
	}

	void ConvertPoint(VPoint &point, const DecoderConfig &config) {

		float x_coord = point.x;
		float y_coord = point.y;
		float z_coord = point.z;
		point.x = config.transform[0] * x_coord + config.transform[1] * y_coord + config.transform[2] * z_coord + config.transform[3];
		point.y = config.transform[4] * x_coord + config.transform[5] * y_coord + config.transform[6] * z_coord + config.transform[7];
		point.z = config.transform[8] * x_coord + config.transform[9] * y_coord + config.transform[10] * z_coord + config.transform[11];
		point.angle = point.angle + config.tf_yaw;
		point.angle = point.angle >= 0.0 ? point.angle : point.angle + 360.0;
		point.angle = point.angle <= 360.0 ? point.angle : point.angle - 360.0;
	}

private:
	int vertical_angle_[MAX_LIDAR_RING];
	int vertical_angle_offset_[480][4];
	int horizontal_angle_offset_[MAX_LIDAR_RING][4];

	std::vector<std::pair< unsigned int, unsigned int>> vec_data_id_;
	std::vector<bool> data_id_flag_;

	int32_t frame_id_ = -1;
	int32_t udp_pkg_id_ = -1;
	int32_t udp_pkg_count_ = -1;
	bool	is_new = false;
	std::atomic<bool> one_frame_;

	VPoint           point;
	gassd_msgs::CommonUdpPacket            msop_buf, difop_buf;
	char supplementary_buf[SUPPLE_BUFFER_SIZE];
	// LockQueue<gassd_msgs::CommonUdpPacket> udp_recv_msop_queue;
	// LockQueue<gassd_msgs::CommonUdpPacket> udp_recv_difop_queue;

	double sin_table[CULCULATE_TABLE_SIZE];
	double cos_table[CULCULATE_TABLE_SIZE];

public:
	DecoderConfig decoder_config;

};


}
}

#endif  // __AVOS_DRIVER_RS_BP_DECODER_H__