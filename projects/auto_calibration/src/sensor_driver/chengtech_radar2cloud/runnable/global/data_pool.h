#ifndef __RS_RADAR_DATA_POOL_H__
#define __RS_RADAR_DATA_POOL_H__
#include <condition_variable>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <vector>
#include <atomic>

#include <Eigen/Dense>
#include "point_types.h"

#define SUPPLE_BUFFER_SIZE	3024
#define UDP_BUFFER_SIZE 1500
#define CULCULATE_TABLE_SIZE 360*100
#define DRIVER_MAX_NUM_THREAD 7

#define ADSFI_TO_SEC(sec, nsec) (double)(sec + (double)nsec / 1e9)
#define ADSFI_GET_SEC(sec) (unsigned int)sec
#define ADSFI_GET_NSEC(sec) (unsigned int)((sec - (unsigned int)sec)*1e9)

namespace avos {
namespace driver {

typedef struct DecoderConfig
{
	int pre_rot_axis_0 = 0;
	int pre_rot_axis_1 = 1;
	int pre_rot_axis_2 = 2;
	double pre_rot_degree_0 = 0.0;
	double pre_rot_degree_1 = 0.0;
	double pre_rot_degree_2 = 0.0;
	double tf_x;
	double tf_y;
	double tf_z;
	double tf_roll;
	double tf_pitch;
	double tf_yaw;
	double transform[16];

	double car_modle_front_x;
	double car_modle_back_x;
	double car_modle_left_y;
	double car_modle_right_y;
	double car_modle_top_z;
	double car_modle_button_z;
	int    filter_obj;
	int    debug;
	// HWLidarDriverConfig driver_cfg;

	DecoderConfig() {
		tf_x = 0.0;
		tf_y = 0.0;
		tf_z = 0.0;
		tf_roll = 0.0;
		tf_pitch = 0.0;
		tf_yaw = 0.0;

		car_modle_front_x = 0.0;
		car_modle_back_x = 0.0;
		car_modle_left_y = 0.0;
		car_modle_right_y = 0.0;
		car_modle_top_z = 0.0;
		car_modle_button_z = 0.0;
		filter_obj = 0;
		debug = 0;
	}

	void Init() {
		Eigen::Matrix4f pre_rotation_matrix = Eigen::Matrix4f::Identity();

		Eigen::Quaternionf pre_rotation_q = Eigen::Quaternionf::Identity();

		// 1st-axis
		Eigen::Vector3f rot_axis = Eigen::Vector3f::Zero();
		rot_axis[pre_rot_axis_0] = 1;
		Eigen::AngleAxis<float> axis_rot = Eigen::AngleAxis<float>(pre_rot_degree_0 / 180.0 * M_PI,
		                                   rot_axis);
		pre_rotation_q = axis_rot * pre_rotation_q;
		// 2rd-axis
		rot_axis = Eigen::Vector3f::Zero();
		rot_axis[pre_rot_axis_1] = 1;
		axis_rot = Eigen::AngleAxis<float>(pre_rot_degree_1 / 180.0 * M_PI,
		                                   rot_axis);
		pre_rotation_q = axis_rot * pre_rotation_q;
		// 3rd-axis
		rot_axis = Eigen::Vector3f::Zero();
		rot_axis[pre_rot_axis_2] = 1;
		axis_rot = Eigen::AngleAxis<float>(pre_rot_degree_2 / 180.0 * M_PI,
		                                   rot_axis);
		pre_rotation_q = axis_rot * pre_rotation_q;

		pre_rotation_matrix.block<3, 3>(0, 0) =
		    pre_rotation_q.normalized().toRotationMatrix();

		// translate
		Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
		transform_matrix.block<3, 1>(0, 3) = Eigen::Vector3f(
		        tf_x, tf_y, tf_z);
		// euler angle
		Eigen::Matrix3f rotation_m = Eigen::Matrix3f::Identity();
		rotation_m = Eigen::AngleAxis<float>(tf_yaw / 180.0 * M_PI, Eigen::Vector3f::UnitZ())
		             * Eigen::AngleAxis<float>(tf_pitch / 180.0 * M_PI, Eigen::Vector3f::UnitY())
		             * Eigen::AngleAxis<float>(tf_roll / 180.0 * M_PI, Eigen::Vector3f::UnitX());

		transform_matrix.block<3, 3>(0, 0) = rotation_m;
		transform_matrix = transform_matrix * pre_rotation_matrix;
		//
		for (int r = 0; r < 4; ++r) {
			for (int c = 0; c < 4; ++c) {
				transform[c + r * 4] = transform_matrix(r, c);
			}
		}
	}
} DecoderConfig;

typedef struct HWRadarDriverConfig
{
	std::string radar_name;
	std::string native_ip;
	std::string target_ip;
	std::string packets_topic;
	std::string pointcloud_topic;
	int udp_buf_size;
	int data_port;
	int heart_port;
	int vehicle_port;
	std::string config_name;
	std::string vehicle_param;
	int  debug;
	int  interval_ms;

	DecoderConfig decoder_config;

	HWRadarDriverConfig() {
		radar_name = "";
		udp_buf_size = 0;
		native_ip = "";
		target_ip = "";
		data_port = 0;
		heart_port = 0;
		vehicle_port = 0;
		config_name = "";
		vehicle_param = "";
		interval_ms   	= 50;
		debug    		= 0;
	}
} HWRadarDriverConfig;

typedef struct PacketData
{
	unsigned char data[UDP_BUFFER_SIZE];
	uint16_t size;
	PacketData() {
		size = 0;
		memset(data, 0, UDP_BUFFER_SIZE);
	}

	PacketData(const PacketData & c)
	{
		memcpy(data, c.data, UDP_BUFFER_SIZE);
		size = c.size;
	}

	PacketData& operator=(const PacketData& c)
	{
		// 避免自赋值
		if (this != &c)
		{
			memcpy(data, c.data, UDP_BUFFER_SIZE);
			size = c.size;
		}
		return *this;
	}
} PacketData;

typedef struct PacketPointData
{
	VPointCloud points;
} PacketPointData;

typedef struct SupplementaryData
{
	int vertical_angle[192]; // 0.00390625 du
	int vertical_angle_offset[480][4]; // 0.001  du
	int horizontal_angle_offset[192][4]; // 0.001  du

	int size()
	{
		return sizeof(SupplementaryData);
	}

} SupplementaryData;

}
}


#endif
