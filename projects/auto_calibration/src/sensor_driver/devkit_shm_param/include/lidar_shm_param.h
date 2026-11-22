#ifndef __RSLIDAR_LIDAR_SHM_PARAM_H__
#define __RSLIDAR_LIDAR_SHM_PARAM_H__


#include <string>
#include <sys/time.h>
#include "param_module.h"

class LidarShmParam
{
public:
	LidarShmParam() {

	}
	~LidarShmParam() {

	}

	static double get_ts()
	{
		timeval tv{};
		gettimeofday(&tv, nullptr);
		return (tv.tv_sec + tv.tv_usec / 1000000.0); //double ts
	}

	static void SetLidarModuleParam(const std::string &node_name, int prerotaxis_0, int prerotaxis_1, int prerotaxis_2,
	                                double prerotdegree_0, double prerotdegree_1, double prerotdegree_2,
	                                double tf_x, double tf_y, double tf_z,
	                                double roll, double pitch, double yaw) {
		std::cout << "set node_name:" << node_name << std::endl;
		SHMParamApi::Instance()->AddModuleParam("global");
		SHMParamApi::Instance()->SetIntModuleParam(node_name + "_prerotaxis_0", prerotaxis_0);
		SHMParamApi::Instance()->SetIntModuleParam(node_name + "_prerotaxis_1", prerotaxis_1);
		SHMParamApi::Instance()->SetIntModuleParam(node_name + "_prerotaxis_2", prerotaxis_2);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "prerotdegree_0", prerotdegree_0);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "prerotdegree_1", prerotdegree_1);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "prerotdegree_2", prerotdegree_2);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_xoffset", tf_x);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_yoffset", tf_y);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_zoffset", tf_z);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_roll", roll);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_pitch", pitch);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_yaw", yaw);
	}

	static void GetLidarModuleParam(const std::string &node_name, int &prerotaxis_0, int &prerotaxis_1, int &prerotaxis_2,
	                                double &prerotdegree_0, double &prerotdegree_1, double &prerotdegree_2,
	                                double &tf_x, double &tf_y, double &tf_z,
	                                double &roll, double &pitch, double &yaw) {
		std::cout << "get node_name:" << node_name << std::endl;
		SHMParamApi::Instance()->QueryIntModuleParam(node_name + "_prerotaxis_0", prerotaxis_0);
		SHMParamApi::Instance()->QueryIntModuleParam(node_name + "_prerotaxis_1", prerotaxis_1);
		SHMParamApi::Instance()->QueryIntModuleParam(node_name + "_prerotaxis_2", prerotaxis_2);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "prerotdegree_0", prerotdegree_0);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "prerotdegree_1", prerotdegree_1);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "prerotdegree_2", prerotdegree_2);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_xoffset", tf_x);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_yoffset", tf_y);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_zoffset", tf_z);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_roll", roll);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_pitch", pitch);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_yaw", yaw);
	}

	static void GetModuleParamSimple(const std::string &node_name,
	                                double &tf_x, double &tf_y, double &tf_z,
	                                double &roll, double &pitch, double &yaw) {
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_xoffset", tf_x);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_yoffset", tf_y);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_zoffset", tf_z);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_roll", roll);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_pitch", pitch);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_yaw", yaw);

		printf("---------------------------------- [GET %.3f] %s's xoffset: %.5f, yoffset: %.5f, zoffset: %.5f, roll: %.5f, pitch: %.5f, yaw: %.5f ----------------------------------\n",
				get_ts(), node_name.c_str(), tf_x, tf_y, tf_z, roll, pitch, yaw);
	}

	static void SetModuleParamSimple(const std::string &node_name,
	                                double tf_x, double tf_y, double tf_z,
	                                double roll, double pitch, double yaw) {
		SHMParamApi::Instance()->AddModuleParam("global");
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_xoffset", tf_x);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_yoffset", tf_y);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_zoffset", tf_z);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_roll", roll);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_pitch", pitch);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_yaw", yaw);

		printf("---------------------------------- [SET %.3f] %s's xoffset: %.5f, yoffset: %.5f, zoffset: %.5f, roll: %.5f, pitch: %.5f, yaw: %.5f ----------------------------------\n",
				get_ts(), node_name.c_str(), tf_x, tf_y, tf_z, roll, pitch, yaw);
	}

	static void SetModuleParam_roll_pitch_z(const std::string &node_name, double tf_z, double roll, double pitch) {
		SHMParamApi::Instance()->AddModuleParam("global");
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_zoffset", tf_z);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_roll", roll);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_pitch", pitch);
		printf("---------------------------------- [SET %.3f] %s's zoffset: %.5f, roll: %.5f, pitch: %.5f ----------------------------------\n",
				get_ts(), node_name.c_str(), tf_z, roll, pitch);
	}

	static void GetModuleParam_roll_pitch_z(const std::string &node_name, double &tf_z, double &roll, double &pitch) {
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_zoffset", tf_z);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_roll", roll);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_pitch", pitch);
		printf("---------------------------------- [GET %.3f] %s's zoffset: %.5f, roll: %.5f, pitch: %.5f ----------------------------------\n",
				get_ts(), node_name.c_str(), tf_z, roll, pitch);
	}

	static void SetModuleParam_yaw(const std::string &node_name, double yaw) {
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_yaw", yaw);
		printf("---------------------------------- [SET %.3f] %s's yaw: %.5f ----------------------------------\n", get_ts(), node_name.c_str(), yaw);
	}

	static void GetModuleParam_yaw(const std::string &node_name, double &yaw) {
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_yaw", yaw);
		printf("---------------------------------- [GET %.3f] %s's yaw: %.5f ----------------------------------\n", get_ts(), node_name.c_str(), yaw);
	}

	static void SetModuleParam_x_y_yaw(const std::string &node_name, double tf_x, double tf_y, double yaw) {
		SHMParamApi::Instance()->AddModuleParam("global");
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_xoffset", tf_x);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_yoffset", tf_y);
		SHMParamApi::Instance()->SetDoubleModuleParam(node_name + "_yaw", yaw);
		printf("---------------------------------- [SET %.3f] %s's xoffset: %.5f, yoffset: %.5f, yaw: %.5f ----------------------------------\n",
				get_ts(), node_name.c_str(), tf_x, tf_y, yaw);
	}

	static void GetModuleParam_x_y_yaw(const std::string &node_name, double &tf_x, double &tf_y, double &yaw) {
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_xoffset", tf_x);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_yoffset", tf_y);
		SHMParamApi::Instance()->QueryDoubleModuleParam(node_name + "_yaw", yaw);
		printf("---------------------------------- [GET %.3f] %s's xoffset: %.5f, yoffset: %.5f, yaw: %.5f ----------------------------------\n",
				get_ts(), node_name.c_str(), tf_x, tf_y, yaw);
	}

};

#endif
