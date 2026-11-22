#include <sensor_msgs/PointCloud2.h>
#include "radar_driver.h"
#include "lidar_shm_param.h"

namespace avos {
namespace driver {

RadarDriver::RadarDriver() {
	// node_name_ = ros::this_node::getName();
	// node_name_ = node_name_.erase(0, 1); // erase /
}


RadarDriver::~RadarDriver() {
}

int RadarDriver::SetThreadCorePri(pthread_t thread_handler,  const std::vector<int> &core, int pri) {
	// pthread_t thread = t.native_handle();
	// 设置线程绑定到CPU核心0

	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	for (auto &p : core) {
		CPU_SET(p, &cpuset); // 将CPU 0加入集合
	}
	pthread_setaffinity_np(thread_handler, sizeof(cpu_set_t), &cpuset);
	// 设置线程调度为先进先出
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
	// 设置线程优先级为99
	struct sched_param param;
	param.sched_priority = pri;
	pthread_attr_setschedparam(&attr, &param);
	// 绑定线程属性到线程
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	pthread_setschedparam(thread_handler, SCHED_FIFO, &param);
	return 0;
}

int RadarDriver::Init(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
	ReadConfig(private_nh);
	printf("============================ share memary init ============================\n");

	pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2>(driver_cfg.pointcloud_topic, 10);

	sub_packet = nh.subscribe(driver_cfg.packets_topic, 1, &RadarDriver::RadarPacketsCallBack, this);

	point_cloud_data.reset(new VPointCloud());

	return 0 ;
}


int RadarDriver::Start() {
	while (ros::ok()) {
		ros::spinOnce();
		usleep(25*1000); //sleep 100ms
	}

	return 0;
}


void RadarDriver::ReadConfig(ros::NodeHandle& private_nh)
{
	private_nh.param("interval_ms", driver_cfg.interval_ms, 100);
	private_nh.param("filter_obj", radar_cali_config.filter_obj, 3);
	private_nh.param("calibration_key", node_name_, std::string("front_left_radar"));
	private_nh.param("packets_topic", driver_cfg.packets_topic, std::string("/driver/radar/inno_rw_packets"));
	private_nh.param("ros_send_topic", driver_cfg.pointcloud_topic, std::string("/driver/radar/cloud/top"));
	private_nh.param("config_name", driver_cfg.config_name, std::string("vehicle_sensor_calibration.conf"));
	private_nh.param("vehicle_param", driver_cfg.vehicle_param, std::string("vehicle_param.conf"));

	printf("------------------- calibration_key: %s --------------------\n",node_name_.c_str());
	printf("------------------- packets_topic: %s --------------------\n",driver_cfg.packets_topic.c_str());
	printf("------------------- ros_send_topic: %s --------------------\n",driver_cfg.pointcloud_topic.c_str());
	printf("------------------- config_name: %s --------------------\n",driver_cfg.config_name.c_str());
	printf("------------------- vehicle_param: %s --------------------\n",driver_cfg.vehicle_param.c_str());

	// private_nh.param("debug", driver_cfg.debug, 0);
	private_nh.param("debug", debug_, 0);
	radar_cali_config.debug = debug_;
	driver_cfg.debug     = debug_;

	std::cout << "driver_cfg.interval_ms = " << driver_cfg.interval_ms << "\n";
	std::cout << "debug_ = " << debug_ << "\n";
	std::cout << "radar_cali_config.filter_obj = " << radar_cali_config.filter_obj << "\n";
	std::cout << "radar_cali_config.debug = " << radar_cali_config.debug << "\n";
	std::cout << "driver_cfg.debug = " << driver_cfg.debug << "\n";

	private_nh.param("prerotaxis_0", radar_cali_config.pre_rot_axis_0, 0);
	private_nh.param("prerotaxis_1", radar_cali_config.pre_rot_axis_1, 1);
	private_nh.param("prerotaxis_2", radar_cali_config.pre_rot_axis_2, 2);
	private_nh.param("prerotdegree_0", radar_cali_config.pre_rot_degree_0, 0.0);
	private_nh.param("prerotdegree_1", radar_cali_config.pre_rot_degree_1, 0.0);
	private_nh.param("prerotdegree_2", radar_cali_config.pre_rot_degree_2, 0.0);
	private_nh.param("xoffset", radar_cali_config.tf_x, 0.0);
	private_nh.param("yoffset", radar_cali_config.tf_y, 0.0);
	private_nh.param("zoffset", radar_cali_config.tf_z, 0.0);
	private_nh.param("roll", radar_cali_config.tf_roll, 0.0);
	private_nh.param("pitch", radar_cali_config.tf_pitch, 0.0);
	private_nh.param("yaw", radar_cali_config.tf_yaw, 0.0);
	private_nh.param("top_z_ground_offset", radar_cali_config.car_modle_button_z, 1.0);
	printf("%s's default calib_cfg for launch_file as follow:\n", node_name_.c_str());
	std::cout << "\tprerotaxis_0: " << radar_cali_config.pre_rot_axis_0<< "\n";
	std::cout << "\tprerotaxis_1: " << radar_cali_config.pre_rot_axis_1<< "\n";
	std::cout << "\tprerotaxis_2: " << radar_cali_config.pre_rot_axis_2<< "\n";
	std::cout << "\tprerotdegree_0: " << radar_cali_config.pre_rot_degree_0<< "\n";
	std::cout << "\tprerotdegree_1: " << radar_cali_config.pre_rot_degree_1<< "\n";
	std::cout << "\tprerotdegree_2: " << radar_cali_config.pre_rot_degree_2<< "\n";

	std::cout << "\txoffset: " << radar_cali_config.tf_x<< "\n";
	std::cout << "\tyoffset: " << radar_cali_config.tf_y<< "\n";
	std::cout << "\tzoffset: " << radar_cali_config.tf_z<< "\n";
	std::cout << "\troll: " << radar_cali_config.tf_roll << "\n";
	std::cout << "\tpitch: " << radar_cali_config.tf_pitch << "\n";
	std::cout << "\tyaw: " << radar_cali_config.tf_yaw << "\n";
	std::cout << "\ttop_z_ground_offset: " << radar_cali_config.car_modle_button_z<< "\n";
	radar_cali_config.Init();

	// shm calib_para interactive
	// ros::param::set("/"+node_name_+"/radar/x", radar_cali_config.tf_x);
	// ros::param::set("/"+node_name_+"/radar/y", radar_cali_config.tf_y);
	// ros::param::set("/"+node_name_+"/radar/z", radar_cali_config.tf_z);
	// ros::param::set("/"+node_name_+"/radar/roll", radar_cali_config.tf_roll);
	// ros::param::set("/"+node_name_+"/radar/pitch", radar_cali_config.tf_pitch);
	// ros::param::set("/"+node_name_+"/radar/yaw", radar_cali_config.tf_yaw);

	double front_axle_tofront = 0.0;
	double wheel_base_length = 0.0;
	double rear_axle_torear = 0.0;
	double car_width = 0.0;
	private_nh.param("front_axle_tofront", front_axle_tofront, 1.191);
	private_nh.param("wheel_base_length", wheel_base_length, 4.140);
	private_nh.param("rear_axle_torear", rear_axle_torear, 1.953);
	private_nh.param("car_width", car_width, 2.505);
	radar_cali_config.car_modle_front_x = front_axle_tofront + wheel_base_length;
	radar_cali_config.car_modle_back_x = -rear_axle_torear;
	radar_cali_config.car_modle_left_y = car_width / 2.0;
	radar_cali_config.car_modle_right_y = -car_width / 2.0;
	radar_cali_config.car_modle_top_z = 0.1;
	radar_cali_config.car_modle_button_z = -radar_cali_config.car_modle_button_z;
	printf("vehicle_cfg as follow:\n");
	std::cout << "\tcar_modle_front_x: " << radar_cali_config.car_modle_front_x
			<< ". car_modle_back_x: " << radar_cali_config.car_modle_back_x << "\n";
	std::cout << "\tcar_modle_left_y: " << radar_cali_config.car_modle_left_y
			<< ". car_modle_right_y: " << radar_cali_config.car_modle_right_y << "\n";
	std::cout << "\tcar_modle_button_z: " << radar_cali_config.car_modle_button_z
			<< ". car_modle_top_z: " << radar_cali_config.car_modle_top_z << "\n";

	return;
}

void RadarDriver::RadarPacketsCallBack(const gassd_msgs_gsds::ChengTechRadarObjs::ConstPtr & packets) {
	if(debug_ & 1024)
		std::cout << "objs size: " << packets->objs.size() << std::setprecision(17) << ", ts: " << packets->header.time << "\n";

	// shm calib_para interactive
	// ros::param::get("/"+node_name_+"/radar/x", radar_cali_config.tf_x);
	// ros::param::get("/"+node_name_+"/radar/y", radar_cali_config.tf_y);
	// ros::param::get("/"+node_name_+"/radar/z", radar_cali_config.tf_z);
	// ros::param::get("/"+node_name_+"/radar/roll", radar_cali_config.tf_roll);
	// ros::param::get("/"+node_name_+"/radar/pitch", radar_cali_config.tf_pitch);
	// ros::param::get("/"+node_name_+"/radar/yaw", radar_cali_config.tf_yaw);
	LidarShmParam::GetModuleParamSimple(node_name_, radar_cali_config.tf_x, radar_cali_config.tf_y, radar_cali_config.tf_z,
										radar_cali_config.tf_roll, radar_cali_config.tf_pitch, radar_cali_config.tf_yaw);
	radar_cali_config.Init();

	point_cloud_data->clear();
	point_cloud_data->width = 0;
	double header_ts_s = packets->header.time;
	VPoint point;
	for (int i = 0; i < packets->objs.size(); i++) {
		// const gassd_msgs_gsds::ChengTechRadarObj & obj = packets->objs().Get(i);
		point.ring = packets->objs[i].id; //float ts
		point.x = packets->objs[i].x;
		point.y = packets->objs[i].y;
		point.z = 1.0;

		// calibration
		ConvertPoint(point, radar_cali_config);
		if(debug_&1)
		{
			printf("id: %d, x: %.3f, y: %.3f, z: 1.0\n", point.ring, point.x, point.y);
		}

		point_cloud_data->push_back(point);
	}
	ros::Time ros_time = ros::Time(header_ts_s);
	pcl_conversions::toPCL(ros_time, point_cloud_data->header.stamp);

	point_cloud_data->header.frame_id = "base_link";
	point_cloud_data->header.seq = seq_++;
	//printf("point_cloud_data->header.seq is %llu\n", point_cloud_data->header.seq);
	point_cloud_data->height = 1;
	point_cloud_data->width = point_cloud_data->size();
	pub_point_cloud.publish(*point_cloud_data);
	// printf("pub cloud once\n");

	printf("frame header ts: %.3f, point cloud size: %d\n", point_cloud_data->header.stamp/1000000.0, point_cloud_data->points.size());

	return;
}

}
}