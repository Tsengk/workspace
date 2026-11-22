#include <sensor_msgs/PointCloud2.h>
#include "lidar_driver.h"
#include "lidar_shm_param.h"

namespace avos {
namespace driver {

LidarDriver::LidarDriver() {
	// node_name_ = ros::this_node::getName();
	// node_name_ = node_name_.erase(0, 1); // erase /
}


LidarDriver::~LidarDriver() {
}

int LidarDriver::SetThreadCorePri(pthread_t thread_handler,  const std::vector<int> &core, int pri) {
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

int LidarDriver::Init(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
	ReadConfig(private_nh);
	printf("============================ share memary init ============================\n");
	decoder = std::make_shared<avos::driver::HWDecoder>();
	decoder->Init(lidar_cali_config);

	pub_point_cloud = nh.advertise<sensor_msgs::PointCloud2>(driver_cfg.pointcloud_topic, 10);

	sub_packet = nh.subscribe(driver_cfg.packets_topic, 1, &LidarDriver::LidarPacketsCallBack, this);

	point_cloud_data.reset(new VPointCloud());

	return 0 ;
}


int LidarDriver::Start() {
	while (ros::ok()) {
		ros::spinOnce();
		usleep(50*1000); //sleep 100ms
	}

	return 0;
}


void LidarDriver::ReadConfig(ros::NodeHandle& private_nh)
{
	private_nh.param("ten_ms_filter_l", driver_cfg.ten_ms_filter_l, 110);
	private_nh.param("ten_ms_filter_r", driver_cfg.ten_ms_filter_r, 130);
	private_nh.param("phase_lock", driver_cfg.phase_lock, 6000);
	private_nh.param("interval_ms", driver_cfg.interval_ms, 100);
	private_nh.param("calibration_key", node_name_, std::string("front_left_lidar"));
	private_nh.param("packets_topic", driver_cfg.packets_topic, std::string("/driver/lidar/inno_rw_packets"));
	private_nh.param("ros_send_topic", driver_cfg.pointcloud_topic, std::string("/driver/lidar/cloud/top"));
	private_nh.param("config_name", driver_cfg.config_name, std::string("vehicle_sensor_calibration.conf"));
	private_nh.param("vehicle_param", driver_cfg.vehicle_param, std::string("vehicle_param.conf"));

	printf("------------------- calibration_key: %s --------------------\n",node_name_.c_str());
	printf("------------------- packets_topic: %s --------------------\n",driver_cfg.packets_topic.c_str());
	printf("------------------- ros_send_topic: %s --------------------\n",driver_cfg.pointcloud_topic.c_str());
	printf("------------------- config_name: %s --------------------\n",driver_cfg.config_name.c_str());
	printf("------------------- vehicle_param: %s --------------------\n",driver_cfg.vehicle_param.c_str());

	// private_nh.param("debug", driver_cfg.debug, 0);
	private_nh.param("debug", debug_, 0);
	lidar_cali_config.decoder_debug = (debug_ & 4);
	lidar_cali_config.error_debug = (debug_ & 8);
	lidar_cali_config.print_data = (debug_ & 16);
	driver_cfg.debug_ts = (debug_ & 1);
	driver_cfg.debug     = (debug_ & 2);

	private_nh.param("set_shm_cfg", set_shm_cfg_, false);
	private_nh.param("get_shm_cfg", get_shm_cfg_, true);
	std::cout << "set_shm_cfg = " << set_shm_cfg_ << "\n";
	std::cout << "get_shm_cfg = " << get_shm_cfg_ << "\n";

	std::cout << "driver_cfg.ten_ms_filter_l = " << driver_cfg.ten_ms_filter_l << "\n";
	std::cout << "driver_cfg.ten_ms_filter_r = " << driver_cfg.ten_ms_filter_r << "\n";
	std::cout << "driver_cfg.phase_lock = " << driver_cfg.phase_lock << "\n";
	std::cout << "driver_cfg.interval_ms = " << driver_cfg.interval_ms << "\n";
	std::cout << "debug_ = " << debug_ << "\n";
	std::cout << "lidar_cali_config.print_data = " << lidar_cali_config.print_data << "\n";
	std::cout << "lidar_cali_config.decoder_debug = " << lidar_cali_config.decoder_debug << "\n";
	std::cout << "lidar_cali_config.error_debug = " << lidar_cali_config.error_debug << "\n";
	std::cout << "driver_cfg.debug_ts = " << driver_cfg.debug_ts << "\n";
	std::cout << "driver_cfg.debug = " << driver_cfg.debug << "\n";

	private_nh.param("prerotaxis_0", driver_cfg.decoder_config.pre_rot_axis_0, 0);
	private_nh.param("prerotaxis_1", driver_cfg.decoder_config.pre_rot_axis_1, 1);
	private_nh.param("prerotaxis_2", driver_cfg.decoder_config.pre_rot_axis_2, 2);
	private_nh.param("prerotdegree_0", driver_cfg.decoder_config.pre_rot_degree_0, 0.0);
	private_nh.param("prerotdegree_1", driver_cfg.decoder_config.pre_rot_degree_1, 0.0);
	private_nh.param("prerotdegree_2", driver_cfg.decoder_config.pre_rot_degree_2, 0.0);
	private_nh.param("xoffset", driver_cfg.decoder_config.tf_x, 0.0);
	private_nh.param("yoffset", driver_cfg.decoder_config.tf_y, 0.0);
	private_nh.param("zoffset", driver_cfg.decoder_config.tf_z, 0.0);
	private_nh.param("roll", driver_cfg.decoder_config.tf_roll, 0.0);
	private_nh.param("pitch", driver_cfg.decoder_config.tf_pitch, 0.0);
	private_nh.param("yaw", driver_cfg.decoder_config.tf_yaw, 0.0);
	private_nh.param("top_z_ground_offset", driver_cfg.decoder_config.car_modle_button_z, 1.0);
	printf("%s's default calib_cfg for launch_file as follow:\n", node_name_.c_str());
	std::cout << "\tprerotaxis_0: " << driver_cfg.decoder_config.pre_rot_axis_0<< "\n";
	std::cout << "\tprerotaxis_1: " << driver_cfg.decoder_config.pre_rot_axis_1<< "\n";
	std::cout << "\tprerotaxis_2: " << driver_cfg.decoder_config.pre_rot_axis_2<< "\n";
	std::cout << "\tprerotdegree_0: " << driver_cfg.decoder_config.pre_rot_degree_0<< "\n";
	std::cout << "\tprerotdegree_1: " << driver_cfg.decoder_config.pre_rot_degree_1<< "\n";
	std::cout << "\tprerotdegree_2: " << driver_cfg.decoder_config.pre_rot_degree_2<< "\n";

	std::cout << "\txoffset: " << driver_cfg.decoder_config.tf_x<< "\n";
	std::cout << "\tyoffset: " << driver_cfg.decoder_config.tf_y<< "\n";
	std::cout << "\tzoffset: " << driver_cfg.decoder_config.tf_z<< "\n";
	std::cout << "\troll: " << driver_cfg.decoder_config.tf_roll << "\n";
	std::cout << "\tpitch: " << driver_cfg.decoder_config.tf_pitch << "\n";
	std::cout << "\tyaw: " << driver_cfg.decoder_config.tf_yaw << "\n";
	std::cout << "\ttop_z_ground_offset: " << driver_cfg.decoder_config.car_modle_button_z<< "\n";
	driver_cfg.decoder_config.Init();

	// shm calib_para interactive
	// ros::param::set("/"+node_name_+"/lidar/x", driver_cfg.decoder_config.tf_x);
	// ros::param::set("/"+node_name_+"/lidar/y", driver_cfg.decoder_config.tf_y);
	// ros::param::set("/"+node_name_+"/lidar/z", driver_cfg.decoder_config.tf_z);
	// ros::param::set("/"+node_name_+"/lidar/roll", driver_cfg.decoder_config.tf_roll);
	// ros::param::set("/"+node_name_+"/lidar/pitch", driver_cfg.decoder_config.tf_pitch);
	// ros::param::set("/"+node_name_+"/lidar/yaw", driver_cfg.decoder_config.tf_yaw);
	if(set_shm_cfg_)
	{
		LidarShmParam::SetModuleParamSimple(node_name_, driver_cfg.decoder_config.tf_x, driver_cfg.decoder_config.tf_y, driver_cfg.decoder_config.tf_z,
										driver_cfg.decoder_config.tf_roll, driver_cfg.decoder_config.tf_pitch, driver_cfg.decoder_config.tf_yaw);
	}

	double front_axle_tofront = 0.0;
	double wheel_base_length = 0.0;
	double rear_axle_torear = 0.0;
	double car_width = 0.0;
	private_nh.param("front_axle_tofront", front_axle_tofront, 1.191);
	private_nh.param("wheel_base_length", wheel_base_length, 4.140);
	private_nh.param("rear_axle_torear", rear_axle_torear, 1.953);
	private_nh.param("car_width", car_width, 2.505);
	driver_cfg.decoder_config.car_modle_front_x = front_axle_tofront + wheel_base_length;
	driver_cfg.decoder_config.car_modle_back_x = -rear_axle_torear;
	driver_cfg.decoder_config.car_modle_left_y = car_width / 2.0;
	driver_cfg.decoder_config.car_modle_right_y = -car_width / 2.0;
	driver_cfg.decoder_config.car_modle_top_z = 0.1;
	driver_cfg.decoder_config.car_modle_button_z = -driver_cfg.decoder_config.car_modle_button_z;
	printf("vehicle_cfg as follow:\n");
	std::cout << "\tcar_modle_front_x: " << driver_cfg.decoder_config.car_modle_front_x
			<< ". car_modle_back_x: " << driver_cfg.decoder_config.car_modle_back_x << "\n";
	std::cout << "\tcar_modle_left_y: " << driver_cfg.decoder_config.car_modle_left_y
			<< ". car_modle_right_y: " << driver_cfg.decoder_config.car_modle_right_y << "\n";
	std::cout << "\tcar_modle_button_z: " << driver_cfg.decoder_config.car_modle_button_z
			<< ". car_modle_top_z: " << driver_cfg.decoder_config.car_modle_top_z << "\n";

	return;
}

void LidarDriver::LidarPacketsCallBack(const gassd_msgs::SensorLidarPacket::ConstPtr & packets) {
	if(debug_ & 1024)
		std::cout << "msop_packets size: " << packets->msop_packet.size() << std::setprecision(17) << ", ts: "
			<< ADSFI_TO_SEC(packets->header.time.sec, packets->header.time.nsec) << "\n";

	PacketPointData packet_point_cloud;
	packet_point_cloud.points.clear();
	// shm calib_para interactive
	// ros::param::get("/"+node_name_+"/lidar/x", decoder->decoder_config.tf_x);
	// ros::param::get("/"+node_name_+"/lidar/y", decoder->decoder_config.tf_y);
	// ros::param::get("/"+node_name_+"/lidar/z", decoder->decoder_config.tf_z);
	// ros::param::get("/"+node_name_+"/lidar/roll", decoder->decoder_config.tf_roll);
	// ros::param::get("/"+node_name_+"/lidar/pitch", decoder->decoder_config.tf_pitch);
	// ros::param::get("/"+node_name_+"/lidar/yaw", decoder->decoder_config.tf_yaw);
	if(get_shm_cfg_)
	{
		LidarShmParam::GetModuleParamSimple(node_name_, decoder->decoder_config.tf_x, decoder->decoder_config.tf_y, decoder->decoder_config.tf_z,
											decoder->decoder_config.tf_roll, decoder->decoder_config.tf_pitch, decoder->decoder_config.tf_yaw);
		decoder->decoder_config.Init();
	}
	if(false == call_back_init)
	{
		if(0 == decoder->SetSupplementaryData(packets, debug_))
		{
			printf("===================== SetSupplementaryData success =====================\n");
			call_back_init = true;
		}
		else
		{
			printf("===================== SetSupplementaryData fail =====================\n");
		}
	}
	point_cloud_data->clear();
	point_cloud_data->width = 0;
	double header_ts_s = 0;
	if(true == call_back_init)
	{
		for (int i = 0; i < packets->msop_packet.size(); i++) {
			packet_point_cloud.points.clear();
			decoder->ConvertHuaweiPacktsToPoint(packets->msop_packet[i], packet_point_cloud);
			point_cloud_data->insert(point_cloud_data->end(), packet_point_cloud.points.begin(), packet_point_cloud.points.end());
		}
		if(point_cloud_data->size()>0)
		{
			header_ts_s = point_cloud_data->at(0).timestamp;
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

	}
	printf("frame header ts: %.3f, point cloud size: %d\n", point_cloud_data->header.stamp/1000000.0, point_cloud_data->points.size());

	// for (auto pkt : packets->msop_packet)	{
	// 	memset((unsigned char*)decoder->data_buf.data, 0, sizeof(decoder->data_buf.data));
	// 	memcpy((unsigned char*)decoder->data_buf.data, &(pkt.data[0]), sizeof(decoder->data_buf.data));
	// 	decoder->udp_recv_data_queue.push(decoder->data_buf);
	// }

	return;
}

}
}