#include <math.h>
#include "huawei_decode.h"

namespace avos {
namespace driver {

HWDecoder::HWDecoder()  {
	for (int i = 0; i < CULCULATE_TABLE_SIZE; i++) {
		sin_table[i] = sin(i / 180.0 / (CULCULATE_TABLE_SIZE / 360.0) * 3.1415926);
	}

	for (int i = 0; i < CULCULATE_TABLE_SIZE; i++) {
		cos_table[i] = cos(i / 180.0 / (CULCULATE_TABLE_SIZE / 360.0) * 3.1415926);
	}

	std::cout << "HWDecoder" << std::endl;
	memset((char *)vertical_angle_, 0, sizeof(int) * MAX_LIDAR_RING);
	memset((char *)vertical_angle_offset_, 0, sizeof(int) * 480 * 4);
	memset((char *)horizontal_angle_offset_, 0, sizeof(int) * MAX_LIDAR_RING * 4);

	vec_data_id_.clear();
	data_id_flag_.resize(9, false);

	vec_data_id_.push_back(std::make_pair(0x0001, 0x667c1ae0));
	vec_data_id_.push_back(std::make_pair(0x0010, 0x06f383a2));
	vec_data_id_.push_back(std::make_pair(0x0011, 0x4089a102));
	vec_data_id_.push_back(std::make_pair(0x0012, 0x8cc49379));
	vec_data_id_.push_back(std::make_pair(0x0013, 0xcabeb1d9));
	vec_data_id_.push_back(std::make_pair(0x0020, 0x08320075));
	vec_data_id_.push_back(std::make_pair(0x0021, 0x4e4822d5));
	vec_data_id_.push_back(std::make_pair(0x0022, 0x8f83bb98));
	vec_data_id_.push_back(std::make_pair(0x0023, 0xc9f99938));

	one_frame_.store(false);

}

HWDecoder::~HWDecoder() {

}

int HWDecoder::Init(const DecoderConfig &config) {
	difop_buf.udp_packet.resize(sizeof(SupplementaryData));
	msop_buf.udp_packet.resize(UDP_BUFFER_SIZE);
	decoder_config = config;
	return 0;
}

int HWDecoder::SetSupplementaryData(const gassd_msgs::SensorLidarPacket::ConstPtr &packets_ptr, int debug) {
	// if (packets_ptr->get() == nullptr) {
	// 	return -1;
	// }
	if (packets_ptr->difop_packet.udp_packet.size() != sizeof(SupplementaryData))
	{
		printf("error: invalid difop_packet size[%d], unqual SupplementaryData's size[%d]\n", packets_ptr->difop_packet.udp_packet.size(), sizeof(SupplementaryData));
		// exit(1);
	}
	// SupplementaryData *supple_data = (SupplementaryData *)(&(packets_ptr->difop_packet.udp_packet[0]));
	SupplementaryData supple_data;
	memcpy(&supple_data, &(packets_ptr->difop_packet.udp_packet[0]), sizeof(SupplementaryData));

	for (int i = 0; i < MAX_LIDAR_RING; i++) {
		vertical_angle_[i] = supple_data.vertical_angle[i];
		if(debug)
		{
			printf("   vertical_angle_[%d] is: %d; ", i, vertical_angle_[i]);
			if (0 == (i % 10))
			{
				printf("\n");
			}
		}
	}
	printf("\n");

	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < 480; i++) {
			vertical_angle_offset_[i][j] = supple_data.vertical_angle_offset[i][j];
			if(debug)
			{
				printf("   vertical_angle_offset_[%d][%d] is: %d; ", i, j, vertical_angle_offset_[i][j]);
				if (0 == (i % 10))
				{
					printf("\n");
				}
			}
		}
		printf("\n");
	}

	for (int j = 0; j < 4; j++) {
		for (int i = 0; i < MAX_LIDAR_RING; i++) {
			horizontal_angle_offset_[i][j] = supple_data.horizontal_angle_offset[i][j];
			if(debug)
			{
				printf("   horizontal_angle_offset_[%d][%d] is: %d; ", i, j, horizontal_angle_offset_[i][j]);
				if (0 == (i % 10))
				{
					printf("\n");
				}
			}
		}
		printf("\n");
	}

	return 0;
}

// convert avos huawei_packtes.udp packet to cloud
void HWDecoder::ConvertHuaweiPacktsToPoint(const gassd_msgs::CommonUdpPacket &packet, PacketPointData &point_cloud) {
	HWPointCloudProtocol *pkt = (HWPointCloudProtocol *)(&(packet.udp_packet[0]));
	if (decoder_config.print_data)
	{
		std::stringstream its_message;
		its_message << "udp buf size:[" << std::dec << UDP_BUFFER_SIZE << "], data:\n ";
		for (uint32_t i = 0; i < UDP_BUFFER_SIZE; ++i)
			its_message << std::hex << std::setw(2) << std::setfill('0')
						<< (int) (packet.udp_packet)[i] << " ";
		std::cout << its_message.str() << "\n";
	}

	point_cloud.points.clear();
	double time_stamp = pkt->data_header.utc_time.GetSecondValue();
	// point_cloud.points.header.time = time_stamp;

	if (decoder_config.decoder_debug)
	{
		printf("   FrameID %d, PkgID %d, PkgPosition %d, ts %.4f\n", pkt->data_header.FrameID(),
				pkt->data_header.PackageSeqNum(), pkt->data_header.PkgPosition(), time_stamp);
	}
	unsigned char LUTIndex = pkt->data_header.lut_index;
	unsigned short Column_ID = pkt->data_header.AzimuthIDX0();//pkt->data_header.azimuth_idx0;
	int Azimuth = pkt->data_block.Azimuth() * (100 / 256.0);

	if (Column_ID >= 480 || LUTIndex > 3 ) {
		std::cout << "*********error Column_ID || LUTIndex*************************" << "\n";
		return;
	}


	point_cloud.points.reserve(MAX_LIDAR_RING + 1);
	// static VPoint point;
	for (int i = 0; i < MAX_LIDAR_RING; i++) {	// 192 data blocks

		static int horizontal_angle = 0;
		horizontal_angle = (Azimuth + horizontal_angle_offset_[i][LUTIndex] + MAX_ANGLE) % MAX_ANGLE;
		static int vertical_angle = 0;
		vertical_angle = (vertical_angle_[i] + vertical_angle_offset_[Column_ID][LUTIndex] + MAX_ANGLE) % MAX_ANGLE;;
		static float distance = 0;
		distance = pkt->data_block.channel_data[i].Distance() * (1 / 256.0);
		point.timestamp = time_stamp; //float ts
		point.angle = horizontal_angle;
		point.ring = i;
		if (distance < 255) {
			point.x = distance * GetActCosValue(vertical_angle) * GetActCosValue(horizontal_angle);
			point.y = distance * GetActCosValue(vertical_angle) * GetActSinValue(horizontal_angle);
			point.z = distance * GetActSinValue(vertical_angle) ; ;
			point.intensity = pkt->data_block.channel_data[i].reflectivity;;
		} else {
			point.x = 0.0;
			point.y = 0.0;
			point.z = 0.0;
			point.intensity = 0;
		}
		// calibration
		ConvertPoint(point, decoder_config);
		// remove noise point
		if (point.x < decoder_config.car_modle_front_x &&
		                point.x > decoder_config.car_modle_back_x  &&
		                point.y < decoder_config.car_modle_left_y &&
		                point.y > decoder_config.car_modle_right_y &&
		                point.z < decoder_config.car_modle_top_z &&
		                point.z > decoder_config.car_modle_button_z  ) {
			point.x = 0.0;
			point.y = 0.0;
			point.z = 0.0;
			point.intensity = 0;
		}
		point_cloud.points.push_back(point);
		// point_cloud.points.width++;
	}
}
}
}
