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

	one_frame_.store(false);
}

HWDecoder::~HWDecoder() {

}

int HWDecoder::Init(const DecoderConfig &config) {
	msop_buf.udp_packet.resize(UDP_BUFFER_SIZE);
	decoder_config = config;
	return 0;
}

void HWDecoder::ConvertPacktsToPoint(const gassd_msgs::CommonUdpPacket &packet, PacketPointData &point_cloud, uint64_t &first_pkg_ts, int &pub_logic) {
	//pub_logic: 0 not fill not pub; 1 first pub then fill cloud poind; 2 only fill cloud poind; 3 fill cloud poind then pub
	pub_logic = -1;
	if(packet.udp_packet.size() == 0)
	{
		return;
	}
	RadarPointHeader  *header = (RadarPointHeader *)(&(packet.udp_packet[0]));
	// parse header
	if(header->header.isHeader())
	{
		timeval tv{};
		gettimeofday(&tv, nullptr);
		long long cur_ts = tv.tv_sec*1000 + tv.tv_usec / 1000;

		frame_ts_ = header->frame_ts_ms();

		if(header->frame_id() != frame_id_)//new frame
		{
			if(-1==frame_id_)//first frame
			{
				is_new = true;
				udp_pkg_count_ = 0;
				udp_pkg_num_ = header->pkg_num();
				obj_num_ = header->obj_num();
				first_pkg_ts = header->frame_ts_ms(); //ms
				frame_id_ = header->frame_id() % 65536;
				pub_logic = 0;
			}
			else //not first frame
			{
				if((((frame_id_+1)%65536) != header->frame_id())&&(-1!=frame_id_))
				{
					std::cout << "   lost frame, last:current -> " << frame_id_ << ":" <<   header->frame_id() << "\n";
				}
				if(!is_new)//finish last frame process
				{
					is_new = true;
					udp_pkg_count_ = 0;
					udp_pkg_num_ = header->pkg_num();
					obj_num_ = header->obj_num();
					first_pkg_ts = header->frame_ts_ms(); //ms
					frame_id_ = header->frame_id() % 65536;
					pub_logic = 0;
				}
				else //not finish last frame process
				{
					is_new = true;
					udp_pkg_count_ = 0;
					udp_pkg_num_ = header->pkg_num();
					obj_num_ = header->obj_num();
					first_pkg_ts = header->frame_ts_ms(); //ms
					frame_id_ = header->frame_id() % 65536;
					pub_logic = 1;
				}
			}
			if(1 == header->block_flag)
			{
				printf("-------------------- there is block! --------------------\n");
			}
		}
		return;
	}

	RadarPointContent  *content = (RadarPointContent *)(&(packet.udp_packet[0]));
	// parse content
	if(content->header.isContent())
	{
		if((1+udp_pkg_id_) != content->GetPackageId())
		{
			if(decoder_config.debug & 2)
				printf("   frame %d lost udp packet, last %d, current %d \n", frame_id_, udp_pkg_id_, content->GetPackageId());
		}
		udp_pkg_count_ ++;
		udp_pkg_id_ = content->GetPackageId();
		obj_num_in_pkg_ = content->GetPackageClusterNum();
		obj_recv_sum_+=obj_num_in_pkg_;

		if(is_new&&(udp_pkg_num_ > content->GetPackageId()))
		{
			pub_logic = 2;
		}
		else if(is_new&&(udp_pkg_num_ == content->GetPackageId())){
			if(content->GetPackageId() != udp_pkg_count_)
			{
				printf("   recv end pkg, pkg recv sum is: %d, less than pkg num: %d\n", udp_pkg_count_, udp_pkg_num_);
			}
			if(obj_recv_sum_ != obj_num_)
			{
				printf("   recv end pkg, obj recv sum is: %d, less than obj num: %d\n", obj_recv_sum_, obj_num_);
			}
			pub_logic = 3;
			is_new = false;
			udp_pkg_count_ = 0;
			udp_pkg_id_ = 0;
			obj_num_ = -1;
			obj_recv_sum_ = 0;
		}

		// point_cloud.points.reserve(50);
		point_cloud.points.clear();
		// point_cloud.header.time = frame_ts_/1000.0;
		for(int j=0; j< content->GetPackageClusterNum(); j++)
		{
			/*
				x=rcos(a)cos(b)
				y=rcos(a)sin(b)
				z=rsin(a)
				r为距离，a为俯仰角，b为方位角
			*/
			// point.id = content->cluster_info[j].GetClusterId();
			// point.motionStatus = content->cluster_info[j].motion_state;
			// point.existStatus  = content->cluster_info[j].exist_status;
			// point.falseAlarmProb  = content->cluster_info[j].false_alarm_prob;
			// point.heightConfidence = content->cluster_info[j].height_confidence;
			// point.modeFlag = content->cluster_info[j].mode_flag;

			point.intensity = content->cluster_info[j].motion_state;
			point.ring  = content->cluster_info[j].false_alarm_prob;
			point.timestamp = frame_ts_;

			/*SNR：信噪比，值越大，信噪比越高
			  点云多普勒状态：解模糊状态，3为解模糊，使用时需要进一步解模糊或者弃用；0为无模糊
			  虚警概率：虚假点的概率，和attribState结合使用，attribState非0，falseAlarmProb大于0.75这种点可以弃用，低于0.75相对可信
			*/
			if(decoder_config.filter_obj&1)
			{

				int ambg_state = content->cluster_info[j].ambg_state;
				if(3 == ambg_state)
				{
					continue;
				}
			}
			if(decoder_config.filter_obj&2)
			{
				int attrib_state = content->cluster_info[j].attrib_state;
				if((0 != attrib_state)&&(point.ring>3))
				{
					continue;
				}
			}
			float vx, vy, vz;
			content->cluster_info[j].GetObjectData(point.x, point.y, point.z, vx, vy, vz);

			// calibration
			ConvertPoint(point, decoder_config);
			if(decoder_config.debug & 1)
			{
				// if(1 == point.motionStatus)
				if(1 == point.intensity)
				{
					point_cloud.points.push_back(point);
				}
			}
			else
			{
				point_cloud.points.push_back(point);
			}
		}
	}
}
}
}


