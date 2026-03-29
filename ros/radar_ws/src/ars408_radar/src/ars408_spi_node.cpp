#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <map>

#define SPI_TRANS_SIZE          2880
#define SPI_BUFFER_SIZE         720
#define CAN3_RECV_MSG_ID_POS    0    // CAN3 (Front) 起始位置
#define CAN2_RECV_MSG_ID_POS    360  // CAN2 (Rear)  起始位置
#define CAN_SEGMENT_LENGTH      360  // 每路 CAN 占用的数组长度

struct RadarObject {
    int id;
    float dist_long;
    float dist_lat;
    float vrel_long;
    float vrel_lat;
};

class ARS408Node {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 两路雷达的发布者
    ros::Publisher pub_front_; // CAN3
    ros::Publisher pub_rear_;  // CAN2
    
    int spi_fd_;
    uint32_t spi_tx_buf_[SPI_BUFFER_SIZE];
    uint32_t spi_rx_buf_[SPI_BUFFER_SIZE];

    // 两路雷达的独立目标存储
    std::map<int, RadarObject> objects_front_;
    std::map<int, RadarObject> objects_rear_;

    std::string device_path_;
    int spi_speed_hz_;
    bool is_mock_;
    
    uint32_t mode_ = SPI_CPHA;
    uint8_t bits_ = 8;

public:
    ARS408Node() : private_nh_("~") {
        // 分别注册前/后雷达的 Topic
        pub_front_ = nh_.advertise<visualization_msgs::MarkerArray>("/radar_front/objects", 10);
        pub_rear_  = nh_.advertise<visualization_msgs::MarkerArray>("/radar_rear/objects", 10);
        
        private_nh_.param<std::string>("device_path", device_path_, "/dev/spidev0.0");
        private_nh_.param<int>("spi_speed_hz", spi_speed_hz_, 10000000);
        private_nh_.param<bool>("is_mock", is_mock_, false);

        initSPI();
    }

    ~ARS408Node() {
        if (spi_fd_ >= 0) close(spi_fd_);
    }

    void initSPI() {
        spi_fd_ = open(device_path_.c_str(), is_mock_ ? O_RDONLY : O_RDWR);
        if (spi_fd_ < 0) {
            ROS_ERROR("Can't open device/file: %s", device_path_.c_str());
            ros::shutdown();
            return;
        }

        if (!is_mock_) {
            ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode_);
            ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_);
            uint32_t speed_u32 = (uint32_t)spi_speed_hz_;
            ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_u32);
            ROS_INFO("Hardware SPI initialized at %d Hz on %s", spi_speed_hz_, device_path_.c_str());
        } else {
            ROS_INFO("Mock mode enabled. Reading from local file: %s", device_path_.c_str());
        }
    }

    uint32_t swapEndian(uint32_t data) {
        return ((data >> 24) & 0xFF) | ((data >> 8) & 0xFF00) | 
               ((data << 8) & 0xFF0000) | ((data << 24) & 0xFF000000);
    }

    // 独立解包函数：解包后存入指定的 map 中
    void parseObjectGeneral(uint8_t* data, std::map<int, RadarObject>& target_map) {
        RadarObject obj;
        obj.id = data[0];

        uint16_t raw_dist_long = (data[1] << 5) | (data[2] >> 3);
        obj.dist_long = raw_dist_long * 0.2f - 500.0f;

        uint16_t raw_dist_lat = ((data[2] & 0x07) << 8) | data[3];
        obj.dist_lat = raw_dist_lat * 0.2f - 204.6f;

        uint16_t raw_vrel_long = (data[4] << 2) | (data[5] >> 6);
        obj.vrel_long = raw_vrel_long * 0.25f - 128.0f;

        uint16_t raw_vrel_lat = ((data[5] & 0x3F) << 3) | (data[6] >> 5);
        obj.vrel_lat = raw_vrel_lat * 0.25f - 64.0f;

        target_map[obj.id] = obj;
    }

    // 独立发布函数：从指定的 map 中读取目标，并打上指定的 frame_id
    void publishMarkers(ros::Publisher& pub, std::map<int, RadarObject>& target_map, const std::string& frame_id) {
        visualization_msgs::MarkerArray marker_array;

        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = frame_id;
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.action = 3; // DELETEALL
        marker_array.markers.push_back(clear_marker);

        for (const auto& pair : target_map) {
            const auto& obj = pair.second;

            visualization_msgs::Marker box;
            box.header.frame_id = frame_id;
            box.header.stamp = ros::Time::now();
            box.ns = "radar_boxes";
            box.id = obj.id;
            box.type = visualization_msgs::Marker::CUBE;
            box.action = visualization_msgs::Marker::ADD;
            box.pose.position.x = obj.dist_long;
            box.pose.position.y = obj.dist_lat;
            box.pose.position.z = 0.5; 
            box.pose.orientation.w = 1.0;
            box.scale.x = 1.0; box.scale.y = 1.0; box.scale.z = 1.0;
            
            // 为了区分，前雷达用绿色，后雷达用红色
            if (frame_id == "radar_front_link") {
                box.color.r = 0.0f; box.color.g = 1.0f; box.color.b = 0.0f; box.color.a = 0.8f;
            } else {
                box.color.r = 1.0f; box.color.g = 0.0f; box.color.b = 0.0f; box.color.a = 0.8f;
            }
            box.lifetime = ros::Duration(0.2);

            visualization_msgs::Marker text = box;
            text.ns = "radar_text";
            text.id = obj.id + 1000;
            text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text.pose.position.z = 1.5;
            char text_buf[64];
            snprintf(text_buf, sizeof(text_buf), "ID:%d\nVx:%.1f", obj.id, obj.vrel_long);
            text.text = text_buf;
            text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f;

            marker_array.markers.push_back(box);
            marker_array.markers.push_back(text);
        }
        pub.publish(marker_array);
    }

    // 核心处理模块：传入起始位置和绑定的变量，解耦 CAN2/CAN3 的逻辑
    void processCanSegment(int start_pos, int length, std::map<int, RadarObject>& target_map, 
                           ros::Publisher& pub, const std::string& frame_id) {
        for (int i = 0; i < length - 2; i++) {
            uint32_t can_id = spi_rx_buf_[start_pos + i];
            
            if (can_id == 0x60A) {
                // 收到新帧，发布旧帧并清空 Map
                publishMarkers(pub, target_map, frame_id);
                target_map.clear();
                i += 2; 
            } 
            else if (can_id == 0x60B) {
                uint8_t data[8];
                uint32_t data_high = spi_rx_buf_[start_pos + i + 1];
                uint32_t data_low  = spi_rx_buf_[start_pos + i + 2];
                
                data[0] = data_high >> 24; data[1] = data_high >> 16;
                data[2] = data_high >> 8;  data[3] = data_high;
                data[4] = data_low >> 24;  data[5] = data_low >> 16;
                data[6] = data_low >> 8;   data[7] = data_low;

                parseObjectGeneral(data, target_map);
                i += 2; 
            }
            // 过滤掉 0x60C, 0x60D，避免误读后续数据
            else if (can_id == 0x60C || can_id == 0x60D) {
                i += 2;
            }
        }
    }

    void spin() {
        ros::Rate rate(50); 
        
        while (ros::ok()) {
            int ret = -1;

            if (!is_mock_) {
                struct spi_ioc_transfer tr = {};
                tr.tx_buf = (unsigned long)spi_tx_buf_;
                tr.rx_buf = (unsigned long)spi_rx_buf_;
                tr.len = SPI_TRANS_SIZE;
                tr.speed_hz = (uint32_t)spi_speed_hz_;
                tr.bits_per_word = bits_;
                ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
            } else {
                lseek(spi_fd_, 0, SEEK_SET); 
                int bytes_read = read(spi_fd_, spi_rx_buf_, SPI_TRANS_SIZE);
                ret = (bytes_read == SPI_TRANS_SIZE) ? 1 : -1;
            }

            if (ret >= 1) {
                for (int i = 0; i < SPI_BUFFER_SIZE; i++) {
                    spi_rx_buf_[i] = swapEndian(spi_rx_buf_[i]);
                }

                // 1. 处理 CAN3 段 (前雷达)，索引 0 ~ 359
                processCanSegment(CAN3_RECV_MSG_ID_POS, CAN_SEGMENT_LENGTH, 
                                  objects_front_, pub_front_, "radar_front_link");

                // 2. 处理 CAN2 段 (后雷达)，索引 360 ~ 719
                processCanSegment(CAN2_RECV_MSG_ID_POS, CAN_SEGMENT_LENGTH, 
                                  objects_rear_, pub_rear_, "radar_rear_link");
            }
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ars408_radar_node");
    ARS408Node node;
    node.spin();
    return 0;
}