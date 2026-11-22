#ifndef __TanwayFocusT_DECODE_H__
#define __TanwayFocusT_DECODE_H__
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <array>
#include <unordered_map>
#include "common_protocol.h"
#include "runnable/global/data_pool.h"

#define CULCULATE_TABLE_SIZE 360 * 100
#define DEG2ARC 3.14159265 / 180.0
// 双回波
#define FOCUS_T_MSOP_UDP_BUFFER_SIZE 1020
// 单回波
#define FOCUS_T_MSOP_UDP_BUFFER_SIZE_SINGLE 1236
#define FOCUS_T_DIFOP_UDP_BUFFER_SIZE 1024
static Eigen::Matrix<float, 3, 1> vec_t_;
static Eigen::Matrix<float, 3, 3> matrix_R_;

class FocusTDecoder {
public:
    FocusTDecoder() {
        printf("====================== FocusTDecoder ======================\n");

        for (int i = 0; i < CULCULATE_TABLE_SIZE; i++) {
            sin_table[i] = sin(i / 180.0 / (CULCULATE_TABLE_SIZE / 360.0) * 3.1415926);
        }

        for (int i = 0; i < CULCULATE_TABLE_SIZE; i++) {
            cos_table[i] = cos(i / 180.0 / (CULCULATE_TABLE_SIZE / 360.0) * 3.1415926);
        }
    }

    ~FocusTDecoder() {
    }

private:
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

    void ConvertPoint(VPoint &point, const avos::driver::LidarCaliConfig &config) {

		// float x_coord = point.x;
		// float y_coord = point.y;
        float y_coord = -point.x;
        float x_coord = point.y;
		float z_coord = point.z;
		point.x = config.transform[0] * x_coord + config.transform[1] * y_coord + config.transform[2] * z_coord + config.transform[3];
		point.y = config.transform[4] * x_coord + config.transform[5] * y_coord + config.transform[6] * z_coord + config.transform[7];
		point.z = config.transform[8] * x_coord + config.transform[9] * y_coord + config.transform[10] * z_coord + config.transform[11];
		point.angle = point.angle + config.tf_yaw;
		point.angle = point.angle >= 0.0 ? point.angle : point.angle + 360.0;
		point.angle = point.angle <= 360.0 ? point.angle : point.angle - 360.0;
	}

public:
    double GetCurrentAngle() {
        return 0.0;
    }

    bool IsEqualityFloat3(const double value1, const double value2) {
        return fabs(value1 - value2) < 0.001;
    }

    void SetEchoNum(avos::driver::EchoNum echoNum) { _echoNum = echoNum; }

    void SetMirrorVerAngleOffset(float a, float b, float c) {
        m_MirrorVerAngleOffset[0] = a;
        m_MirrorVerAngleOffset[1] = b;
        m_MirrorVerAngleOffset[2] = c;
    }

    void SetMirrorHorAngleOffset(float a, float b, float c) {
        m_MirrorHorAngleOffset[0] = a;
        m_MirrorHorAngleOffset[1] = b;
        m_MirrorHorAngleOffset[2] = c;
    }

    void SetCoreVerAngleOffset(float l, float r) {
        m_LeftRightMechCoreVerAngleOffset[0] = l;
        m_LeftRightMechCoreVerAngleOffset[1] = r;
    }

    void SetCoreHorAngleOffset(float l, float r) {
        m_LeftRightMechCoreHorAngleOffset[0] = l;
        m_LeftRightMechCoreHorAngleOffset[1] = r;
    }

    void SetXYZRotateAngle(float x, float y, float z) {
        m_transformSinRotateX = sin(x * m_calRA);
        m_transformCosRotateX = cos(x * m_calRA);
        m_transformSinRotateY = sin(y * m_calRA);
        m_transformCosRotateY = cos(y * m_calRA);
        m_transformSinRotateZ = sin(z * m_calRA);
        m_transformCosRotateZ = cos(z * m_calRA);
        // first roll; second pitch; third yaw;
        m_rotationMatrix[0][0] = m_transformCosRotateZ * m_transformCosRotateY;
        m_rotationMatrix[0][1] =
            m_transformCosRotateZ * m_transformSinRotateY * m_transformSinRotateX -
            m_transformSinRotateZ * m_transformCosRotateX;
        m_rotationMatrix[0][2] =
            m_transformCosRotateZ * m_transformSinRotateY * m_transformCosRotateX +
            m_transformSinRotateZ * m_transformSinRotateX;
        m_rotationMatrix[1][0] = m_transformSinRotateZ * m_transformCosRotateY;
        m_rotationMatrix[1][1] =
            m_transformSinRotateZ * m_transformSinRotateY * m_transformSinRotateX +
            m_transformCosRotateZ * m_transformCosRotateX;
        m_rotationMatrix[1][2] =
            m_transformSinRotateZ * m_transformSinRotateY * m_transformCosRotateX -
            m_transformCosRotateZ * m_transformSinRotateX;
        m_rotationMatrix[2][0] = -m_transformSinRotateY;
        m_rotationMatrix[2][1] = m_transformCosRotateY * m_transformSinRotateX;
        m_rotationMatrix[2][2] = m_transformCosRotateY * m_transformCosRotateX;
    }

    void InitBasicVariables() {
        // transform
        m_transformSinRotateX = sin(0 * m_calRA);
        m_transformCosRotateX = cos(0 * m_calRA);
        m_transformSinRotateY = sin(0 * m_calRA);
        m_transformCosRotateY = cos(0 * m_calRA);
        m_transformSinRotateZ = sin(0 * m_calRA);
        m_transformCosRotateZ = cos(0 * m_calRA);
        m_transformMoveX = 0;
        m_transformMoveY = 0;
        m_transformMoveZ = 0;

        // FocusT
        for (int i = 0; i < 3; i++) {
            m_skewing_sin_FocusT[i] = sin(m_FocusTMirrorABCAmend[i] * m_calRA);
            m_skewing_cos_FocusT[i] = cos(m_FocusTMirrorABCAmend[i] * m_calRA);
        }

        for (int i = 0; i < 96; i++) {
            double vA = m_verticalChannelAngle_FocusT[i];
            m_verticalChannelAngle_FocusT_cos_vA_RA[i] = cos(vA * m_calRA);
            m_verticalChannelAngle_FocusT_sin_vA_RA[i] = sin(vA * m_calRA);
        }
    }

    void DecodeDIFImp(const char *udpData, avos::driver::DeviceInfoFrame &deviceInfoFrame) {
        deviceInfoFrame.header =
            avos::driver::FourHexToInt(udpData[0], udpData[1], udpData[2], udpData[3]);
        deviceInfoFrame.deviceType =
            avos::driver::FourHexToInt(udpData[8], udpData[9], udpData[10], udpData[11]);
        deviceInfoFrame.deviceNumber =
            avos::driver::FourHexToInt(udpData[12], udpData[13], udpData[14], udpData[15]);

        char version[16] = {'\0'};
        snprintf(version, sizeof(version), "%d.%d.%d", udpData[17], udpData[18],
                udpData[19]);

        deviceInfoFrame.psVersion = version;

        snprintf(version, sizeof(version), "%d.%d.%d", udpData[21], udpData[22],
                udpData[23]);

        deviceInfoFrame.plVersion = version;

        deviceInfoFrame.workMode = udpData[32];
        deviceInfoFrame.workStatus = udpData[33];
    }

    void UseDecodeFocusT(const char *udpData, avos::driver::PacketPointData &point_cloud, const avos::driver::LidarCaliConfig &decoder_config, uint64_t &first_pkg_ts, int &pub_logic, bool is_single) {
        //pub_logic: 0 not fill not pub; 1 first pub then fill cloud poind; 2 only fill cloud poind; 3 fill cloud poind then pub
        // ptp
        // check ret mode
        if(false == get_ret_mode)
        {
            get_ret_mode = true;
            ret_mode = udpData[10];
            printf("\n\n=============================== ret_mode is: %d [%s] ===============================\n\n", ret_mode, ret_mode_str.c_str());
        }

        unsigned int frameSecond =
            avos::driver::FourHexToInt(udpData[13], udpData[14], udpData[15], udpData[16]);
        double frameMicrosecond =
            avos::driver::FourHexToInt(udpData[17], udpData[18], udpData[19], udpData[20]) * 0.1;

        // point_cloud.points.clear();
	    double time_stamp = frameSecond + frameMicrosecond * 1e-6;
        pub_logic = 0;
        if(avos::driver::TwoHextoInt(udpData[4], udpData[5]) != frame_id_)//new frame, frame_idx(byte 4/5)::[0~65535]
        {
            if(decoder_config.debug & 1)
            {
                std::cout << "\n/***********************************/\nreceive new frame, FrameID: " <<  avos::driver::TwoHextoInt(udpData[4], udpData[5]) << "\n";
            }
            if((((frame_id_+1)%65536) != avos::driver::TwoHextoInt(udpData[4], udpData[5]))&&(-1!=frame_id_))
            {
                std::cout << "lost frame, last:current -> " << frame_id_ << ":" <<  avos::driver::TwoHextoInt(udpData[4], udpData[5]) << "\n";
            }

            if(-1==frame_id_)//first frame
            {
                if(avos::driver::TwoHextoInt(udpData[21], udpData[22]) == avos::driver::TwoHextoInt(udpData[6], udpData[7])) // udp_idx(byte 6/7):[1~1024], udp_sum(byte 21/22):750
                {
                    if(decoder_config.debug & 1)
                    {
                        std::cout << "first frame , first packet is end packet, drop it" << "\n";
                    }
                    pub_logic = 0;
                    return;
                }
                else
                {
                    if(decoder_config.debug & 1)
                    {
                        std::cout << "first frame , first packet is middle packet" << "\n";
                    }
                    pub_logic = 2;
                }
                is_new = true;
                udp_pkg_count_ = 0;
                first_pkg_ts = time_stamp * 1000; //ms
                frame_id_ = avos::driver::TwoHextoInt(udpData[4], udpData[5]) % 65536;
                udp_pkg_id_ = avos::driver::TwoHextoInt(udpData[6], udpData[7]);
            }
            else //not first frame
            {
                if((avos::driver::TwoHextoInt(udpData[4], udpData[5])+1)%65536 == frame_id_) // recv old frame's udp, drop
                {
                    if(decoder_config.debug & 16)
                    {
                        printf("recv old frame's udp, drop it. recv udp frame is: %d, frame_id_ is: %d \n ", avos::driver::TwoHextoInt(udpData[4], udpData[5]), frame_id_);
                    }
                    pub_logic = 0;
                    return;
                }
                if(!is_new)//finish last frame process
                {
                    if(1 == avos::driver::TwoHextoInt(udpData[6], udpData[7]))//new frame, first packet
                    {
                        if(decoder_config.debug & 1)
                        {
                            std::cout << "finish last frame process, receive new frame , first packet is head packet" << "\n";
                        }
                        pub_logic = 2;
                    }
                    else if(avos::driver::TwoHextoInt(udpData[21], udpData[22]) != avos::driver::TwoHextoInt(udpData[6], udpData[7]))
                    {
                        if(decoder_config.debug & 1)
                        {
                            std::cout << "finish last frame process, receive new frame , first packet is middle packet" << "\n";
                        }
                        pub_logic = 2;
                    }
                    else if(avos::driver::TwoHextoInt(udpData[21], udpData[22]) == avos::driver::TwoHextoInt(udpData[6], udpData[7]))
                    {
                        if(decoder_config.debug & 1)
                        {
                            std::cout << "finish last frame process, receive new frame , first packet is end packet, drop it" << "\n";
                        }
                        pub_logic = 0;
                        return;
                    }
                    is_new = true;
                    udp_pkg_count_ = 0;
                    first_pkg_ts = time_stamp * 1000;
                    frame_id_ = avos::driver::TwoHextoInt(udpData[4], udpData[5]) % 65536;
                    udp_pkg_id_ = avos::driver::TwoHextoInt(udpData[6], udpData[7]);
                }
                else //not finish last frame process
                {
                    if(decoder_config.debug & 1)
                    {
                        std::cout << "receive a new frame , while not finish receive last frame's packets" << "\n";
                    }
                    is_new = true;
                    pub_logic = 1;
                    udp_pkg_count_ = 0;
                    first_pkg_ts = time_stamp * 1000; //only exceed at new frame
                    frame_id_ = avos::driver::TwoHextoInt(udpData[4], udpData[5]) % 65536;
                    udp_pkg_id_ = avos::driver::TwoHextoInt(udpData[6], udpData[7]);
                }
            }
        }
        else //same frame
        {
            if((1+udp_pkg_id_) != avos::driver::TwoHextoInt(udpData[6], udpData[7]))
            {
                if(decoder_config.debug & 1)
                printf("frame %d lost packet, last %d, current %d ts %.3f\n", avos::driver::TwoHextoInt(udpData[4], udpData[5]), udp_pkg_id_, avos::driver::TwoHextoInt(udpData[6], udpData[7]),time_stamp);
            }
            udp_pkg_count_ ++;
            udp_pkg_id_ = avos::driver::TwoHextoInt(udpData[6], udpData[7]);

            if((false == not_use_udp_idx) && (avos::driver::TwoHextoInt(udpData[21], udpData[22]) < avos::driver::TwoHextoInt(udpData[6], udpData[7])))
            {
                not_use_udp_idx = true; // udp in a frame execeed udp_bag_sum, donot use udp_bag_sum as flag for "recv all udp bag"
            }
            if(not_use_udp_idx)
            {
                pub_logic = 2;
            }
            else
            {
                if(is_new&&(avos::driver::TwoHextoInt(udpData[21], udpData[22]) != avos::driver::TwoHextoInt(udpData[6], udpData[7])))//middle udp bag
                {
                    pub_logic = 2;
                }
                else if(is_new&&(avos::driver::TwoHextoInt(udpData[21], udpData[22]) == avos::driver::TwoHextoInt(udpData[6], udpData[7])))//last udp bag
                {
                    if(decoder_config.debug & 1)
                    {
                        std::cout << "receive end packet\n";
                    }
                    // if(avos::driver::TwoHextoInt(udpData[6], udpData[7]) != 750)// one frame 750 udps.
                    // {
                    //     std::cout << "end PkgID should be 750, but real id is: " << udp_pkg_id_ << "\n";
                    // }
                    pub_logic = 3;
                    is_new = false; //finish recv all udp
                    udp_pkg_count_ = -1;
                    udp_pkg_id_ = -1;
                }
            }
        }

        if(decoder_config.debug & 2)
        {
            printf("   FrameID %d, PkgID %d, PkgCnt %d, PkgSum %d, ts %.4f\n", avos::driver::TwoHextoInt(udpData[4], udpData[5]),
                    avos::driver::TwoHextoInt(udpData[6], udpData[7]), udp_pkg_count_, avos::driver::TwoHextoInt(udpData[21], udpData[22]), time_stamp);
        }

        // point_cloud.header.time = time_stamp; ///////////////
        if(is_single)
        {
            double horizontalAngle = 0;
            unsigned short mirror = 0;
            double Nx = 0.0, Ny = 0.0, Nz = 0.0;

            int frame_cycle_cout = avos::driver::TwoHextoInt(udpData[4], udpData[5]);
            int data_block = 6; // 每个data block 6字节
            for (int blocks_num = 0; blocks_num < 12; blocks_num++) {
                int offset = blocks_num * 100;

                int offsetMicrosecond =
                    avos::driver::TwoHextoInt(udpData[32 + offset], udpData[33 + offset]);
                // ptp
                double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
                unsigned int blockSecond =
                    (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
                unsigned int blockMicrosecond =
                    (totalMicrosecond >= 1000000)
                        ? (unsigned int)(totalMicrosecond - 1000000)
                        : (unsigned int)totalMicrosecond;

                mirror = (udpData[35 + offset] & 0x06) >> 1;

                for (int seq = 0; seq < 16; seq++) {
                // horizontalAngle表示点云水平角度(φ)
                int HextoAngle = avos::driver::TwoHextoInt(udpData[36 + offset + seq * data_block],
                                            udpData[37 + offset + seq * data_block]);
                horizontalAngle = HextoAngle * 0.01;
                horizontalAngle += m_MirrorHorAngleOffset[mirror];

                // 点云径向距离L_1
                double hexToInt1 = avos::driver::TwoHextoInt(udpData[38 + offset + seq * data_block],
                                                udpData[39 + offset + seq * data_block]);
                double L_1 = hexToInt1 * m_calSimpleFPGA;

                unsigned char hexChar1 = udpData[40 + offset + seq * data_block];
                double intensity_1 = ((double)hexChar1);

                int channel =
                    16 * (blocks_num < 6 ? blocks_num : blocks_num - 6) + seq + 1;
                int laser_column = 0;
                if ((26 <= channel && channel <= 40 && channel % 2 == 0) ||
                    (57 <= channel && channel <= 64))
                    laser_column = 1;
                else if ((9 <= channel && channel <= 23 && channel % 2 == 1) ||
                        (41 <= channel && channel <= 55 && channel % 2 == 1) ||
                        (65 <= channel && channel <= 72) ||
                        (81 <= channel && channel <= 88))
                    laser_column = 2;
                else if ((1 <= channel && channel <= 8) ||
                        (25 <= channel && channel <= 39 && channel % 2 == 1) ||
                        (73 <= channel && channel <= 80) ||
                        (89 <= channel && channel <= 96))
                    laser_column = 3;
                else if ((10 <= channel && channel <= 24 && channel % 2 == 0) ||
                        (42 <= channel && channel <= 56 && channel % 2 == 0))
                    laser_column = 4;
                double theta = 0; // 转镜法线转过的角度
                double alpha = m_alphaMap_FocusT.count(channel)
                                    ? m_alphaMap_FocusT.at(channel)
                                    : 0.0; // 修正角度

                double horizontalAngle_modify = 0;
                // 转镜法线转过的角度 θ
                switch (laser_column) {
                case 1:
                    theta = (horizontalAngle - 1.63 + alpha) * m_calRA / 2;
                    horizontalAngle_modify = horizontalAngle - 1.63;
                    break;
                case 2:
                    theta = (horizontalAngle - 0.82 + alpha) * m_calRA / 2;
                    horizontalAngle_modify = horizontalAngle - 0.82;
                    break;
                case 3:
                    theta = (horizontalAngle + alpha) * m_calRA / 2;
                    horizontalAngle_modify = horizontalAngle;
                    break;
                case 4:
                    theta = (horizontalAngle + 0.82 + alpha) * m_calRA / 2;
                    horizontalAngle_modify = horizontalAngle + 0.82;
                    break;
                default:
                    break;
                }

                // TWPoint basic_point;
                // basic_point.angle = horizontalAngle_modify;
                // basic_point.mirror = mirror;
                // basic_point.channel = channel;
                // basic_point.cycle_count = frame_cycle_cout;

                basic_point.timestamp = blockSecond + blockMicrosecond * 1e-6;
                basic_point.angle = horizontalAngle_modify;
                basic_point.ring = channel;

                // 计算各通道竖直角度（β）的正弦和余弦
                double cos_beta = m_verticalChannelAngle_FocusT_cos_vA_RA[channel - 1];
                double sin_beta = m_verticalChannelAngle_FocusT_sin_vA_RA[channel - 1];

                // 机芯发射光线单位向量 li = (-cosβ,0,sinβ)
                double Lx = -cos_beta;
                double Ly = 0;
                double Lz = sin_beta;

                // Nx = cosδcosθ
                // Ny = cosδsinθ
                // Nz = sinδ
                Nx = m_skewing_cos_FocusT[mirror] * cos(theta);
                Ny = m_skewing_cos_FocusT[mirror] * sin(theta);
                Nz = m_skewing_sin_FocusT[mirror];

                double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

                int apd_temp = (udpData[23] & 0x7F);
                if (udpData[23] & 0x80)
                    apd_temp = -apd_temp;

                // 计算笛卡尔坐标X,Y,Z
                if (_echoNum & avos::driver::EchoNum::Echo1) {
                    basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
                    basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
                    basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

                    // basic_point.echo = 1;
                    // basic_point.distance = L_1;
                    basic_point.intensity = intensity_1;
                    // basic_point.confidence = confidence1;
                    // basic_point.encoding_interval = _encoding_interval;
                    // basic_point.state = 0;
                    // basic_point.label = 0;
                    // basic_point.apd_temp = apd_temp;
                    // basic_point.pulseCodeInterval = udpData[24] * 10;
                    // points.push_back(std::move(basic_point));

                    // calibration
                    ConvertPoint(basic_point, decoder_config);
                    // remove noise points
                    if (basic_point.x < decoder_config.car_modle_front_x &&
                            basic_point.x > decoder_config.car_modle_back_x  &&
                            basic_point.y < decoder_config.car_modle_left_y &&
                            basic_point.y > decoder_config.car_modle_right_y &&
                            basic_point.z < decoder_config.car_modle_top_z &&
                            basic_point.z > decoder_config.car_modle_button_z  ) {
                        basic_point.x = 0.0;
                        basic_point.y = 0.0;
                        basic_point.z = 0.0;
                        basic_point.intensity = 0;
                    }

                    point_cloud.points.push_back(basic_point);
                    if(decoder_config.debug & 8)
                    {
                        printf("echo1 x=%.3f, y=%.3f, z=%.3f\n", basic_point.x, basic_point.y, basic_point.z);
                    }
                }
                }
            }
        }
        else
        {
            double horizontalAngle = 0;
            unsigned short mirror = 0;
            double Nx = 0.0, Ny = 0.0, Nz = 0.0;

            int frame_cycle_cout = avos::driver::TwoHextoInt(udpData[4], udpData[5]);
            int data_block = 10; // 每个data block 10字节
            for (int blocks_num = 0; blocks_num < 6; blocks_num++) {
                int offset = blocks_num * 164;

                int offsetMicrosecond =
                    avos::driver::TwoHextoInt(udpData[32 + offset], udpData[33 + offset]);
                // ptp
                double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
                unsigned int blockSecond =
                    (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
                unsigned int blockMicrosecond =
                    (totalMicrosecond >= 1000000)
                        ? (unsigned int)(totalMicrosecond - 1000000)
                        : (unsigned int)totalMicrosecond;

                mirror = (udpData[35 + offset] & 0x06) >> 1;
                // unsigned short faceIndex = mirror;

                for (int seq = 0; seq < 16; seq++) {
                // horizontalAngle表示点云水平角度(φ)
                int HextoAngle = avos::driver::TwoHextoInt(udpData[36 + offset + seq * data_block],
                                            udpData[37 + offset + seq * data_block]);
                horizontalAngle = HextoAngle * 0.01;
                horizontalAngle += m_MirrorHorAngleOffset[mirror];

                // 点云径向距离L_1
                double hexToInt1 = avos::driver::TwoHextoInt(udpData[38 + offset + seq * data_block],
                                                udpData[39 + offset + seq * data_block]);
                double L_1 = hexToInt1 * m_calSimpleFPGA;

                unsigned char hexChar1 = udpData[40 + offset + seq * data_block];
                double intensity_1 = ((double)hexChar1);

                // double confidence1 = udpData[41 + offset + seq * 10];  //预留

                // 点云径向距离L_2
                double hexToInt2 = avos::driver::TwoHextoInt(udpData[42 + offset + seq * data_block],
                                                udpData[43 + offset + seq * data_block]);
                double L_2 = hexToInt2 * m_calSimpleFPGA;

                unsigned char hexChar2 = udpData[44 + offset + seq * data_block];
                double intensity_2 = ((double)hexChar2);

                // double confidence2 = udpData[45 + offset + seq * 10];

                int channel = 16 * blocks_num + seq + 1;
                int laser_column = 0;
                if ((26 <= channel && channel <= 40 && channel % 2 == 0) ||
                    (57 <= channel && channel <= 64))
                    laser_column = 1;
                else if ((9 <= channel && channel <= 23 && channel % 2 == 1) ||
                        (41 <= channel && channel <= 55 && channel % 2 == 1) ||
                        (65 <= channel && channel <= 72) ||
                        (81 <= channel && channel <= 88))
                    laser_column = 2;
                else if ((1 <= channel && channel <= 8) ||
                        (25 <= channel && channel <= 39 && channel % 2 == 1) ||
                        (73 <= channel && channel <= 80) ||
                        (89 <= channel && channel <= 96))
                    laser_column = 3;
                else if ((10 <= channel && channel <= 24 && channel % 2 == 0) ||
                        (42 <= channel && channel <= 56 && channel % 2 == 0))
                    laser_column = 4;
                double theta = 0; // 转镜法线转过的角度
                double alpha = m_alphaMap_FocusT.count(channel)
                                    ? m_alphaMap_FocusT.at(channel)
                                    : 0.0; // 修正角度

                double horizontalAngle_modify = 0;
                switch (laser_column) {
                case 1:
                    theta = (horizontalAngle + 5.0 - 1.63 + alpha) * m_calRA / 2;
                    horizontalAngle_modify = horizontalAngle - 1.63;
                    break;
                case 2:
                    theta = (horizontalAngle + 5.0 - 0.82 + alpha) * m_calRA / 2;
                    horizontalAngle_modify = horizontalAngle - 0.82;
                    break;
                case 3:
                    theta = (horizontalAngle + 5.0 + alpha) * m_calRA / 2;
                    horizontalAngle_modify = horizontalAngle;
                    break;
                case 4:
                    theta = (horizontalAngle + 5.0 + 0.82 + alpha) * m_calRA / 2;
                    horizontalAngle_modify = horizontalAngle + 0.82;
                    break;
                default:
                    break;
                }

                // TWPoint basic_point;
                // basic_point.angle = horizontalAngle_modify; // 需要修正
                // basic_point.mirror = mirror;
                // basic_point.channel = channel;
                // basic_point.cycle_count = frame_cycle_cout;

                basic_point.timestamp = blockSecond + blockMicrosecond * 1e-6;
                basic_point.angle = horizontalAngle_modify;
                basic_point.ring = channel;

                // 计算各通道竖直角度（β）的正弦和余弦
                double cos_beta = m_verticalChannelAngle_FocusT_cos_vA_RA[channel - 1];
                double sin_beta = m_verticalChannelAngle_FocusT_sin_vA_RA[channel - 1];

                // 机芯发射光线单位向量 li = (-cosβ,0,sinβ)
                double Lx = -cos_beta;
                double Ly = 0;
                double Lz = sin_beta;

                // 转镜法线转过的角度 θ = φ / 2
                // double theta = horizontalAngle * m_calRA / 2;

                // Nx = cosδcosθ
                // Ny = cosδsinθ
                // Nz = sinδ
                Nx = m_skewing_cos_FocusT[mirror] * cos(theta);
                Ny = m_skewing_cos_FocusT[mirror] * sin(theta);
                Nz = m_skewing_sin_FocusT[mirror];

                double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

                // APD温度，背面点标定使用
                int apd_temp = (udpData[23] & 0x7F);
                if (udpData[23] & 0x80)
                    apd_temp = -apd_temp;

                // 计算笛卡尔坐标X,Y,Z
                if (_echoNum & avos::driver::EchoNum::Echo1) {
                    basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
                    basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
                    basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

                    // basic_point.echo = 1;
                    // basic_point.distance = L_1;
                    basic_point.intensity = intensity_1;
                    // basic_point.confidence = confidence1;
                    // basic_point.encoding_interval = _encoding_interval;
                    // basic_point.state = 0;
                    // basic_point.label = 0;
                    // basic_point.apd_temp = apd_temp;
                    // basic_point.pulseCodeInterval = udpData[24] * 10;
                    // points.push_back(std::move(basic_point));

                    // calibration
                    ConvertPoint(basic_point, decoder_config);
                    // remove noise points
                    if (basic_point.x < decoder_config.car_modle_front_x &&
                            basic_point.x > decoder_config.car_modle_back_x  &&
                            basic_point.y < decoder_config.car_modle_left_y &&
                            basic_point.y > decoder_config.car_modle_right_y &&
                            basic_point.z < decoder_config.car_modle_top_z &&
                            basic_point.z > decoder_config.car_modle_button_z  ) {
                        basic_point.x = 0.0;
                        basic_point.y = 0.0;
                        basic_point.z = 0.0;
                        basic_point.intensity = 0;
                    }

                    point_cloud.points.push_back(basic_point);
                    if(decoder_config.debug & 8)
                    {
                        printf("echo1 x=%.3f, y=%.3f, z=%.3f\n", basic_point.x, basic_point.y, basic_point.z);
                    }
                }

                if (_echoNum & avos::driver::EchoNum::Echo2) {
                    basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
                    basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
                    basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

                    // basic_point.echo = 2;
                    // basic_point.distance = L_2;
                    basic_point.intensity = intensity_2;
                    // basic_point.confidence = confidence2;
                    // basic_point.encoding_interval = _encoding_interval;
                    // basic_point.state = 0;
                    // basic_point.label = 0;
                    // basic_point.apd_temp = apd_temp;
                    // basic_point.pulseCodeInterval = udpData[24] * 10;
                    // points.push_back(std::move(basic_point));

                    // calibration
                    ConvertPoint(basic_point, decoder_config);
                    // remove noise points
                    if (basic_point.x < decoder_config.car_modle_front_x &&
                            basic_point.x > decoder_config.car_modle_back_x  &&
                            basic_point.y < decoder_config.car_modle_left_y &&
                            basic_point.y > decoder_config.car_modle_right_y &&
                            basic_point.z < decoder_config.car_modle_top_z &&
                            basic_point.z > decoder_config.car_modle_button_z  ) {
                        basic_point.x = 0.0;
                        basic_point.y = 0.0;
                        basic_point.z = 0.0;
                        basic_point.intensity = 0;
                    }

                    point_cloud.points.push_back(basic_point);
                    if(decoder_config.debug & 8)
                    {
                        printf("echo2 x=%.3f, y=%.3f, z=%.3f\n", basic_point.x, basic_point.y, basic_point.z);
                    }
                }
                }
            }
        }
    }

    void DecodeDIFData_FocusT(const char *udpData) {
        // 计算ABC面转镜俯仰角（δ）

        double mirrorABCAmend[3] = {0};
        // double mirrorABCOffset = 0.2;

        int base = 508 + 4 * 14 + 3; // 倒序计算，先计算C镜面
        for (int i = 0; i < 3; i++) {
            // 每个面占用连续的10位，计算起始位: i=0→0, i=1→10, i=2→20
            int start_bit = i * 10;

            // 计算起始字节偏移和位偏移
            int byte_offset = start_bit / 8; // 确定起始所在的字节位置,0,1,2
            int bit_offset = start_bit % 8;  // 确定位偏移,0,2,4

            // 合并两个连续的字节数据
            unsigned short merged_value =
                ((unsigned char)(udpData[base - byte_offset - 1])
                << 8) |                                    // 高字节左移8位
                (unsigned char)udpData[base - byte_offset]; // // 低字节补在末尾
            // 截取目标10位
            merged_value = (merged_value >> (bit_offset)) & 0x3FF; // 右移后，保留低10位

            // 计算修正量 δ = (K-300)*0.001
            mirrorABCAmend[2 - i] = (merged_value - 300) * 0.001;
        }

        for (int i = 0; i < 3; i++) {
            m_FocusTMirrorABCAmend[i] = mirrorABCAmend[i] + m_MirrorVerAngleOffset[i];
            // 计算sinδ和cosδ
            m_skewing_sin_FocusT[i] = sin(m_FocusTMirrorABCAmend[i] * m_calRA);
            m_skewing_cos_FocusT[i] = cos(m_FocusTMirrorABCAmend[i] * m_calRA);
        }

        // m_tempoA4UserMode = (udpData[31] >> 4) == 0;

        // DeviceInfoFrame deviceInfoFrame;
        // DecodeDIFImp(udpData, deviceInfoFrame);

        // _lidarObserver->OnDeviceInfoFrame(_lidarInfo, deviceInfoFrame);
    }
private:
    void InitMapTable() {
        return;
    }

private:
    // SensorLidarPoint           basic_point;
    VPoint                     basic_point;
    double                     sin_table[CULCULATE_TABLE_SIZE];
    double                     cos_table[CULCULATE_TABLE_SIZE];
    Eigen::Matrix<float, 3, 1> point_sensor;
    Eigen::Matrix<float, 3, 1> point_vcs;
    std::map<std::string, std::uint32_t>  ec_map;

    int32_t frame_id_ = -1;
	int32_t udp_pkg_id_ = -1;
	int32_t udp_pkg_count_ = -1;
    bool    not_use_udp_idx = false;
	bool	is_new = false;
public:
    double m_calRA = (double)(3.14159265f / 180.0f);
    avos::driver::EchoNum _echoNum = avos::driver::EchoNum::Echo1;
    int _lidarTypeFromDIF = -1; // 从DIF中读到的雷达型号,需要转换成字符串
    unsigned char PCF_type = 0x0;
    double m_startAngle = 30.0;
    double m_endAngle = 150.0;

    double m_calSimpleFPGA = 0.004796; // 0.032 * 2.997924 / 10.f / 2;
    double m_calPulseFPGA = 0.032;

    // transform
    double m_transformSinRotateX = 0;
    double m_transformCosRotateX = 1;
    double m_transformSinRotateY = 0;
    double m_transformCosRotateY = 1;
    double m_transformSinRotateZ = 0;
    double m_transformCosRotateZ = 1;
    double m_transformMoveX = 0;
    double m_transformMoveY = 0;
    double m_transformMoveZ = 0;

    // 俯仰与水平修正角度
    double m_MirrorVerAngleOffset[3] = {0.0};
    double m_MirrorHorAngleOffset[3] = {0.0};

    float m_verticalChannelAngle_FocusT[96] = {
      12.41f, 12.06f,  11.72f,  11.37f,  11.03f,  10.68f,  10.33f,  9.98f,
      9.64f,  9.46f,   9.29f,   9.12f,   8.94f,   8.77f,   8.59f,   8.42f,
      8.24f,  8.07f,   7.90f,   7.72f,   7.55f,   7.37f,   7.20f,   7.02f,
      6.85f,  6.67f,   6.50f,   6.32f,   6.15f,   5.97f,   5.80f,   5.62f,
      5.45f,  5.27f,   5.10f,   4.92f,   4.75f,   4.57f,   4.39f,   4.22f,
      4.04f,  3.87f,   3.69f,   3.52f,   3.34f,   3.17f,   2.99f,   2.81f,
      2.64f,  2.46f,   2.29f,   2.11f,   1.93f,   1.76f,   1.58f,   1.41f,
      1.23f,  0.88f,   0.53f,   0.18f,   -0.18f,  -0.53f,  -0.88f,  -1.23f,
      -1.58f, -1.93f,  -2.29f,  -2.64f,  -2.99f,  -3.34f,  -3.69f,  -4.04f,
      -4.39f, -4.75f,  -5.10f,  -5.45f,  -5.80f,  -6.15f,  -6.50f,  -6.85f,
      -7.20f, -7.55,   -7.90f,  -8.24f,  -8.59f,  -8.94f,  -9.29f,  -9.64f,
      -9.98f, -10.33f, -10.68f, -11.03f, -11.37f, -11.72f, -12.06f, -12.41f};
  double m_verticalChannelAngle_FocusT_cos_vA_RA[96] = {0.f};
  double m_verticalChannelAngle_FocusT_sin_vA_RA[96] = {0.f};

  double m_FocusTMirrorABCAmend[3] = {0.0};
  double m_skewing_sin_FocusT[3];
  double m_skewing_cos_FocusT[3];

  // 定义 alpha 值映射
  const std::unordered_map<int, double> m_alphaMap_FocusT = {
      {10, 0.00171693},  {25, 0.00171693},  {81, 0.00171693},
      {9, 0.005324973},  {26, 0.005324973}, {89, 0.005324973},
      {12, 0.008933015}, {27, 0.008933015}, {82, 0.008933015},
      {11, 0.012416642}, {28, 0.012416642}, {90, 0.012416642},
      {14, 0.015900269}, {29, 0.015900269}, {83, 0.015900269},
      {13, 0.019508311}, {30, 0.019508311}, {91, 0.019508311},
      {16, 0.023116353}, {31, 0.023116353}, {84, 0.023116353},
      {15, 0.02659998},  {32, 0.02659998},  {92, 0.02659998},
      {18, 0.030083607}, {33, 0.030083607}, {85, 0.030083607},
      {17, 0.033193988}, {34, 0.033193988}, {93, 0.033193988},
      {20, 0.036304369}, {35, 0.036304369}, {86, 0.036304369},
      {19, 0.039290335}, {36, 0.039290335}, {94, 0.039290335},
      {22, 0.042276301}, {37, 0.042276301}, {87, 0.042276301},
      {21, 0.045386683}, {38, 0.045386683}, {95, 0.045386683},
      {24, 0.048497064}, {39, 0.048497064}, {88, 0.048497064},
      {23, 0.05148303},  {40, 0.05148303},  {96, 0.05148303},
      {42, 0.054468996}, {65, 0.054468996}, {1, 0.054468996},
      {41, 0.056335224}, {73, 0.056335224}, {57, 0.056335224},
      {44, 0.058201453}, {66, 0.058201453}, {2, 0.058201453},
      {43, 0.059943267}, {74, 0.059943267}, {58, 0.059943267},
      {46, 0.06168508},  {67, 0.06168508},  {3, 0.06168508},
      {45, 0.063551309}, {75, 0.063551309}, {59, 0.063551309},
      {48, 0.065417538}, {68, 0.065417538}, {4, 0.065417538},
      {47, 0.067159351}, {76, 0.067159351}, {60, 0.067159351},
      {50, 0.068901165}, {69, 0.068901165}, {5, 0.068901165},
      {49, 0.070767393}, {77, 0.070767393}, {61, 0.070767393},
      {52, 0.072633622}, {70, 0.072633622}, {6, 0.072633622},
      {51, 0.074375435}, {78, 0.074375435}, {62, 0.074375435},
      {54, 0.076117249}, {71, 0.076117249}, {7, 0.076117249},
      {53, 0.077983478}, {79, 0.077983478}, {63, 0.077983478},
      {56, 0.079849706}, {72, 0.079849706}, {8, 0.079849706},
      {55, 0.08159152},  {80, 0.08159152},  {64, 0.08159152}};

    // 机芯垂直与水平角度
    double m_LeftRightMechCoreVerAngleOffset[2] = {0.0};
    double m_LeftRightMechCoreHorAngleOffset[2] = {0.0};

    double m_rotationMatrix[3][3] = {
      {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};


    /*
        0:双回波/强零: 1:双回波/前零
        2:双回波/后零 3:双回波/强后
        4:双回波/强前 5:双回波/透传
        6:单回波/单回波(默认最强)
    */
    bool    get_ret_mode = false;
    uint8_t ret_mode = 6; //
    std::string ret_mode_str = "0:双回波/强零 1:双回波/前零 2:双回波/后零 3:双回波/强后 4:双回波/强前 5:双回波/透传 6:单回波/单回波(默认最强)";
};

#endif
