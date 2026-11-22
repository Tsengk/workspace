#ifndef __Scope128_DECODE_H__
#define __Scope128_DECODE_H__
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <array>
#include <unordered_map>
#include "common_protocol.h"
#include "runnable/global/data_pool.h"

#define CULCULATE_TABLE_SIZE 360 * 100
#define DEG2ARC 3.14159265 / 180.0
// 双回波
#define SCOPE128_MSOP_UDP_BUFFER_SIZE 1348
// 单回波
#define SCOPE128_MSOP_UDP_BUFFER_SIZE_SINGLE 1236

#define SCOPE128_DIFOP_UDP_BUFFER_SIZE 1024
static Eigen::Matrix<float, 3, 1> vec_t_;
static Eigen::Matrix<float, 3, 3> matrix_R_;

class Scope128Decoder {
public:
    Scope128Decoder() {
        printf("====================== Scope128Decoder ======================\n");

        for (int i = 0; i < CULCULATE_TABLE_SIZE; i++) {
            sin_table[i] = sin(i / 180.0 / (CULCULATE_TABLE_SIZE / 360.0) * 3.1415926);
        }

        for (int i = 0; i < CULCULATE_TABLE_SIZE; i++) {
            cos_table[i] = cos(i / 180.0 / (CULCULATE_TABLE_SIZE / 360.0) * 3.1415926);
        }
    }

    ~Scope128Decoder() {
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

        // LT_Scope128
        for (int i = 0; i < 32; i++) {
            m_verticalChannelsAngle_SCP128P[i] = m_verticalChannelsAngle_SCP128P_v2[i];
            m_verticalChannelsAngle_SCP128N[i] = m_verticalChannelsAngle_SCP128N_v2[i];

            double vA_P = m_verticalChannelsAngle_SCP128P[i]; // 默认使用新的β角
            m_verticalChannelAngle_scp128P_sin_vA_RA[i] = sin(vA_P * m_calRA);
            m_verticalChannelAngle_scp128P_cos_vA_RA[i] = cos(vA_P * m_calRA);
            double vA_N = m_verticalChannelsAngle_SCP128N[i];
            m_verticalChannelAngle_scp128N_sin_vA_RA[i] = sin(vA_N * m_calRA);
            m_verticalChannelAngle_scp128N_cos_vA_RA[i] = cos(vA_N * m_calRA);
        }

        for (int i = 0; i < 3; i++) {
            m_skewing_sin_scope128[i] = sin(m_skewing_scope128_Angle[i] * m_calRA);
            m_skewing_cos_scope128[i] = cos(m_skewing_scope128_Angle[i] * m_calRA);
        }

        m_rotate_scp128P_sin = sin(m_scp128MoveAngleP * m_calRA);
        m_rotate_scp128P_cos = cos(m_scp128MoveAngleP * m_calRA);
        m_rotate_scp128N_sin = sin(m_scp128MoveAngleN * m_calRA);
        m_rotate_scp128N_cos = cos(m_scp128MoveAngleN * m_calRA);

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

    void DecodeDIFData_Scope128(const char *udpData) {
        _lidarTypeFromDIF = (unsigned int)(udpData[11]); // 1:Tensor48,2:Scope256,8:Scope128
        // ABC
        int hex8_mirrorA = (unsigned char)udpData[508 + 4 * 14 + 1]; //565，转镜 A 面俯仰角
        int hex8_mirrorB = (unsigned char)udpData[508 + 4 * 14 + 2]; // 转镜 B 面俯仰角
        int hex8_mirrorC = (unsigned char)udpData[508 + 4 * 14 + 3]; // 转镜 C 面俯仰角

        // 提取该字节的高4位（第4位到第7位）,将结果右移4位便于读取,β角标志位
        int scp128BetaAnglePS = (udpData[750] & 0xF0) >> 4;

        if (scp128BetaAnglePS != m_scp128BetaAnglePS) {
            for (int i = 0; i < 32; i++) {
            if (scp128BetaAnglePS == 1) {
                m_verticalChannelsAngle_SCP128P[i] =
                    m_verticalChannelsAngle_SCP128P_v2[i];
                m_verticalChannelsAngle_SCP128N[i] =
                    m_verticalChannelsAngle_SCP128N_v2[i];
            } else if (scp128BetaAnglePS == 0) {
                m_verticalChannelsAngle_SCP128P[i] =
                    m_verticalChannelsAngle_SCP128P_v1[i];
                m_verticalChannelsAngle_SCP128N[i] =
                    m_verticalChannelsAngle_SCP128N_v1[i];
            }
            double vA_P = m_verticalChannelsAngle_SCP128P[i];
            m_verticalChannelAngle_scp128P_sin_vA_RA[i] = sin(vA_P * m_calRA);
            m_verticalChannelAngle_scp128P_cos_vA_RA[i] = cos(vA_P * m_calRA);
            double vA_N = m_verticalChannelsAngle_SCP128N[i];
            m_verticalChannelAngle_scp128N_sin_vA_RA[i] = sin(vA_N * m_calRA);
            m_verticalChannelAngle_scp128N_cos_vA_RA[i] = cos(vA_N * m_calRA);
            }
            m_scp128BetaAnglePS = scp128BetaAnglePS;
        }
        float mirrorA = (hex8_mirrorA - 128) * 0.01;
        float mirrorB = (hex8_mirrorB - 128) * 0.01;
        float mirrorC = (hex8_mirrorC - 128) * 0.01;

        if (!IsEqualityFloat3(mirrorA + m_MirrorVerAngleOffset[0],
                                m_skewing_scope128_Angle[0])) {
            m_skewing_scope128_Angle[0] = mirrorA + m_MirrorVerAngleOffset[0];
            m_skewing_sin_scope128[0] = sin(m_skewing_scope128_Angle[0] * m_calRA);
            m_skewing_cos_scope128[0] = cos(m_skewing_scope128_Angle[0] * m_calRA);
        }

        if (!IsEqualityFloat3(mirrorB + m_MirrorVerAngleOffset[1],
                                m_skewing_scope128_Angle[1])) {
            m_skewing_scope128_Angle[1] = mirrorB + m_MirrorVerAngleOffset[1];
            m_skewing_sin_scope128[1] = sin(m_skewing_scope128_Angle[1] * m_calRA);
            m_skewing_cos_scope128[1] = cos(m_skewing_scope128_Angle[1] * m_calRA);
        }

        if (!IsEqualityFloat3(mirrorC + m_MirrorVerAngleOffset[2],
                                m_skewing_scope128_Angle[2])) {
            m_skewing_scope128_Angle[2] = mirrorC + m_MirrorVerAngleOffset[2];
            m_skewing_sin_scope128[2] = sin(m_skewing_scope128_Angle[2] * m_calRA);
            m_skewing_cos_scope128[2] = cos(m_skewing_scope128_Angle[2] * m_calRA);
        }

        for (int i = 0; i < 4; i++) {
            m_Scope128PosLaserOffset[3 - i] = udpData[508 + 0xE8 + i] / 6.0 - 3.0;
            m_Scope128NegLaserOffset[3 - i] = udpData[508 + 0xEC + i] / 6.0 - 3.0;
        }

        double scope128NegBetaAngleOffset =
            ((((unsigned char)udpData[508 + 0x7A] << 8) |
                (unsigned char)udpData[508 + 0x7B]) &
            0x01ff) *
                0.01 -
            2.56;

        double scope128PosBetaAngleOffset =
            (((((unsigned char)udpData[508 + 0x79] << 8) |
                (unsigned char)udpData[508 + 0x7A]) >>
                1) &
            0x01ff) *
                0.01 -
            2.56;

        if (scope128NegBetaAngleOffset != m_Scope128NegBetaAngleOffset) {
            for (int i = 0; i < 32; i++) {
            double vA_N =
                m_verticalChannelsAngle_SCP128N[i] + scope128NegBetaAngleOffset;
            m_verticalChannelAngle_scp128N_sin_vA_RA[i] = sin(vA_N * m_calRA);
            m_verticalChannelAngle_scp128N_cos_vA_RA[i] = cos(vA_N * m_calRA);
            }

            m_Scope128NegBetaAngleOffset = scope128NegBetaAngleOffset;
        }

        if (scope128PosBetaAngleOffset != m_Scope128PosBetaAngleOffset) {
            for (int i = 0; i < 32; i++) {
            double vA_N =
                m_verticalChannelsAngle_SCP128P[i] + scope128PosBetaAngleOffset;
            m_verticalChannelAngle_scp128P_sin_vA_RA[i] = sin(vA_N * m_calRA);
            m_verticalChannelAngle_scp128P_cos_vA_RA[i] = cos(vA_N * m_calRA);
            }

            m_Scope128PosBetaAngleOffset = scope128PosBetaAngleOffset;
        }

        m_negtive_back_startAngle =
            avos::driver::TwoHextoInt(udpData[508 + 0xB2], udpData[508 + 0xB3]) * 0.01 -
            1.0; // 由于pl计算误差，左右各扩充1°
        m_negtive_back_endAngle =
            avos::driver::TwoHextoInt(udpData[508 + 0xB6], udpData[508 + 0xB7]) * 0.01 + 1.0;
        m_positive_back_startAngle =
            avos::driver::TwoHextoInt(udpData[508 + 0xBA], udpData[508 + 0xBB]) * 0.01 - 1.0;
        m_positive_back_endAngle =
            avos::driver::TwoHextoInt(udpData[508 + 0xBE], udpData[508 + 0xBF]) * 0.01 + 1.0;

        PCF_type = (udpData[31] & 0xF0) >> 4;
    }

    void UseDecodeScope128(const char *udpData, avos::driver::PacketPointData &point_cloud, const avos::driver::LidarCaliConfig &decoder_config, uint64_t &first_pkg_ts, int &pub_logic, bool is_single) {
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
        if(avos::driver::TwoHextoInt(udpData[4], udpData[5]) != frame_id_)//new frame, frame_idx(byte 4/5)::[0~65535]
        {
            if(decoder_config.debug & 1)
            {
                std::cout << "\n/***********************************/\nreceive new frame, FrameID: " <<  avos::driver::TwoHextoInt(udpData[4], udpData[5]) << "\n";
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
                if((((frame_id_+1)%65536) != avos::driver::TwoHextoInt(udpData[4], udpData[5]))&&(-1!=frame_id_))
                {
                    std::cout << "lost frame, last:current -> " << frame_id_ << ":" <<  avos::driver::TwoHextoInt(udpData[4], udpData[5]) << "\n";
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
                is_new = false;
                udp_pkg_count_ = -1;
                udp_pkg_id_ = -1;
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
            for (int blocks_num = 0; blocks_num < 12; blocks_num++) {
                int offset_block = blocks_num * 100;

                // ptp
                int offsetMicrosecond =
                    avos::driver::TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
                double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
                unsigned int blockSecond =
                    (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
                unsigned int blockMicrosecond =
                    (totalMicrosecond >= 1000000)
                        ? (unsigned int)(totalMicrosecond - 1000000)
                        : (unsigned int)totalMicrosecond;

                // Positive or Negative 0:Neg 1:Pos
                unsigned short posOrNeg = udpData[35 + offset_block] & 0x01;

                // mirror
                unsigned short mirror = (udpData[35 + offset_block] >> 1) & 0x03;
                int blockID = (udpData[35 + offset_block] & 0xF0) >> 4;

                if (blockID != 0 && blockID != 1)
                continue;

                if (mirror != 0 && mirror != 1 && mirror != 2)
                continue;

                for (int seq = 0; seq < 16; seq++) {
                // 2Byte 36-37 解析点云水平角度φ
                double hexHorAngle = avos::driver::TwoHextoInt(udpData[36 + offset_block + seq * 6],
                                                udpData[37 + offset_block + seq * 6]);
                double horAngle = hexHorAngle * 0.01;
                horAngle += m_MirrorHorAngleOffset[mirror];

                // L1 2Byte 40-41
                double hexL1 = avos::driver::TwoHextoInt(udpData[38 + offset_block + seq * 6],
                                            udpData[39 + offset_block + seq * 6]);
                double L_1 = hexL1 * m_calSimpleFPGA;

                // intensity 0-255
                unsigned char hexIntensity1 = udpData[40 + offset_block + seq * 6];
                double intensity_1 = hexIntensity1;

                int confidence1 = udpData[41 + offset_block + seq * 6] & 0x01;

                // 通道计算
                int channel = -1;
                int signal_mirror_seq = blockID * 16 + seq + 1;

                if (posOrNeg) {
                    if (0 == mirror) {
                    channel = signal_mirror_seq * 2 - 1;
                    } else if (1 == mirror || 2 == mirror) {
                    channel = signal_mirror_seq * 2;
                    }
                } else {
                    if (0 == mirror) {
                    channel = 64 + signal_mirror_seq * 2 - 1;
                    } else if (2 == mirror || 1 == mirror) {
                    channel = 64 + signal_mirror_seq * 2;
                    }
                }

                // 转镜角度 三角函数计算 （θ：参照PDF定义）
                double theta = 0;

                if (1 == posOrNeg) // 左机芯 正机芯
                {
                    theta = horAngle / 2; // + 30;
                    if (channel >= 1 && channel <= 16) {
                    theta += (-1.25) + m_Scope128PosLaserOffset[0];
                    horAngle += 1.5;
                    } else if (channel >= 17 && channel <= 32) {
                    theta += (-1.25) + m_Scope128PosLaserOffset[1];
                    horAngle += 1.5;
                    } else if (channel >= 33 && channel <= 48) {
                    theta += (-2.75) + m_Scope128PosLaserOffset[2];
                    horAngle -= 1.5;
                    } else if (channel >= 49 && channel <= 64) {
                    theta += (-2.75) + m_Scope128PosLaserOffset[3];
                    horAngle -= 1.5;
                    }
                } else // 右机芯 负机芯
                {
                    theta = horAngle / 2 + 90; //+ 120;
                    if (channel >= 65 && channel <= 80) {
                    theta += (2.75) + m_Scope128NegLaserOffset[3];
                    horAngle += 1.5;
                    } else if (channel >= 81 && channel <= 96) {
                    theta += (2.75) + m_Scope128NegLaserOffset[2];
                    horAngle += 1.5;
                    } else if (channel >= 97 && channel <= 112) {
                    theta += (1.25) + m_Scope128NegLaserOffset[1];
                    horAngle -= 1.5;
                    } else if (channel >= 113 && channel <= 128) {
                    theta += (1.25) + m_Scope128NegLaserOffset[0];
                    horAngle -= 1.5;
                    }
                }

                double Lx = 0.0;
                double Ly = 0.0;
                double Lz = 0.0;

                double Nx = 0.0;
                double Ny = 0.0;
                double Nz = 0.0;

                // Positive
                if (1 == posOrNeg) {
                    // 计算各通道竖直角度（β）的正弦和余弦
                    double sin_beta =
                        m_verticalChannelAngle_scp128P_sin_vA_RA[signal_mirror_seq - 1];
                    double cos_beta =
                        m_verticalChannelAngle_scp128P_cos_vA_RA[signal_mirror_seq - 1];

                    // 计算出射光线与X轴夹角（γ）的正弦和余弦
                    double sin_gamma = m_rotate_scp128P_sin;
                    double cos_gamma = m_rotate_scp128P_cos;

                    // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
                    Lx = cos_beta * cos_gamma;
                    Ly = -cos_beta * sin_gamma;
                    Lz = sin_beta;

                    // 转镜法线转过的角度（θ）的正弦和余弦
                    double sin_theta = sin(theta * m_calRA);
                    double cos_theta = cos(theta * m_calRA);

                    Nx = m_skewing_cos_scope128[mirror] * cos_theta;
                    Ny = m_skewing_cos_scope128[mirror] * sin_theta;
                    Nz = m_skewing_sin_scope128[mirror];

                } else {
                    // 计算各通道竖直角度（β）的正弦和余弦
                    double sin_beta =
                        m_verticalChannelAngle_scp128N_sin_vA_RA[signal_mirror_seq - 1];
                    double cos_beta =
                        m_verticalChannelAngle_scp128N_cos_vA_RA[signal_mirror_seq - 1];

                    // 计算出射光线与X轴夹角（γ）的正弦和余弦
                    double sin_gamma = m_rotate_scp128N_sin;
                    double cos_gamma = m_rotate_scp128N_cos;

                    // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
                    Lx = cos_beta * cos_gamma;
                    Ly = -cos_beta * sin_gamma;
                    Lz = sin_beta;

                    double sin_theta = sin(theta * m_calRA);
                    double cos_theta = cos(theta * m_calRA);

                    Nx = m_skewing_cos_scope128[mirror] * cos_theta;
                    Ny = m_skewing_cos_scope128[mirror] * sin_theta;
                    Nz = m_skewing_sin_scope128[mirror];
                }

                double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

                // TWPoint basic_point;
                // basic_point.angle = horAngle;
                // basic_point.mirror = mirror;
                // basic_point.left_right = posOrNeg;
                // basic_point.channel = channel;

                // basic_point.back_point = IsBackPoint(basic_point);

                basic_point.timestamp = blockSecond + blockMicrosecond * 1e-6;
                basic_point.angle = horAngle;
                basic_point.ring = channel;

                if (_echoNum & avos::driver::EchoNum::Echo1) {
                    basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
                    basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
                    basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);

                    basic_point.intensity = intensity_1;
                    // basic_point.distance = L_1;
                    // basic_point.echo = 1;
                    // basic_point.confidence = confidence1;
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
                    // point_cloud.points.width++;
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
            for (int blocks_num = 0; blocks_num < 8; blocks_num++) {
                int offset_block = blocks_num * 164;

                // ptp
                int offsetMicrosecond =
                    avos::driver::TwoHextoInt(udpData[32 + offset_block], udpData[33 + offset_block]);
                double totalMicrosecond = frameMicrosecond + offsetMicrosecond * 0.1;
                unsigned int blockSecond =
                    (totalMicrosecond >= 1000000) ? (frameSecond + 1) : frameSecond;
                unsigned int blockMicrosecond =
                    (totalMicrosecond >= 1000000)
                        ? (unsigned int)(totalMicrosecond - 1000000)
                        : (unsigned int)totalMicrosecond;

                // Positive or Negative 0:Neg 1:Pos
                unsigned short posOrNeg = udpData[35 + offset_block] & 0x01;

                // mirror
                unsigned short mirror = (udpData[35 + offset_block] >> 1) & 0x03;
                int blockID = (udpData[35 + offset_block] & 0xF0) >> 4;

                if (blockID != 0 && blockID != 1)
                continue;

                if (mirror != 0 && mirror != 1 && mirror != 2)
                continue;

                for (int seq = 0; seq < 16; seq++) {
                // 2Byte 36-37 解析点云水平角度φ
                double hexHorAngle = avos::driver::TwoHextoInt(udpData[36 + offset_block + seq * 10],
                                                udpData[37 + offset_block + seq * 10]);
                double horAngle = hexHorAngle * 0.01;
                horAngle += m_MirrorHorAngleOffset[mirror];

                // L1 2Byte 40-41
                double hexL1 = avos::driver::TwoHextoInt(udpData[38 + offset_block + seq * 10],
                                            udpData[39 + offset_block + seq * 10]);
                double L_1 = hexL1 * m_calSimpleFPGA;

                // intensity 0-255
                unsigned char hexIntensity1 = udpData[40 + offset_block + seq * 10];
                double intensity_1 = hexIntensity1;

                int confidence1 = udpData[41 + offset_block + seq * 10] & 0x01;

                // L2 2Byte
                double hexL2 = avos::driver::TwoHextoInt(udpData[42 + offset_block + seq * 10],
                                            udpData[43 + offset_block + seq * 10]);
                double L_2 = hexL2 * m_calSimpleFPGA;

                // intensity 0-255
                unsigned char hexIntensity2 = udpData[44 + offset_block + seq * 10];
                double intensity_2 = hexIntensity2;

                int confidence2 = udpData[45 + offset_block + seq * 10] & 0x01;

                // 通道计算
                int channel = -1;
                int signal_mirror_seq = blockID * 16 + seq + 1;

                if (posOrNeg) {
                    if (0 == mirror) {
                    channel = signal_mirror_seq * 2 - 1;
                    } else if (1 == mirror || 2 == mirror) {
                    channel = signal_mirror_seq * 2;
                    }
                } else {
                    if (0 == mirror) {
                    channel = 64 + signal_mirror_seq * 2 - 1;
                    } else if (2 == mirror || 1 == mirror) {
                    channel = 64 + signal_mirror_seq * 2;
                    }
                }

                // 转镜角度 三角函数计算 （θ：参照PDF定义）
                double theta = 0;

                if (1 == posOrNeg) // 左机芯 正机芯
                {
                    theta = horAngle / 2; // + 30;
                    if (channel >= 1 && channel <= 16) {
                    theta += (-1.25) + m_Scope128PosLaserOffset[0];
                    horAngle += 1.5;
                    } else if (channel >= 17 && channel <= 32) {
                    theta += (-1.25) + m_Scope128PosLaserOffset[1];
                    horAngle += 1.5;
                    } else if (channel >= 33 && channel <= 48) {
                    theta += (-2.75) + m_Scope128PosLaserOffset[2];
                    horAngle -= 1.5;
                    } else if (channel >= 49 && channel <= 64) {
                    theta += (-2.75) + m_Scope128PosLaserOffset[3];
                    horAngle -= 1.5;
                    }
                } else // 右机芯 负机芯
                {
                    theta = horAngle / 2 + 90; //+ 120;
                    if (channel >= 65 && channel <= 80) {
                    theta += (2.75) + m_Scope128NegLaserOffset[3];
                    horAngle += 1.5;
                    } else if (channel >= 81 && channel <= 96) {
                    theta += (2.75) + m_Scope128NegLaserOffset[2];
                    horAngle += 1.5;
                    } else if (channel >= 97 && channel <= 112) {
                    theta += (1.25) + m_Scope128NegLaserOffset[1];
                    horAngle -= 1.5;
                    } else if (channel >= 113 && channel <= 128) {
                    theta += (1.25) + m_Scope128NegLaserOffset[0];
                    horAngle -= 1.5;
                    }
                }

                double Lx = 0.0;
                double Ly = 0.0;
                double Lz = 0.0;

                double Nx = 0.0;
                double Ny = 0.0;
                double Nz = 0.0;

                // Positive
                if (1 == posOrNeg) {
                    // 计算各通道竖直角度（β）的正弦和余弦
                    double sin_beta =
                        m_verticalChannelAngle_scp128P_sin_vA_RA[signal_mirror_seq - 1];
                    double cos_beta =
                        m_verticalChannelAngle_scp128P_cos_vA_RA[signal_mirror_seq - 1];

                    // 计算出射光线与X轴夹角（γ）的正弦和余弦
                    double sin_gamma = m_rotate_scp128P_sin;
                    double cos_gamma = m_rotate_scp128P_cos;

                    // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
                    Lx = cos_beta * cos_gamma;
                    Ly = -cos_beta * sin_gamma;
                    Lz = sin_beta;

                    // 转镜法线转过的角度（θ）的正弦和余弦
                    double sin_theta = sin(theta * m_calRA);
                    double cos_theta = cos(theta * m_calRA);

                    Nx = m_skewing_cos_scope128[mirror] * cos_theta;
                    Ny = m_skewing_cos_scope128[mirror] * sin_theta;
                    Nz = m_skewing_sin_scope128[mirror];

                } else {
                    // 计算各通道竖直角度（β）的正弦和余弦
                    double sin_beta =
                        m_verticalChannelAngle_scp128N_sin_vA_RA[signal_mirror_seq - 1];
                    double cos_beta =
                        m_verticalChannelAngle_scp128N_cos_vA_RA[signal_mirror_seq - 1];

                    // 计算出射光线与X轴夹角（γ）的正弦和余弦
                    double sin_gamma = m_rotate_scp128N_sin;
                    double cos_gamma = m_rotate_scp128N_cos;

                    // 机芯发射光线单位向量li = (cosβcosγ,-cosβsinγ,sinβ)
                    Lx = cos_beta * cos_gamma;
                    Ly = -cos_beta * sin_gamma;
                    Lz = sin_beta;

                    double sin_theta = sin(theta * m_calRA);
                    double cos_theta = cos(theta * m_calRA);

                    Nx = m_skewing_cos_scope128[mirror] * cos_theta;
                    Ny = m_skewing_cos_scope128[mirror] * sin_theta;
                    Nz = m_skewing_sin_scope128[mirror];
                }

                double dot_product = Lx * Nx + Ly * Ny + Lz * Nz;

                // TWPoint basic_point;
                // basic_point.angle = horAngle;
                // basic_point.mirror = mirror;
                // basic_point.left_right = posOrNeg;
                // basic_point.channel = channel;
                // basic_point.t_sec = blockSecond;
                // basic_point.t_usec = blockMicrosecond;

                basic_point.timestamp = blockSecond + blockMicrosecond * 1e-6;
                basic_point.angle = horAngle;
                basic_point.ring = channel;

                if (_echoNum & avos::driver::EchoNum::Echo1) {
                    basic_point.x = L_1 * (Lx - 2 * dot_product * Nx);
                    basic_point.y = L_1 * (Ly - 2 * dot_product * Ny);
                    basic_point.z = L_1 * (Lz - 2 * dot_product * Nz);
                    basic_point.intensity = intensity_1;
                    // basic_point.distance = L_1;
                    // basic_point.pulse = intensity_1 * 0.128;
                    // basic_point.echo = 1;
                    // basic_point.confidence = confidence1;

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
                    // point_cloud.points.width++;
                    if(decoder_config.debug & 8)
                    {
                        printf("echo1 x=%.3f, y=%.3f, z=%.3f\n", basic_point.x, basic_point.y, basic_point.z);
                    }
                }

                if (_echoNum & avos::driver::EchoNum::Echo2) {
                    basic_point.x = L_2 * (Lx - 2 * dot_product * Nx);
                    basic_point.y = L_2 * (Ly - 2 * dot_product * Ny);
                    basic_point.z = L_2 * (Lz - 2 * dot_product * Nz);

                    basic_point.intensity = intensity_2;
                    // basic_point.distance = L_2;
                    // basic_point.pulse = intensity_2 * 0.128;
                    // basic_point.echo = 2;
                    // basic_point.confidence = confidence2;

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
                    // point_cloud.points.width++;
                    if(decoder_config.debug & 8)
                    {
                        printf("echo2 x=%.3f, y=%.3f, z=%.3f\n", basic_point.x, basic_point.y, basic_point.z);
                    }
                }
                }
            }
        }
    }

private:
    void InitMapTable() {
        return;
    }

private:
    VPoint           basic_point;
    double                     sin_table[CULCULATE_TABLE_SIZE];
    double                     cos_table[CULCULATE_TABLE_SIZE];
    Eigen::Matrix<float, 3, 1> point_sensor;
    Eigen::Matrix<float, 3, 1> point_vcs;
    std::map<std::string, std::uint32_t>  ec_map;

    int32_t frame_id_ = -1;
	int32_t udp_pkg_id_ = -1;
	int32_t udp_pkg_count_ = -1;
	bool	is_new = false;
    // avos::driver::DecoderConfig decoder_config;
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

    double m_verticalChannelsAngle_SCP128P[32] = {};
    double m_verticalChannelsAngle_SCP128N[32] = {};
    int m_scp128BetaAnglePS = 1; // PS标志物为1则使用新的β角，为0则使用老的β角

    // Scope128，v1是老版β角，v2是75mm雷达更改后的β角
    const double m_verticalChannelsAngle_SCP128P_v1[32] = {
        -34.43, -33.35, -32.27, -31.19, -30.11, -29.03, -27.95, -26.85,
        -25.75, -24.64, -23.52, -22.39, -21.25, -20.1,  -18.95, -17.79,
        -16.93, -15.85, -14.77, -13.69, -12.61, -11.53, -10.45, -9.35,
        -8.25,  -7.14,  -6.02,  -4.89,  -3.775, -2.6,   -1.45,  -0.29};

    const double m_verticalChannelsAngle_SCP128N_v1[32] = {
        0.87,  2.02,  3.17,  4.32,  5.45,  6.58,  7.7,   8.8,
        9.9,   10.99, 12.07, 13.15, 14.23, 15.31, 16.39, 17.46,
        18.37, 19.52, 20.67, 21.82, 22.95, 24.08, 25.2,  26.3,
        27.4,  28.49, 29.57, 30.65, 31.73, 32.81, 33.89, 34.96};

    const double m_verticalChannelsAngle_SCP128P_v2[32] = {
        -34.73, -33.65, -32.57, -31.49, -30.41, -29.33, -28.25, -27.15,
        -26.05, -24.94, -23.82, -22.69, -21.55, -20.40, -19.25, -18.09,
        -17.23, -16.15, -15.07, -13.99, -12.91, -11.83, -10.75, -9.65,
        -8.55,  -7.44,  -6.32,  -5.19,  -4.075, -2.90,  -1.75,  -0.59};

    const double m_verticalChannelsAngle_SCP128N_v2[32] = {
        0.590,  1.750,  2.900,  4.075,  5.190,  6.320,  7.440,  8.550,
        9.650,  10.750, 11.830, 12.910, 13.990, 15.070, 16.150, 17.230,
        18.090, 19.250, 20.400, 21.550, 22.690, 23.820, 24.940, 26.050,
        27.150, 28.250, 29.330, 30.410, 31.490, 32.570, 33.650, 34.730};

    double m_verticalChannelAngle_scp128P_sin_vA_RA[32] = {0.f}; // beta
    double m_verticalChannelAngle_scp128P_cos_vA_RA[32] = {0.f};
    double m_verticalChannelAngle_scp128N_sin_vA_RA[32] = {0.f};
    double m_verticalChannelAngle_scp128N_cos_vA_RA[32] = {0.f};

    double m_skewing_scope128_Angle[3] = {0.0, 0.4, 0.4};
    double m_skewing_sin_scope128[3];
    double m_skewing_cos_scope128[3];

    double m_scp128MoveAngleP = 184.0; // gamma
    double m_scp128MoveAngleN = 356.0;
    double m_rotate_scp128P_sin;
    double m_rotate_scp128P_cos;
    double m_rotate_scp128N_sin;
    double m_rotate_scp128N_cos;

    // 俯仰与水平修正角度
    double m_MirrorVerAngleOffset[3] = {0.0};
    double m_MirrorHorAngleOffset[3] = {0.0};

    double m_Scope128PosLaserOffset[4] = {0, 0, 0, 0};
    double m_Scope128NegLaserOffset[4] = {0, 0, 0, 0};

    // 机芯垂直与水平角度
    double m_LeftRightMechCoreVerAngleOffset[2] = {0.0};
    double m_LeftRightMechCoreHorAngleOffset[2] = {0.0};

    double m_Scope128PosBetaAngleOffset = 0.0;
    double m_Scope128NegBetaAngleOffset = 0.0;

    double m_positive_back_startAngle = 0; // 正机芯背面点起始角度
    double m_positive_back_endAngle = 0;   // 正机芯背面点结束角度
    double m_negtive_back_startAngle = 0; // 负机芯背面点起始角度
    double m_negtive_back_endAngle = 0;   // 负机芯背面点结束角度

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
