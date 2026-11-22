#ifndef __COMMON_PROTOCOL_H__
#define __COMMON_PROTOCOL_H__
#include <sys/time.h>
#include <time.h>

#include <Eigen/Dense>
#include <atomic>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace avos {
    namespace driver {
        enum EchoNum { EchoNone = 0, Echo1 = 0x01, Echo2 = 0x02 };

        typedef struct DeviceInfoFrame
        {
            uint32_t header;
            uint32_t deviceType;
            uint32_t deviceNumber;
            std::string psVersion;
            std::string plVersion;
            uint8_t workMode;
            uint8_t workStatus;
        } DeviceInfoFrame;

        inline std::string Dec2Hex(int data, int width = 2) {
            std::stringstream ioss;
            std::string       s_temp;
            ioss << std::hex << data;
            ioss >> s_temp;

            if (width > s_temp.size()) {
                std::string s_0(width - s_temp.size(), '0'); //位数不够则补0
                s_temp = s_0 + s_temp;                       //合并
            }

            std::string s = s_temp.substr(s_temp.length() - width, s_temp.length()); //取右width位
            return s;
        }

#define MAX_ANGLE 36000
#define REVERSE_SHORT(value) (unsigned short)(((value)&0x00ff) << 8 | ((value)&0xff00) >> 8)
#define REVERSE_INT(value)                                                                              \
    (unsigned int)(((value)&0x000000ff) << 24 | ((value)&0x0000ff00) << 8 | ((value)&0x00ff0000) >> 8 | \
                   ((value)&0xff000000) >> 24)

        inline float FourHexToFloat(unsigned char high, unsigned char highmiddle,
                            unsigned char middle, unsigned char low) {
            unsigned char sz_float[4] = {low, middle, highmiddle, high};
            float addr = 0;
            memcpy(&addr, sz_float, 4);
            return addr;
        }

        inline int TwoHextoInt(unsigned char high, unsigned char low) {
            int addr = low & 0xFF;
            addr |= ((high << 8) & 0XFF00);
            return addr;
        }

        // 处理有符号两字节数据的函数
        inline int SignedTwoHextoInt(unsigned char high, unsigned char low) {
            // 先组合成16位无符号整数
            int addr = low & 0xFF;
            addr |= ((high << 8) & 0xFF00);

            // 检查最高位(第15位)是否为1，如果是则进行符号扩展
            if (addr & 0x8000) {
                // 符号位为1，说明是负数，需要符号扩展到32位int
                addr |= 0xFFFF0000;
            }

            return addr;
        }

        inline unsigned int FourHexToInt(unsigned char high, unsigned char highmiddle,
                                        unsigned char middle, unsigned char low) {
            unsigned int addr = low & 0xFF;
            addr |= ((middle << 8) & 0xFF00);
            addr |= ((highmiddle << 16) & 0xFF0000);
            addr |= ((high << 24) & 0xFF000000);
            return addr;
        }

        inline unsigned short OneHexHalfHextoInt(unsigned char high,
                                                unsigned char halflow) {
            unsigned short addr = halflow & 0xF0;
            addr = addr >> 4;
            addr |= ((high << 4) & 0x0FF0);
            return addr;
        }

#pragma pack(push, 1)
        typedef struct DeviceTimeUTC {
            char         sec[6];
            uint32_t     us;

            std::string GetDeviceTime() {
                double     timestamp = GetSecondValue();
                time_t     tt = (time_t)timestamp;
                struct tm *ttime;
                char       now[64];
                ttime = localtime(&tt);
                strftime(now, 64, "%Y-%m-%d %H:%M:%S", ttime);
                std::string str(now);
                std::string res;
                res = str + "---" + std::to_string(timestamp - int(timestamp));
                return res;
            }

            double GetSecondValue() {
                union u_ts {
                    uint8_t  data[8];
                    uint64_t ts;
                } timestamp;
                timestamp.data[7] = 0;
                timestamp.data[6] = 0;
                timestamp.data[5] = sec[0];
                timestamp.data[4] = sec[1];
                timestamp.data[3] = sec[2];
                timestamp.data[2] = sec[3];
                timestamp.data[1] = sec[4];
                timestamp.data[0] = sec[5];

                double res = (double)timestamp.ts + (double)((double)(REVERSE_INT(us)) / 1000000.0);

                return res;
            }

            uint16_t size() {
                return sizeof(DeviceTimeUTC);
            }

            void Print(const std::string &prefix = "") {
                std::cout << prefix << GetDeviceTime();
                std::cout << prefix << REVERSE_INT(us);
            }
        } DeviceTimeUTC;

        typedef struct EthProtocol {
            unsigned char  lidar_ip[4];
            unsigned char  dest_ip[4];
            unsigned char  lidar_mac[6];
            unsigned short msop_native_out_port;
            unsigned short msop_pc_in_port;
            unsigned short difop_native_out_port;
            unsigned short difop_pc_in_port;

            void Print(const std::string &prefix = "") {
                std::cout << prefix << "lidar_ip:" << (int)lidar_ip[0] << "." << (int)lidar_ip[1] << "." << (int)lidar_ip[2]
                      << "." << (int)lidar_ip[3];
                std::cout << prefix << "dest_ip:" << (int)dest_ip[0] << "." << (int)dest_ip[1] << "." << (int)dest_ip[2]
                      << "." << (int)dest_ip[3];
                std::cout << prefix << "lidar_mac:" << (int)lidar_mac[0] << "." << (int)lidar_mac[1] << "."
                      << (int)lidar_mac[2] << "." << (int)lidar_mac[3] << "." << (int)lidar_mac[4] << "."
                      << (int)lidar_mac[5];
                std::cout << prefix << "msop_native_out_port:" << REVERSE_SHORT(msop_native_out_port);
                std::cout << prefix << "msop_pc_in_port:" << REVERSE_SHORT(msop_pc_in_port);
                std::cout << prefix << "difop_native_out_port:" << REVERSE_SHORT(difop_native_out_port);
                std::cout << prefix << "difop_pc_in_port:" << REVERSE_SHORT(difop_pc_in_port);
            }
        } EthProtocol;

#pragma pack(pop)

    } // namespace driver
} // namespace avos
#endif
