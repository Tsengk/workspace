#ifndef GKD_CONFIG_H1
#define GKD_CONFIG_H1

#include "common.hpp"
using namespace std;
struct zy{
    string zb_id;
    string name;
};
struct GKDCommonCfg
{

    string publish_topic_101 = "ZK/101"; //指控对2-1任务规划请求
    string publish_topic_102   = "ZK/102"; //指控对操控的直播拉流请求
    string publish_topic_zzrwjh_103   = "ZK/103"; //指控对操控作战任务计划
    string publicsh_topic_zzptjh_104 = "ZK/104" ; // 指控对操控 作战行动计划
    string subscribe_topic_rwgh_901 = "GH/901" ; // 任务规划对指控 作战任务规划
    string subscribe_topic_bltj_905 = "GH/905" ; // 任务规划对指控  兵力推荐集请求
    string subscribe_topic_dzmbqb_301 = "QB/301" ; // 情报系统对 指控 电子目标情报
    string subscribe_topic_dzmbxsb_302 = "QB/302" ; // 情报系统对 指控 电子目标消失报
    string subscribe_topic_zzptjzbzt_303 = "QB/303" ; // 情报系统对 指控 作战平台及装备状态
    string subscribe_topic_rwzxzt_501 = "CK/501" ; // 情报系统对 指控 作战平台及装备状态
    string subscribe_topic_rwzxzt_502 = "CK/502" ; // 操控系统返回任务确认信息

    string subscribe_topic_dt_801 ="SZDT/801"; //导调作战目标
    string subscribe_topic_dt_802 ="SZDT/802"; //下发导调
    string subscribe_topic_dt_803 ="SZDT/803"; //寻飞弹

    string imsi_list="460110230819098,460001234567891,460001234567892";
    string zg_target_list="460075315188985,460075315188986,460075315188987";

    // 聊天系统配置
    string chat_server_ip = "27.11.11.30";
    string chat_server_port = "18666";
    string chat_user_name = "ZK";
    string chat_user_mobile = "13834129017";
    string chat_user_avatar = "";
    string rtmpserver_ip  ="127.0.0.1";
    std::vector<std::string> ys_video_streams; // 演示任务固定推送的视频流地址列表
    std::map<std::string,std::vector<zy>>  zyc; //资源池缓存


    string subscribe_topic_push = "situation/push"; // 情报系统对 指控 情报推送
    string subscribe_topic_prediction = "situation/prediction"; // 情报系统对 指控 预测
    string user_id = "field_operator"; // 用户ID，区分不同用户

    string dc_operator_params = "46001"; //电磁手机探测任务配置项 移动46000，电信46011，联调：46001

    QMap<QString, int> deviceVelocityMap; // 设备默认速度映射
    QMap<QString, int> deviceTaskVelocityMap; // 设备+任务类型速度映射
    GKDCommonCfg() {}



    // 从ini文件加载配置
    bool loadFromIni(const QString& filePath = QString("%1/gkd_config.ini").arg(QCoreApplication::applicationDirPath())) {
        QSettings settings(filePath, QSettings::IniFormat);

        if (!settings.contains("GKD_MQTT/publish_topic_101")) {
            qDebug() << "配置文件不存在或格式错误:" << filePath;

            // 如果文件存在，先删除
            QFile file(filePath);
            if (file.exists()) {
                file.remove();
            }

            // 手动创建并写入配置文件
            QFile newFile(filePath);
            if (!newFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                qDebug() << "无法创建配置文件:" << filePath;
                return false;
            }

            QTextStream out(&newFile);
            out.setCodec("UTF-8");  // 设置UTF-8编码

            // 写入配置内容
            out << "[GKD_MQTT]\n"
                << QString("publish_topic_101=%1\n").arg(QString::fromStdString( publish_topic_101))
                << QString("publish_topic_102=%1\n").arg(QString::fromStdString(publish_topic_102))
                << QString("publish_topic_zzrwjh_103=%1\n").arg(QString::fromStdString(publish_topic_zzrwjh_103))
                << QString("publicsh_topic_zzptjh_104=%1\n").arg(QString::fromStdString(publicsh_topic_zzptjh_104))
                << QString("subscribe_topic_rwgh_901=%1\n").arg(QString::fromStdString(subscribe_topic_rwgh_901))
                << QString("subscribe_topic_dzmbqb_301=%1\n").arg(QString::fromStdString(subscribe_topic_dzmbqb_301))
                << QString("subscribe_topic_dzmbxsb_302=%1\n").arg(QString::fromStdString(subscribe_topic_dzmbxsb_302))
                << QString("subscribe_topic_zzptjzbzt_303=%1\n").arg(QString::fromStdString(subscribe_topic_zzptjzbzt_303))
                << QString("subscribe_topic_rwzxzt_501=%1\n").arg(QString::fromStdString(subscribe_topic_rwzxzt_501))
                << QString("subscribe_topic_dt_801=%1\n").arg(QString::fromStdString(subscribe_topic_dt_801))
                << QString("subscribe_topic_dt_802=%1\n").arg(QString::fromStdString(subscribe_topic_dt_802))
                << QString("subscribe_topic_dt_803=%1\n").arg(QString::fromStdString(subscribe_topic_dt_803))
                << QString("subscribe_topic_rwzxzt_502=%1\n").arg(QString::fromStdString(subscribe_topic_rwzxzt_502))
                << QString("imsi_list=%1\n").arg(QString::fromStdString(imsi_list))
                << QString("zg_target_list=%1\n").arg(QString::fromStdString(zg_target_list))
                << "\n[GKD_VIDEO]\n"
                << QString("rtmpserver_ip =%1\n").arg(QString::fromStdString(rtmpserver_ip))
                << QString("video_stream_1=rtmp://172.16.26.48:1935/live/zs01\n")
                << QString("video_stream_2=rtmp://172.16.26.48:1935/live/zs02\n")
                << QString("video_stream_3=rtmp://172.16.26.48:1935/live/zs03\n")
                << QString("video_stream_4=rtmp://172.16.26.48:1935/live/wz01\n")
                << "\n[CHAT_CONFIG]\n"
                << QString("chat_server_ip=%1\n").arg(QString::fromStdString(chat_server_ip))
                << QString("chat_server_port=%1\n").arg(QString::fromStdString(chat_server_port))
                << QString("chat_user_name=%1\n").arg(QString::fromStdString(chat_user_name))
                << QString("chat_user_mobile=%1\n").arg(QString::fromStdString(chat_user_mobile))
                << QString("chat_user_avatar=%1\n").arg(QString::fromStdString(chat_user_avatar))
                << "\n[USER_CONFIG]\n"
                << QString("subscribe_topic_push=%1\n").arg(QString::fromStdString(subscribe_topic_push))
                << QString("subscribe_topic_prediction=%1\n").arg(QString::fromStdString(subscribe_topic_prediction))
                << QString("user_id=%1\n").arg(QString::fromStdString(user_id))
                << "\n[TASK_CONFIG]\n"
                << QString("dc_operator_params=%1\n").arg(QString::fromStdString(dc_operator_params))
                << "\n[JSBY_Performance]\n"
                << QString::fromUtf8("挂载弹药=80_50_4_4\n")
                << QString::fromUtf8("油量=80_50_4_4\n")
                << QString::fromUtf8("速度=80_50_4_4\n")
                << QString::fromUtf8("升限=80_50_4_4\n")
                << QString::fromUtf8("加速度=80_50_4_4\n")
                << QString::fromUtf8("打击半径=80_50_4_4\n")
                << QString::fromUtf8("目标感知能力=80_50_4_4\n")
                << QString::fromUtf8("隐身能力=80_50_4_4\n")
                << "\n[ZYC]\n"
                << QString("1002_1=%1\n").arg(QString::fromStdString("dy99"))
                << QString("1002_2=%1\n").arg(QString::fromStdString("dy100"))
                << QString("1002_3=%1\n").arg(QString::fromStdString("dy101"));

            newFile.close();
            qDebug() << "已创建新的配置文件并写入默认值:" << filePath;

            // 重新加载设置
            settings.sync();
        }

        //GKD_MQTT topic配置
        publish_topic_101 = settings.value("GKD_MQTT/publish_topic_101", publish_topic_101.c_str()).toString().toStdString();
        publish_topic_102 = settings.value("GKD_MQTT/publish_topic_102", publish_topic_102.c_str()).toString().toStdString();
        publish_topic_zzrwjh_103 = settings.value("GKD_MQTT/publish_topic_zzrwjh_103", publish_topic_zzrwjh_103.c_str()).toString().toStdString();
        publicsh_topic_zzptjh_104 = settings.value("GKD_MQTT/publicsh_topic_zzptjh_104", publicsh_topic_zzptjh_104.c_str()).toString().toStdString();
        subscribe_topic_rwgh_901 = settings.value("GKD_MQTT/subscribe_topic_rwgh_901", subscribe_topic_rwgh_901.c_str()).toString().toStdString();
        subscribe_topic_dzmbqb_301 = settings.value("GKD_MQTT/subscribe_topic_dzmbqb_301", subscribe_topic_dzmbqb_301.c_str()).toString().toStdString();
        subscribe_topic_dzmbxsb_302 = settings.value("GKD_MQTT/subscribe_topic_dzmbxsb_302", subscribe_topic_dzmbxsb_302.c_str()).toString().toStdString();
        subscribe_topic_zzptjzbzt_303 = settings.value("GKD_MQTT/subscribe_topic_zzptjzbzt_303", subscribe_topic_zzptjzbzt_303.c_str()).toString().toStdString();
        subscribe_topic_rwzxzt_501 = settings.value("GKD_MQTT/subscribe_topic_rwzxzt_501", subscribe_topic_rwzxzt_501.c_str()).toString().toStdString();
        subscribe_topic_rwzxzt_502 = settings.value("GKD_MQTT/subscribe_topic_rwzxzt_502", subscribe_topic_rwzxzt_502.c_str()).toString().toStdString();
        subscribe_topic_dt_801 = settings.value("GKD_MQTT/subscribe_topic_dt_801", subscribe_topic_dt_801.c_str()).toString().toStdString();
        subscribe_topic_dt_802 = settings.value("GKD_MQTT/subscribe_topic_dt_802", subscribe_topic_dt_802.c_str()).toString().toStdString();
        subscribe_topic_dt_803 = settings.value("GKD_MQTT/subscribe_topic_dt_803", subscribe_topic_dt_803.c_str()).toString().toStdString();
        imsi_list = settings.value("GKD_MQTT/imsi_list", imsi_list.c_str()).toString().toStdString();
        zg_target_list = settings.value("GKD_MQTT/zg_target_list", zg_target_list.c_str()).toString().toStdString();

        // 聊天系统配置
        chat_server_ip = settings.value("CHAT_CONFIG/chat_server_ip", chat_server_ip.c_str()).toString().toStdString();
        chat_server_port = settings.value("CHAT_CONFIG/chat_server_port", chat_server_port.c_str()).toString().toStdString();
        chat_user_name = settings.value("CHAT_CONFIG/chat_user_name", chat_user_name.c_str()).toString().toStdString();
        chat_user_mobile = settings.value("CHAT_CONFIG/chat_user_mobile", chat_user_mobile.c_str()).toString().toStdString();
        chat_user_avatar = settings.value("CHAT_CONFIG/chat_user_avatar", chat_user_avatar.c_str()).toString().toStdString();
        // 用户配置
        subscribe_topic_push = settings.value("USER_CONFIG/subscribe_topic_push", subscribe_topic_push.c_str()).toString().toStdString();
        subscribe_topic_prediction = settings.value("USER_CONFIG/subscribe_topic_prediction", subscribe_topic_prediction.c_str()).toString().toStdString();
        user_id = settings.value("USER_CONFIG/user_id", user_id.c_str()).toString().toStdString();
        //任务配置
        dc_operator_params = settings.value("TASK_CONFIG/dc_operator_params", user_id.c_str()).toString().toInt();
        //视频流地址
        rtmpserver_ip  = settings.value("GKD_VIDEO/rtmpserver_ip", rtmpserver_ip.c_str()).toString().toStdString();
        // 演示任务固定推送的视频流地址列表：GKD_VIDEO 段下以 video_stream_ 开头的键，一个键一路流，个数动态
        ys_video_streams.clear();
        settings.beginGroup("GKD_VIDEO");
        QStringList vkeys = settings.childKeys();
        for (const QString& key : vkeys) {
            if(!key.startsWith("video_stream_")) continue;
            QString addr = settings.value(key).toString().trimmed();
            if(!addr.isEmpty()) ys_video_streams.push_back(addr.toStdString());
        }
        settings.endGroup();
        settings.beginGroup("ZYC");
        QStringList keys = settings.childKeys();
        for (const QString &key : keys) {
            int index = key.indexOf("_");
            if(index == -1) continue;
            QVariant value = settings.value(key);
            std::string zc_type = key.left(index).toStdString();
            if(zyc.find(zc_type) != zyc.end()){
                zy zy1;
                zy1.zb_id =value.toString().toStdString();
                zyc[zc_type].push_back(zy1);
            }else{
                // std::vector<zy> zb_list;
                // zb_list.push_back();
                zy  z{value.toString().toStdString(),""};
                zyc.insert(std::make_pair(zc_type, std::vector<zy>{z}));

            }

            qDebug() << "Key:" << key << ", Value:" << value.toString();
        }
        // 加载设备速度映射配置
        loadDeviceVelocityConfig();
        
        
        qDebug() << "配置文件加载成功:" << filePath<<",target_file:"<<filePath;
        return true;
    }

    // 加载设备速度映射配置
    void loadDeviceVelocityConfig(const QString& filePath = QString("%1/device_velocity_config.ini").arg(QCoreApplication::applicationDirPath())) {
        deviceVelocityMap.clear();
        deviceTaskVelocityMap.clear();

        QSettings velocitySettings(filePath, QSettings::IniFormat);

        // 检查配置文件是否存在，如果不存在则创建默认配置
        if (!QFile::exists(filePath)) {
            qDebug() << "设备速度配置文件不存在，创建默认配置:" << filePath;

            QFile defaultFile(filePath);
            if (defaultFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
                QTextStream out(&defaultFile);
                out.setCodec("UTF-8");
                out << "[DeviceTaskVelocity]\n"
                    << "# 设备+任务类型速度映射配置\n"
                    << "# 格式: 设备编号_任务类型=速度值\n\n"
                    << "# dy01设备的不同任务类型速度\n"
                    << "dy01_220=10\n"
                    << "dy01_221=10\n"
                    << "dy01_222=10\n\n"
                    << "# dy02设备的不同任务类型速度\n"
                    << "dy02_15=10\n\n"
                    << "# dy03设备的不同任务类型速度\n"
                    << "dy03_12=10\n"
                    << "dy03_18=10\n"
                    << "dy03_19=10\n"
                    << "dy03_120=10\n\n"
                    << "# dy04设备的不同任务类型速度\n"
                    << "dy04_18=10\n"
                    << "dy04_121=10\n\n"
                    << "# dy05设备\n"
                    << "dy05_220=10\n"
                    << "dy05_221=10\n"
                    << "dy05_222=10\n\n"
                    << "# dy06设备\n"
                    << "dy06_11=10\n\n"
                    << "# dy07设备\n"
                    << "dy07_13=10\n\n"
                    << "# dy08设备\n"
                    << "dy08_14=10\n"
                    << "dy08_19=10\n"
                    << "dy08_45=10\n\n"
                    << "# dy09设备\n"
                    << "dy09_14=10\n\n"
                    << "# dy11设备\n"
                    << "dy11_160=10\n"
                    << "dy11_161=10\n\n"
                    << "# dy12设备\n"
                    << "dy12_162=10\n\n"
                    << "# dy13设备\n"
                    << "dy13_41=10\n"
                    << "dy13_42=10\n"
                    << "dy13_43=10\n"
                    << "dy13_44=10\n\n"
                    << "# dy14设备\n"
                    << "dy14_41=10\n"
                    << "dy14_42=10\n"
                    << "dy14_44=10\n\n"
                    << "# dy17设备\n"
                    << "dy17_20=10\n\n"
                    << "# dy21设备\n"
                    << "dy21_20=10\n\n"
                    << "# dy25设备\n"
                    << "dy25_20=10\n\n"
                    << "[DefaultVelocity]\n"
                    << "# 默认设备速度（当找不到具体任务类型组合时使用）\n"
                    << "dy01=10\n"
                    << "dy02=10\n"
                    << "dy03=10\n"
                    << "dy04=10\n"
                    << "dy05=10\n"
                    << "dy06=10\n"
                    << "dy07=10\n"
                    << "dy08=10\n"
                    << "dy09=10\n"
                    << "dy11=10\n"
                    << "dy12=10\n"
                    << "dy13=10\n"
                    << "dy14=10\n"
                    << "dy17=10\n"
                    << "dy21=10\n"
                    << "dy25=10\n";
                defaultFile.close();
            }

            // 重新加载设置
            velocitySettings.sync();
        }

        // 读取设备+任务类型速度配置
        velocitySettings.beginGroup("DeviceTaskVelocity");
        QStringList taskKeys = velocitySettings.allKeys();
        foreach (const QString &key, taskKeys) {
            bool ok;
            int velocity = velocitySettings.value(key).toInt(&ok);
            if (ok) {
                deviceTaskVelocityMap[key] = velocity;
                qDebug() << "加载设备任务速度配置:" << key << "=" << velocity;
            }
        }
        velocitySettings.endGroup();

        // 读取默认设备速度配置
        velocitySettings.beginGroup("DefaultVelocity");
        QStringList defaultKeys = velocitySettings.allKeys();
        foreach (const QString &key, defaultKeys) {
            bool ok;
            int velocity = velocitySettings.value(key).toInt(&ok);
            if (ok) {
                deviceVelocityMap[key] = velocity;
                qDebug() << "加载设备默认速度配置:" << key << "=" << velocity;
            }
        }
        velocitySettings.endGroup();

        qDebug() << "设备速度配置加载完成，设备+任务类型:" << deviceTaskVelocityMap.size()
                 << "个，默认设备:" << deviceVelocityMap.size() << "个";
    }

    // 获取设备速度（默认方法，向后兼容）
    int getDeviceVelocity(const QString& deviceId) const {
        return deviceVelocityMap.value(deviceId, 0); // 如果找不到则返回0
    }

    // 获取设备+任务类型的速度（优先查找精确匹配）
    int getDeviceTaskVelocity(const QString& deviceId, int taskType) const {
        // 安全检查：确保对象未被销毁
        if (this == nullptr) {
            qDebug() << "错误：GKDCommonCfg对象为空指针";
            return 0;
        }

        // 首先尝试查找设备+任务类型的精确匹配
        QString taskKey = QString("%1_%2").arg(deviceId).arg(taskType);
        if (deviceTaskVelocityMap.contains(taskKey)) {
            int velocity = deviceTaskVelocityMap.value(taskKey);
            qDebug() << QString("找到精确匹配: %1 -> 速度 %2").arg(taskKey).arg(velocity);
            return velocity;
        }

        // 如果没有找到精确匹配，使用设备的默认速度
        if (deviceVelocityMap.contains(deviceId)) {
            int velocity = deviceVelocityMap.value(deviceId);
            qDebug() << QString("使用设备默认速度: %1 -> 速度 %2").arg(deviceId).arg(velocity);
            return velocity;
        }

        // 如果都没找到，返回0
        qDebug() << QString("未找到设备配置: %1，使用默认速度0").arg(deviceId);
        return 0;
    }
    //军事博弈相关
    void getPerformanceParams( std::map<std::string,std::string>& params ,const QString& filePath = QString("%1/gkd_config.ini").arg(QCoreApplication::applicationDirPath()))
    {
        QSettings settings(filePath,QSettings::IniFormat);

        QFile file(filePath);

        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            qDebug() << "无法打开配置文件进行读取:" << filePath;

        }

        QTextStream in(&file);

        in.setCodec("UTF-8");

        bool inPerformanceSection = false;

        while (!in.atEnd()) {
            QString line = in.readLine().trimmed();

            // 忽略空行和注释行（支持 ';' 和 '#' 注释）
            if (line.isEmpty() || line.startsWith(";") || line.startsWith("#")) {
                continue;
            }

            // 检测是否进入或离开了 [JSBY_Performance] 节点
            if (line.startsWith("[") && line.endsWith("]")) {
                if (line == "[JSBY_Performance]") {
                    inPerformanceSection = true;
                } else {
                    inPerformanceSection = false; // 进入了其他节点，停止录入
                }
                continue;
            }

            // 如果当前处于 [JSBY_Performance] 节点内，则解析键值对
            if (inPerformanceSection) {
                int equalsIndex = line.indexOf('=');
                if (equalsIndex != -1) {
                    QString key = line.left(equalsIndex).trimmed();
                    QString value = line.mid(equalsIndex + 1).trimmed();

                    // 使用 toUtf8().toStdString() 防止 Windows/Linux 跨平台 local8Bit 编码不一致导致的乱码
                    params[key.toUtf8().toStdString()] = value.toStdString();
                }
            }
        }

        file.close();

        if (params.empty()) {
            qDebug() << "提示：在" << filePath << "中未找到[performance]节点或该节点下为空！";
        } else {
            qDebug() << "成功从" << filePath << "读取到" << params.size() << "个性能参数！";
        }

        return;
    }
};
#endif //GKD_CONFIG_H1
