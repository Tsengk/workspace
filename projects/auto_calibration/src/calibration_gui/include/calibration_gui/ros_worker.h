#pragma once
#include <QObject>
#include <QString>
#include <QProcess>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <mutex>
#include <functional>
#include <map>

namespace calibration_gui {

/// ROS 通信工作对象（在 Qt 事件循环中通过定时器轮询）
/// 封装对各标定节点的服务调用与话题订阅
class RosWorker : public QObject {
    Q_OBJECT
public:
    explicit RosWorker(QObject* parent = nullptr);
    ~RosWorker();

    void init(int argc, char** argv);
    void spinOnce();  // 由主窗口定时器调用

    // ---- 服务调用（同步，在 Qt 线程中调用） ----
    bool callTrigger(const std::string& service_name, std::string& msg_out);
    bool callReset(const std::string& service_name, std::string& msg_out);
    bool callSave(const std::string& service_name, std::string& msg_out);

    // ---- rosparam 读写 ----
    void setParam(const std::string& key, double value);
    bool getParam(const std::string& key, double& value);

    // ---- 订阅管理 ----
    void subscribeStatus(const std::string& topic,
                         std::function<void(const std::string&)> cb);
    void subscribeCloud(const std::string& topic,
                        std::function<void(pcl::PointCloud<pcl::PointXYZI>::Ptr)> cb);
    void subscribeCloudRGB(const std::string& topic,
                           std::function<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr)> cb);
    void subscribeImage(const std::string& topic,
                        std::function<void(cv::Mat)> cb);
    void subscribeResult(const std::string& topic,
                         std::function<void(double,double,double)> cb);
    void unsubscribeAll();

    // ---- 发布点（用于 GUI 选点 → 标定节点）----
    void publishPoint(const std::string& topic, float x, float y, float z,
                      const std::string& frame_id = "");

    // ---- 节点进程管理 ----
    /// 启动节点：roslaunch <pkg> <launch_file> [extra_args]，node_key 用于后续 kill/查询
    bool launchNode(const std::string& pkg, const std::string& launch_file,
                    const std::string& node_key,
                    const QStringList& extra_args = QStringList());
    /// 通过绝对路径启动 launch 文件：roslaunch <abs_launch_path> [extra_args]
    bool launchNodeByPath(const std::string& abs_launch_path,
                          const std::string& node_key,
                          const QStringList& extra_args = QStringList());
    /// 停止节点（SIGTERM → 等待3s → SIGKILL）
    void killNode(const std::string& node_key);
    /// 检查节点进程是否仍在运行
    bool isNodeRunning(const std::string& node_key) const;

    bool isRosOk() const { return ros::ok(); }

signals:
    void rosShutdown();
    void nodeStarted(const QString& node_key);
    void nodeStopped(const QString& node_key);
    void nodeLogOutput(const QString& node_key, const QString& text);

private:
    ros::NodeHandle* nh_{nullptr};

    // 活跃订阅
    std::vector<ros::Subscriber> subscribers_;
    std::mutex sub_mutex_;

    // 节点进程管理
    std::map<std::string, QProcess*> node_processes_;

    // 点发布器缓存
    std::map<std::string, ros::Publisher> point_publishers_;

    void statusCb(const std_msgs::String::ConstPtr& msg,
                  std::function<void(const std::string&)> cb);
    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg,
                 std::function<void(pcl::PointCloud<pcl::PointXYZI>::Ptr)> cb);
    void cloudRgbCb(const sensor_msgs::PointCloud2::ConstPtr& msg,
                    std::function<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr)> cb);
    void imageCb(const sensor_msgs::Image::ConstPtr& msg,
                 std::function<void(cv::Mat)> cb);
    void resultCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg,
                  std::function<void(double,double,double)> cb);
};

} // namespace calibration_gui
