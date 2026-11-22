#include "calibration_gui/ros_worker.h"
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <QMetaObject>

namespace calibration_gui {

RosWorker::RosWorker(QObject* parent) : QObject(parent) {}

RosWorker::~RosWorker()
{
    // 停止所有托管的节点进程
    for (auto& kv : node_processes_) {
        if (kv.second) {
            kv.second->terminate();
            kv.second->waitForFinished(3000);
            if (kv.second->state() != QProcess::NotRunning)
                kv.second->kill();
            delete kv.second;
        }
    }
    node_processes_.clear();
    delete nh_;
}

void RosWorker::init(int argc, char** argv)
{
    ros::init(argc, argv, "calibration_gui",
              ros::init_options::AnonymousName |
              ros::init_options::NoSigintHandler);
    nh_ = new ros::NodeHandle();
}

void RosWorker::spinOnce()
{
    if (ros::ok()) {
        ros::spinOnce();
    } else {
        emit rosShutdown();
    }
}

bool RosWorker::callTrigger(const std::string& service_name, std::string& msg_out)
{
    ros::ServiceClient client = nh_->serviceClient<std_srvs::Trigger>(service_name);
    std_srvs::Trigger srv;
    if (client.call(srv)) {
        msg_out = srv.response.message;
        return srv.response.success;
    }
    msg_out = "Service call failed: " + service_name;
    return false;
}

bool RosWorker::callReset(const std::string& service_name, std::string& msg_out)
{
    return callTrigger(service_name, msg_out);
}

bool RosWorker::callSave(const std::string& service_name, std::string& msg_out)
{
    return callTrigger(service_name, msg_out);
}

void RosWorker::setParam(const std::string& key, double value)
{
    nh_->setParam(key, value);
}

bool RosWorker::getParam(const std::string& key, double& value)
{
    return nh_->getParam(key, value);
}

void RosWorker::subscribeStatus(const std::string& topic,
                                 std::function<void(const std::string&)> cb)
{
    std::lock_guard<std::mutex> lk(sub_mutex_);
    auto sub = nh_->subscribe<std_msgs::String>(topic, 10,
        [this, cb](const std_msgs::String::ConstPtr& msg) {
            statusCb(msg, cb);
        });
    subscribers_.push_back(sub);
}

void RosWorker::subscribeCloud(const std::string& topic,
                                std::function<void(pcl::PointCloud<pcl::PointXYZI>::Ptr)> cb)
{
    std::lock_guard<std::mutex> lk(sub_mutex_);
    auto sub = nh_->subscribe<sensor_msgs::PointCloud2>(topic, 1,
        [this, cb](const sensor_msgs::PointCloud2::ConstPtr& msg) {
            cloudCb(msg, cb);
        });
    subscribers_.push_back(sub);
}

void RosWorker::subscribeCloudRGB(const std::string& topic,
                                   std::function<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr)> cb)
{
    std::lock_guard<std::mutex> lk(sub_mutex_);
    auto sub = nh_->subscribe<sensor_msgs::PointCloud2>(topic, 1,
        [this, cb](const sensor_msgs::PointCloud2::ConstPtr& msg) {
            cloudRgbCb(msg, cb);
        });
    subscribers_.push_back(sub);
}

void RosWorker::subscribeImage(const std::string& topic,
                                std::function<void(cv::Mat)> cb)
{
    std::lock_guard<std::mutex> lk(sub_mutex_);
    auto sub = nh_->subscribe<sensor_msgs::Image>(topic, 1,
        [this, cb](const sensor_msgs::Image::ConstPtr& msg) {
            imageCb(msg, cb);
        });
    subscribers_.push_back(sub);
}

void RosWorker::subscribeResult(const std::string& topic,
                                 std::function<void(double,double,double)> cb)
{
    std::lock_guard<std::mutex> lk(sub_mutex_);
    auto sub = nh_->subscribe<geometry_msgs::Vector3Stamped>(topic, 1,
        [this, cb](const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
            resultCb(msg, cb);
        });
    subscribers_.push_back(sub);
}

void RosWorker::unsubscribeAll()
{
    std::lock_guard<std::mutex> lk(sub_mutex_);
    for (auto& s : subscribers_) s.shutdown();
    subscribers_.clear();
}

void RosWorker::publishPoint(const std::string& topic, float x, float y, float z,
                              const std::string& frame_id)
{
    if (!nh_) return;
    auto it = point_publishers_.find(topic);
    if (it == point_publishers_.end()) {
        point_publishers_[topic] =
            nh_->advertise<geometry_msgs::PointStamped>(topic, 1);
        it = point_publishers_.find(topic);
    }
    geometry_msgs::PointStamped msg;
    msg.header.stamp    = ros::Time::now();
    msg.header.frame_id = frame_id;
    msg.point.x = x;
    msg.point.y = y;
    msg.point.z = z;
    it->second.publish(msg);
}

bool RosWorker::launchNode(const std::string& pkg,
                            const std::string& launch_file,
                            const std::string& node_key,
                            const QStringList& extra_args)
{
    // 若已在运行，直接返回
    if (isNodeRunning(node_key)) return true;

    auto* proc = new QProcess(this);
    QString qkey = QString::fromStdString(node_key);

    // 进程结束时发出 nodeStopped 信号，并清理 map
    connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this, qkey, proc](int, QProcess::ExitStatus) {
                node_processes_.erase(qkey.toStdString());
                proc->deleteLater();
                emit nodeStopped(qkey);
            });

    QStringList args;
    args << QString::fromStdString(pkg) << QString::fromStdString(launch_file);
    args << extra_args;

    proc->setProcessChannelMode(QProcess::MergedChannels);

    connect(proc, &QProcess::readyReadStandardOutput,
            this, [this, proc, qkey]() {
                QByteArray data = proc->readAllStandardOutput();
                QString text = QString::fromLocal8Bit(data);
                fprintf(stdout, "%s", text.toLocal8Bit().constData());
                fflush(stdout);
                emit nodeLogOutput(qkey, text);
            });

    // 打印完整 roslaunch 命令到终端
    fprintf(stdout, "[RosWorker] launchNode [%s]: roslaunch %s\n",
            node_key.c_str(), args.join(' ').toLocal8Bit().constData());
    fflush(stdout);

    proc->start("roslaunch", args);

    if (!proc->waitForStarted(5000)) {
        delete proc;
        return false;
    }

    node_processes_[node_key] = proc;
    emit nodeStarted(qkey);
    return true;
}

bool RosWorker::launchNodeByPath(const std::string& abs_launch_path,
                                  const std::string& node_key,
                                  const QStringList& extra_args)
{
    if (isNodeRunning(node_key)) return true;

    auto* proc = new QProcess(this);
    QString qkey = QString::fromStdString(node_key);

    connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this, qkey, proc](int, QProcess::ExitStatus) {
                node_processes_.erase(qkey.toStdString());
                proc->deleteLater();
                emit nodeStopped(qkey);
            });

    QStringList args;
    args << QString::fromStdString(abs_launch_path);
    args << extra_args;

    proc->setProcessChannelMode(QProcess::MergedChannels);

    connect(proc, &QProcess::readyReadStandardOutput,
            this, [this, proc, qkey]() {
                QByteArray data = proc->readAllStandardOutput();
                QString text = QString::fromLocal8Bit(data);
                fprintf(stdout, "%s", text.toLocal8Bit().constData());
                fflush(stdout);
                emit nodeLogOutput(qkey, text);
            });

    // 打印完整 roslaunch 命令到终端
    fprintf(stdout, "[RosWorker] launchNodeByPath [%s]: roslaunch %s\n",
            node_key.c_str(), args.join(' ').toLocal8Bit().constData());
    fflush(stdout);

    proc->start("roslaunch", args);

    if (!proc->waitForStarted(5000)) {
        delete proc;
        return false;
    }

    node_processes_[node_key] = proc;
    emit nodeStarted(qkey);
    return true;
}

void RosWorker::killNode(const std::string& node_key)
{
    auto it = node_processes_.find(node_key);
    if (it == node_processes_.end()) return;

    fprintf(stdout, "[RosWorker] killNode [%s]\n", node_key.c_str());
    fflush(stdout);

    QProcess* proc = it->second;
    proc->terminate();
    if (!proc->waitForFinished(3000))
        proc->kill();
    // finished 信号会自动清理 map 和 delete proc
}

bool RosWorker::isNodeRunning(const std::string& node_key) const
{
    auto it = node_processes_.find(node_key);
    return it != node_processes_.end() &&
           it->second->state() == QProcess::Running;
}

void RosWorker::statusCb(const std_msgs::String::ConstPtr& msg,
                          std::function<void(const std::string&)> cb)
{
    std::string data = msg->data;
    cb(data);
}

void RosWorker::cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg,
                         std::function<void(pcl::PointCloud<pcl::PointXYZI>::Ptr)> cb)
{
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);
    cb(cloud);
}

void RosWorker::cloudRgbCb(const sensor_msgs::PointCloud2::ConstPtr& msg,
                            std::function<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr)> cb)
{
    auto cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);
    cb(cloud);
}

void RosWorker::imageCb(const sensor_msgs::Image::ConstPtr& msg,
                         std::function<void(cv::Mat)> cb)
{
    try {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
        cb(img);
    } catch (...) {}
}

void RosWorker::resultCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg,
                          std::function<void(double,double,double)> cb)
{
    double x = msg->vector.x, y = msg->vector.y, z = msg->vector.z;
    cb(x, y, z);
}

} // namespace calibration_gui
