#ifndef XSJSIMUI_H
#define XSJSIMUI_H

#include <QMainWindow>
#include <QTimer>
#include <QResizeEvent>
#include "xjsim/xjsim.h"
#include "customtitlebar.h"
//发送目标信息
struct TargetInfo11{
    int tarhet_type;
    QString name;
    QString target_id;
    double lon;
    double lat;
};
namespace Ui {
class XsJSimUi;
}


class XsJSimUi : public QMainWindow
{
    Q_OBJECT

public:
    explicit XsJSimUi(QWidget *parent = nullptr);
    ~XsJSimUi();
    void startTimer(MqClient* mqClient,rest_rcp_client* _rcp_client,const CommonCfg& _cfg,const GKDCommonCfg& gkd_cfg);

    void clearPlannedRoutes();

private slots:
    void on_btn_clear_targets_routes_clicked();

    void on_btn_publish303_clicked();

    void on_btn_stop_uav_clicked();

    void on_btn_start_uav_clicked();


    void on_action_exit_triggered();

    void on_btn_start_fpv_clicked();

    void on_btn_stop_fpv_clicked();

    void on_cmd_reset_clicked();

    void on_btn_open_cabin_clicked();

    void on_btn_close_cabin_clicked();

    void on_btn_top_clicked();

    void on_btn_right_clicked();

    void on_btn_left_clicked();

    void on_btn_bottom_clicked();

    void updateObjectsList();

    void onObjectSelected(int index);
    void onTargetSelected(int index);

    void on_btn_scout_clicked();

    void on_btn_strike_clicked();

    void on_btn_uav_strike_clicked();

    void on_btn_uav_land_clicked();

    void on_btn_test_load_task_clicked();

    void on_btn_test_location_clicked();
    void on_btn_test_location2_clicked();
    void on_btn_test_location3_clicked();
    void on_btn_test_location4_clicked();
    void on_pushButton_gante_clicked();
    void on_pushButton_pushtarget_clicked(); //发送目标



    void on_btn_test_ers_clicked();

    void on_btn_show_fire_points_clicked();

    void on_btn_show_safe_zone_clicked();

    void on_btn_show_avoidance_area_clicked();

    void on_btn_show_yield_points_clicked();
    
    void on_btn_vechile_clicked();

protected:
    void resizeEvent(QResizeEvent *event) override;

private:
    CustomTitleBar *titleBar;
    Ui::XsJSimUi *ui;
    XjSim sim_controller_;
    MqClient* mqttClient_ = nullptr;
    rest_rcp_client* rpcClient_ = nullptr;
    void showOperationResult(const ApiResponse& response, const QString& operation);
    QTimer* update_timer_;
    QString selected_uid_;  // 存储当前选中的智能体UID
    TargetInfo11 _targetInfo;
    int targetidIndex = 200;

    //定义测试数据
    const std::string test_run_nav = R"({
  "cmd": "RUN_UAV",
  "sn": "uav_1751106428.323309",
  "data": [
    {
      "lat": 29.729701200328144,
      "lon": 106.51951126679494,
      "hegiht": 17.748696895364905
    },
    {
      "lat": 29.730017189201778,
      "lon": 106.51782774147395,
      "hegiht": 180.0
    },
    {
      "lat": 29.730017189201778,
      "lon": 106.51777760266471,
      "hegiht": 180.0
    },
    {
      "lat": 29.730017189201778,
      "lon": 106.51772746385545,
      "hegiht": 180.0
    },
    {
      "lat": 29.730017189201778,
      "lon": 106.51767732504621,
      "hegiht": 180.0
    },
    {
      "lat": 29.730017189201778,
      "lon": 106.51762718623695,
      "hegiht": 180.0
    },
    {
      "lat": 29.730017189201778,
      "lon": 106.51757704742771,
      "hegiht": 180.0
    },
    {
      "lat": 29.730017189201778,
      "lon": 106.51752690861846,
      "hegiht": 180.0
    },
    {
      "lat": 29.730017189201778,
      "lon": 106.5174767698092,
      "hegiht": 180.0
    },
    {
      "lat": 29.730017189201778,
      "lon": 106.51742663099996,
      "hegiht": 180.0
    },
    {
      "lat": 29.730017189201778,
      "lon": 106.5173764921907,
      "hegiht": 180.0
    },
    {
      "lat": 29.730017189201778,
      "lon": 106.51732635338146,
      "hegiht": 269.99985121319116
    },
    {
      "lat": 29.73032757230665,
      "lon": 106.51732635338146,
      "hegiht": -0.0
    },
    {
      "lat": 29.73032757230665,
      "lon": 106.5173764921907,
      "hegiht": -0.0
    },
    {
      "lat": 29.73032757230665,
      "lon": 106.51742663099996,
      "hegiht": -0.0
    },
    {
      "lat": 29.73032757230665,
      "lon": 106.5174767698092,
      "hegiht": -0.0
    },
    {
      "lat": 29.73032757230665,
      "lon": 106.51752690861846,
      "hegiht": -0.0
    },
    {
      "lat": 29.73032757230665,
      "lon": 106.51757704742771,
      "hegiht": -0.0
    },
    {
      "lat": 29.73032757230665,
      "lon": 106.51762718623695,
      "hegiht": -0.0
    },
    {
      "lat": 29.73032757230665,
      "lon": 106.51767732504621,
      "hegiht": -0.0
    },
    {
      "lat": 29.73032757230665,
      "lon": 106.51772746385545,
      "hegiht": -0.0
    },
    {
      "lat": 29.73032757230665,
      "lon": 106.51777760266471,
      "hegiht": -0.0
    },
    {
      "lat": 29.73032757230665,
      "lon": 106.51782774147395,
      "hegiht": -0.0
    }
  ],
  "timestamp": 1751106441
})";
    const std::string test_run_fpv = R"({"cmd":"RUN_FPV","sn":"sn","data":"","timestamp":1750995507})";

};

#endif // XSJSIMUI_H
