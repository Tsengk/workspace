#ifndef GkdDataFusion_BY_H
#define GkdDataFusion_BY_H

#include <QWidget>
#include <QListWidgetItem>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QResizeEvent>
#include <QMouseEvent>
#include <QMenu>
#include <QAction>
#include <memory>
#include"common/gkd_config.h"
#include "mqtt/mq_client.h"
#include "../../rest_rcp_client.h"
#include"gkd/task_response_define.h"
#include"utm/geo_algo.h"
#include <algorithm> // 包含reverse函数
#include<vector>
#include<iostream>
#include<QRadioButton>
#include<mutex>
#include <algorithm> // 包含 std::find
#include <thread>
#include <atomic>

struct FusionData_by{
    string name;
    std::vector<APIBridgeSDK::Position> path;
    void clear() {
        name = "";
        path.clear();
    }
};

//记录要素当前位置
struct CurrentPos{
    string name;
    APIBridgeSDK::Position pos;
};
class CustomTitleBar;

namespace Ui {
class GkdDataFusion_by;
}

/**
高差融合： 点击停止->点击开始->点击高差融合
球面距离融合：点击停止->点击开始->点击球面距离融合
**/
class GkdDataFusion_by : public QWidget
{
    Q_OBJECT

public:
    explicit GkdDataFusion_by(QWidget *parent,CommonCfg cfg);
    ~GkdDataFusion_by();

private slots:
    // void onAISClicked();
    // void onADSClicked();
    // void onStopAISClicked();
    // void onStopADSClicked();
    void onLocateClicked();
    void onStartPushFusion();
    void onResetPushFusion();
    void onStopPushFusion();
    void OnFusionDistance();
    void OnFusionHeigh();
    void onDuokongPushClicked();
    void showDuokongDataEmptyWarning();
protected:
    void showEvent(QShowEvent *event) override;
    void hideEvent(QHideEvent *event) override;

private:
    void CacheFusionData();
    void ClearFunctionData();
    // void planAIS(FusionData_by& _fusionData);
    // void PlanADS(FusionData_by& _fusionData);
    void pushFusion(std::vector<FusionData_by>& _fusionData); //目标融合
    void pushFusion_OwerPlantform(std::vector<FusionData_by> _fusionData); //我方装备
    void CalBoYi(); //博弈计算逻辑
    bool PackageDzmbqb_301(QString name,QJsonObject& root,APIBridgeSDK::Position& pos); //电子目标情报
    bool PackageDzmbxsb_302(string name,QJsonObject& root); // 电子目标消失报
    bool PackageZzptjzbzt_303(QString name,QJsonObject& root,APIBridgeSDK::Position& pos); // 作战平台,飞机
    bool PackageZzptjzbzt_303_CB(QJsonObject& root,APIBridgeSDK::Position& pos); // 作战平台，船舶
    bool PackageDzmbxsb_304(string name,QJsonObject& root); // 作战平台消失
    // // 设置C2Manager实例

 signals:
        void setBYType(bool _isShowThreatHemisphere,bool r1,bool r2, bool r3, bool r4); //军事博弈状态类型设定 基于零和模型
private:
    void addHeader(QJsonObject& root); // 加入报文头 SID TM
    void readHKFusion(const QString &filePath);
    void readFusionData(const QString &filePath);
    void pushDuokongData(QJsonArray redArray, QJsonArray blueArray);
    QJsonArray loadJsonArray(const QString &filePath);
    QString resolveJsonPath(const QString &fileName) const;
    // void setupContextMenu();
    // Qt::CursorShape getCursorShape(const QPoint &point);
    // int getResizeDirection(const QPoint &point);
    int pc_i = 10;
    bool _stopAISThread = false;
    bool _stopADSThread = false;
    bool _stopFusionDataThread = false;
    bool _resetPushFusionDataThread = false;
    bool _fusionDistance =false;
    bool _fusionHeigh = false;
    std::vector<FusionData_by> _hk_fusion_data;
    std::vector<FusionData_by> _fusion_data;
    std::vector<FusionData_by> _current_OwnPlatForm_Pos_list; //jS博弈当前位置，我方
    std::vector<FusionData_by > _current_target_Pos_list; //军事博弈当前位置，敌方

    int target_index = 0;
    int platform_index = 0;

    int sn = 0; // 抽吸表示
 //   std::mutex mtx; // 全局互斥锁
    GEO_ALGO _geo_algo;
    QJsonArray _duokongRedData;
    QJsonArray _duokongBlueData;
    QButtonGroup  _readBGroup;
    QButtonGroup  _blueBGroup;
    // QJsonObject temp_dzmbqb_301;
    // QJsonObject temp_dzmbxsb_302;
    // QJsonObject temp_zzptjzbzt_303;

    Ui::GkdDataFusion_by *ui;
    CustomTitleBar *titleBar;

    CommonCfg _cfg;
   std::shared_ptr <MqClient>   _pgkd_mqtt_client;
    std::shared_ptr <rest_rcp_client> _prcp_client;
    std::chrono::steady_clock::time_point last_time;

    int test_id = 0;
    // 右键菜单相关
    QMenu *contextMenu;
    QAction *stayOnTopAction;
    bool isStayOnTop;

    // 窗口拉伸相关
    bool isResizing;
    int resizeDirection;
    QPoint lastMousePos;
    QRect originalGeometry;
    static const int RESIZE_BORDER = 8; // 拉伸边界宽度

    // 拉伸方向常量
    enum ResizeDirection {
        None = 0,
        Left = 1,
        Right = 2,
        Top = 4,
        Bottom = 8,
        TopLeft = Top | Left,
        TopRight = Top | Right,
        BottomLeft = Bottom | Left,
        BottomRight = Bottom | Right
    };
};

#endif // GkdDataFusion_BY_H
