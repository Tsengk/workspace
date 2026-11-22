#ifndef GkdDataFusion_H
#define GkdDataFusion_H

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

struct FusionData{
    string name;
    std::vector<APIBridgeSDK::Position> path;
    void clear() {
        name = "";
        path.clear();
    }
};

class CustomTitleBar;

namespace Ui {
class GkdDataFusion;
}

/**
高差融合： 点击停止->点击开始->点击高差融合
球面距离融合：点击停止->点击开始->点击球面距离融合
**/
class GkdDataFusion : public QWidget
{
    Q_OBJECT

public:
    explicit GkdDataFusion(QWidget *parent,CommonCfg cfg);
    ~GkdDataFusion();

private slots:
    void onAISClicked();
    void onADSClicked();
    void onStopAISClicked();
    void onStopADSClicked();
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
    void planAIS(FusionData& _fusionData);
    void PlanADS(FusionData& _fusionData);
    void pushFusion(std::vector<FusionData>& _fusionData); //目标融合
    bool PackageDzmbqb_301(QString name,QJsonObject& root,APIBridgeSDK::Position& pos); //电子目标情报
    bool PackageDzmbxsb_302(string name,QJsonObject& root); // 电子目标消失报
    bool PackageZzptjzbzt_303(QJsonObject& root,APIBridgeSDK::Position& pos); // 作战平台,飞机
    bool PackageZzptjzbzt_303_CB(QJsonObject& root,APIBridgeSDK::Position& pos); // 作战平台，船舶

    // // 设置C2Manager实例
    // void setMqttAndRCP(std::shared_ptr <MqClient> gkd_client, std::shared_ptr <rest_rcp_client> rcp_client);

// protected:
//     void resizeEvent(QResizeEvent *event) override;
//     void mousePressEvent(QMouseEvent *event) override;
//     void mouseMoveEvent(QMouseEvent *event) override;
//     void mouseReleaseEvent(QMouseEvent *event) override;
//     bool eventFilter(QObject *obj, QEvent *event) override;

// private slots:
//     void onPlanFileLoaded(const QString &filePath);
//     void onTaskInfoChanged(const QJsonObject &taskInfo);
//     void onTaskDelivered(const QJsonObject &taskInfo);
//     void onToggleStayOnTop();
//     void onTitleBarContextMenu(const QPoint &pos);

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
    std::vector<FusionData> _hk_fusion_data;
    std::vector<FusionData> _fusion_data;
    GEO_ALGO _geo_algo;
    QJsonArray _duokongRedData;
    QJsonArray _duokongBlueData;
    // QJsonObject temp_dzmbqb_301;
    // QJsonObject temp_dzmbxsb_302;
    // QJsonObject temp_zzptjzbzt_303;

    Ui::GkdDataFusion *ui;
    CustomTitleBar *titleBar;

    CommonCfg _cfg;
    std::shared_ptr <MqClient> _pgkd_mqtt_client;
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

#endif // GkdDataFusion_H
