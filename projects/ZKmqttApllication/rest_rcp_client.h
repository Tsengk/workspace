#pragma once

#include <QtWidgets/QWidget>
#include "ui_rest_rcp_client.h"
#include "RcpClientAdapter.h"
#include "common/common_config.h"
#include "Src/gkd/task_response_define.h"
typedef std::function<void(QString& msg)> MsgCallback;
class rest_rcp_client : public QWidget
{
	Q_OBJECT

public:
	rest_rcp_client(QWidget *parent = Q_NULLPTR);
private:
	Ui::rest_rcp_clientClass ui;
public:
    RcpClientAdapter* m_rcpClientAdapter;
    void startConnectRcpServer(const CommonCfg& _cfg);

    void subscribeTask(const string& key,TPCallback& cb);

    void slotLoadScheme();
private slots:
    void on_pushButton_updateRoute_clicked();

    void on_pushButton_goto_clicked();

    void on_pushButton_north_clicked();

    void on_pushButton_zoom_clicked();

    void on_pushButton_updatePointMark_clicked();

    void on_pushButton_updateRegionMark_clicked();

    void on_pushButton_setTargetVisible_clicked();

    void on_pushButton_showChatReply_clicked();

    void on_pushButton_showAlertData_clicked();

    void on_btn_sch_start_clicked();

    void on_btn_sch_pause_clicked();

    void on_btn_sch_continue_clicked();

    void on_btn_sch_end_clicked();

    void on_btn_showScheme_clicked();

    void on_btn_addbeh_clicked();

    void on_btn_addhocBeh_clicked();

    void on_btn_abandonbeh_clicked();

private:
	void buildConnect();

	void slotSDKGoto();
	void slotSDKZoom();
	void slotSDKUptoNorth();
	void slotSDKUpdatePointMark();
	void slotSDKUpdateRegionMark();
	void slotSDKSetTargetVisible();
	void slotSDKUpdateRoute();
	void slotSDKShowChatReply();
	void slotSDKShowAlertData();
    void slotSchemeStart();

    void slotSchemePause();

    void slotSchemeContinue();
    void slotSchemeEnd();


    void slotAddEntityMission();
    void slotAddAdHocMisson();
    void slotAbondonBeh();
    void slotReomveEntityMission();
    void slotRemoveScheme();
	#pragma region 
//**********************************************测试例子代码begin**********************************************
	/*void slotGetPersonSync();
	void slotGetPersonAsync();

	void slotGetDoubleStrSync();
	void slotGetDoubleStrAsync();

	void slotCallFunctionSync();*/
	//**********************************************测试例子代码end**********************************************
#pragma endregion 
};
