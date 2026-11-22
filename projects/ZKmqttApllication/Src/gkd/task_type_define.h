#ifndef TASK_TYPE_DEFINES_H
#define TASK_TYPE_DEFINES_H

// 任务类型定义
#define ZG_DJ_TASK_TYPE 43 // 抵近任务类型ID
#define ZG_KK_TASK_TYPE 44 //
#define ZG_KD_TASK_TYPE 45 //

// 临机干预（105），任务干预标识
#define LJGY_DC_LOCATION  5; //手机电磁，锁定电磁目标
#define LJGY_DC_CJ_PLAN 6; //手机电磁，电磁从机规划路线
#define LJGH_ZG_TARGET_LOCATION 8; //侦查干扰 目标锁定
#define LJGH_ZG_JIAJIAO 9;
#define LJGH_ZG_TARGET_KM 10; // 侦察干扰-3科目干扰阶段标志
#define LJGH_KDXT_GZ 11; //空地协同 目标识别转跟踪打击
#define LJGH_KDXT_SQ 12; //空地协同，授权打击
#define LJGH_KDXT_JS 13; //空地协同结束

#endif // TASK_TYPE_DEFINES_H
