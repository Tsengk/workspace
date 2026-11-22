#pragma once
#ifndef LOG_SYS_H
#define LOG_SYS_H

#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include<QString>
#include<QDir>
#include <string>
#include<common/common.hpp>


using std::cout;
using std::string;
using std::endl;
using std::to_string;
using std::ios;

static string  currTime() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    auto timer = std::chrono::system_clock::to_time_t(now);
    std::tm bt;
#ifdef _WIN32
    localtime_s(&bt, &timer);  // Windows
#else
    localtime_r(&timer, &bt);  // Linux/Unix
#endif

    char buf[80];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &bt);

    std::ostringstream oss;
    oss << buf << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}
enum log_level { debug, info, warning, error };
enum log_target { file, terminal, file_and_terminal };
class Loggers {


public:
   std::ofstream outfile;
    log_target target;
   string path;
   log_level level;
   bool is_debug_mode = false;
    void output(string text, log_level act_level){
        if(!is_debug_mode) return;
       string prefix;
       if (act_level == debug) prefix = "[DEBUG] ";
       else if (act_level == info) prefix = "[INFO] ";
       else if (act_level == warning) prefix = "[WARNING] ";
       else if (act_level == error) prefix = "[ERROR] ";
       else prefix = "";
    //   prefix += __FILE__;
     //  prefix += " ";
       string output_content = prefix + currTime() + " : " + text;
       if (level <= act_level && target != file) {
           cout << output_content<<std::endl;
           qDebug()<<QString::fromStdString(output_content);
       }
       if (target != terminal)
           outfile << output_content << std::endl;
       outfile.flush();

   }
public:
   void  initLogger(){
       SetConsoleOutputCP(65001);
       path = common::getcurrentDir();

       qDebug()<<QString::fromStdString(path);

       // 创建log目录（如果不存在）- 使用Qt的QDir兼容VS2015
       QString log_dir_qstr = QString::fromStdString(path) + "/log";
       QDir log_dir(log_dir_qstr);
       if (!log_dir.exists()) {
           if (QDir().mkpath(log_dir_qstr)) {
               std::cout << "[LOG] 创建日志目录: " << log_dir_qstr.toStdString() << std::endl;
           } else {
               std::cerr << "[ERROR] 创建日志目录失败: " << log_dir_qstr.toStdString() << std::endl;
               qDebug() << QString("[ERROR] 创建日志目录失败: ") + log_dir_qstr;
           }
       }

       // 打开日志文件
       std::string log_file = path + "\\log\\log_.log";
       outfile.open(log_file, std::ios::out | std::ios::app);

       if(outfile.is_open()){
           is_debug_mode = true;  // 文件打开成功后才启用调试模式
           std::cout << "[LOG] 日志系统初始化成功: " << log_file << std::endl;
           qDebug() << QString::fromStdString("[LOG] 日志系统初始化成功: " + log_file);
       } else {
           std::cerr << "[ERROR] 日志文件打开失败: " << log_file << std::endl;
           qDebug() << QString::fromStdString("[ERROR] 日志文件打开失败: " + log_file);
           is_debug_mode = false;
       }

       target = log_target::file_and_terminal;
       level = log_level::debug;
       cout << "[WELCOME] " << __FILE__ << " " << currTime() << " : " << "=== Start logging ===" << endl;
   }
   //static Logger(log_target target, log_level level, string path);
   void DEBUG(string text){
        output(text, debug);
   }
    void INFO(string text){
         output(text, info);
   }
   void WARNING(string text){
          output(text, warning);
   }
  //  void ERROR(string text);
};
// void logger_test() {
//     Logger logger(Logger::file_and_terminal, Logger::debug, "result.log");
//     logger.DEBUG("What happened?");
//     logger.INFO("This is good.");
//     logger.WARNING("Yes...");
//     logger.ERROR("IO Error!");
// }
#endif
