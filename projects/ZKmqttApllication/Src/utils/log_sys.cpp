#include "log_sys.h"

// void Logger::initLogger() {

// }

// Logger::Logger(log_target target, log_level level, string path) {
//     this->target = target;
//     this->path = path;
//     this->level = level;
//     string tmp = "";
//     string welcome_dialog = tmp + "[Welcome] " + __FILE__ + " " + currTime() + " : " + "=== Start logging ===\n";
//     if (target != terminal) {
//         this->outfile.open(path, ios::out | ios::app);
//         this->outfile << welcome_dialog;
//     }
//     if (target != file) {
//         cout << welcome_dialog;
//     }
// }

//  void Logger::output(string text, log_level act_level) {
//     string prefix;
//     if (act_level == debug) prefix = "[DEBUG] ";
//     else if (act_level == info) prefix = "[INFO] ";
//     else if (act_level == warning) prefix = "[WARNING] ";
//     else if (act_level == error) prefix = "[ERROR] ";
//     else prefix = "";
//     prefix += __FILE__;
//     prefix += " ";
//     string output_content = prefix + currTime() + " : " + text + "\n";
//     if (level <= act_level && target != file) {
//         cout << output_content;
//     }
//     if (target != terminal)
//         outfile << output_content;
// }

// void Logger::DEBUG(string text) {
//     output(text, debug);
// }

// void Logger::INFO(string text) {
//     output(text, info);
// }

// void Logger::WARNING(string text) {
//     output(text, warning);
// }

// void Logger::ERROR(string text) {
//     this->output(text, error);
// }
