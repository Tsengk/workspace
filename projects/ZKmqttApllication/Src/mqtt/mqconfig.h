#ifndef MQCONFIG_H
#define MQCONFIG_H
#include<string>
using namespace std;
struct MqCfg
{
    std::string ip = "8.137.149.129";
    string username = "";
    string passwrd ="";
    int16_t port = 1883;
    int16_t keepalive = 60;

    MqCfg()
    {
    }
    MqCfg(std::string m_ip, int16_t m_port,string m_username,string m_passwrd, int16_t m_keepalive)
    {
        ip = m_ip;
        port = m_port;
        username = m_username;
        passwrd = m_passwrd;
        keepalive = m_keepalive;
    }
};
#endif // MQCONFIG_H
