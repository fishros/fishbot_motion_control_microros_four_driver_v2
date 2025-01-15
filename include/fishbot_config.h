/**
 * @file fishbot_config.h
 * @author fishros (fishros@foxmail.com)
 * @brief 机器人配置汇总
 * @version V1.0.0
 * @date 2023-01-04
 *
 * @copyright Copyright (c) fishros.com & fishros.org.cn 2023
 *
 */
#ifndef __FISHBOT_CONFIG_H__
#define __FISHBOT_CONFIG_H__

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include "fishlog.h"

/*=========================================常量值定义========================================*/
#define CONFIG_TRANSPORT_MODE_SERIAL "serial"              // 串口模式，0
#define CONFIG_TRANSPORT_MODE_WIFI_UDP_CLIENT "udp_client" // UDP客户端模式，1

#define CONFIG_MOTION_MODE_MECANUM "mecanum" // 麦轮模式
#define CONFIG_MOTION_MODE_DIFFERENTIAL "differential" // 差速轮模式

/*=========================================默认值定义=====================================*/
#define CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_IP "192.168.2.105" // 默认UDP服务端IP
#define CONFIG_DEFAULT_TRANSPORT_MODE_WIFI_SERVER_PORT "8888"        // 默认UDP服务端端口号
#define CONFIG_DEFAULT_TRANSPORT_MODE "udp_client"                   // 默认传输模式-udp_client模式
#define CONFIG_DEFAULT_SERIAL_ID "0"                                 // 可选使用0或者2,使用2则需要使用GPIO16和17作为RXTX
#define CONFIG_DEFAULT_TRANSPORT_SERIAL_BAUD "921600"

//------------------------------------WIFI SSID-----------------------------------------
#define CONFIG_DEFAULT_WIFI_STA_SSID "m5"
#define CONFIG_DEFAULT_WIFI_STA_PSWK "88888888"

//--------------------------------------电机相关配置---------------------------------------
#define CONFIG_DEFAULT_MOTOR_PID_KP "0.625"
#define CONFIG_DEFAULT_MOTOR_PID_KI "0.125"
#define CONFIG_DEFAULT_MOTOR_PID_KD "0.0"
#define CONFIG_DEFAULT_MOTOR_OUT_LIMIT_LOW "-100"
#define CONFIG_DEFAULT_MOTOR_OUT_LIMIT_HIGH "100"
#define CONFIG_DEFAULT_MOTOR_PARAM_REDUCATION_RATIO "45"
#define CONFIG_DEFAULT_MOTOR_PARAM_PULSE_RATION "44"
#define CONFIG_DEFAULT_MOTOR_PARAM_WHEEL_DIAMETER "48"
//-------------------------------------默认轮距----------------------------------------------
#define CONFIG_DEFAULT_KINEMATIC_WHEEL_DISTANCE_A "216" 
#define CONFIG_DEFAULT_KINEMATIC_WHEEL_DISTANCE_B "177"

//------------------------------------IO相关配置----------------------------------------------

//-----------------------------ROS2节点相关配置-------------------------------------------
#define CONFIG_DEFAULT_ROS2_NODE_NAME "fishbot_motion_control"
#define CONFIG_DEFAULT_ROS2_NAMESPACE ""
#define CONFIG_DEFAULT_ROS2_ODOM_TOPIC_NAME "odom"
#define CONFIG_DEFAULT_ROS2_ODOM_FRAME_ID "odom"
#define CONFIG_DEFAULT_ROS2_ODOM_CHILD_FRAME_ID "base_footprint"
#define CONFIG_DEFAULT_ROS2_CMD_VEL_TOPIC_NAME "cmd_vel"
#define CONFIG_DEFAULT_ROS2_ODOM_PUBLISH_PERIOD "50"
#define CONFIG_DEFAULT_ROS2_HANDLE_NUM "2"
#define CONFIG_DEFAULT_ROS2_RCL_SPIN_TIME "100"
#define CONFIG_DEFAULT_ROS2_TIME_SYNC_TIMEOUT "5000"

#define CONFIG_NAME_NAMESPACE "fishbot"
#define VERSION_CODE "{VERSION}"
#define FIRST_START_TIP "=================================================\n     wwww.fishros.com        \nfishbot-motion-control-{VERSION}\n=================================================\n"

typedef enum
{
    CONFIG_PARSE_ERROR = -1,
    CONFIG_PARSE_NODATA = 0,
    CONFIG_PARSE_OK = 1,
} fishbot_config_status;

class FishBotConfig
{
private:
    /* data */
    Preferences preferences;

public:
    void init(String namespace_);
    uint32_t is_first_startup();

    bool config(String key, String value);

    String motion_mode();
    String config_str();
    String board_name();
    // 基础配置
    uint32_t serial_baudrate();
    String wifi_sta_ssid();
    String wifi_sta_pswd();
    String wifi_ap_ssid();
    String wifi_ap_pswd();
    // MicroROS相关
    String microros_transport_mode();
    String microros_uclient_server_ip();
    uint32_t microros_uclient_server_port();
    uint32_t microros_serial_id();
    // ROS2相关
    String ros2_nodename();
    String ros2_namespace();
    String ros2_odom_topic_name();
    String ros2_odom_frameid();
    String ros2_odom_child_frameid();
    String ros2_twist_topic_name();
    uint32_t odom_publish_period();
    // 运动学相关配置
    float kinematics_reducation_ration();
    uint32_t kinematics_pulse_ration();
    uint32_t kinematics_wheel_diameter();
    float kinematics_wheel_distance_a();
    float kinematics_wheel_distance_b();
    
    float kinematics_pid_kp();
    float kinematics_pid_ki();
    float kinematics_pid_kd();
    float kinematics_pid_out_limit();

    int8_t loop_config_uart(int c, char result[][32]);
    int8_t split_str(const char *line, char result[][32]);

    FishBotConfig(/* args */) = default;
    ~FishBotConfig() = default;
};

#endif // __FISHBOT_CONFIG_H__