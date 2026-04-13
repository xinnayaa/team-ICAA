#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath> // 用于计算距离

// 全局变量用于存储无人机状态和位置
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pos;

// 状态回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// 位置回调函数：用于判断是否到达目标点
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pos = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "off1_node"); // 这里的节点名建议与你的包名一致
    ros::NodeHandle nh;

    // 订阅者
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);

    // 发布者：发送目标位置
    ros::Publisher local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    // 服务客户端：解锁和切模式
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0); // PX4 建议频率在 20Hz 以上

    // 等待连接
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // --- 方形路径点定义 (x, y, z) ---
    // ENU 坐标系：x-东, y-北, z-天
    double goals[4][3] = {
        {20.0, 0.0, 20.0},  // A点
        {20.0, 20.0, 20.0},  // B点
        {0.0, 20.0, 20.0},  // C点
        {0.0, 0.0, 20.0}   // D点（回到起飞点上方）
    };
    int goal_idx = 0; // 当前目标点索引

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // 预发布 100 个点，防止切 OFFBOARD 失败
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pose_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        // 1. 自动切换到 OFFBOARD 模式
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        // 2. 自动解锁
        else {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // --- 核心逻辑：方形点切换 ---
        if (current_state.mode == "OFFBOARD" && current_state.armed) {
            // 计算当前位置与目标点的距离误差
            double dx = current_pos.pose.position.x - goals[goal_idx][0];
            double dy = current_pos.pose.position.y - goals[goal_idx][1];
            double dz = current_pos.pose.position.z - goals[goal_idx][2];
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            // 如果距离小于 0.3 米，切换到下一个点
            if (dist < 0.3) {
                goal_idx = (goal_idx + 1) % 4; // 在 0,1,2,3 之间循环
                ROS_INFO("Reached Goal! Heading to Goal %d: [%.1f, %.1f, %.1f]", 
                          goal_idx, goals[goal_idx][0], goals[goal_idx][1], goals[goal_idx][2]);
            }

            // 更新发布的位置信息
            pose.pose.position.x = goals[goal_idx][0];
            pose.pose.position.y = goals[goal_idx][1];
            pose.pose.position.z = goals[goal_idx][2];
        }

        local_pose_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}