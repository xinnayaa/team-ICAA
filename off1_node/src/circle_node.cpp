#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "circle_node");
    ros::NodeHandle nh;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    ros::Rate rate(20.0);
    
    // 等待连接
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    
    // 圆形参数
    const double CENTER_X = 10.0;
    const double CENTER_Y = 10.0;
    const double RADIUS = 8.0;
    const double HEIGHT = 3.0;
    const double SPEED = 0.3;  // 弧度/秒 (约 17度/秒，完整一圈约 21秒)
    
    geometry_msgs::PoseStamped pose;
    double angle = 0.0;
    double angle_step = SPEED / 20.0;
    
    // 起始点
    pose.pose.position.x = CENTER_X + RADIUS;
    pose.pose.position.y = CENTER_Y;
    pose.pose.position.z = HEIGHT;
    
    // 预发布
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pose_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    // 切模式和解锁
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
    
    ROS_INFO("Starting circle trajectory...");
    
    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Armed");
            }
            last_request = ros::Time::now();
        }
        
        if (current_state.mode == "OFFBOARD" && current_state.armed) {
            // 更新角度
            angle += angle_step;
            if (angle >= 2 * M_PI) angle -= 2 * M_PI;
            
            // 计算圆形上的点
            pose.pose.position.x = CENTER_X + RADIUS * cos(angle);
            pose.pose.position.y = CENTER_Y + RADIUS * sin(angle);
            
            local_pose_pub.publish(pose);
        } else {
            local_pose_pub.publish(pose);
        }
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}