/*
exploration_with_graph_planner.cpp
the interface for drrt planner

Created and maintained by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
 */

#include <chrono>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_datatypes.h>

#include "dsvplanner/clean_frontier_srv.h"
#include "dsvplanner/dsvplanner_srv.h"
#include "graph_planner/GraphPlannerCommand.h"
#include "graph_planner/GraphPlannerStatus.h"

using namespace std::chrono;
#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

geometry_msgs::Point wayPoint;//目标点
geometry_msgs::Point wayPoint_pre;
geometry_msgs::Point goal_point;//当前阶段的目标点，从服务器中获得（回家模式下为home点）
geometry_msgs::Point home_point;//机器人探索的出发点
graph_planner::GraphPlannerCommand graph_planner_command;
std_msgs::Float32 effective_time;//全局规划器本次规划耗时（s）
std_msgs::Float32 total_time;//运行总时长

bool simulation = false;   // control whether use graph planner to follow path
bool begin_signal = false; // trigger the planner
bool gp_in_progress = false; //全局状态（正在进行）
bool wp_state = false; //获得新的waypoint
bool return_home = false;//开始回家的标志
double current_odom_x = 0;//当前车里程计的x坐标
double current_odom_y = 0;//当前车里程计的y坐标
double current_odom_z = 0;//当前车里程计的z坐标
double previous_odom_x = 0;//上一次车移动后里程计的x坐标
double previous_odom_y = 0;//上一次车移动后里程计的y坐标
double previous_odom_z = 0;//上一次车移动后里程计的z坐标
double dtime = 0.0;//给以规划器规划的时间
double init_x = 2;//初始目标点x
double init_y = 0;//初始目标点y
double init_z = 2;//初始目标点z
double init_time = 2;//初始时长
double return_home_threshold = 1.5;//回到家的阈值
double robot_moving_threshold = 6;//视机器人位置移动的阈值
std::string map_frame = "map";//创建世界地图的frame
std::string waypoint_topic = "/way_point";
std::string cmd_vel_topic = "/cmd_vel";
std::string gp_command_topic = "/graph_planner_command";
std::string effective_plan_time_topic = "/runtime";
std::string total_plan_time_topic = "/totaltime";
std::string gp_status_topic = "/graph_planner_status";
std::string odom_topic = "/state_estimation";
std::string begin_signal_topic = "/start_exploring";
std::string stop_signal_topic = "/stop_exploring";

tf::StampedTransform transformToMap;

steady_clock::time_point plan_start;//全局规划器开始的时刻
steady_clock::time_point plan_over;//全局规划器结束的时刻
steady_clock::duration time_span;//全局规划器消耗时长(c++内部)

ros::Publisher waypoint_pub;
ros::Publisher gp_command_pub;
ros::Publisher effective_plan_time_pub;
ros::Publisher total_plan_time_pub;
ros::Subscriber gp_status_sub;
ros::Subscriber waypoint_sub;
ros::Subscriber odom_sub;
ros::Subscriber begin_signal_sub;//发布开始探索的信号
ros::Publisher stop_signal_pub;//发布停止探索的标志

/**
 * @brief 订阅全局的状态
*/
void gp_status_callback(const graph_planner::GraphPlannerStatus::ConstPtr &msg) {
    if (msg->status == graph_planner::GraphPlannerStatus::STATUS_IN_PROGRESS)
        gp_in_progress = true;
    else {
        gp_in_progress = false;
    }
}

/**
 * @brief 订阅发布的目标点
*/
void waypoint_callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
    wayPoint = msg->point;
    wp_state = true;
}

/**
 * @brief 订阅当天odom的状态，并存储到tf中（相对于map的）
*/
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
    current_odom_x = msg->pose.pose.position.x;
    current_odom_y = msg->pose.pose.position.y;
    current_odom_z = msg->pose.pose.position.z;

    transformToMap.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                        msg->pose.pose.position.y,
                                        msg->pose.pose.position.z));
    transformToMap.setRotation(tf::Quaternion(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
}

/**
 * @brief 订阅开始探索的信号
*/
void begin_signal_callback(const std_msgs::Bool::ConstPtr &msg) {
    begin_signal = msg->data;
}

/**
 * @brief 判断车是否移动
*/
bool robotPositionChange() {
    double dist = sqrt(
        (current_odom_x - previous_odom_x) * (current_odom_x - previous_odom_x) +
        (current_odom_y - previous_odom_y) * (current_odom_y - previous_odom_y) +
        (current_odom_z - previous_odom_z) * (current_odom_z - previous_odom_z));
    if (dist < robot_moving_threshold)
        return false;
    previous_odom_x = current_odom_x;
    previous_odom_y = current_odom_y;
    previous_odom_z = current_odom_z;
    return true;
}

/**
 * @brief 保证机器人向前行驶一段距离，以排除home的干扰的初始化操作
*/
void initilization() {
    tf::Vector3 vec_init(init_x, init_y, init_z);
    tf::Vector3 vec_goal;
    vec_goal = transformToMap * vec_init;
    geometry_msgs::PointStamped wp;
    wp.header.frame_id = map_frame;
    wp.header.stamp = ros::Time::now();
    wp.point.x = vec_goal.x();
    wp.point.y = vec_goal.y();
    wp.point.z = vec_goal.z();
    home_point.x = current_odom_x;
    home_point.y = current_odom_y;
    home_point.z = current_odom_z;

    ros::Duration(0.5).sleep(); // wait for sometime to make sure waypoint can be
                                // published properly

    waypoint_pub.publish(wp);
    bool wp_ongoing = true;
    int init_time_count = 0;
    while (wp_ongoing) {//让机器人向前行走一段距离，直到远离家且到达目标点附近（后行驶足够长的时间）
        init_time_count++;
        ros::Duration(0.1).sleep();
        ros::spinOnce();
        vec_goal = transformToMap * vec_init;
        wp.point.x = vec_goal.x();
        wp.point.y = vec_goal.y();
        wp.point.z = vec_goal.z();
        waypoint_pub.publish(wp);

        double dist = sqrt((wp.point.x - current_odom_x) * (wp.point.x - current_odom_x) + (wp.point.y - current_odom_y) * (wp.point.y - current_odom_y));
        double dist_to_home = sqrt((home_point.x - current_odom_x) * (home_point.x - current_odom_x) + (home_point.y - current_odom_y) * (home_point.y - current_odom_y));
        if (dist < 0.5 && dist_to_home > 0.5)//到目标点附近并远离家
            wp_ongoing = false;
        if (init_time_count >= init_time / 0.1 && dist_to_home > 0.5)//行驶了足够长的时间，且远离家
            wp_ongoing = false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "exploration");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    waypoint_pub = nh.advertise<geometry_msgs::PointStamped>(waypoint_topic, 5);
    gp_command_pub = nh.advertise<graph_planner::GraphPlannerCommand>(gp_command_topic, 1);
    effective_plan_time_pub = nh.advertise<std_msgs::Float32>(effective_plan_time_topic, 1);
    total_plan_time_pub = nh.advertise<std_msgs::Float32>(total_plan_time_topic, 1);

    gp_status_sub = nh.subscribe<graph_planner::GraphPlannerStatus>(gp_status_topic, 1, gp_status_callback);
    waypoint_sub = nh.subscribe<geometry_msgs::PointStamped>(waypoint_topic, 1, waypoint_callback);
    odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1, odom_callback);
    begin_signal_sub = nh.subscribe<std_msgs::Bool>(begin_signal_topic, 1, begin_signal_callback);

    stop_signal_pub = nh.advertise<std_msgs::Bool>(stop_signal_topic, 1);

    nhPrivate.getParam("simulation", simulation);
    nhPrivate.getParam("/interface/dtime", dtime);
    nhPrivate.getParam("/interface/initX", init_x);
    nhPrivate.getParam("/interface/initY", init_y);
    nhPrivate.getParam("/interface/initZ", init_z);
    nhPrivate.getParam("/interface/initTime", init_time);
    nhPrivate.getParam("/interface/returnHomeThres", return_home_threshold);
    nhPrivate.getParam("/interface/robotMovingThres", robot_moving_threshold);
    nhPrivate.getParam("/interface/tfFrame", map_frame);
    nhPrivate.getParam("/interface/autoExp", begin_signal);
    nhPrivate.getParam("/interface/waypointTopic", waypoint_topic);
    nhPrivate.getParam("/interface/cmdVelTopic", cmd_vel_topic);
    nhPrivate.getParam("/interface/graphPlannerCommandTopic", gp_command_topic);
    nhPrivate.getParam("/interface/effectivePlanTimeTopic", effective_plan_time_topic);
    nhPrivate.getParam("/interface/totalPlanTimeTopic", total_plan_time_topic);
    nhPrivate.getParam("/interface/gpStatusTopic", gp_status_topic);
    nhPrivate.getParam("/interface/odomTopic", odom_topic);
    nhPrivate.getParam("/interface/beginSignalTopic", begin_signal_topic);
    nhPrivate.getParam("/interface/stopSignalTopic", stop_signal_topic);

    ros::Duration(1.0).sleep();
    ros::spinOnce();
    //等待开始信号
    while (!begin_signal) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        ROS_INFO("Waiting for Odometry");
    }

    ROS_INFO("Starting the planner: Performing initialization motion");
    initilization();
    ros::Duration(1.0).sleep();

    std::cout << std::endl << "\033[1;32mExploration Started\033[0m\n" << std::endl;
    total_time.data = 0;
    plan_start = steady_clock::now();
    // Start planning: The planner is called and the computed goal point sent to
    // the graph planner.
    int iteration = 0;
    while (ros::ok()) {
        if (!return_home) {//
            if (iteration != 0) {
                for (int i = 0; i < 8; i++) {
                    printf(cursup);
                    printf(cursclean);
                }
            }
            std::cout << "Planning iteration " << iteration << std::endl;
            dsvplanner::dsvplanner_srv planSrv;
            dsvplanner::clean_frontier_srv cleanSrv;
            planSrv.request.header.stamp = ros::Time::now();
            planSrv.request.header.seq = iteration;
            planSrv.request.header.frame_id = map_frame;
            //根据服务器的应答进行相应的规划，主要是回家和探索的区分
            if (ros::service::call("drrtPlannerSrv", planSrv)) {//服务器回应
                if (planSrv.response.goal.size() == 0) { // usually the size should be 1 if planning successfully没有目标点，则等待
                    ros::Duration(1.0).sleep();
                    continue;
                }

                if (planSrv.response.mode.data == 2) {//回应的模式为2表示探索完成，进行回家操作
                    return_home = true;
                    goal_point = home_point;
                    std::cout << std::endl << "\033[1;32mExploration completed, returning home\033[0m" << std::endl << std::endl;
                    effective_time.data = 0;
                    effective_plan_time_pub.publish(effective_time);
                } else {//探索模式
                    return_home = false;
                    goal_point = planSrv.response.goal[0];
                    //记录规划耗时
                    plan_over = steady_clock::now();
                    time_span = plan_over - plan_start;
                    effective_time.data = float(time_span.count()) *
                                        steady_clock::period::num /
                                        steady_clock::period::den;
                    effective_plan_time_pub.publish(effective_time);
                }
                total_time.data += effective_time.data;
                total_plan_time_pub.publish(total_time);

                if (!simulation) { // when not in simulation mode, the robot will go to the goal point according to graph planner
                    //向全局规划器中，发布目标点
                    graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
                    graph_planner_command.location = goal_point;
                    gp_command_pub.publish(graph_planner_command);
                    ros::Duration(dtime).sleep(); // give sometime to graph planner for searching path to goal point
                    ros::spinOnce();              // update gp_in_progree

                    int count = 200;
                    previous_odom_x = current_odom_x;
                    previous_odom_y = current_odom_y;
                    previous_odom_z = current_odom_z;
                    while (gp_in_progress) {      // if the waypoint keep the same for 20 (200*0.1) 20秒内车没有移动，表示目标点不可到达，移除该目标点
                        ros::Duration(0.1).sleep(); // seconds, then give up the goal
                        wayPoint_pre = wayPoint;
                        ros::spinOnce();
                        bool robotMoving = robotPositionChange();
                        if (robotMoving) {//20s内车移动了，重置标志位
                            count = 200;
                        } else {
                            count--;
                        }
                        if (count <= 0) { // when the goal point cannot be reached, clean its correspoinding frontier if there is
                            cleanSrv.request.header.stamp = ros::Time::now();
                            cleanSrv.request.header.frame_id = map_frame;
                            ros::service::call("cleanFrontierSrv", cleanSrv);
                            ros::Duration(0.1).sleep();
                            break;
                        }
                    }
                    //FIXME：全局规划完成，进行。。。
                    graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_DISABLE;
                    gp_command_pub.publish(graph_planner_command);
                } else { // simulation mode is used when testing this planning algorithm
                            // with bagfiles where robot will
                    // not move to the planned goal. When in simulation mode, robot will
                    // keep replanning every two seconds
                    //当使用记录的文件 测试该规划算法时，使用模拟模式。机器人不会移动到规划目标。在模拟模式下，机器人将每两秒钟重新规划一次 
                    for (size_t i = 0; i < planSrv.response.goal.size(); i++) {
                        graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
                        graph_planner_command.location = planSrv.response.goal[i];
                        gp_command_pub.publish(graph_planner_command);
                        ros::Duration(2).sleep();
                        break;
                    }
                }
                plan_start = steady_clock::now();
            } else {//服务器没有回应则等待
                std::cout << "Cannot call drrt planner." << std::flush;

                ros::Duration(1.0).sleep();
            }
            iteration++;
        } else {//进行回家操作
            ros::spinOnce();
            if (fabs(current_odom_x - home_point.x) +  fabs(current_odom_y - home_point.y) + fabs(current_odom_z - home_point.z) 
                    <= return_home_threshold) {//到家了，停止一切操作
                printf(cursclean);
                std::cout << "\033[1;32mReturn home completed\033[0m" << std::endl;
                printf(cursup);
                std_msgs::Bool stop_exploring;
                stop_exploring.data = true;
                stop_signal_pub.publish(stop_exploring);
            } else {//没到家了，回家操作
                while (!gp_in_progress) {//图规划完成
                    ros::spinOnce();
                    ros::Duration(2.0).sleep();

                    graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
                    graph_planner_command.location = goal_point;//该处目标点是home点
                    gp_command_pub.publish(graph_planner_command);
                }
            }
            ros::Duration(0.1).sleep();
        }
    }
}
