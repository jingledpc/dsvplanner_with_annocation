/**************************************************************************
grid.h
Header of the OccupancyGrid class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
05/25/2020

**************************************************************************/

#ifndef GRID_H_
#define GRID_H_

#include "dsvplanner/drrt_base.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace Eigen;
namespace dsvplanner_ns {
    typedef Vector3d StateVec;
    typedef Vector2i GridIndex;
    class OccupancyGrid {
        typedef std::shared_ptr<OccupancyGrid> Ptr;

        public:
            OccupancyGrid(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
            ~OccupancyGrid();

            /**
             * @brief 设置地图参数
            */
            bool readParameters();
            
            /**
             * @brief 地图初始化，包含设置参数、初始化ros话题、设置地图大小
            */
            bool initialize();

            public:
            ros::NodeHandle nh_;
            ros::NodeHandle nh_private_;

            // ROS subscribers
            message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;//订阅里程计点云
            message_filters::Subscriber<sensor_msgs::PointCloud2> terrain_point_cloud_sub_;//订阅地形点云
            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
            typedef message_filters::Synchronizer<syncPolicy> Sync;
            boost::shared_ptr<Sync> sync_;

            // ROS publishers
            ros::Publisher grid_cloud_pub_;//发布栅格地图（点云式）

            // String constants
            std::string world_frame_id_;//世界frame
            std::string sub_odom_topic_;//订阅odom的话题
            std::string sub_terrain_point_cloud_topic_;//订阅地形点云的话题
            std::string pub_grid_points_topic_;//发布地图点的话题

            // Constants
            double kMapWidth;//地图的宽
            double kGridSize;//地图的大小
            double kDownsampleSize;//下采样的大小
            double kObstacleHeightThre;//障碍物的高度阈值
            double kFlyingObstacleHeightThre;//飞行障碍物的高度阈值
            double kCollisionCheckX;//碰撞检测x值
            double kCollisionCheckY;//碰撞检测y值

            // Variables
            enum gridStatus { unknown = 0, free = 1, occupied = 2, near_occupied = 3 };//栅格的状态
            std::vector<std::vector<int>> gridState_;//栅格地图
            int map_width_grid_num_;//栅格地图的宽（格数）
            int map_half_width_grid_num_;//栅格地图的宽的一半（格数）

            ros::Time terrain_time_;
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());//地形点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_ds = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());//地形点云下采样
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_traversable_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());//地形点云中可通过的点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_obstacle_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());//地形点云中视为障碍物的点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr grid_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

            StateVec robot_position_;//机器人位置

            // Functions

            /**
             * @brief 以p对应栅格的中心生成点
            */
            geometry_msgs::Point getPoint(GridIndex p);

            /**
             * @brief 计算点在栅格地图中的坐标
            */
            GridIndex getIndex(StateVec point);
            
            /**
             * @brief 将栅格地图的所有点，置为未知
            */
            void clearGrid();

            /**
             * @brief 根据terrain_cloud_obstacle_和terrain_cloud_traversable_更新栅格的状态
            */
            void updateGrid();

            /**
             * @brief 将每个栅格中的状态填入到点的属性中，并中心位置为点坐标发布出去
            */
            void publishGridMap();

            /**
             * @brief 检测起点和终点之间是否有障碍物（射线法）
            */
            bool collisionCheckByTerrainWithVector(StateVec origin_point, StateVec goal_point);

            /**
             * @brief 检测起点和终点之间是否有障碍物
            */
            bool collisionCheckByTerrain(geometry_msgs::Point origin, geometry_msgs::Point goal);
            
            /**
             * @brief 点是否在指定的范围内
            */
            bool InRange(const GridIndex sub, const GridIndex max_sub, const GridIndex min_sub);
            
            /**
             * @brief 根据对应点对应的附近是否有被占用的点
            */
            bool updateFreeGridWithSurroundingGrids(int indx, int indy);
            
            /**
             * @brief signmod函数 小于0为-1，大于0为1
            */
            int signum(int x);

            /**
             * @brief 
            */
            double intbound(double s, double ds);

            /**
             * @brief 取余，采用fmod进行了两次取余，//FIXME:不知道两次的意义
            */
            double mod(double value, double modulus);

            /**
             * @brief 射线法计算起点和终点之间所有的点的坐标
            */
            std::vector<GridIndex> rayCast(GridIndex origin, GridIndex goal, GridIndex max_grid, GridIndex min_grid);

            // Callback Functions
            /**
             * @brief 将获得的地形点云分类为可通过和障碍物
            */
            void terrainCloudAndOdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg, const sensor_msgs::PointCloud2::ConstPtr &terrain_msg);
    };
}

#endif // GRID_H
