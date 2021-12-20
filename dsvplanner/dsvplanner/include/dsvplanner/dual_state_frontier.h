/**************************************************************************
dual_state_frontier.h
Header of the dual_state_frontier class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/

#ifndef DUAL_STATE_FRONTIER_H
#define DUAL_STATE_FRONTIER_H

#include "dsvplanner/grid.h"
#include "octomap_world/octomap_manager.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
namespace dsvplanner_ns {
    typedef Eigen::Vector3d StateVec;
    class DualStateFrontier {
        typedef std::shared_ptr<DualStateFrontier> Ptr;

        public:
            ros::NodeHandle nh_;
            ros::NodeHandle nh_private_;

            // ROS subscribers
            ros::Subscriber graph_points_sub_;//订阅 图
            message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;//订阅里程计话题
            message_filters::Subscriber<sensor_msgs::PointCloud2> terrain_point_cloud_sub_;//订阅地形点云话题
            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
            typedef message_filters::Synchronizer<syncPolicy> Sync;
            boost::shared_ptr<Sync> sync_;
            // ROS publishers
            ros::Publisher unknown_points_pub_;//发布未知点话题
            ros::Publisher global_frontier_points_pub_;//全局 前沿点 话题
            ros::Publisher local_frontier_points_pub_;//局部前沿点 话题
            ros::Publisher terrain_elev_cloud_pub_;//发布地形高度 话题

            // String constants
            std::string world_frame_id_;//世界坐标系 框架
            std::string sub_odom_topic_;//里程计 话题名
            std::string sub_graph_points_topic_;//
            std::string sub_terrain_point_cloud_topic_;//订阅 地形点云 话题名
            std::string pub_unknown_points_topic_;//发布未知点话题名
            std::string pub_global_frontier_points_topic_;//发布的 全局 前沿点 话题名
            std::string pub_local_frontier_points_topic_;//发布的 局部前沿点 话题名

            // Constants
            double kExecuteFrequency_;//执行频率
            double kFrontierResolution;//前沿点 分辨率
            double kFrontierFilterSize;//前沿点 滤波大小
            double kSearchRadius;//搜索半径
            double kSensorVerticalView;//传感器的竖直视角
            double kSensorHorizontalView;//传感器的水平视角
            double kVehicleHeight;//车的高度
            double kGlobalMaxX;//全局地图的范围的x_max坐标
            double kGlobalMaxY;//全局地图的范围的y_max坐标
            double kGlobalMaxZ;//全局地图的范围的z_max坐标
            double kGlobalMinX;//全局地图的范围的x_min坐标
            double kGlobalMinY;//全局地图的范围的y_min坐标
            double kGlobalMinZ;//全局地图的范围的z_min坐标
            double kFrontierNeighbourSearchRadius;//前沿点邻居搜索半径
            int kEffectiveUnknownNumAroundFrontier;
            bool kEliminateFrontiersAroundRobots;

            StateVec robot_bounding;
            StateVec search_bounding;//搜索框的尺寸

            double kTerrainVoxelSize;//障碍物体素的单位大小
            int kTerrainVoxelHalfWidth;//障碍物体素的宽度的一半
            int kTerrainVoxelWidth; //障碍物体素的宽度

            // Variables

            // general
            ros::Timer executeTimer_;//计时器（以固定频率执行事件）
            std::vector<double> terrain_voxel_elev_; //每个地形体素的高度
            std::vector<int> terrain_voxel_points_num_;//每个地形体素的点的数量
            std::vector<double> terrain_voxel_min_elev_;//每个地形体素的最小高度值
            std::vector<double> terrain_voxel_max_elev_;//每个地形体素的最大高度值
            StateVec robot_position_;//机器人位置
            geometry_msgs::Polygon boundary_polygon_;

            bool planner_status_; // false means exploration and true means relocation
            bool boundaryLoaded_;

            pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());//地形点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_ds = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());//下采样后的地形点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr unknown_points_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());//未知的点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr global_frontier_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());//全局前沿点点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr global_frontier_pcl_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());//全局前沿点点云——pcl
            pcl::PointCloud<pcl::PointXYZ>::Ptr local_frontier_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());//局部前沿点点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr local_frontier_pcl_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());//局部前沿点点云——pcl
            pcl::PointCloud<pcl::PointXYZ>::Ptr cleanedFrontier_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());//TODO:清除的前沿点点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr graphPoints_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());//图规划 点云
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_elev_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());//地形高度点云，z=0，intensity为最小高度（地面的高度）
            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_ = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());//kdTree 点云
            pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr global_frontiers_kdtree_ = pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());//全局前沿点点云 kdTree

            volumetric_mapping::OctomapManager *manager_;//八叉树地图
            OccupancyGrid *grid_;//栅格地图

            // General Functions
            
            /**
            * @brief 前沿点是否在限制范围内（加入的限制框、最大全局范围）
            */
            bool FrontierInBoundry(octomap::point3d point) const;
            
            /**
            * @brief  遍历center周围 box中的每一个点，将八叉树中不存在的点加入到unknown_points_中，将符合要求的前沿点经降采样加入到local_frontier_中
            */
            void getUnknowPointcloudInBoundingBox(const StateVec &center, const StateVec &bounding_box_size);

            /**
            * @brief 获取前沿点
            */
            void getFrontiers();

            /**
            * @brief 将local_frontier_中符合规则（比全局高）的加入到local_frontier_pcl_和global_frontier_中，并下采样更新到local_frontier_中
            */
            void localFrontierUpdate(StateVec &center);
            
            /**
            * @brief 设置规划器的状态
            */
            void setPlannerStatus(bool status);
            
            /**
            * @brief 检测point点是不是前沿点（在八叉树地图中 前后左右四个方向存在"未被占用"的点 且含有一定量的"未知点"）
            */
            bool frontierDetect(octomap::point3d point) const;
            
            /**
            * @brief 计算点所在 地形体素格子中车高度 
            */
            double getZvalue(double x_position, double y_position);

            /**
            * @brief 是否属于被清理的前沿点（cleanedFrontier_中 是否 存在 在point附近(3m)的点）
            */
            bool isCleanedFrontier(pcl::PointXYZ point);
            
            /**
            * @brief point点 是否 为机器人可以到达的点（满足与机器人位置 1较近2在可视范围内 3 点间没有障碍物（八叉树））
            */
            bool inSensorRangeofRobot(StateVec point);
            
            /**
            * @brief （图规划中有可行进点）在图规划的定点集中 “存在” 点 满足与check_point点 1较近2在可视范围内 3 点间没有障碍物（八叉树和栅格）
            */
            bool inSensorRangeofGraphPoints(StateVec point);
            
            /**
            * @brief 清理所有global_frontier_、和local_frontier_的前沿点，发布unknown_pcl, local_frontier_pcl, global_frontier_pcl中的前沿点
            */
            void cleanAllUselessFrontiers();
            
            /**
            * @brief 将global_frontier_中非孤立的前沿点保存到global_frontier_pcl_、global_frontier_中，
            */
            void globalFrontiersNeighbourCheck();
            
            /**
            * @brief 将gloabal_frontier_中 非清理前沿点 且 符合规则的前沿点存在到 global_frontier_pcl中，并下采样返回到gloabal_frontier_中
            */
            void gloabalFrontierUpdate();
            
            /**
            * @brief 发布unknown_pcl, local_frontier_pcl, global_frontier_pcl中的前沿点
            */
            void publishFrontiers();
            
            /**
            * @brief 向cleanedFrontier_中添加点point
            */
            void updateToCleanFrontier(pcl::PointXYZ point);
            
            /**
            * @brief 设置人定的探索范围
            */
            void setBoundary(const geometry_msgs::PolygonStamped &boundary);
            
            // Functions for terrain elevation map
            /**
            * @brief 通过遍历下采样后的地形点云，更新每个地形体素对应的最小高度和最大高度
            */
            void updateTerrainMinElevation();
            
            /**
            * @brief 更新地面地图的高度为最近点的地面高度（给未知区域）
            */
            void updateTerrainElevationForUnknow();
            
            /**
            * @brief （已知是体素中有点）将每个体素的可通行情况 更新到 地形点云中的高度，同时将地形的体素的高度更新最小的高度（不可通过为极大值）
            */
            void updateTerrainElevationForKnown();
            
            /**
            * @brief 获得地形体素的高度
            */
            std::vector<double> getTerrainVoxelElev();

            

            // Callback Functions
            void terrainCloudAndOdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg, const sensor_msgs::PointCloud2::ConstPtr &terrain_msg);
            void graphPointsCallback(const sensor_msgs::PointCloud2 &graph_msg);

        public:
            DualStateFrontier(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, volumetric_mapping::OctomapManager *manager, OccupancyGrid *grid);
            
            /**
            * @brief 读取并设置参数
            */
            bool readParameters();

            /**
            * @brief 初始化
            */
            bool initialize();

            /**
            * @brief 不断执行 获得前沿点 的操作
            */
            void execute(const ros::TimerEvent &e);
            ~DualStateFrontier();
    };
}
#endif // DUAL_STATE_FRONTIER_H