/**************************************************************************
dual_state_graph.h
Header of the dual_state_graph class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/
#ifndef DUAL_STATE_GRAPH_H
#define DUAL_STATE_GRAPH_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "dsvplanner/grid.h"
#include "graph_planner/GraphPlannerStatus.h"
#include "graph_utils/TopologicalGraph.h"
#include "octomap_world/octomap_manager.h"

namespace dsvplanner_ns {
    class DualStateGraph {
        public:
            typedef std::shared_ptr<DualStateGraph> Ptr;
            ros::NodeHandle nh_;
            ros::NodeHandle nh_private_;

            // ROS subscribers
            ros::Subscriber key_pose_sub_;//订阅关键点话题
            ros::Subscriber graph_planner_path_sub_;//订阅图规划路径的话题
            ros::Subscriber graph_planner_status_sub_;//订阅图规划状态的话题

            // ROS publishers
            ros::Publisher local_graph_pub_;//发布局部地图的话题
            ros::Publisher global_graph_pub_;//发布全局第地图的话题
            ros::Publisher graph_points_pub_;//发布图点的话题

            // String constants
            std::string world_frame_id_;
            std::string pub_local_graph_topic_;//发布局部地图的话题名
            std::string pub_global_graph_topic_;//发布全局第地图的话题名
            std::string pub_global_points_topic_;//发布图点的话题名
            std::string sub_keypose_topic_;//订阅关键点的话题名
            std::string sub_path_topic_;//订阅图规划路径的话题名
            std::string sub_graph_planner_status_topic_;//订阅图规划状态的话题名

            // Constants
            bool kCropPathWithTerrain;
            double kConnectVertexDistMax;//顶点之间进行连线的距离最大值
            double kDistBadEdge;
            double kDegressiveCoeff;//计算增益是的衰减因子
            double kDirectionCoeff;//计算增益是的方向因子
            double kExistingPathRatioThreshold;//
            double kExistingPathRatioThresholdGlobal;//判定是否可以视为存在捷径的比例阈值
            double kLongEdgePenaltyMultiplier;
            double kMinVertexDist;//用来判断顶点之间是否距离过近的阈值
            double kMaxLongEdgeDist;//图中边的最大长度值
            double kMaxVertexAngleAlongZ;//图中对应 z方向上 允许的最大漂移角度 
            double kMaxVertexDiffAlongZ;//图中对应 z方向上 允许的最大漂移距离 
            double kMaxDistToPrunedRoot;
            double kMaxPrunedNodeDist;
            double kSurroundRange;//被视为在附近 的范围
            double kMinGainRange;//
            double kMinDistanceToRobotToCheck;//
            Eigen::Vector3d robot_bounding;

            // Variables
            Eigen::Vector3d explore_direction_;//探索的方向
            geometry_msgs::Point robot_pos_;//机器人位置
            geometry_msgs::Pose tempvertex_;//机器人临时姿态
            graph_utils::TopologicalGraph global_graph_;//全局图
            graph_utils::TopologicalGraph local_graph_;//局部图
            graph_utils::TopologicalGraph pruned_graph_;//剪枝后的图（从局部地图中更新出的）
            graph_planner::GraphPlannerStatus graph_planner_status_;//图规划的状态 0：关闭 1：正在进行
            pcl::PointCloud<pcl::PointXYZ>::Ptr graph_point_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
            std::vector<int> gainID_;//储存者 总增益大于0 的顶点
            volumetric_mapping::OctomapManager *manager_;//八叉树地图
            OccupancyGrid *grid_;//栅格地图

            bool planner_status_; // false： 局部规划  true： 全局规划
            int track_localvertex_idx_;//追踪的局部顶点 索引
            int track_globalvertex_idx_;//追踪的全局部顶点 索引
            int prev_track_vertex_idx_;//上次追踪的顶点 索引
            int prev_track_keypose_vertex_idx_;//上次追踪的关键点索引

            int localEdgeSize_;//局部边的数量
            int globalEdgeSize_;//全局边的数量
            int best_vertex_id_;//最佳顶点的 索引
            double best_gain_;//最佳增益
            double DTWValue_;
            double robot_yaw_;//机器人的俯仰角

            // Feedback Functions
            /**
             * @brief 计算每个顶点的增益；更新最佳顶点和对应的增益，并筛选出增益大于的顶点
            */
            double getGain(geometry_msgs::Point robot_position);
            
            /**
             * @brief 获得局部图中 顶点的数量
            */
            int getLocalVertexSize();
            
            /**
             * @brief 获得局部图中 边的数量
            */
            int getLocalEdgeSize();
            
            /**
             * @brief 获得全局图中 顶点的数量
            */
            int getGlobalVertexSize();
            
            /**
             * @brief 获得全局图中 边的数量
            */
            int getGlobalEdgeSize();
            
            /**
             * @brief 获取局部最佳顶点的位置
            */
            geometry_msgs::Point getBestLocalVertexPosition();
            /**
             * @brief 获取全局最佳顶点的位置
            */
            geometry_msgs::Point getBestGlobalVertexPosition();
            /**
             * @brief 获取局部探索的方向（由局部图中0索引点 指向 局部最佳顶点）
            */
            Eigen::Vector3d getExploreDirection();

            // General Functions
            /**
             * @brief 针对 graph中的起止顶点 添加 对应的边（包含属性：到达的顶点、边长(cost)、是否为追踪边）（不检查可通过性）localEdgeSize_计数加1
             * @param start_vertex_idx 起始顶点的索引
             * @param end_vertex_idx 终止顶点的索引
             * @param graph 待添加边的图（一般局部）
            */
            void addEdgeWithoutCheck(int start_vertex_idx, int end_vertex_idx, graph_utils::TopologicalGraph &graph);
            
            /**
             * @brief 添加边 针对 graph中的起止顶点 添加 对应的边（包含属性：到达的顶点、边长(cost)、是否为追踪边）（检查可通过性：可通行，且路径不存在 或 需要建立捷径）
             * @param start_vertex_idx 起始顶点的索引
             * @param end_vertex_idx 终止顶点的索引
             * @param graph 待添加边的图（一般局部）
            */
            void addEdge(int start_vertex_idx, int end_vertex_idx, graph_utils::TopologicalGraph &graph);
            
            /**
             * @brief 将vertex_msg作为顶点加到 graph图中，并建立其对应的父顶点和边(父子边，邻居边)，并更新局部地图追踪的顶点索引
             * @param graph 待加入顶点的图（剪枝图、局部图）
            */
            void addNewLocalVertex(geometry_msgs::Pose &vertex_msg, graph_utils::TopologicalGraph &graph);
            
            /**
             * @brief 根据vertex_msg向graph中添加顶点（不添加边），并更新局部地图追踪的顶点索引
             * @param vertex_msg 顶点信息
             * @param graph 待添加顶点的图（一般局部）
            */
            void addNewLocalVertexWithoutEdge(geometry_msgs::Pose &vertex_msg, graph_utils::TopologicalGraph &graph);
            
            /**
             * @brief 如果vertex_msg点与全局中所有的点保持在合理的距离内，将该点 到全局图中，并添加其和上次追踪顶点之间的边（非追踪边）并更新prev_track_vertex_idx_
            */
            void addNewLocalVertexWithoutDuplicates(geometry_msgs::Pose &vertex_msg, graph_utils::TopologicalGraph &graph);
            
            /**
             * @brief 如果graph中不存在与vertex_msg较近的点 且 该点非远离所有点，且两点间的八叉树地图可通行，在局部图中加入该顶点，并更新上次追踪的顶点 索引
             * @param vertex_msg 待检测顶点
             * @param graph 带加入顶点的图（一般剪枝后的图）
            */
            void addNewPrunedVertex(geometry_msgs::Pose &vertex_msg, graph_utils::TopologicalGraph &graph);
            
            /**
             * @brief 向图中添加由start_vertex_idx到end_vertex_idx的边（通常是父子边）globalEdgeSize_计数加1
             * @param start_vertex_idx 起止顶点的索引
             * @param end_vertex_idx 终止顶点的索引
             * @param trajectory_edge 是否是追踪过的边（两个关键顶点之间的）
            */
            void addGlobalEdgeWithoutCheck(int start_vertex_idx, int end_vertex_idx, bool trajectory_edge);
            
            /**
             * @brief 如果起止顶点之间可通行，且路径不存在 或 需要建立捷径，添加边
            */
            void addGlobalEdge(int start_vertex_idx, int end_vertex_idx);
            
            /**
             * @brief 将vertex_msg作为顶点加到 全局图中，并建立其对应的父顶点和边(父子边，邻居边)，并更新全局地图追踪的顶点索引
            */
            void addNewGlobalVertex(geometry_msgs::Pose &vertex_msg);
            
            /**
             * @brief 如果vertex_msg点与全局中所有的点保持在合理的距离内，将该点 到全局图中，并添加其和上次追踪顶点之间的边（非追踪边）并更新prev_track_vertex_idx_
            */
            void addNewGlobalVertexWithoutDuplicates(geometry_msgs::Pose &vertex_msg);
            
            /**
             * @brief 如果vertex_msg点与全局中所有的点保持在合理的距离内，将该点视为"当前追踪关键顶点"加入到全局图中，并添加其和上次追踪顶点之间的边
            */
            void addNewGlobalVertexWithKeypose(geometry_msgs::Pose &vertex_msg);
           
            /**
             * @brief 清空局部图
            */
            void clearLocalGraph();
            
            /**
             * @brief 采用DTW方法 计算 “图中路径” 与 “直接向目标点行驶路径 ”的  差异大小，赋予DTWValue_（//TODO: 后续验证）
             *                  两个序列间的距离
            */
            void DTW(std::vector<int> path, geometry_msgs::Point robot_position);
            /**
             * @brief 找到从root出发 到达局部图中每一个顶点（在剪枝图范围内）的最短路径 所经过的点，并当他们添加到剪枝图中
            */
            void pruneGraph(geometry_msgs::Point root);
            /**
             * @brief 八叉树地图改变 导致某些顶点间的不可达，清除全局图中 所有从其他点连向改点的边
            */
            void pruneGlobalGraph();
            /**
             * @brief 发布当前的局部图，并将局部图中的顶点添加到graph_point中进行发布（用于前沿点检测）
            */
            void publishLocalGraph();
            /**
             * @brief 发布当前的 global graph
            */
            void publishGlobalGraph();
            
            /**
             * @brief 设置当前规划其的状态
            */
            void setCurrentPlannerStatus(bool status);
            /**
             * @brief 
            */
            void updateGlobalGraph();
            
            /**
             * @brief 计算由局部图中 0索引点 指向 局部最佳顶点 的单位向量
            */
            void updateExploreDirection();
            
            /**
             * @brief 两个顶点是否满足 图对应z的要求（z轴偏移与俯仰角偏移）
            */
            bool zCollisionCheck(int start_vertex_idx, int end_vertex_idx, graph_utils::TopologicalGraph graph);

            // Callback Functions
            /**
             * @brief 将里程计信息转换为位置信息，并作为关键点添加到全局图中
            */
            void keyposeCallback(const nav_msgs::Odometry::ConstPtr &msg);
            
            /**
             * @brief 检测路径每一步 是否 可以通过，如果不能删除对应的边（在全局图和局部图中）
            */
            void pathCallback(const nav_msgs::Path::ConstPtr &graph_path);
            
            /**
             * @brief //图规划状态 由 关闭 转为 正在进行，且规划处于局部规划时，进行全局图(global_graph_)的修剪和发布
            */
            void graphPlannerStatusCallback(const graph_planner::GraphPlannerStatusConstPtr &status);

        public:
            /**
             * @brief 读取参数及初始化
            */
            DualStateGraph(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, volumetric_mapping::OctomapManager *manager, OccupancyGrid *grid);
            /**
             * @brief 读取参数
            */
            bool readParameters();
            /**
             * @brief 初始化
            */
            bool initialize();
            /**
             * @brief 执行publishLocalGraph()
            */
            bool execute();
            ~DualStateGraph();
    };
}
#endif // DUAL_STATE_GRAPH_H
