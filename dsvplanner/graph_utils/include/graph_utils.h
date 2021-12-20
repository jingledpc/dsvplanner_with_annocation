/**************************************************************************
graph_utils.h
graph utility functions

Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
5/25/2019
**************************************************************************/
#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include "graph_utils/Edge.h"
#include "graph_utils/TopologicalGraph.h"
#include "graph_utils/Vertex.h"

namespace graph_utils_ns {
    
    /**
     * @brief Function for getting the shortest path on a graph between two vertexes(两点之间的最短路径)
     * @Input: graph, index of the start vertex and index of the goal vertex
     * @Output: a sequence of vertex ids as the path
     * */ 
    void ShortestPathBtwVertex(std::vector<int> &path, const graph_utils::TopologicalGraph &graph, int start_index, int goal_index);
   
    /**
     * @brief Compute path length, where path represented by sequence of vertex indices（图中路径的长度）
     * */ 
    float PathLength(const std::vector<int> &path, const graph_utils::TopologicalGraph &graph);

    /**
     * @brief Find the vertex idx in graph that is closest (Euclidean distance) to pnt(点)(图中距某点 最近点的索引)
     * */ 
    int GetClosestVertexIdxToPoint(const graph_utils::TopologicalGraph &graph, const geometry_msgs::Point &pnt);
    
    /**
     * @brief  Returns the vertex_index of the first vertex along the path that is beyond threshold distance from the 1st vertex on path.if none exist,
                         it returns the last vertex To be considered, a vertex must have BOTH accumulated and Euclidean distance away. 
                        Euclidean distance only relevant if path wraps back on itself (shouldn't happen if it's a "shortest path") Assumes path is not empty
    */
    int GetFirstVertexBeyondThreshold(const geometry_msgs::Point &start_location, const std::vector<int> &path,
                                    const graph_utils::TopologicalGraph &graph, const float distance_threshold);
    
    /**
     * @brief 
    */
    bool PathCircleDetect(std::vector<int> &path, const graph_utils::TopologicalGraph &graph, int next_vertex_index, geometry_msgs::Point rob_pos);
}

#endif // GRAPH_UTILS_H
