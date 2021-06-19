// 二元边 两个位姿节点之间的距离越小,那么他的误差越小,这就是为什么之前要把局部路径规划点进行重新分割
// 分割成在dt_ref时间以最快速度运行得到的间隔点
#ifndef EDGE_SHORTEST_PATH_H_
#define EDGE_SHORTEST_PATH_H_

#include <float.h>

#include <base_local_planner/BaseLocalPlannerConfig.h>

#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>

#include <Eigen/Core>

namespace teb_local_planner {

/**
 * @class EdgeShortestPath
 * @brief Edge defining the cost function for minimizing the Euclidean distance between two consectuive poses.
 *
 * @see TebOptimalPlanner::AddEdgesShortestPath
 */
class EdgeShortestPath : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose> {
public:
  /**
   * @brief Construct edge.
   */
  EdgeShortestPath() { this->setMeasurement(0.); }

  /**
   * @brief Actual cost function
   */
  void computeError() {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeShortestPath()");
    const VertexPose *pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose *pose2 = static_cast<const VertexPose*>(_vertices[1]);
    _error[0] = (pose2->position() - pose1->position()).norm();

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeShortestPath::computeError() _error[0]=%f\n", _error[0]);
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // end namespace

#endif /* EDGE_SHORTEST_PATH_H_ */
