// 一元边 与通过点相关的边
// cost便是离当前路径点最近的通过点与当前点最近的距离
#ifndef EDGE_VIA_POINT_H_
#define EDGE_VIA_POINT_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>

#include "g2o/core/base_unary_edge.h"


namespace teb_local_planner
{

/**
 * @class EdgeViaPoint
 * @brief Edge defining the cost function for pushing a configuration towards a via point
 * 
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min  dist2point \cdot weight \f$. \n
 * \e dist2point denotes the distance to the via point. \n
 * \e weight can be set using setInformation(). \n
 * @see TebOptimalPlanner::AddEdgesViaPoints
 * @remarks Do not forget to call setTebConfig() and setViaPoint()
 */     
class EdgeViaPoint : public BaseTebUnaryEdge<1, const Eigen::Vector2d*, VertexPose>
{
public:
    
  /**
   * @brief Construct edge.
   */    
  EdgeViaPoint() 
  {
    _measurement = NULL;
  }
 
  /**
   * @brief Actual cost function
   * // 当前节点和观测点之间的距离
   */    
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig(), setViaPoint() on EdgeViaPoint()");
    // 当前的节点
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    // 当前节点和观测点之间的距离
    _error[0] = (bandpt->position() - *_measurement).norm();

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeViaPoint::computeError() _error[0]=%f\n",_error[0]);
  }

  /**
   * @brief Set pointer to associated via point for the underlying cost function 
   * @param via_point 2D position vector containing the position of the via point
   */ 
  void setViaPoint(const Eigen::Vector2d* via_point)
  {
    _measurement = via_point;
  }
    
  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param via_point 2D position vector containing the position of the via point
   */ 
  void setParameters(const TebConfig& cfg, const Eigen::Vector2d* via_point)
  {
    cfg_ = &cfg;
    _measurement = via_point;
  }
  
public: 	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
  
    

} // end namespace

#endif
