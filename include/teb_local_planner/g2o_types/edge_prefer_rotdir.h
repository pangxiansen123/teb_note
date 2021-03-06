// 这个函数就是如果_measurement=1(preferLeft()),那么要做的就是惩罚右转的方向
#ifndef EDGE_PREFER_ROTDIR_H_
#define EDGE_PREFER_ROTDIR_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include "g2o/core/base_unary_edge.h"


namespace teb_local_planner
{

/**
 * @class EdgePreferRotDir
 * @brief Edge defining the cost function for penalzing a specified turning direction, in particular left resp. right turns
 * 边定义惩罚一个指定的转弯方向的代价函数，特别是左转弯。右转
 * The edge depends on two consecutive vertices \f$ \mathbf{s}_i, \mathbf{s}_{i+1} \f$ and penalizes a given rotation direction
 * based on the \e weight and \e dir (\f$ dir \in \{-1,1\} \f$)
 * \e dir should be +1 to prefer left rotations and -1 to prefer right rotations  \n
 * \e weight can be set using setInformation(). \n
 * @see TebOptimalPlanner::AddEdgePreferRotDir
 */     
class EdgePreferRotDir : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose>
{
public:
    
  /**
   * @brief Construct edge.
   */    
  EdgePreferRotDir() 
  {
    _measurement = 1;
  }
 
  /**
   * @brief Actual cost function
   * 这个函数就是如果_measurement=1,那么要做的就是惩罚右转的方向
   */    
  void computeError()
  {
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    // 这个就是左边是正,右边是反
    _error[0] = penaltyBoundFromBelow( _measurement*g2o::normalize_theta(conf2->theta()-conf1->theta()) , 0, 0);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgePreferRotDir::computeError() _error[0]=%f\n",_error[0]);
  }

  /**
   * @brief Specify the prefered direction of rotation
   * @param dir +1 to prefer the left side, -1 to prefer the right side
   */ 
  void setRotDir(double dir)
  {
    _measurement = dir;
  }
  
  /** Prefer rotations to the right */
  void preferRight() {_measurement = -1;}
    
  /** Prefer rotations to the left */
  void preferLeft() {_measurement = 1;}  
    
  
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
  
    

} // end namespace

#endif
