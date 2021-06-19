// 一元边,表示时间节点时间越长,那么误差越大
#ifndef EDGE_TIMEOPTIMAL_H_
#define EDGE_TIMEOPTIMAL_H_

#include <float.h>

#include <base_local_planner/BaseLocalPlannerConfig.h>

#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>

#include <Eigen/Core>

namespace teb_local_planner
{

  
/**
 * 一元边,就是时间节点的时间越长误差越大
 * @class EdgeTimeOptimal
 * @brief Edge defining the cost function for minimizing transition time of the trajectory.
 * 
 * The edge depends on a single vertex \f$ \Delta T_i \f$ and minimizes: \n
 * \f$ \min \Delta T_i^2 \cdot scale \cdot weight \f$. \n
 * \e scale is determined using the penaltyEquality() function, since we experiences good convergence speeds with it. \n
 * \e weight can be set using setInformation() (something around 1.0 seems to be fine). \n
 * @see TebOptimalPlanner::AddEdgesTimeOptimal
 * @remarks Do not forget to call setTebConfig()
 */
class EdgeTimeOptimal : public BaseTebUnaryEdge<1, double, VertexTimeDiff>
{
public:
    
  /**
   * @brief Construct edge.
   */
  EdgeTimeOptimal()
  {
    this->setMeasurement(0.);
  }
  
  /**
   * @brief Actual cost function
   * 就是时间节点的时间越长误差越大
   */
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeTimeOptimal()");
    const VertexTimeDiff* timediff = static_cast<const VertexTimeDiff*>(_vertices[0]);

   _error[0] = timediff->dt();
  
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeTimeOptimal::computeError() _error[0]=%f\n",_error[0]);
  }

#ifdef USE_ANALYTIC_JACOBI
  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeTimeOptimal()");
    _jacobianOplusXi( 0 , 0 ) = 1;
  }
#endif
  
  
public:        
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}; // end namespace

#endif /* EDGE_TIMEOPTIMAL_H_ */
