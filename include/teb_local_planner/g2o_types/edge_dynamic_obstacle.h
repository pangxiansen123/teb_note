// 这个是动态障碍物相关的边
//  计算_measurement以当前速度运行t时间段后与bandpt的距离
//  计算误差值,如果路径点距离障碍物小于cfg_->obstacles.min_obstacle_dist+cfg_->optim.penalty_epsilon,那么就给惩罚,也就是error[0]=正值,否则为0
// 同理路径点距离障碍物小于cfg_->obstacles.dynamic_obstacle_inflation_dist也要进行判断
#ifndef EDGE_DYNAMICOBSTACLE_H
#define EDGE_DYNAMICOBSTACLE_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/robot_footprint_model.h>

namespace teb_local_planner
{
  
/**
 * @class EdgeDynamicObstacle
 * 边定义与动态(移动)障碍物保持距离的代价函数。
 * @brief Edge defining the cost function for keeping a distance from dynamic (moving) obstacles.
 * 
 * The edge depends on two vertices si,∆Ti and minimizes: \n
 * min penaltyBelow(dist2obstacle)·weight \n
 * \e dist2obstacle denotes the minimum distance to the obstacle trajectory (spatial and temporal). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow(). \n
 * @see TebOptimalPlanner::AddEdgesDynamicObstacles
 * @remarks Do not forget to call setTebConfig(), setVertexIdx() and 
 * @warning Experimental
 */  
class EdgeDynamicObstacle : public BaseTebUnaryEdge<2, const Obstacle*, VertexPose>
{
public:
  
  /**
   * @brief Construct edge.
   */    
  EdgeDynamicObstacle() : t_(0)
  {
  }
  
  /**
   * @brief Construct edge and specify the time for its associated pose (neccessary for computeError).
   * @param t_ Estimated time until current pose is reached
   */      
  EdgeDynamicObstacle(double t) : t_(t)
  {
  }
  
  /**
   * @brief Actual cost function
   * 计算_measurement以当前速度运行t时间段后与bandpt的距离
   * 计算误差值,如果路径点距离障碍物小于cfg_->obstacles.min_obstacle_dist+cfg_->optim.penalty_epsilon,那么就给惩罚,也就是error[0]=正值,否则为0
     同理路径点距离障碍物小于cfg_->obstacles.dynamic_obstacle_inflation_dist也要进行判断
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setObstacle() and setRobotModel() on EdgeDynamicObstacle()");
    // 得到连接的节点
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    // 就是计算_measurement以当前速度运行t时间段后与bandpt的距离
    double dist = robot_model_->estimateSpatioTemporalDistance(bandpt->pose(), _measurement, t_);
    // 表示一个线性惩罚:当var大于a+epsilon,不惩罚,当小于的时候线性惩罚
    // 计算误差值,如果路径点距离障碍物小于cfg_->obstacles.min_obstacle_dist+cfg_->optim.penalty_epsilon,那么就给惩罚,也就是error[0]=正值,否则为0
    // 同理路径点距离障碍物小于cfg_->obstacles.dynamic_obstacle_inflation_dist也要进行判断
    _error[0] = penaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundFromBelow(dist, cfg_->obstacles.dynamic_obstacle_inflation_dist, 0.0);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeDynamicObstacle::computeError() _error[0]=%f\n",_error[0]);
  }
  
  
  /**
   * @brief Set Obstacle for the underlying cost function
   * @param obstacle Const pointer to an Obstacle or derived Obstacle
   */     
  void setObstacle(const Obstacle* obstacle)
  {
    _measurement = obstacle;
  }
  
  /**
   * @brief Set pointer to the robot model
   * @param robot_model Robot model required for distance calculation
   */
  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, const Obstacle* obstacle)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = obstacle;
  }

protected:
  
  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  double t_; //!< Estimated time until current pose is reached
  
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
    
 
    

} // end namespace

#endif
