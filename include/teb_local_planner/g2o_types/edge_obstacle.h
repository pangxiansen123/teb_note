// 这个是考虑静态障碍物的边
// 其中下面是两条类型的边,判断使用的条件就是cfg_->obstacles.inflation_dist与cfg_->obstacles.min_obstacle_dist大小
// 如果大于用EdgeInflatedObstacle边:
    // 这个边里面专门加入了一个膨胀距离路径点的cost
    // 计算误差值,如果路径点距离障碍物小于cfg_->obstacles.min_obstacle_dist+cfg_->optim.penalty_epsilon,那么就给惩罚,也就是error[0]=正值,否则为0
    // 同理路径点距离障碍物小于cfg_->obstacles.inflation_dist也要进行判断
// 如果小于用EdgeObstacle:
    // 计算误差值,如果路径点距离障碍物小于cfg_->obstacles.min_obstacle_dist+cfg_->optim.penalty_epsilon,那么就给惩罚,也就是error[0]=正值,否则为0
#ifndef EDGE_OBSTACLE_H_
#define EDGE_OBSTACLE_H_

#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>

namespace teb_local_planner
{

  /**
 * @class EdgeObstacle
 * @brief Edge defining the cost function for keeping a minimum distance from obstacles.
 * 
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2point ) \cdot weight \f$. \n
 * \e dist2point denotes the minimum distance to the point obstacle. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow() \n
 * @see TebOptimalPlanner::AddEdgesObstacles, TebOptimalPlanner::EdgeInflatedObstacle
 * @remarks Do not forget to call setTebConfig() and setObstacle()
 */
  // 注意,当cfg_->obstacles.inflation_dist < cfg_->obstacles.min_obstacle_dist时候执行的是这个边
  // 计算误差值,如果路径点距离障碍物小于cfg_->obstacles.min_obstacle_dist+cfg_->optim.penalty_epsilon,那么就给惩罚,也就是error[0]=正值,否则为0
  class EdgeObstacle : public BaseTebUnaryEdge<1, const Obstacle *, VertexPose>
  {
  public:
    /**
   * @brief Construct edge.
   */
    EdgeObstacle()
    {
      _measurement = NULL;
    }

    /**
   * @brief Actual cost function
   */
    void computeError()
    {
      ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setObstacle() and setRobotModel() on EdgeObstacle()");
      const VertexPose *bandpt = static_cast<const VertexPose *>(_vertices[0]);

      double dist = robot_model_->calculateDistance(bandpt->pose(), _measurement);

      // Original obstacle cost.
      _error[0] = penaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);

      if (cfg_->optim.obstacle_cost_exponent != 1.0 && cfg_->obstacles.min_obstacle_dist > 0.0)
      {
        // Optional non-linear cost. Note the max cost (before weighting) is
        // the same as the straight line version and that all other costs are
        // below the straight line (for positive exponent), so it may be
        // necessary to increase weight_obstacle and/or the inflation_weight
        // when using larger exponents.
        _error[0] = cfg_->obstacles.min_obstacle_dist * std::pow(_error[0] / cfg_->obstacles.min_obstacle_dist, cfg_->optim.obstacle_cost_exponent);
      }

      ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeObstacle::computeError() _error[0]=%f\n", _error[0]);
    }

#ifdef USE_ANALYTIC_JACOBI
#if 0

  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgePointObstacle()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    
    Eigen::Vector2d deltaS = *_measurement - bandpt->position(); 
    double angdiff = atan2(deltaS[1],deltaS[0])-bandpt->theta();
    
    double dist_squared = deltaS.squaredNorm();
    double dist = sqrt(dist_squared);
    
    double aux0 = sin(angdiff);
    double dev_left_border = penaltyBoundFromBelowDerivative(dist*fabs(aux0),cfg_->obstacles.min_obstacle_dist,cfg_->optim.penalty_epsilon);

    if (dev_left_border==0)
    {
      _jacobianOplusXi( 0 , 0 ) = 0;
      _jacobianOplusXi( 0 , 1 ) = 0;
      _jacobianOplusXi( 0 , 2 ) = 0;
      return;
    }
    
    double aux1 = -fabs(aux0) / dist;
    double dev_norm_x = deltaS[0]*aux1;
    double dev_norm_y = deltaS[1]*aux1;
    
    double aux2 = cos(angdiff) * g2o::sign(aux0);
    double aux3 = aux2 / dist_squared;
    double dev_proj_x = aux3 * deltaS[1] * dist;
    double dev_proj_y = -aux3 * deltaS[0] * dist;
    double dev_proj_angle = -aux2;
    
    _jacobianOplusXi( 0 , 0 ) = dev_left_border * ( dev_norm_x + dev_proj_x );
    _jacobianOplusXi( 0 , 1 ) = dev_left_border * ( dev_norm_y + dev_proj_y );
    _jacobianOplusXi( 0 , 2 ) = dev_left_border * dev_proj_angle;
  }
#endif
#endif

    /**
   * @brief Set pointer to associated obstacle for the underlying cost function 
   * @param obstacle 2D position vector containing the position of the obstacle
   */
    void setObstacle(const Obstacle *obstacle)
    {
      _measurement = obstacle;
    }

    /**
   * @brief Set pointer to the robot model 
   * @param robot_model Robot model required for distance calculation
   */
    void setRobotModel(const BaseRobotFootprintModel *robot_model)
    {
      robot_model_ = robot_model;
    }

    /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */
    void setParameters(const TebConfig &cfg, const BaseRobotFootprintModel *robot_model, const Obstacle *obstacle)
    {
      cfg_ = &cfg;
      robot_model_ = robot_model;
      _measurement = obstacle;
    }

  protected:
    const BaseRobotFootprintModel *robot_model_; //!< Store pointer to robot_model

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /**
 * @class EdgeInflatedObstacle
 * 边定义成本函数，以保持与膨胀障碍物的最小距离。
 * @brief Edge defining the cost function for keeping a minimum distance from inflated obstacles.
 * 
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2point, min_obstacle_dist ) \cdot weight_inflation \f$. \n
 * Additional, a second penalty is provided with \n
 * \f$ \min \textrm{penaltyBelow}( dist2point, inflation_dist ) \cdot weight_inflation \f$.
 * It is assumed that inflation_dist > min_obstacle_dist and weight_inflation << weight_inflation.
 * \e dist2point denotes the minimum distance to the point obstacle. \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow() \n
 * @see TebOptimalPlanner::AddEdgesObstacles, TebOptimalPlanner::EdgeObstacle
 * @remarks Do not forget to call setTebConfig() and setObstacle()
 * 表示一元边的子类
 */
  // 注意,当cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist时候执行的是这个边
  // 这个边里面专门加入了一个膨胀距离路径点的cost
  // 计算误差值,如果路径点距离障碍物小于cfg_->obstacles.min_obstacle_dist+cfg_->optim.penalty_epsilon,那么就给惩罚,也就是error[0]=正值,否则为0
  // 同理路径点距离障碍物小于cfg_->obstacles.inflation_dist也要进行判断
  class EdgeInflatedObstacle : public BaseTebUnaryEdge<2, const Obstacle *, VertexPose>
  {
  public:
    /**
   * @brief Construct edge.
   * // 存储观测值
   */
    EdgeInflatedObstacle()
    {
      _measurement = NULL;
    }

    /**
   * @brief Actual cost function
   * 计算误差值,如果路径点距离障碍物小于cfg_->obstacles.min_obstacle_dist+cfg_->optim.penalty_epsilon,那么就给惩罚,也就是error[0]=正值,否则为0
   * 同理路径点距离障碍物小于cfg_->obstacles.inflation_dist也要进行判断
   */
    void computeError()
    {
      ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setObstacle() and setRobotModel() on EdgeInflatedObstacle()");
      // 这个就是路径中的一个点
      const VertexPose *bandpt = static_cast<const VertexPose *>(_vertices[0]);
      // _measurement表示其中的障碍物,是一个点
      double dist = robot_model_->calculateDistance(bandpt->pose(), _measurement);

      // Original "straight line" obstacle cost. The max possible value
      // before weighting is min_obstacle_dist
      // 表示一个线性惩罚:当var大于a+epsilon,不惩罚,当小于的时候线性惩罚
      _error[0] = penaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon);

      if (cfg_->optim.obstacle_cost_exponent != 1.0 && cfg_->obstacles.min_obstacle_dist > 0.0)
      {
        // Optional non-linear cost. Note the max cost (before weighting) is
        // the same as the straight line version and that all other costs are
        // below the straight line (for positive exponent), so it may be
        // necessary to increase weight_obstacle and/or the inflation_weight
        // when using larger exponents.
        //可选的非线性成本。注意，最大成本(权重前)与直线版本相同，所有其他成本都低于直线(对于正指数)，
        // 因此在使用较大的指数时可能需要增加weight_obstacle和inflation_weight。
        _error[0] = cfg_->obstacles.min_obstacle_dist * std::pow(_error[0] / cfg_->obstacles.min_obstacle_dist, cfg_->optim.obstacle_cost_exponent);
      }

      // Additional linear inflation cost
      // 添加额外的膨胀cost
      _error[1] = penaltyBoundFromBelow(dist, cfg_->obstacles.inflation_dist, 0.0);

      ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]), "EdgeInflatedObstacle::computeError() _error[0]=%f, _error[1]=%f\n", _error[0], _error[1]);
    }

    /**
   * @brief Set pointer to associated obstacle for the underlying cost function 
   * @param obstacle 2D position vector containing the position of the obstacle
   * _measurement维度是2,也就是障碍物的xy坐标
   */
    void setObstacle(const Obstacle *obstacle)
    {
      _measurement = obstacle;
    }

    /**
   * @brief Set pointer to the robot model 
   * @param robot_model Robot model required for distance calculation
   */
    void setRobotModel(const BaseRobotFootprintModel *robot_model)
    {
      robot_model_ = robot_model;
    }

    /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */
    void setParameters(const TebConfig &cfg, const BaseRobotFootprintModel *robot_model, const Obstacle *obstacle)
    {
      cfg_ = &cfg;
      robot_model_ = robot_model;
      _measurement = obstacle;
    }

  protected:
    const BaseRobotFootprintModel *robot_model_; //!< Store pointer to robot_model

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // end namespace

#endif
