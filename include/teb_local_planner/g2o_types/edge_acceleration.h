

#ifndef EDGE_ACCELERATION_H_
#define EDGE_ACCELERATION_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>

#include <geometry_msgs/Twist.h>



namespace teb_local_planner
{

/**
 * 这个是五元边:三个节点和两个时间点,通过这个信息计算加速度,如果超过就惩罚
 * @class EdgeAcceleration
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
 * 
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot } ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of all three poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n 
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationStart
 * @see EdgeAccelerationGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationStart() and EdgeAccelerationGoal() for defining boundary values!
 */    
class EdgeAcceleration : public BaseTebMultiEdge<2, double>
{
public:

  /**
   * @brief Construct edge.
   * 设置五元边
   */ 
  EdgeAcceleration()
  {
    this->resize(5);
  }
    
  /**
   * @brief Actual cost function
   * 这个是五元边:三个节点和两个时间点,通过这个信息计算加速度,如果超过就惩罚
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeAcceleration()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // VELOCITY & ACCELERATION
    const Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    const Eigen::Vector2d diff2 = pose3->position() - pose2->position();
        
    double dist1 = diff1.norm();
    double dist2 = diff2.norm();
    const double angle_diff1 = g2o::normalize_theta(pose2->theta() - pose1->theta());
    const double angle_diff2 = g2o::normalize_theta(pose3->theta() - pose2->theta());
    
    if (cfg_->trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
    {
        if (angle_diff1 != 0)
        {
            const double radius =  dist1/(2*sin(angle_diff1/2));
            dist1 = fabs( angle_diff1 * radius ); // actual arg length!
        }
        if (angle_diff2 != 0)
        {
            const double radius =  dist2/(2*sin(angle_diff2/2));
            dist2 = fabs( angle_diff2 * radius ); // actual arg length!
        }
    }
    
    double vel1 = dist1 / dt1->dt();
    double vel2 = dist2 / dt2->dt();
    
    
    // consider directions
//     vel1 *= g2o::sign(diff1[0]*cos(pose1->theta()) + diff1[1]*sin(pose1->theta())); 
//     vel2 *= g2o::sign(diff2[0]*cos(pose2->theta()) + diff2[1]*sin(pose2->theta())); 
    vel1 *= fast_sigmoid( 100*(diff1.x()*cos(pose1->theta()) + diff1.y()*sin(pose1->theta())) ); 
    vel2 *= fast_sigmoid( 100*(diff2.x()*cos(pose2->theta()) + diff2.y()*sin(pose2->theta())) ); 
    // 注意这里面*2了
    const double acc_lin  = (vel2 - vel1)*2 / ( dt1->dt() + dt2->dt() );
   

    _error[0] = penaltyBoundToInterval(acc_lin,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    const double omega1 = angle_diff1 / dt1->dt();
    const double omega2 = angle_diff2 / dt2->dt();
    const double acc_rot  = (omega2 - omega1)*2 / ( dt1->dt() + dt2->dt() );
      
    _error[1] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAcceleration::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAcceleration::computeError() rotational: _error[1]=%f\n",_error[1]);
  }



#ifdef USE_ANALYTIC_JACOBI
#if 0
  /*
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeAcceleration()");
    const VertexPointXY* conf1 = static_cast<const VertexPointXY*>(_vertices[0]);
    const VertexPointXY* conf2 = static_cast<const VertexPointXY*>(_vertices[1]);
    const VertexPointXY* conf3 = static_cast<const VertexPointXY*>(_vertices[2]);
    const VertexTimeDiff* deltaT1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* deltaT2 = static_cast<const VertexTimeDiff*>(_vertices[4]);
    const VertexOrientation* angle1 = static_cast<const VertexOrientation*>(_vertices[5]);
    const VertexOrientation* angle2 = static_cast<const VertexOrientation*>(_vertices[6]);
    const VertexOrientation* angle3 = static_cast<const VertexOrientation*>(_vertices[7]);

    Eigen::Vector2d deltaS1 = conf2->estimate() - conf1->estimate();
    Eigen::Vector2d deltaS2 = conf3->estimate() - conf2->estimate();
    double dist1 = deltaS1.norm();
    double dist2 = deltaS2.norm();
    
    double sum_time = deltaT1->estimate() + deltaT2->estimate();
    double sum_time_inv = 1 / sum_time;
    double dt1_inv = 1/deltaT1->estimate();
    double dt2_inv = 1/deltaT2->estimate();
    double aux0 = 2/sum_time_inv;
    double aux1 = dist1 * deltaT1->estimate();
    double aux2 = dist2 * deltaT2->estimate();

    double vel1 = dist1 * dt1_inv;
    double vel2 = dist2 * dt2_inv;
    double omega1 = g2o::normalize_theta( angle2->estimate() - angle1->estimate() ) * dt1_inv;
    double omega2 = g2o::normalize_theta( angle3->estimate() - angle2->estimate() ) * dt2_inv;
    double acc = (vel2 - vel1) * aux0;
    double omegadot = (omega2 - omega1) * aux0;
    double aux3 = -acc/2;
    double aux4 = -omegadot/2;
    
    double dev_border_acc = penaltyBoundToIntervalDerivative(acc, tebConfig.robot_acceleration_max_trans,optimizationConfig.optimization_boundaries_epsilon,optimizationConfig.optimization_boundaries_scale,optimizationConfig.optimization_boundaries_order);
    double dev_border_omegadot = penaltyBoundToIntervalDerivative(omegadot, tebConfig.robot_acceleration_max_rot,optimizationConfig.optimization_boundaries_epsilon,optimizationConfig.optimization_boundaries_scale,optimizationConfig.optimization_boundaries_order);
    
    _jacobianOplus[0].resize(2,2); // conf1
    _jacobianOplus[1].resize(2,2); // conf2
    _jacobianOplus[2].resize(2,2); // conf3
    _jacobianOplus[3].resize(2,1); // deltaT1
    _jacobianOplus[4].resize(2,1); // deltaT2
    _jacobianOplus[5].resize(2,1); // angle1
    _jacobianOplus[6].resize(2,1); // angle2
    _jacobianOplus[7].resize(2,1); // angle3
    
    if (aux1==0) aux1=1e-20;
    if (aux2==0) aux2=1e-20;
  
    if (dev_border_acc!=0)
    {
      // TODO: double aux = aux0 * dev_border_acc;
      // double aux123 = aux / aux1;
      _jacobianOplus[0](0,0) = aux0 * deltaS1[0] / aux1 * dev_border_acc; // acc x1
      _jacobianOplus[0](0,1) = aux0 * deltaS1[1] / aux1 * dev_border_acc; // acc y1
      _jacobianOplus[1](0,0) = -aux0 * ( deltaS1[0] / aux1 + deltaS2[0] / aux2 ) * dev_border_acc; // acc x2
      _jacobianOplus[1](0,1) = -aux0 * ( deltaS1[1] / aux1 + deltaS2[1] / aux2 ) * dev_border_acc; // acc y2
      _jacobianOplus[2](0,0) = aux0 * deltaS2[0] / aux2 * dev_border_acc; // acc x3
      _jacobianOplus[2](0,1) = aux0 * deltaS2[1] / aux2 * dev_border_acc; // acc y3	
      _jacobianOplus[2](0,0) = 0;
      _jacobianOplus[2](0,1) = 0;
      _jacobianOplus[3](0,0) = aux0 * (aux3 + vel1 * dt1_inv) * dev_border_acc; // acc deltaT1
      _jacobianOplus[4](0,0) = aux0 * (aux3 - vel2 * dt2_inv) * dev_border_acc; // acc deltaT2
    }
    else
    {
      _jacobianOplus[0](0,0) = 0; // acc x1
      _jacobianOplus[0](0,1) = 0; // acc y1	
      _jacobianOplus[1](0,0) = 0; // acc x2
      _jacobianOplus[1](0,1) = 0; // acc y2
      _jacobianOplus[2](0,0) = 0; // acc x3
      _jacobianOplus[2](0,1) = 0; // acc y3	
      _jacobianOplus[3](0,0) = 0; // acc deltaT1
      _jacobianOplus[4](0,0) = 0; // acc deltaT2
    }
    
    if (dev_border_omegadot!=0)
    {
      _jacobianOplus[3](1,0) = aux0 * ( aux4 + omega1 * dt1_inv ) * dev_border_omegadot; // omegadot deltaT1
      _jacobianOplus[4](1,0) = aux0 * ( aux4 - omega2 * dt2_inv ) * dev_border_omegadot; // omegadot deltaT2
      _jacobianOplus[5](1,0) = aux0 * dt1_inv * dev_border_omegadot; // omegadot angle1
      _jacobianOplus[6](1,0) = -aux0 * ( dt1_inv + dt2_inv ) * dev_border_omegadot; // omegadot angle2
      _jacobianOplus[7](1,0) = aux0 * dt2_inv * dev_border_omegadot; // omegadot angle3
    }
    else
    {
      _jacobianOplus[3](1,0) = 0; // omegadot deltaT1
      _jacobianOplus[4](1,0) = 0; // omegadot deltaT2
      _jacobianOplus[5](1,0) = 0; // omegadot angle1
      _jacobianOplus[6](1,0) = 0; // omegadot angle2
      _jacobianOplus[7](1,0) = 0; // omegadot angle3			
    }

    _jacobianOplus[0](1,0) = 0; // omegadot x1
    _jacobianOplus[0](1,1) = 0; // omegadot y1
    _jacobianOplus[1](1,0) = 0; // omegadot x2
    _jacobianOplus[1](1,1) = 0; // omegadot y2
    _jacobianOplus[2](1,0) = 0; // omegadot x3
    _jacobianOplus[2](1,1) = 0; // omegadot y3
    _jacobianOplus[5](0,0) = 0; // acc angle1
    _jacobianOplus[6](0,0) = 0; // acc angle2
    _jacobianOplus[7](0,0) = 0; // acc angle3
    }
#endif
#endif

      
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
};
    
    
/**
 * 加速度边:计算的是从当前速度到想要的速度中间的加速度是否满足要求,不满足要求的话,就给上惩罚
 * @class EdgeAccelerationStart
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
 *  边定义 限制轨迹开始时的平移和转动加速度的代价函数。
 * 需要三个节点si,sip1,∆Ti;一个初始速度需要设置setInitialVelocity()
 * min penaltyInterval([a, omegadot]T)*weight.
 * a表示加速度,位姿的两次差商
 * Omegadot是使用偏航角的差商，然后归一化到[-pi, pi]来计算的。
 * The edge depends on three vertices si,sip1,∆Ti, an initial velocity defined by setInitialVelocity() * and minimizes:
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of the poses. \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAcceleration
 * @see EdgeAccelerationGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationGoal() for defining boundary values at the end of the trajectory!
 */      
class EdgeAccelerationStart : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */	  
  EdgeAccelerationStart()
  {
    _measurement = NULL;
    this->resize(3);
  }
  
  
  /**
   * @brief Actual cost function
   * 计算的是从当前速度到想要的速度中间的加速度是否满足要求,不满足要求的话,就给上惩罚
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationStart()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    // VELOCITY & ACCELERATION
    const Eigen::Vector2d diff = pose2->position() - pose1->position();
    double dist = diff.norm();
    const double angle_diff = g2o::normalize_theta(pose2->theta() - pose1->theta());
    if (cfg_->trajectory.exact_arc_length && angle_diff != 0)
    {
        const double radius =  dist/(2*sin(angle_diff/2));
        dist = fabs( angle_diff * radius ); // actual arg length!
    }
    //一般这个速度就是机器人当前的速度
    const double vel1 = _measurement->linear.x;
    double vel2 = dist / dt->dt();

    // consider directions
    //vel2 *= g2o::sign(diff[0]*cos(pose1->theta()) + diff[1]*sin(pose1->theta())); 
    vel2 *= fast_sigmoid( 100*(diff.x()*cos(pose1->theta()) + diff.y()*sin(pose1->theta())) ); 
    // 表示从vel2到vel1的加速度
    const double acc_lin  = (vel2 - vel1) / dt->dt();
    // 超过加速度范围(加上额外的安全裕度),就会惩罚
    _error[0] = penaltyBoundToInterval(acc_lin,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
   //一般这个速度就是机器人当前的速度
    const double omega1 = _measurement->angular.z;
    const double omega2 = angle_diff / dt->dt();
    const double acc_rot  = (omega2 - omega1) / dt->dt();
    // 超过加速度范围(加上额外的安全裕度),就会惩罚
    _error[1] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationStart::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationStart::computeError() rotational: _error[1]=%f\n",_error[1]);
  }
  
  /**
   * 设置计算加速度时考虑的初始速度
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
   * @param vel_start twist message containing the translational and rotational velocity
   */    
  void setInitialVelocity(const geometry_msgs::Twist& vel_start)
  {
    //一般这个速度就是机器人当前的速度
    _measurement = &vel_start;
  }
  
public:       
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};    
    
    
    

/**
 * 这个是三元边:2个节点和1个时间点,通过这个信息计算加速度,如果超过就惩罚
 * @class EdgeAccelerationGoal
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
 * 
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [a, omegadot ]^T ) \cdot weight \f$. \n
 * \e a is calculated using the difference quotient (twice) and the position parts of the poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 2: the first component represents the translational acceleration and
 * the second one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAcceleration
 * @see EdgeAccelerationStart
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationStart() for defining boundary (initial) values at the end of the trajectory
 */  
class EdgeAccelerationGoal : public BaseTebMultiEdge<2, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */  
  EdgeAccelerationGoal()
  {
    _measurement = NULL;
    this->resize(3);
  }
  

  /**
   * @brief Actual cost function
   * 这个是三元边:2个节点和1个时间点,通过这个信息计算加速度,如果超过就惩罚
   * 一般情况下终点的速度是0
   */ 
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationGoal()");
    const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);//终点的上一个点
    const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);//终点
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);//时间

    // VELOCITY & ACCELERATION

    const Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();  
    double dist = diff.norm();
    const double angle_diff = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta());
    if (cfg_->trajectory.exact_arc_length  && angle_diff != 0)
    {
        double radius =  dist/(2*sin(angle_diff/2));
        dist = fabs( angle_diff * radius ); // actual arg length!
    }
    
    double vel1 = dist / dt->dt();
    const double vel2 = _measurement->linear.x;
    
    // consider directions
    //vel1 *= g2o::sign(diff[0]*cos(pose_pre_goal->theta()) + diff[1]*sin(pose_pre_goal->theta())); 
    vel1 *= fast_sigmoid( 100*(diff.x()*cos(pose_pre_goal->theta()) + diff.y()*sin(pose_pre_goal->theta())) ); 
    
    const double acc_lin  = (vel2 - vel1) / dt->dt();

    _error[0] = penaltyBoundToInterval(acc_lin,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    const double omega1 = angle_diff / dt->dt();
    const double omega2 = _measurement->angular.z;
    const double acc_rot  = (omega2 - omega1) / dt->dt();
      
    _error[1] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationGoal::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationGoal::computeError() rotational: _error[1]=%f\n",_error[1]);
  }
    
  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
   * @param vel_goal twist message containing the translational and rotational velocity
   */    
  void setGoalVelocity(const geometry_msgs::Twist& vel_goal)
  {
    _measurement = &vel_goal;
  }
  
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; 
    



/**
 * @class EdgeAccelerationHolonomic
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration.
 * 
 * The edge depends on five vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \mathbf{s}_{ip2}, \Delta T_i, \Delta T_{ip1} \f$ and minimizes:
 * \f$ \min \textrm{penaltyInterval}( [ax, ay, omegadot } ]^T ) \cdot weight \f$. \n
 * \e ax is calculated using the difference quotient (twice) and the x position parts of all three poses \n
 * \e ay is calculated using the difference quotient (twice) and the y position parts of all three poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi]. \n 
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 3: the first component represents the translational acceleration (x-dir), 
 * the second one the strafing acceleration and the third one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomicStart
 * @see EdgeAccelerationHolonomicGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomicStart() and EdgeAccelerationHolonomicGoal() for defining boundary values!
 */    
class EdgeAccelerationHolonomic : public BaseTebMultiEdge<3, double>
{
public:

  /**
   * @brief Construct edge.
   */    
  EdgeAccelerationHolonomic()
  {
    this->resize(5);
  }
    
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeAcceleration()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexPose* pose3 = static_cast<const VertexPose*>(_vertices[2]);
    const VertexTimeDiff* dt1 = static_cast<const VertexTimeDiff*>(_vertices[3]);
    const VertexTimeDiff* dt2 = static_cast<const VertexTimeDiff*>(_vertices[4]);

    // VELOCITY & ACCELERATION
    Eigen::Vector2d diff1 = pose2->position() - pose1->position();
    Eigen::Vector2d diff2 = pose3->position() - pose2->position();
    
    double cos_theta1 = std::cos(pose1->theta());
    double sin_theta1 = std::sin(pose1->theta()); 
    double cos_theta2 = std::cos(pose2->theta());
    double sin_theta2 = std::sin(pose2->theta()); 
    
    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
    double p1_dx =  cos_theta1*diff1.x() + sin_theta1*diff1.y();
    double p1_dy = -sin_theta1*diff1.x() + cos_theta1*diff1.y();
    // transform pose3 into robot frame pose2 (inverse 2d rotation matrix)
    double p2_dx =  cos_theta2*diff2.x() + sin_theta2*diff2.y();
    double p2_dy = -sin_theta2*diff2.x() + cos_theta2*diff2.y();
    
    double vel1_x = p1_dx / dt1->dt();
    double vel1_y = p1_dy / dt1->dt();
    double vel2_x = p2_dx / dt2->dt();
    double vel2_y = p2_dy / dt2->dt();
    
    double dt12 = dt1->dt() + dt2->dt();
    
    double acc_x  = (vel2_x - vel1_x)*2 / dt12;
    double acc_y  = (vel2_y - vel1_y)*2 / dt12;
   
    _error[0] = penaltyBoundToInterval(acc_x,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(acc_y,cfg_->robot.acc_lim_y,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    double omega1 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt1->dt();
    double omega2 = g2o::normalize_theta(pose3->theta() - pose2->theta()) / dt2->dt();
    double acc_rot  = (omega2 - omega1)*2 / dt12;
      
    _error[2] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAcceleration::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAcceleration::computeError() strafing: _error[1]=%f\n",_error[1]);
    ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAcceleration::computeError() rotational: _error[2]=%f\n",_error[2]);
  }

public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   
};


/**
 * @class EdgeAccelerationHolonomicStart
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the beginning of the trajectory.
 *
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setInitialVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [ax, ay, omegadot ]^T ) \cdot weight \f$. \n
 * \e ax is calculated using the difference quotient (twice) and the x-position parts of the poses. \n
 * \e ay is calculated using the difference quotient (twice) and the y-position parts of the poses. \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 3: the first component represents the translational acceleration,
 * the second one the strafing acceleration and the third one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic
 * @see EdgeAccelerationHolonomicGoal
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomicGoal() for defining boundary values at the end of the trajectory!
 */      
class EdgeAccelerationHolonomicStart : public BaseTebMultiEdge<3, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */   
  EdgeAccelerationHolonomicStart()
  {
    this->resize(3);
    _measurement = NULL;
  }
    
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setStartVelocity() on EdgeAccelerationStart()");
    const VertexPose* pose1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    // VELOCITY & ACCELERATION
    Eigen::Vector2d diff = pose2->position() - pose1->position();
            
    double cos_theta1 = std::cos(pose1->theta());
    double sin_theta1 = std::sin(pose1->theta()); 
    
    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
    double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
    double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();
    
    double vel1_x = _measurement->linear.x;
    double vel1_y = _measurement->linear.y;
    double vel2_x = p1_dx / dt->dt();
    double vel2_y = p1_dy / dt->dt();

    double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
    double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();
    
    _error[0] = penaltyBoundToInterval(acc_lin_x,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(acc_lin_y,cfg_->robot.acc_lim_y,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    double omega1 = _measurement->angular.z;
    double omega2 = g2o::normalize_theta(pose2->theta() - pose1->theta()) / dt->dt();
    double acc_rot  = (omega2 - omega1) / dt->dt();
      
    _error[2] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationStart::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationStart::computeError() strafing: _error[1]=%f\n",_error[1]);
    ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAccelerationStart::computeError() rotational: _error[2]=%f\n",_error[2]);
  }
  
  /**
   * @brief Set the initial velocity that is taken into account for calculating the acceleration
   * @param vel_start twist message containing the translational and rotational velocity
   */    
  void setInitialVelocity(const geometry_msgs::Twist& vel_start)
  {
    _measurement = &vel_start;
  }
        
public:       
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};    
    
       

/**
 * @class EdgeAccelerationHolonomicGoal
 * @brief Edge defining the cost function for limiting the translational and rotational acceleration at the end of the trajectory.
 * 
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$, an initial velocity defined by setGoalVelocity()
 * and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [ax, ay, omegadot ]^T ) \cdot weight \f$. \n
 * \e ax is calculated using the difference quotient (twice) and the x-position parts of the poses \n
 * \e ay is calculated using the difference quotient (twice) and the y-position parts of the poses \n
 * \e omegadot is calculated using the difference quotient of the yaw angles followed by a normalization to [-pi, pi].  \n
 * \e weight can be set using setInformation() \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval() \n
 * The dimension of the error / cost vector is 3: the first component represents the translational acceleration,
 * the second one is the strafing velocity and the third one the rotational acceleration.
 * @see TebOptimalPlanner::AddEdgesAcceleration
 * @see EdgeAccelerationHolonomic
 * @see EdgeAccelerationHolonomicStart
 * @remarks Do not forget to call setTebConfig()
 * @remarks Refer to EdgeAccelerationHolonomicStart() for defining boundary (initial) values at the end of the trajectory
 */  
class EdgeAccelerationHolonomicGoal : public BaseTebMultiEdge<3, const geometry_msgs::Twist*>
{
public:

  /**
   * @brief Construct edge.
   */  
  EdgeAccelerationHolonomicGoal()
  {
    _measurement = NULL;
    this->resize(3);
  }
  
  /**
   * @brief Actual cost function
   */ 
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setGoalVelocity() on EdgeAccelerationGoal()");
    const VertexPose* pose_pre_goal = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* pose_goal = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* dt = static_cast<const VertexTimeDiff*>(_vertices[2]);

    // VELOCITY & ACCELERATION

    Eigen::Vector2d diff = pose_goal->position() - pose_pre_goal->position();    
    
    double cos_theta1 = std::cos(pose_pre_goal->theta());
    double sin_theta1 = std::sin(pose_pre_goal->theta()); 
    
    // transform pose2 into robot frame pose1 (inverse 2d rotation matrix)
    double p1_dx =  cos_theta1*diff.x() + sin_theta1*diff.y();
    double p1_dy = -sin_theta1*diff.x() + cos_theta1*diff.y();
   
    double vel1_x = p1_dx / dt->dt();
    double vel1_y = p1_dy / dt->dt();
    double vel2_x = _measurement->linear.x;
    double vel2_y = _measurement->linear.y;
    
    double acc_lin_x  = (vel2_x - vel1_x) / dt->dt();
    double acc_lin_y  = (vel2_y - vel1_y) / dt->dt();

    _error[0] = penaltyBoundToInterval(acc_lin_x,cfg_->robot.acc_lim_x,cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(acc_lin_y,cfg_->robot.acc_lim_y,cfg_->optim.penalty_epsilon);
    
    // ANGULAR ACCELERATION
    double omega1 = g2o::normalize_theta(pose_goal->theta() - pose_pre_goal->theta()) / dt->dt();
    double omega2 = _measurement->angular.z;
    double acc_rot  = (omega2 - omega1) / dt->dt();
      
    _error[2] = penaltyBoundToInterval(acc_rot,cfg_->robot.acc_lim_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeAccelerationGoal::computeError() translational: _error[0]=%f\n",_error[0]);
    ROS_ASSERT_MSG(std::isfinite(_error[1]), "EdgeAccelerationGoal::computeError() strafing: _error[1]=%f\n",_error[1]);
    ROS_ASSERT_MSG(std::isfinite(_error[2]), "EdgeAccelerationGoal::computeError() rotational: _error[2]=%f\n",_error[2]);
  }
  
  
  /**
   * @brief Set the goal / final velocity that is taken into account for calculating the acceleration
   * @param vel_goal twist message containing the translational and rotational velocity
   */    
  void setGoalVelocity(const geometry_msgs::Twist& vel_goal)
  {
    _measurement = &vel_goal;
  }
  

public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; 
    



}; // end namespace

#endif /* EDGE_ACCELERATION_H_ */
