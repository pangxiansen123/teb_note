// 速度惩罚,超过设定的速度就会进行惩罚,注意误差是两个值,一个是线速度一个是角速度
#ifndef EDGE_VELOCITY_H
#define EDGE_VELOCITY_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>


#include <iostream>

namespace teb_local_planner
{

  
/**
 * @class EdgeVelocity
 * 三元边 边定义限制平移和旋转速度的代价函数。速度惩罚,超过设定的速度就会进行惩罚,注意误差是两个值,一个是线速度一个是角速度
 * @brief Edge defining the cost function for limiting the translational and rotational velocity.
 * 
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [v,omega]^T ) \cdot weight \f$. \n
 * 依赖三个节点:两个位姿一个时间s_i,s_ip1,∆Ti
 * V是通过差商和两个姿态的位置部分来计算的。
 * 用两个偏航角的差商计算，然后归一化到[-pi, pi]。
 * penaltyInterval表示惩罚函数，参见penaltyBoundToInterval()。
 * The  edge  depends  on  three  vertices s_i,s_ip1,∆Ti and minimizes:  min penaltyInterval([v, omega]T)*weight.
 * \e v is calculated using the difference quotient and the position parts of both poses. \n
 * \e omega is calculated using the difference quotient of both yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 2: the first component represents the translational velocity and
 * the second one the rotational velocity.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */  
class EdgeVelocity : public BaseTebMultiEdge<2, double>
{
public:
  
  /**
   * @brief Construct edge.
   */	      
  EdgeVelocity()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }
  
  /**
   * @brief Actual cost function
   * 速度惩罚,超过设定的速度就会进行惩罚,注意误差是两个值,一个是线速度一个是角速度
   */  
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocity()");
    // 前两个是节点,后一个是时间
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
    // 位移差值
    const Eigen::Vector2d deltaS = conf2->estimate().position() - conf1->estimate().position();
    // 距离
    double dist = deltaS.norm();
    // 角度差值
    const double angle_diff = g2o::normalize_theta(conf2->theta() - conf1->theta());
    // 计算弧线距离,这一个标识符
    if (cfg_->trajectory.exact_arc_length && angle_diff != 0)
    {
        double radius =  dist/(2*sin(angle_diff/2));
        dist = fabs( angle_diff * radius ); // actual arg length!
    }
    // 计算这个两点之间的速度
    double vel = dist / deltaT->estimate();
    
//     vel *= g2o::sign(deltaS[0]*cos(conf1->theta()) + deltaS[1]*sin(conf1->theta())); // consider direction
    // 得到运行的方向
    vel *= fast_sigmoid( 100 * (deltaS.x()*cos(conf1->theta()) + deltaS.y()*sin(conf1->theta())) ); // consider direction
    // 角速度
    const double omega = angle_diff / deltaT->estimate();
    // 超过速度范围(加上额外的安全裕度),就会惩罚
    _error[0] = penaltyBoundToInterval(vel, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,cfg_->optim.penalty_epsilon);
    // 超过速度范围(加上额外的安全裕度),就会惩罚
    _error[1] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVelocity::computeError() _error[0]=%f _error[1]=%f\n",_error[0],_error[1]);
  }

#ifdef USE_ANALYTIC_JACOBI
#if 0 //TODO the hardcoded jacobian does not include the changing direction (just the absolute value)
      // Change accordingly...

  /**
   * @brief Jacobi matrix of the cost function specified in computeError().
   */
  void linearizeOplus()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocity()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
    
    Eigen::Vector2d deltaS = conf2->position() - conf1->position();
    double dist = deltaS.norm();
    double aux1 = dist*deltaT->estimate();
    double aux2 = 1/deltaT->estimate();
    
    double vel = dist * aux2;
    double omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) * aux2;
    
    double dev_border_vel = penaltyBoundToIntervalDerivative(vel, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x,cfg_->optim.penalty_epsilon);
    double dev_border_omega = penaltyBoundToIntervalDerivative(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);
    
    _jacobianOplus[0].resize(2,3); // conf1
    _jacobianOplus[1].resize(2,3); // conf2
    _jacobianOplus[2].resize(2,1); // deltaT
    
//  if (aux1==0) aux1=1e-6;
//  if (aux2==0) aux2=1e-6;
    
    if (dev_border_vel!=0)
    {
      double aux3 = dev_border_vel / aux1;
      _jacobianOplus[0](0,0) = -deltaS[0] * aux3; // vel x1
      _jacobianOplus[0](0,1) = -deltaS[1] * aux3; // vel y1	
      _jacobianOplus[1](0,0) = deltaS[0] * aux3; // vel x2
      _jacobianOplus[1](0,1) = deltaS[1] * aux3; // vel y2
      _jacobianOplus[2](0,0) = -vel * aux2 * dev_border_vel; // vel deltaT
    }
    else
    {
      _jacobianOplus[0](0,0) = 0; // vel x1
      _jacobianOplus[0](0,1) = 0; // vel y1	
      _jacobianOplus[1](0,0) = 0; // vel x2
      _jacobianOplus[1](0,1) = 0; // vel y2	
      _jacobianOplus[2](0,0) = 0; // vel deltaT
    }
    
    if (dev_border_omega!=0)
    {
      double aux4 = aux2 * dev_border_omega;
      _jacobianOplus[2](1,0) = -omega * aux4; // omega deltaT
      _jacobianOplus[0](1,2) = -aux4; // omega angle1
      _jacobianOplus[1](1,2) = aux4; // omega angle2
    }
    else
    {
      _jacobianOplus[2](1,0) = 0; // omega deltaT
      _jacobianOplus[0](1,2) = 0; // omega angle1
      _jacobianOplus[1](1,2) = 0; // omega angle2			
    }

    _jacobianOplus[0](1,0) = 0; // omega x1
    _jacobianOplus[0](1,1) = 0; // omega y1
    _jacobianOplus[1](1,0) = 0; // omega x2
    _jacobianOplus[1](1,1) = 0; // omega y2
    _jacobianOplus[0](0,2) = 0; // vel angle1
    _jacobianOplus[1](0,2) = 0; // vel angle2
  }
#endif
#endif
 
  
public:
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};






/**
 * @class EdgeVelocityHolonomic
 * 这个是全向底盘,先不用管
 * @brief Edge defining the cost function for limiting the translational and rotational velocity according to x,y and theta.
 * 
 * The edge depends on three vertices \f$ \mathbf{s}_i, \mathbf{s}_{ip1}, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyInterval}( [vx,vy,omega]^T ) \cdot weight \f$. \n
 * \e vx denotes the translational velocity w.r.t. x-axis (computed using finite differneces). \n
 * \e vy denotes the translational velocity w.r.t. y-axis (computed using finite differneces). \n
 * \e omega is calculated using the difference quotient of both yaw angles followed by a normalization to [-pi, pi]. \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyInterval denotes the penalty function, see penaltyBoundToInterval(). \n
 * The dimension of the error / cost vector is 3: the first component represents the translational velocity w.r.t. x-axis,
 * the second one w.r.t. the y-axis and the third one the rotational velocity.
 * @see TebOptimalPlanner::AddEdgesVelocity
 * @remarks Do not forget to call setTebConfig()
 */  
class EdgeVelocityHolonomic : public BaseTebMultiEdge<3, double>
{
public:
  
  /**
   * @brief Construct edge.
   */       
  EdgeVelocityHolonomic()
  {
    this->resize(3); // Since we derive from a g2o::BaseMultiEdge, set the desired number of vertices
  }
  
  /**
   * @brief Actual cost function
   */  
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVelocityHolonomic()");
    const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
    const VertexTimeDiff* deltaT = static_cast<const VertexTimeDiff*>(_vertices[2]);
    Eigen::Vector2d deltaS = conf2->position() - conf1->position();
    
    double cos_theta1 = std::cos(conf1->theta());
    double sin_theta1 = std::sin(conf1->theta()); 
    
    // transform conf2 into current robot frame conf1 (inverse 2d rotation matrix)
    double r_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double r_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
    
    double vx = r_dx / deltaT->estimate();
    double vy = r_dy / deltaT->estimate();
    double omega = g2o::normalize_theta(conf2->theta() - conf1->theta()) / deltaT->estimate();
    
    _error[0] = penaltyBoundToInterval(vx, -cfg_->robot.max_vel_x_backwards, cfg_->robot.max_vel_x, cfg_->optim.penalty_epsilon);
    _error[1] = penaltyBoundToInterval(vy, cfg_->robot.max_vel_y, 0.0); // we do not apply the penalty epsilon here, since the velocity could be close to zero
    _error[2] = penaltyBoundToInterval(omega, cfg_->robot.max_vel_theta,cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]) && std::isfinite(_error[2]),
                   "EdgeVelocityHolonomic::computeError() _error[0]=%f _error[1]=%f _error[2]=%f\n",_error[0],_error[1],_error[2]);
  }
 
  
public:
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};


} // end namespace

#endif
