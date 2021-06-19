// 这些类都是虚类,都需要重写
// 如果不适用多线程并行处理,那么这个实现就是TebOptimalPlanner
#ifndef PLANNER_INTERFACE_H_
#define PLANNER_INTERFACE_H_

// boost
#include <boost/shared_ptr.hpp>

// ros
#include <tf/transform_datatypes.h>
#include <base_local_planner/costmap_model.h>

// this package
#include <teb_local_planner/pose_se2.h>

// messages
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace teb_local_planner
{

  /**
 * @class PlannerInterface
 * @brief This abstract class defines an interface for local planners
 * 这个抽象类为局部规划器定义了一个接口,也就是teb采用这个规划器进行规划的
 */
  class PlannerInterface
  {
  public:
    /**
   * @brief Default constructor
   */
    PlannerInterface()
    {
    }
    /**
   * @brief Virtual destructor.
   */
    virtual ~PlannerInterface()
    {
    }

    /** @name Plan a trajectory */
    //@{

    /**
     * 根据初始的参照规划（位姿序列）、或者起点终点，提供方法去规划和创造一条轨迹。
   * @brief Plan a trajectory based on an initial reference plan.
   * 
   * Provide this method to create and optimize a trajectory that is initialized
   * according to an initial reference plan (given as a container of poses).
   * @param initial_plan vector of geometry_msgs::PoseStamped
   * @param start_vel Current start velocity (e.g. the velocity of the robot, only linear.x and angular.z are used)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *        otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
    virtual bool plan(const std::vector<geometry_msgs::PoseStamped> &initial_plan, const geometry_msgs::Twist *start_vel = NULL, bool free_goal_vel = false) = 0;

    /**
   * @brief Plan a trajectory between a given start and goal pose (tf::Pose version).
   * 
   * Provide this method to create and optimize a trajectory that is initialized between a given start and goal pose.
   * @param start tf::Pose containing the start pose of the trajectory
   * @param goal tf::Pose containing the goal pose of the trajectory
   * @param start_vel Current start velocity (e.g. the velocity of the robot, only linear.x and angular.z are used)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *        otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
    virtual bool plan(const tf::Pose &start, const tf::Pose &goal, const geometry_msgs::Twist *start_vel = NULL, bool free_goal_vel = false) = 0;

    /**
   * @brief Plan a trajectory between a given start and goal pose.
   * 
   * Provide this method to create and optimize a trajectory that is initialized between a given start and goal pose.
   * @param start PoseSE2 containing the start pose of the trajectory
   * @param goal PoseSE2 containing the goal pose of the trajectory
   * @param start_vel Initial velocity at the start pose (twist msg containing the translational and angular velocity).
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed,
   *        otherwise the final velocity will be zero (default: false)
   * @return \c true if planning was successful, \c false otherwise
   */
    virtual bool plan(const PoseSE2 &start, const PoseSE2 &goal, const geometry_msgs::Twist *start_vel = NULL, bool free_goal_vel = false) = 0;

    /**
     * 获取用于控制机器人的速度命令
   * @brief Get the velocity command from a previously optimized plan to control the robot at the current sampling interval.
   * @warning Call plan() first and check if the generated plan is feasible.
   * @param[out] vx translational velocity [m/s]
   * @param[out] vy strafing velocity which can be nonzero for holonomic robots [m/s] 
   * @param[out] omega rotational velocity [rad/s]
   * @param[in] look_ahead_poses index of the final pose used to compute the velocity command.
   * @return \c true if command is valid, \c false otherwise
   */
    virtual bool getVelocityCommand(double &vx, double &vy, double &omega, int look_ahead_poses) const = 0;

    //@}

    /**
   * @brief Reset the planner.
   */
    virtual void clearPlanner() = 0;

    /**
   * @brief Prefer a desired initial turning direction (by penalizing the opposing one)
   * 
   * A desired (initial) turning direction might be specified in case the planned trajectory oscillates between two 
   * solutions (in the same equivalence class!) with similar cost. Check the parameters in order to adjust the weight of the penalty.
   * Initial means that the penalty is applied only to the first few poses of the trajectory.
   * @param dir This parameter might be RotType::left (prefer left), RotType::right (prefer right) or RotType::none (prefer none)
   */
    virtual void setPreferredTurningDir(RotType dir) { ROS_WARN("setPreferredTurningDir() not implemented for this planner."); }

    /**
   * @brief Visualize planner specific stuff.
   * Overwrite this method to provide an interface to perform all planner related visualizations at once.
   */
    virtual void visualize()
    {
    }

    /**
     * 通过检测轨迹的一部分是否与障碍碰撞而检查轨迹是否可行。
   * @brief Check whether the planned trajectory is feasible or not.
   * 
   * This method currently checks only that the trajectory, or a part of the trajectory is collision free.
   * Obstacles are here represented as costmap instead of the internal ObstacleContainer.
   * @param costmap_model Pointer to the costmap model
   * @param footprint_spec The specification of the footprint of the robot in world coordinates
   * @param inscribed_radius The radius of the inscribed circle of the robot
   * @param circumscribed_radius The radius of the circumscribed circle of the robot
   * @param look_ahead_idx Number of poses along the trajectory that should be verified, if -1, the complete trajectory will be checked.
   * @return \c true, if the robot footprint along the first part of the trajectory intersects with 
   *         any obstacle in the costmap, \c false otherwise.
   */
    virtual bool isTrajectoryFeasible(base_local_planner::CostmapModel *costmap_model, const std::vector<geometry_msgs::Point> &footprint_spec,
                                      double inscribed_radius = 0.0, double circumscribed_radius = 0.0, int look_ahead_idx = -1) = 0;

    /**
     * 返回目前的图优化的花费。
   * Compute and return the cost of the current optimization graph (supports multiple trajectories)
   * @param[out] cost current cost value for each trajectory
   *                  [for a planner with just a single trajectory: size=1, vector will not be cleared]
   * @param obst_cost_scale Specify extra scaling for obstacle costs
   * @param alternative_time_cost Replace the cost for the time optimal objective by the actual (weighted) transition time
   */
    virtual void computeCurrentCost(std::vector<double> &cost, double obst_cost_scale = 1.0, bool alternative_time_cost = false)
    {
    }
  };

  //! Abbrev. for shared instances of PlannerInterface or it's subclasses
  typedef boost::shared_ptr<PlannerInterface> PlannerInterfacePtr;

} // namespace teb_local_planner

#endif /* PLANNER_INTERFACE_H__ */
