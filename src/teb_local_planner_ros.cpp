#include <teb_local_planner/teb_local_planner_ros.h>

#include <tf_conversions/tf_eigen.h>

#include <boost/algorithm/string.hpp>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace teb_local_planner
{

  TebLocalPlannerROS::TebLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL),
                                             costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                             dynamic_recfg_(NULL), custom_via_points_active_(false), goal_reached_(false), no_infeasible_plans_(0),
                                             last_preferred_rotdir_(RotType::none), initialized_(false)
  {
  }

  TebLocalPlannerROS::~TebLocalPlannerROS()
  {
  }

  void TebLocalPlannerROS::reconfigureCB(TebLocalPlannerReconfigureConfig &config, uint32_t level)
  {
    cfg_.reconfigure(config);
  }
  //cfg_就是存储和管理所有相关参数的配置类
  void TebLocalPlannerROS::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
  {
    // check if the plugin is already initialized
    if (!initialized_)
    {
      // create Node Handle with name of plugin (as used in move_base for loading)
      //这个函数就是从参数服务器加载数据
      ros::NodeHandle nh("~/" + name);

      // get parameters of TebConfig via the nodehandle and override the default config
      //这个函数就是从参数服务器加载数据
      cfg_.loadRosParamFromNodeHandle(nh);

      // reserve some memory for obstacles
      // 为障碍物储备一些记忆
      // 开辟新的空间
      obstacles_.reserve(500);

      // create visualization instance
      // 创建可视化实例,最后再看
      visualization_ = TebVisualizationPtr(new TebVisualization(nh, cfg_));

      // create robot footprint/contour model for optimization
      //从参数服务器上获取机器人的模型，存在变量robot_model中，根据机器人是什么形状的对对应的参数进行赋值
      // 得到机器人当前的轮廓,我们使用的圆,圆的参数就是只有圆的半径
      // RobotFootprintModelPtr这个是一个父类,
      RobotFootprintModelPtr robot_model = getRobotFootprintFromParamServer(nh);

      // create the planner instance
      //判断是否激活并行规划（就是出现多条路径）
      // 首先看是否是使用多路径同时规划,假设我们不适用
      if (cfg_.hcp.enable_homotopy_class_planning)
      {
        //初始化并行规划器
        planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_));
        ROS_INFO("Parallel planning in distinctive topologies enabled.");
      }
      else
      {
        //初始化普通的规划器
        //obstacles_是一个空的
        //robot_model存储着机器人的形状信息
        //visualization_创建可视化实例
        //via_points_在局部轨迹优化过程中需要考虑的节点容器
        planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_));
        ROS_INFO("Parallel planning in distinctive topologies disabled.");
      }

      // init other variables
      tf_ = tf;
      //获取costmap地图
      costmap_ros_ = costmap_ros;
      //costmap_2d::Costmap2D* costmap_;
      //是一个指针，指向2d地图
      //就是主代价地图
      costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

      costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);
      //获取世界坐标系
      global_frame_ = costmap_ros_->getGlobalFrameID();
      //给cfg_
      cfg_.map_frame = global_frame_; // TODO
      //机器人坐标系
      robot_base_frame_ = costmap_ros_->getBaseFrameID();

      //Initialize a costmap to polygon converter
      //初始化costmap到多边形转换器
      //这个函数的作用就是对costmap进行实时的转换
      //这些插件将占用的costmap_2d单元格转换为几何图元（点，线，多边形）作为障碍物。
      //在不激活任何插件的情况下，每个占用的costmap单元被视为单点障碍。
      //一般是不激活的。就是把它作为点障碍
      if (!cfg_.obstacles.costmap_converter_plugin.empty())
      {
        try
        {
          //创建一个转换器，并且进行存储
          costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
          std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
          // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
          boost::replace_all(converter_name, "::", "/");
          costmap_converter_->setOdomTopic(cfg_.odom_topic);
          costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
          costmap_converter_->setCostmap2D(costmap_);

          costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
          ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");
        }
        catch (pluginlib::PluginlibException &ex)
        {
          ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
          costmap_converter_.reset();
        }
      }
      else
        //没有指定costmap转换插件。所有被占据的costmap单元都被视为点障碍
        ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");

      // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
      //获取机器人的轮廓，以及从机器人中心到其足迹顶点的最小和最大距离。
      footprint_spec_ = costmap_ros_->getRobotFootprint();
      //就是根据机器人的robot_inscribed_radius_内切圆半径和robot_circumscribed_radius外切圆半径还有机器人的足迹计算最小和最大距离。
      //不知道是怎么计算的，感觉也不需要知道
      costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);

      // init the odom helper to receive the robot's velocity from odom messages
      //设置里程计话题，话题为odom_topic
      odom_helper_.setOdomTopic(cfg_.odom_topic);

      // setup dynamic reconfigure
      dynamic_recfg_ = boost::make_shared<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>>(nh);
      dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&TebLocalPlannerROS::reconfigureCB, this, _1, _2);
      dynamic_recfg_->setCallback(cb);

      // validate optimization footprint and costmap footprint
      //验证优化占用空间和成本图占用空间
      //足迹验证函数
      // 这个函数的作用就是比较teb参数中的轮廓+min_obstacle_dist与costmap中的半径大小
      // 如果加起来还没有costmap中大,就发出警告
      // 测试robot_inscribed_radius_为0.303975
      validateFootprints(robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);

      // 这两个custom_obst_sub_和via_points_sub_一般情况下是不会运行的

      // setup callback for custom obstacles
      //订阅obstacles
      // 应该是加入自己的障碍物，表示自己加的是动态的障碍物
      custom_obst_sub_ = nh.subscribe("obstacles", 1, &TebLocalPlannerROS::customObstacleCB, this);
      // setup callback for custom via-points
      //订阅via_points,调用函数customViaPointsCB
      // 如果通过点是空的,那么custom_via_points_active_就是false
      via_points_sub_ = nh.subscribe("via_points", 1, &TebLocalPlannerROS::customViaPointsCB, this);

      // initialize failure detector
      //初始化故障检测
      ros::NodeHandle nh_move_base("~");
      double controller_frequency = 5;
      nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
      // 就是设置oscillation_filter_duration乘以controller_frequency
      // 就是oscillation_filter_duration时间内,要进行多少次局部路径规划
      // 对boost::circular_buffer<VelMeasurement> buffer_进行设置
      // oscillation_filter_duration这个参数可以设置环形缓存区的大小
      failure_detector_.setBufferLength(std::round(cfg_.recovery.oscillation_filter_duration * controller_frequency));

      // set initialized flag
      initialized_ = true;

      ROS_DEBUG("teb_local_planner plugin initialized.");
    }
    else
    {
      ROS_WARN("teb_local_planner has already been initialized, doing nothing.");
    }
  }

  //就是把接收到的全局规划路径给global_plan_
  bool TebLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
  {
    // check if plugin is initialized
    if (!initialized_)
    {
      ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    // store the global plan
    global_plan_.clear();
    //就是把接收到的全局规划路径给global_plan_
    global_plan_ = orig_global_plan;

    // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
    // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

    // reset goal_reached_ flag
    goal_reached_ = false;

    return true;
  }

  bool TebLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    // check if plugin initialized
    if (!initialized_)
    {
      ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    goal_reached_ = false;

    // Get robot pose
    //从代价地图中获得机器人的全局坐标系下的位置
    //存在robot_pose_
    tf::Stamped<tf::Pose> robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    //robot_pose_这个函数存储的就是xy坐标和旋转角theta
    // 统一数据类型
    robot_pose_ = PoseSE2(robot_pose);

    // Get robot velocity
    //存储速度
    tf::Stamped<tf::Pose> robot_vel_tf;
    odom_helper_.getRobotVel(robot_vel_tf);
    robot_vel_.linear.x = robot_vel_tf.getOrigin().getX();
    robot_vel_.linear.y = robot_vel_tf.getOrigin().getY();
    robot_vel_.angular.z = tf::getYaw(robot_vel_tf.getRotation());
    // prune global plan to cut off parts of the past (spatially before the robot)
    //这个函数的作用就是删除全局路径中通过的部分，不是全部删除，保留global_plan_prune_distance大小的，
    //就是直接在global_plan_上进行删除，就是机器人之前的路径
    // 就是删除之前的路径，不是全部删除，而是保留了dist_thresh_sq面积大小的
    // 这个是跟其他的不一样,其他的是1m.global_plan_prune_distance默认也是1m
    pruneGlobalPlan(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);

    // Transform global plan to the frame of interest (w.r.t. the local costmap)
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    int goal_idx;
    tf::StampedTransform tf_plan_to_global;

    // 这个函数和其他算法上的函数一样就是将global_plan进行判断
    // 得到global_plan里面离机器人最近的路径点到局部地图边界中间的点然后放在transformed_plan中
    // goal_idx是局部地图中的终点值
    // tf_plan_to_global这个是坐标变换矩阵,一般是值很小,除非中间的frame更换
    if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist,
                             transformed_plan, &goal_idx, &tf_plan_to_global))
    {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    // update via-points container
    // 更新路径上的航迹点
    // custom_via_points_active_这个一般为false,所以这个是进去的
    if (!custom_via_points_active_)
      //其中transformed_plan就是已经弄好的局部路径规划
      //global_plan_viapoint_sep两个连续点之间的最小距离,处理后把通过点放到via_points_中
      updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);

    // check if global goal is reached
    tf::Stamped<tf::Pose> global_goal;
    //global_goal这个值就是将局部路径规划的global_plan_（已经被修剪过了）最后一个元素给global_goal
    //如果是到达目标或者几乎接近目标就会执行下面的函数
    tf::poseStampedMsgToTF(global_plan_.back(), global_goal);
    // 坐标系变换,一般变化很小
    global_goal.setData(tf_plan_to_global * global_goal);
    double dx = global_goal.getOrigin().getX() - robot_pose_.x();
    double dy = global_goal.getOrigin().getY() - robot_pose_.y();
    // 把角度设置到-pi到pi之间
    double delta_orient = g2o::normalize_theta(tf::getYaw(global_goal.getRotation()) - robot_pose_.theta());
    // 判断是否到达目标点
    if (fabs(std::sqrt(dx * dx + dy * dy)) < cfg_.goal_tolerance.xy_goal_tolerance && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0))
    {
      goal_reached_ = true;
      return true;
    }

    // check if we should enter any backup mode and apply settings
    //检查我们是否应该进入备份模式并应用设置
    // 如果不合法的路径点生成,那么要做的就是减少预测域的长度,也就是将transformed_plan(局部路径规划)缩短
    // 根据是否震荡,储存最近首选的转向方向
    configureBackupModes(transformed_plan, goal_idx);

    // Return false if the transformed global plan is empty
    if (transformed_plan.empty())
    {
      ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
      return false;
    }

    // Get current goal point (last point of the transformed plan)
    //获得当前目标点(已转换计划的最后一点)
    tf::Stamped<tf::Pose> goal_point;
    //获得局部路径规划的目标点，也就是transformed_plan最后一个点，给goal_point
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    robot_goal_.x() = goal_point.getOrigin().getX();
    robot_goal_.y() = goal_point.getOrigin().getY();
    //下面的判断语句就是是否重写局部子目标的方向，
    //看不懂也没事，就是把目标方向给 robot_goal_.theta()
    //所以下面就是让机器人向着robot_goal_前进
    // http://wiki.ros.org/teb_local_planner/Tutorials/Frequently%20Asked%20Questions
    // 里面有一段是对这个进行说明的
    if (cfg_.trajectory.global_plan_overwrite_orientation)
    {
      // 根据当前的目标点和未来的目标点,就算出合适的目标角度
      robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, goal_point, goal_idx, tf_plan_to_global);
      // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
      transformed_plan.back().pose.orientation = tf::createQuaternionMsgFromYaw(robot_goal_.theta());
    }
    else
    {
      robot_goal_.theta() = tf::getYaw(goal_point.getRotation());
    }

    // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
    // 用机器人的实际位置覆盖/更新transformed_plan的开始点的位姿(允许使用计划作为初始轨迹)
    //就是如果只有一个，就是只包含目标
    //这样是不好的，要做的就是再里面再加入一个值，加入的这个值就是机器人的实际位置
    if (transformed_plan.size() == 1) // plan only contains the goal
    {
      //就是再transformed_plan.begin()这个位置前面插入geometry_msgs::PoseStamped()值,注意这个值还没有进行初始化
      transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
    }
    //和begin不一样，begin是返回一个迭代器，而front是返回一个直接引用。
    //就是将robot_pose赋值给transformed_plan.front().pose,就是刚才没初始化的
    //就是上面说的用机器人的实际位置覆盖/更新transformed_plan的开始
    tf::poseTFToMsg(robot_pose, transformed_plan.front().pose); // update start;

    // clear currently existing obstacles
    //清理上一次处理的障碍物的信息
    obstacles_.clear();

    // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
    // 用costmap_converter插件提供的costmap信息或多边形更新障碍物容器
    // 这个是运行不进去的,一般运行下面的
    if (costmap_converter_)
      updateObstacleContainerWithCostmapConverter();
    else //一般是运行这个
      //这个函数就是将障碍物信息放进到obstacles_里面
      //注意 在机器人后面,并且超过了costmap_obstacles_behind_robot_dist的障碍物不予考虑
      updateObstacleContainerWithCostmap();

    // also consider custom obstacles (must be called after other updates, since the container is not cleared)
    //也考虑自定义障碍(必须在其他更新后调用，因为容器没有被清除
    //这个函数也是更新障碍物信息，同样存储在obstacles_里面
    // 这个一般是不运行的,因为这个话题不会发布
    updateObstacleContainerWithCustomObstacles();

    // Do not allow config changes during the following optimization step
    // 在下列优化步骤中不允许更改配置
    boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

    // Now perform the actual planning
    //   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_.goal_tolerance.free_goal_vel); // straight line init
    //robot_vel_就是开始速度
    // transformed_plan这个就是局部路径规划的路径点
    bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
    if (!success)
    {
      planner_->clearPlanner(); // force reinitialization for next time
      ROS_WARN("teb_local_planner was not able to obtain a local plan for the current setting.");
      // 如果不成功,那么不成功次数加1
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      // 更新时间
      time_last_infeasible_plan_ = ros::Time::now();
      last_cmd_ = cmd_vel;
      return false;
    }

    // Check feasibility (but within the first few states only)
    //检查可行性(但仅限于最初的几个状态)
    //is_footprint_dynamic一般为false
    if (cfg_.robot.is_footprint_dynamic)
    {
      // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
      // 更新机器人的足迹footprint_spec_，以及从机器人中心到足迹顶点的最小和最大距离。
      footprint_spec_ = costmap_ros_->getRobotFootprint();
      costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
    }
    // 检查轨迹的冲突情况
    // footprint_spec机器人的轮廓
    // look_ahead_idx指定每个采样间隔应在预测计划的哪个位置进行可行性检查。检测姿态在规划路径的可行性的时间间隔
    // 检查两个姿态之间的距离是否大于机器人半径或方向差值大于指定的阈值，并在这种情况下进行插值。
    // 这个为什么进行差值呢,因为图中可能两个位姿节点之间的距离变大,障碍物位于中间
    // 所以要做的就是插值,在两个节点之间插值,然后判断是否和障碍物相撞
    bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses);
    // 表示轨迹是冲突的
    if (!feasible)
    {
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0;
      cmd_vel.angular.z = 0;

      // now we reset everything to start again with the initialization of new trajectories.
      planner_->clearPlanner();
      ROS_WARN("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...");
      // 如果轨迹不正常,那么也加1
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();
      last_cmd_ = cmd_vel;
      return false;
    }

    // Get the velocity command for this sampling interval
    //获取这个采样间隔的速度命令
    //检查速度指令是否有效
    // control_look_ahead_poses用于提取速度命令的姿态索引
    // look_ahead_poses什么意思呢?就是计算好图之后,位姿节点和时间节点都是会更新的,那么计算速度就是计算前几个(control_look_ahead_poses)位姿节点和时间节点
    // 默认control_look_ahead_poses为1
    // 根据图,计算速度放在cmd_vel
    if (!planner_->getVelocityCommand(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_.trajectory.control_look_ahead_poses))
    {
      planner_->clearPlanner();
      ROS_WARN("TebLocalPlannerROS: velocity command invalid. Resetting planner...");
      // 说明这个计算出来的最优解不正常,进行计数
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();
      // 不正常的速度是0,记录这个速度
      last_cmd_ = cmd_vel;
      return false;
    }

    // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
    //速度饱和，如果优化结果违反约束(可能是由于软约束)。
    //就是对速度进行一定的限制
    //超过一定速度的时候，就将速度设置为最大值
    saturateVelocity(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_.robot.max_vel_x, cfg_.robot.max_vel_y,
                     cfg_.robot.max_vel_theta, cfg_.robot.max_vel_x_backwards);

    // convert rot-vel to steering angle if desired (carlike robot).如果需要的话，将rot-vel转换为转向角度(类车机器人)
    // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
    // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
    //对于全向机器人底盘差速轮是false，这个是不用的cmd_angle_instead_rotvel这个为应该是false
    // 对于车辆类型的这个是true
    if (cfg_.robot.cmd_angle_instead_rotvel)
    {
      cmd_vel.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.linear.x, cmd_vel.angular.z, cfg_.robot.wheelbase, 0.95 * cfg_.robot.min_turning_radius);
      if (!std::isfinite(cmd_vel.angular.z))
      {
        cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;
        last_cmd_ = cmd_vel;
        planner_->clearPlanner();
        ROS_WARN("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
        ++no_infeasible_plans_; // increase number of infeasible solutions in a row
        time_last_infeasible_plan_ = ros::Time::now();
        return false;
      }
    }

    // a feasible solution should be found, reset counter
    // 能走到这,说明一切正常,那么no_infeasible_plans_就会设置为0
    no_infeasible_plans_ = 0;

    // store last command (for recovery analysis etc.)
    //存储在computevelocitycommands中生成的最后一个控制命令。
    //但是给computevelocitycommands的速度还是cmd_vel
    last_cmd_ = cmd_vel;

    // Now visualize everything
    planner_->visualize();
    visualization_->publishObstacles(obstacles_);
    // 通过点
    //transformed_plan就是已经弄好的局部路径规划
    //min_separation两个连续点之间的最小距离,处理后把通过点放到via_points_中
    visualization_->publishViaPoints(via_points_);
    visualization_->publishGlobalPlan(global_plan_);
    return true;
  }

  bool TebLocalPlannerROS::isGoalReached()
  {
    if (goal_reached_)
    {
      ROS_INFO("GOAL Reached!");
      planner_->clearPlanner();
      return true;
    }
    return false;
  }

  //这个函数就是将障碍物信息放进到obstacles_里面
  //注意 在机器人后面,并且超过了costmap_obstacles_behind_robot_dist的障碍物不予考虑
  void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
  {
    // Add costmap obstacles if desired
    // 是否考虑 local_costmap 中的障碍。标记为障碍的每个单元格被认为是一个点障碍。
    // 因此,不要选择一个非常小的成本图分辨率,因为它增加了计算时间。默认 true。
    if (cfg_.obstacles.include_costmap_obstacles)
    {
      // 方向向量
      Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

      for (unsigned int i = 0; i < costmap_->getSizeInCellsX() - 1; ++i)
      {
        for (unsigned int j = 0; j < costmap_->getSizeInCellsY() - 1; ++j)
        {
          if (costmap_->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
          {
            Eigen::Vector2d obs;
            costmap_->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

            // check if obstacle is interesting (e.g. not far behind the robot)
            // 检查障碍物是否有用(例如，离机器人不远)
            Eigen::Vector2d obs_dir = obs - robot_pose_.position();
            // obs_dir.dot(robot_orient) < 0表示障碍物在机器人的后方
            // obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist表示障碍物离机器人太远了
            // 限制在机器人后面规划时考虑到的被占用的局部costmap障碍物(指定距离以米为单位)
            // 这个函数的意思是在机器人后面,并且超过了costmap_obstacles_behind_robot_dist,那么就不考虑了,直接删除
            if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist)
              continue;

            obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
          }
        }
      }
    }
  }
  // 这个一般不会运行的
  void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
  {
    if (!costmap_converter_)
      return;

    //Get obstacles from costmap converter
    costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
    if (!obstacles)
      return;

    for (std::size_t i = 0; i < obstacles->obstacles.size(); ++i)
    {
      const costmap_converter::ObstacleMsg *obstacle = &obstacles->obstacles.at(i);
      const geometry_msgs::Polygon *polygon = &obstacle->polygon;

      if (polygon->points.size() == 1 && obstacle->radius > 0) // Circle
      {
        obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
      }
      else if (polygon->points.size() == 1) // Point
      {
        obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
      }
      else if (polygon->points.size() == 2) // Line
      {
        obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                          polygon->points[1].x, polygon->points[1].y)));
      }
      else if (polygon->points.size() > 2) // Real polygon
      {
        PolygonObstacle *polyobst = new PolygonObstacle;
        for (std::size_t j = 0; j < polygon->points.size(); ++j)
        {
          polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
      }

      // Set velocity, if obstacle is moving
      if (!obstacles_.empty())
        obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
    }
  }

  //这个函数也是更新障碍物信息，同样存储在obstacles_里面
  // 自己没有发布这个obstacle的话题,所以这个进去之后立马就返回
  void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
  {
    // Add custom obstacles obtained via message
    //  添加通过消息获得的自定义障碍
    boost::mutex::scoped_lock l(custom_obst_mutex_);

    if (!custom_obstacle_msg_.obstacles.empty())
    {
      // We only use the global header to specify the obstacle coordinate system instead of individual ones
      //我们只使用全局头来指定障碍坐标系统，而不是单独的
      Eigen::Affine3d obstacle_to_map_eig;
      try
      {
        tf::StampedTransform obstacle_to_map;
        tf_->waitForTransform(global_frame_, ros::Time(0),
                              custom_obstacle_msg_.header.frame_id, ros::Time(0),
                              custom_obstacle_msg_.header.frame_id, ros::Duration(0.5));
        tf_->lookupTransform(global_frame_, ros::Time(0),
                             custom_obstacle_msg_.header.frame_id, ros::Time(0),
                             custom_obstacle_msg_.header.frame_id, obstacle_to_map);
        //obstacle_to_map转换为特征Affine3d--obstacle_to_map_eig。
        tf::transformTFToEigen(obstacle_to_map, obstacle_to_map_eig);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        obstacle_to_map_eig.setIdentity();
      }

      for (size_t i = 0; i < custom_obstacle_msg_.obstacles.size(); ++i)
      {
        if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0) // circle
        {
          Eigen::Vector3d pos(custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z);
          obstacles_.push_back(ObstaclePtr(new CircularObstacle((obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
        }
        else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1) // point
        {
          Eigen::Vector3d pos(custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z);
          obstacles_.push_back(ObstaclePtr(new PointObstacle((obstacle_to_map_eig * pos).head(2))));
        }
        else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2) // line
        {
          Eigen::Vector3d line_start(custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                     custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                     custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z);
          Eigen::Vector3d line_end(custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                   custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                   custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z);
          obstacles_.push_back(ObstaclePtr(new LineObstacle((obstacle_to_map_eig * line_start).head(2),
                                                            (obstacle_to_map_eig * line_end).head(2))));
        }
        else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
        {
          ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
          continue;
        }
        else // polygon
        {
          PolygonObstacle *polyobst = new PolygonObstacle;
          for (size_t j = 0; j < custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
          {
            Eigen::Vector3d pos(custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                                custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                                custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z);
            polyobst->pushBackVertex((obstacle_to_map_eig * pos).head(2));
          }
          polyobst->finalizePolygon();
          obstacles_.push_back(ObstaclePtr(polyobst));
        }

        // Set velocity, if obstacle is moving
        if (!obstacles_.empty())
          obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
      }
    }
  }

  //这个函数就是将transformed_plan中的里的太近的点去除最后合适的点放入到via_points_中
  //基于当前的参考路径更新内部via_points_容器
  //updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);
  //其中transformed_plan就是已经弄好的局部路径规划
  //min_separation两个连续点之间的最小距离,处理后把通过点放到via_points_中
  void TebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double min_separation)
  {
    via_points_.clear();

    if (min_separation <= 0)
      return;

    std::size_t prev_idx = 0;

    // skip first one, since we do not need any point before the first min_separation [m]
    //这个循环的作用就是将transformed_plan中的里的太近的点去除最后放入到via_points_中
    // 这个循环就是遍历所有的点每隔min_separation距离,往via_points_里面插入一个点
    for (std::size_t i = 1; i < transformed_plan.size(); ++i)
    {
      // check separation to the previous via-point inserted
      // 检查分离到前面插入的通过点
      if (distance_points2d(transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position) < min_separation)
        continue;

      // add via-point
      via_points_.push_back(Eigen::Vector2d(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y));
      prev_idx = i;
    }
  }

  Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose &tf_vel)
  {
    Eigen::Vector2d vel;
    vel.coeffRef(0) = std::sqrt(tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY());
    vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
    return vel;
  }

  //pruneGlobalPlan(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);
  // 就是删除之前的路径，不是全部删除，而是保留了dist_thresh_sq面积大小的
  // 这个是跟其他的不一样,其他的是1m
  bool TebLocalPlannerROS::pruneGlobalPlan(const tf::TransformListener &tf, const tf::Stamped<tf::Pose> &global_pose, std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot)
  {
    if (global_plan.empty())
      return true;

    try
    {
      // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
      // 将机器人的姿态转换为plan frame(我们不在这里等待，因为修剪不重要，可以错过了几次)
      tf::StampedTransform global_to_plan_transform;
      tf.lookupTransform(global_plan.front().header.frame_id, global_pose.frame_id_, ros::Time(0), global_to_plan_transform);
      tf::Stamped<tf::Pose> robot;
      //robot就是实际位置=全局到计划的转换矩阵*机器人位置
      robot.setData(global_to_plan_transform * global_pose);

      double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

      // iterate plan until a pose close the robot is found
      //具体的看下面的描述
      std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
      std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
      while (it != global_plan.end())
      {
        double dx = robot.getOrigin().x() - it->pose.position.x;
        double dy = robot.getOrigin().y() - it->pose.position.y;
        double dist_sq = dx * dx + dy * dy;
        //就是机器人现在的位置从路径的开始带末尾位置计算
        //就是删除之前的路径，不是全部删除，而是保留了dist_thresh_sq面积大小的
        //dist_thresh_sq这个应该很小
        if (dist_sq < dist_thresh_sq)
        {
          erase_end = it;
          break;
        }
        ++it;
      }
      // 表示已经到达末端,还没有找到
      if (erase_end == global_plan.end())
        return false;

      if (erase_end != global_plan.begin())
        //删除向量中[first,last)中元素
        //就是把这一段删除
        // erase_end是离机器人dist_thresh_sq远的路径点(已经走过的路径)
        global_plan.erase(global_plan.begin(), erase_end);
    }
    catch (const tf::TransformException &ex)
    {
      ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
      return false;
    }
    return true;
  }

  // 这个函数和其他算法上的函数一样就是将global_plan进行判断
  // 得到global_plan里面离机器人最近的路径点到局部地图边界中间的点然后放在transformed_plan中
  // current_goal_idx是局部地图中的终点值
  // tf_plan_to_global这个是坐标变换矩阵,一般是单位阵(或者接近单位阵),除非中间的frame更换
  bool TebLocalPlannerROS::transformGlobalPlan(const tf::TransformListener &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                               const tf::Stamped<tf::Pose> &global_pose, const costmap_2d::Costmap2D &costmap, const std::string &global_frame, double max_plan_length,
                                               std::vector<geometry_msgs::PoseStamped> &transformed_plan, int *current_goal_idx, tf::StampedTransform *tf_plan_to_global) const
  {
    // this method is a slightly modified version of base_local_planner/goal_functions.h
    // 这个方法是基本base_local_planner/goal_functions.h的一个稍微修改版本
    const geometry_msgs::PoseStamped &plan_pose = global_plan[0];

    transformed_plan.clear();

    try
    {
      if (global_plan.empty())
      {
        ROS_ERROR("Received plan with zero length");
        *current_goal_idx = 0;
        return false;
      }

      // get plan_to_global_transform from plan frame to global_frame
      tf::StampedTransform plan_to_global_transform;
      tf.waitForTransform(global_frame, ros::Time::now(),
                          plan_pose.header.frame_id, plan_pose.header.stamp,
                          plan_pose.header.frame_id, ros::Duration(0.5));
      tf.lookupTransform(global_frame, ros::Time(),
                         plan_pose.header.frame_id, plan_pose.header.stamp,
                         plan_pose.header.frame_id, plan_to_global_transform);

      //let's get the pose of the robot in the frame of the plan
      //让我们得到机器人在计划框架中的姿势
      tf::Stamped<tf::Pose> robot_pose;
      tf.transformPose(plan_pose.header.frame_id, global_pose, robot_pose);

      //we'll discard points on the plan that are outside the local costmap
      ///我们将放弃计划上当地成本图以外的一些点
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                       costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
      //只需考虑成本图大小的85％，以更好地合并位于本地成本图边界上的点障碍物
      dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                              // located on the border of the local costmap

      int i = 0;
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = 1e10;

      //we need to loop to a point on the plan that is within a certain distance of the robot
      //我们需要循环到路径上距离机器人一定距离内的一点
      //就是只要局部路径的点，范围是sq_dist_threshold

      //这个函数就是找到global_plan（就是修剪过的全局路径）中距离现在机器人最近的一个点
      for (int j = 0; j < (int)global_plan.size(); ++j)
      {
        double x_diff = robot_pose.getOrigin().x() - global_plan[j].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[j].pose.position.y;
        double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
        //如果我们到达成本图的边界，强制停止
        if (new_sq_dist > sq_dist_threshold)
          break; // force stop if we have reached the costmap border

        if (new_sq_dist < sq_dist) // find closest distance
        {
          sq_dist = new_sq_dist;
          //这个i值就是距离机器人最近的一个点
          i = j;
        }
      }

      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;

      double plan_length = 0; // check cumulative Euclidean distance along the plan

      //now we'll transform until points are outside of our distance threshold
      //现在我们将变换直到点超出了距离阈值
      //这个是从机器人的点往以后的点推迟sq_dist_threshold
      while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || plan_length <= max_plan_length))
      {
        const geometry_msgs::PoseStamped &pose = global_plan[i];
        tf::poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(plan_to_global_transform * tf_pose);
        tf_pose.stamp_ = plan_to_global_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        tf::poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;

        // caclulate distance to previous pose
        // 计算到前一个姿势的距离
        if (i > 0 && max_plan_length > 0)
          plan_length += distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);

        ++i;
      }

      // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
      // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
      if (transformed_plan.empty())
      {
        tf::poseStampedMsgToTF(global_plan.back(), tf_pose);
        tf_pose.setData(plan_to_global_transform * tf_pose);
        tf_pose.stamp_ = plan_to_global_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        tf::poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        // Return the index of the current goal point (inside the distance threshold)
        if (current_goal_idx)
          *current_goal_idx = int(global_plan.size()) - 1;
      }
      else
      {
        // Return the index of the current goal point (inside the distance threshold)
        if (current_goal_idx)
          *current_goal_idx = i - 1; // subtract 1, since i was increased once before leaving the loop
      }

      // Return the transformation from the global plan to the global planning frame if desired
      if (tf_plan_to_global)
        *tf_plan_to_global = plan_to_global_transform;
    }
    catch (tf::LookupException &ex)
    {
      ROS_ERROR("No Transform available Error: %s\n", ex.what());
      return false;
    }
    catch (tf::ConnectivityException &ex)
    {
      ROS_ERROR("Connectivity Error: %s\n", ex.what());
      return false;
    }
    catch (tf::ExtrapolationException &ex)
    {
      ROS_ERROR("Extrapolation Error: %s\n", ex.what());
      if (global_plan.size() > 0)
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }
  // 根据当前的目标点和未来的目标点,就算出合适的目标角度
  double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped> &global_plan, const tf::Stamped<tf::Pose> &local_goal,
                                                          int current_goal_idx, const tf::StampedTransform &tf_plan_to_global, int moving_average_length) const
  {
    int n = (int)global_plan.size();

    // check if we are near the global goal already
    // 检查一下我们是否已经接近了global目标
    // 如果快到目标,就直接用目标点的方向就行了
    if (current_goal_idx > n - moving_average_length - 2)
    {
      if (current_goal_idx >= n - 1) // we've exactly reached the goal
      {
        return tf::getYaw(local_goal.getRotation());
      }
      else
      {
        tf::Quaternion global_orientation;
        tf::quaternionMsgToTF(global_plan.back().pose.orientation, global_orientation);
        return tf::getYaw(tf_plan_to_global.getRotation() * global_orientation);
      }
    }

    // reduce number of poses taken into account if the desired number of poses is not available
    // 如果所需的姿态数量不可用，则减少考虑的姿态数量
    // 也许是多余的，因为我们之前检查过goal附近
    moving_average_length = std::min(moving_average_length, n - current_goal_idx - 1); // maybe redundant, since we have checked the vicinity of the goal before

    std::vector<double> candidates;
    tf::Stamped<tf::Pose> tf_pose_k = local_goal;
    tf::Stamped<tf::Pose> tf_pose_kp1;

    int range_end = current_goal_idx + moving_average_length;
    for (int i = current_goal_idx; i < range_end; ++i)
    {
      // Transform pose of the global plan to the planning frame
      const geometry_msgs::PoseStamped &pose = global_plan.at(i + 1);
      tf::poseStampedMsgToTF(pose, tf_pose_kp1);
      tf_pose_kp1.setData(tf_plan_to_global * tf_pose_kp1);

      // calculate yaw angle
      // 计算局部地图中的目标点和其在全局地图中的后面几个点的atan值,
      candidates.push_back(std::atan2(tf_pose_kp1.getOrigin().getY() - tf_pose_k.getOrigin().getY(),
                                      tf_pose_kp1.getOrigin().getX() - tf_pose_k.getOrigin().getX()));

      if (i < range_end - 1)
        tf_pose_k = tf_pose_kp1;
    }
    // 然后进行平均
    return average_angles(candidates);
  }

  //就是对速度进行一定的限制
  //超过一定速度的时候，就将速度设置为最大值
  void TebLocalPlannerROS::saturateVelocity(double &vx, double &vy, double &omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards) const
  {
    // Limit translational velocity for forward driving
    if (vx > max_vel_x)
      vx = max_vel_x;

    // limit strafing velocity
    if (vy > max_vel_y)
      vy = max_vel_y;
    else if (vy < -max_vel_y)
      vy = -max_vel_y;

    // Limit angular velocity
    if (omega > max_vel_theta)
      omega = max_vel_theta;
    else if (omega < -max_vel_theta)
      omega = -max_vel_theta;

    // Limit backwards velocity
    if (max_vel_x_backwards <= 0)
    {
      //通过增加对向后驾驶处罚的优化权重来禁用向后驾驶
      ROS_WARN_ONCE("TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
    }
    else if (vx < -max_vel_x_backwards)
      vx = -max_vel_x_backwards;


  }

  //全向轮底盘不用这个
  double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
  {
    if (omega == 0 || v == 0)
      return 0;

    double radius = v / omega;

    if (fabs(radius) < min_turning_radius)
      radius = double(g2o::sign(radius)) * min_turning_radius;

    return std::atan(wheelbase / radius);
  }
  // 检查自己设置的机器人轮廓是否符合要求
  void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
  {
    ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!",
                  opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
  }
  // 如果不合法的路径点生成,那么要做的就是减少预测域的长度,也就是将transformed_plan(局部路径规划)缩短
  // 根据是否震荡,储存最近首选的转向方向
  void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::PoseStamped> &transformed_plan, int &goal_idx)
  {
    ros::Time current_time = ros::Time::now();

    // reduced horizon backup mode
    // 允许规划器在自动检测到问题时缩小预测域临时(50%)。
    // 这个就是如果不合法的路径点生成,那么要做的就是减少预测域的长度,也就是将transformed_plan(局部路径规划)缩短
    if (cfg_.recovery.shrink_horizon_backup &&
        goal_idx < (int)transformed_plan.size() - 1 &&                                                                                 //如果目标已经被选择，我们不会减少(因为方向可能会改变->会引入振荡)。    // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
        (no_infeasible_plans_ > 0 || (current_time - time_last_infeasible_plan_).toSec() < cfg_.recovery.shrink_horizon_min_duration)) // keep short horizon for at least a few seconds
    {
      ROS_INFO_COND(no_infeasible_plans_ == 1, "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_.recovery.shrink_horizon_min_duration);

      // Shorten horizon if requested
      // reduce to 50 percent:
      int horizon_reduction = goal_idx / 2;
      // 就是降低4倍
      if (no_infeasible_plans_ > 9)
      {
        ROS_INFO_COND(no_infeasible_plans_ == 10, "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
        horizon_reduction /= 2;
      }
      //我们在这里有一个小开销，因为我们已经转换了50%以上的轨迹。
      //但是现在还可以，因为我们不需要让transformGlobalPlan变得更复杂，减少的视野应该很少发生。
      // we have a small overhead here, since we already transformed 50% more of the trajectory.
      // But that's ok for now, since we do not need to make transformGlobalPlan more complex
      // and a reduced horizon should occur just rarely.
      int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
      goal_idx -= horizon_reduction;
      if (new_goal_idx_transformed_plan > 0 && goal_idx >= 0)
        transformed_plan.erase(transformed_plan.begin() + new_goal_idx_transformed_plan, transformed_plan.end());
      else
        goal_idx += horizon_reduction; // this should not happen, but safety first ;-)
    }

    // detect and resolve oscillations
    if (cfg_.recovery.oscillation_recovery)
    {
      double max_vel_theta;
      double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_.robot.max_vel_x : cfg_.robot.max_vel_x_backwards;
      // 对于全向底盘和非全向底盘
      if (cfg_.robot.min_turning_radius != 0 && max_vel_current > 0)
        max_vel_theta = std::max(max_vel_current / std::abs(cfg_.robot.min_turning_radius), cfg_.robot.max_vel_theta);
      else
        max_vel_theta = cfg_.robot.max_vel_theta;

      failure_detector_.update(last_cmd_, cfg_.robot.max_vel_x, cfg_.robot.max_vel_x_backwards, max_vel_theta,
                               cfg_.recovery.oscillation_v_eps, cfg_.recovery.oscillation_omega_eps);
      // 返回机器人是否震荡标识符
      bool oscillating = failure_detector_.isOscillating();
      // 检查一下我们最近是否已经检测到振荡
      bool recently_oscillated = (ros::Time::now() - time_last_oscillation_).toSec() < cfg_.recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently

      if (oscillating)
      {
        // 最近没有震荡
        if (!recently_oscillated)
        {
          // save current turning direction
          // 保存机器人的旋转方向
          if (robot_vel_.angular.z > 0)
            last_preferred_rotdir_ = RotType::left;
          else
            // 保存机器人的旋转方向
            last_preferred_rotdir_ = RotType::right;
          ROS_WARN("TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
        }
        time_last_oscillation_ = ros::Time::now();
        planner_->setPreferredTurningDir(last_preferred_rotdir_);
      }
      // 就是最近没有震荡并且有最完美的旋转方向,那么就清空恢复行为
      else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
      {
        last_preferred_rotdir_ = RotType::none;
        // 储存最近首选的转向方向
        planner_->setPreferredTurningDir(last_preferred_rotdir_);
        ROS_INFO("TebLocalPlannerROS: oscillation recovery disabled/expired.");
      }
    }
  }
  // 一般不运行
  void TebLocalPlannerROS::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg)
  {
    boost::mutex::scoped_lock l(custom_obst_mutex_);
    custom_obstacle_msg_ = *obst_msg;
  }
  // 如果通过点是空的,那么custom_via_points_active_就是false
  // 一般不运行
  void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::Path::ConstPtr &via_points_msg)
  {
    ROS_INFO_ONCE("Via-points received. This message is printed once.");
    if (cfg_.trajectory.global_plan_viapoint_sep > 0)
    {
      ROS_WARN("Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
               "Ignoring custom via-points.");
      custom_via_points_active_ = false;
      return;
    }

    boost::mutex::scoped_lock l(via_point_mutex_);
    via_points_.clear();
    for (const geometry_msgs::PoseStamped &pose : via_points_msg->poses)
    {
      via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    // 如果是空的,那么custom_via_points_active_就是false
    custom_via_points_active_ = !via_points_.empty();
  }
  // 得到机器人当前的轮廓,我们使用的圆,圆的参数就是只有圆的半径
  RobotFootprintModelPtr TebLocalPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle &nh)
  {
    std::string model_name;
    if (!nh.getParam("footprint_model/type", model_name))
    {
      ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
      return boost::make_shared<PointRobotFootprint>();
    }

    // point
    if (model_name.compare("point") == 0)
    {
      ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
      return boost::make_shared<PointRobotFootprint>();
    }

    // circular
    // 这个就是我们需要采用的圆形
    // 从而我们创建类型是CircularRobotFootprint类型的轮廓模型
    if (model_name.compare("circular") == 0)
    {
      // get radius
      double radius;
      if (!nh.getParam("footprint_model/radius", radius))
      {
        ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                                  << "/footprint_model/radius' does not exist. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
      ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius << "m) loaded for trajectory optimization.");
      return boost::make_shared<CircularRobotFootprint>(radius);
    }

    // line
    if (model_name.compare("line") == 0)
    {
      // check parameters
      if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
      {
        ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                              << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
      // get line coordinates
      std::vector<double> line_start, line_end;
      nh.getParam("footprint_model/line_start", line_start);
      nh.getParam("footprint_model/line_end", line_end);
      if (line_start.size() != 2 || line_end.size() != 2)
      {
        ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                              << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }

      ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] << "]m, line_end: ["
                                                              << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
      return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()));
    }

    // two circles
    if (model_name.compare("two_circles") == 0)
    {
      // check parameters
      if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
      {
        ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                                                                                                                      << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
      double front_offset, front_radius, rear_offset, rear_radius;
      nh.getParam("footprint_model/front_offset", front_offset);
      nh.getParam("footprint_model/front_radius", front_radius);
      nh.getParam("footprint_model/rear_offset", rear_offset);
      nh.getParam("footprint_model/rear_radius", rear_radius);
      ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset << "m, front_radius: " << front_radius
                                                                      << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
      return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
    }

    // polygon
    if (model_name.compare("polygon") == 0)
    {

      // check parameters
      XmlRpc::XmlRpcValue footprint_xmlrpc;
      if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc))
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                                 << "/footprint_model/vertices' does not exist. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
      // get vertices
      if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        try
        {
          Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
          ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
          return boost::make_shared<PolygonRobotFootprint>(polygon);
        }
        catch (const std::exception &ex)
        {
          ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
          return boost::make_shared<PointRobotFootprint>();
        }
      }
      else
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                                 << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
    }

    // otherwise
    ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
    return boost::make_shared<PointRobotFootprint>();
  }
  // 机器人设置的是圆型轮廓,所以不用关心这个
  Point2dContainer TebLocalPlannerROS::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc, const std::string &full_param_name)
  {
    // Make sure we have an array of at least 3 elements.
    if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        footprint_xmlrpc.size() < 3)
    {
      ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                               "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
    }

    Point2dContainer footprint;
    Eigen::Vector2d pt;

    for (int i = 0; i < footprint_xmlrpc.size(); ++i)
    {
      // Make sure each element of the list is an array of size 2. (x and y coordinates)
      XmlRpc::XmlRpcValue point = footprint_xmlrpc[i];
      if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
          point.size() != 2)
      {
        ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                  "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                  full_param_name.c_str());
        throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                                 "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
      }

      pt.x() = getNumberFromXMLRPC(point[0], full_param_name);
      pt.y() = getNumberFromXMLRPC(point[1], full_param_name);

      footprint.push_back(pt);
    }
    return footprint;
  }
  // 机器人设置的是圆型轮廓,所以不用关心这个
  double TebLocalPlannerROS::getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name)
  {
    // Make sure that the value we're looking at is either a double or an int.
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
        value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      std::string &value_string = value;
      ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
                full_param_name.c_str(), value_string.c_str());
      throw std::runtime_error("Values in the footprint specification must be numbers");
    }
    return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
  }

} // end namespace teb_local_planner
