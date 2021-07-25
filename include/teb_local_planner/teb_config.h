#ifndef TEB_CONFIG_H_
#define TEB_CONFIG_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

#include <teb_local_planner/TebLocalPlannerReconfigureConfig.h>


// Definitions
#define USE_ANALYTIC_JACOBI // if available for a specific edge, use analytic jacobi


namespace teb_local_planner
{

/**
 * @class TebConfig
 * @brief Config class for the teb_local_planner and its components.
 */
class TebConfig
{
public:

  std::string odom_topic; //!< Topic name of the odometry message, provided by the robot driver or simulator
  std::string map_frame; //!< Global planning frame

  //! Trajectory related parameters
  struct Trajectory
  {
    // 启用自动调整轨迹w.r.t到时间分辨率(推荐)
    double teb_autosize; //!< Enable automatic resizing of the trajectory w.r.t to the temporal resolution (recommended)
    // 期望的轨迹的时间分辨率(应该是基础控制率的大小)
    // 规划轨迹的时间分辨率,运行过程中会根据实际情况调整。局部路径规划的解析度。该值越小,运动越缓慢。默认 0.3。
    double dt_ref; //!< Desired temporal resolution of the trajectory (should be in the magniture of the underlying control rate)
    // 根据当前时间分辨率(dt)自动调整大小的滞后:通常为dt_ref的10%
    double dt_hysteresis; //!< Hysteresis for automatic resizing depending on the current temporal resolution (dt): usually 10% of dt_ref
    int min_samples; //!< Minimum number of samples (should be always greater than 2)
    int max_samples; //!< Maximum number of samples; Warning: if too small the discretization/resolution might not be sufficient for the given robot model or obstacle avoidance does not work anymore.
    // 如果为真 那么根据当前的目标点和未来的目标点,就算出合适的目标角度
    bool global_plan_overwrite_orientation; //!< Overwrite orientation of local subgoals provided by the global planner
    // 如果为真，底层轨迹可能会被初始化为反向运动，以防目标在局部costmap中的起始位置之后(只有当机器人配备了后方传感器时，才建议这样做)
    bool allow_init_with_backwards_motion; //!< If true, the underlying trajectories might be initialized with backwards motions in case the goal is behind the start within the local costmap (this is only recommended if the robot is equipped with rear sensors)
    // 从全局计划中提取的每两个连续通过点之间的分离(如果为负数:禁用)
    // 可能的意思就是化简全局路径规划点,每隔多少距离取一个
    // # * 如果为正，则从全局计划中引出通孔点（路径遵循模式）。 该值确定参考路径的分辨率（如果为负值，则禁用全局规划中每两个连续通孔之间的最小距离）。
    // 如果是负数,那么就直接看局部地图中的终点不看过程点
    // 这个与dt_ref没有关系,dt_ref是计算出来的路径点,而global_plan_viapoint_sep是通过点约束边,是为了使dt_ref计算出来的点与global_plan_viapoint_sep尽量贴合
    double global_plan_viapoint_sep; //!< Min. separation between each two consecutive via-points extracted from the global plan (if negative: disabled)
    // 如果为真，计划器将遵循存储容器中通过点的顺序
    // 这个判断可以减少计算量,但是可能引发错误
    bool via_points_ordered; //!< If true, the planner adheres to the order of via-points in the storage container
    double max_global_plan_lookahead_dist; //!< Specify maximum length (cumulative Euclidean distances) of the subset of the global plan taken into account for optimization [if <=0: disabled; the length is also bounded by the local costmap size!]
    // 这个就是裁剪的距离,表示走过的距离应该留下多少,默认的也是1
    double global_plan_prune_distance; //!< Distance between robot and via_points of global plan which is used for pruning
    // 如果为真，规划器在速度、加速和转弯率计算中使用精确的弧长[不是简单计算,这样会增加计算量]，否则使用欧几里得近似。
    bool exact_arc_length; //!< If true, the planner uses the exact arc length in velocity, acceleration and turning rate computations [-> increased cpu time], otherwise the euclidean approximation is used.
    // 如果先前的目标更新的间隔大于指定的值(以米为单位)，则重新初始化轨迹(跳过热启动)
    // 这个是同伦类里面计算的,就是局部路径的目标点和teb中的目标点相差太大,那么要做的就是重新设置
    double force_reinit_new_goal_dist; //!< Reinitialize the trajectory if a previous goal is updated with a seperation of more than the specified value in meters (skip hot-starting)
    double force_reinit_new_goal_angular; //!< Reinitialize the trajectory if a previous goal is updated with an angular difference of more than the specified value in radians (skip hot-starting)
    // 指定每个采样间隔应在预测计划的哪个位置进行可行性检查。检测姿态在规划路径的可行性的时间间隔
    // feasibility_check_no_poses表示要向前方看多远(这段范围进行判断)
    int feasibility_check_no_poses; //!< Specify up to which pose on the predicted plan the feasibility should be checked each sampling interval.
    // 发布包含完整轨迹和活动障碍列表的计划器反馈(应仅用于评估或调试目的)
    bool publish_feedback; //!< Publish planner feedback containing the full trajectory and a list of active obstacles (should be enabled only for evaluation or debugging purposes)
    // 在costmap碰撞检查中使用的最小角度分辨率。如果不符合，则添加中间进行插值。(rad)
    // 检查两个姿态之间的距离是否大于机器人半径或方向差值大于指定的阈值，并在这种情况下进行插值。
    // 这个为什么进行差值呢,因为图中可能两个位姿节点之间的距离变大,障碍物位于中间
    // 所以要做的就是插值,在两个节点之间插值,然后判断是否和障碍物相撞
    double min_resolution_collision_check_angular; //! Min angular resolution used during the costmap collision check. If not respected, intermediate samples are added. [rad]
    // 用于提取速度命令的姿态索引
    // 什么意思呢?就是计算好图之后,位姿节点和时间节点都是会更新的,那么计算速度就是计算前几个(control_look_ahead_poses)位姿节点和时间节点
    // 累计时间不能过长也就是control_look_ahead_poses*dt_ref
    int control_look_ahead_poses; //! Index of the pose used to extract the velocity command
  } trajectory; //!< Trajectory related parameters

  //! Robot related parameters
  struct Robot
  {
    double max_vel_x; //!< Maximum translational velocity of the robot
    double max_vel_x_backwards; //!< Maximum translational velocity of the robot for driving backwards
    double max_vel_y; //!< Maximum strafing velocity of the robot (should be zero for non-holonomic robots!)
    double max_vel_theta; //!< Maximum angular velocity of the robot
    double acc_lim_x; //!< Maximum translational acceleration of the robot
    double acc_lim_y; //!< Maximum strafing acceleration of the robot
    double acc_lim_theta; //!< Maximum angular acceleration of the robot
    double min_turning_radius; //!< Minimum turning radius of a carlike robot (diff-drive robot: zero);
    double wheelbase; //!< The distance between the drive shaft and steering axle (only required for a carlike robot with 'cmd_angle_instead_rotvel' enabled); The value might be negative for back-wheeled robots!
    bool cmd_angle_instead_rotvel; //!< Substitute the rotational velocity in the commanded velocity message by the corresponding steering angle (check 'axles_distance')
    bool is_footprint_dynamic; //<! If true, updated the footprint before checking trajectory feasibility
  } robot; //!< Robot related parameters

  //! Goal tolerance related parameters
  struct GoalTolerance
  {
    double yaw_goal_tolerance; //!< Allowed final orientation error
    double xy_goal_tolerance; //!< Allowed final euclidean distance to the goal position
    // 为了进行规划，允许机器人的速度不为零(通常为max_vel)
    // 如果为true就是对终点的加速度不进行限制,意思可以直接停下来
    bool free_goal_vel; //!< Allow the robot's velocity to be nonzero (usally max_vel) for planning purposes
    bool complete_global_plan; // true prevents the robot from ending the path early when it cross the end goal
  } goal_tolerance; //!< Goal tolerance related parameters

  //! Obstacle related parameters
  struct Obstacles
  {
    // 根据cfg_->obstacles.inflation_dist与cfg_->obstacles.min_obstacle_dist大小进行判断到底是加入EdgeInflatedObstacle还是EdgeObstacle
    // 障碍物的话:只考虑 cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor内的障碍物和不远处最近的左右两边的障碍物
    double min_obstacle_dist; //!< Minimum desired separation from obstacles
    double inflation_dist; //!< buffer zone around obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
    double dynamic_obstacle_inflation_dist; //!< Buffer zone around predicted locations of dynamic obstacles with non-zero penalty costs (should be larger than min_obstacle_dist in order to take effect)
    // #如果将此参数设置为true，则将通过等速模型在优化过程中预测并考虑速度为非零的障碍物的运动（通过用户在主题〜/障碍物上提供的障碍物或从costmap_converter获得）。
    bool include_dynamic_obstacles; //!< Specify whether the movement of dynamic obstacles should be predicted by a constant velocity model (this also effects homotopy class planning); If false, all obstacles are considered to be static.
    bool include_costmap_obstacles; //!< Specify whether the obstacles in the costmap should be taken into account directly
    double costmap_obstacles_behind_robot_dist; //!< Limit the occupied local costmap obstacles taken into account for planning behind the robot (specify distance in meters)
    int obstacle_poses_affected; //!< The obstacle position is attached to the closest pose on the trajectory to reduce computational effort, but take a number of neighbors into account as well
    // 如果为真，则使用旧的关联策略(对于每个障碍物，找到最近的TEB姿势)，否则使用新的关联策略(对于每个障碍物，只找到“相关的”障碍)。
    bool legacy_obstacle_association; //!< If true, the old association strategy is used (for each obstacle, find the nearest TEB pose), otherwise the new one (for each teb pose, find only "relevant" obstacles).
    // 这个就是在执行AddEdgesObstacles()函数的时候,会在最小的障碍物半径*obstacle_association_force_inclusion_factor(也就是扩大还是缩小)
    double obstacle_association_force_inclusion_factor; //!< The non-legacy obstacle association technique tries to connect only relevant obstacles with the discretized trajectory during optimization, all obstacles within a specifed distance are forced to be included (as a multiple of min_obstacle_dist), e.g. choose 2.0 in order to consider obstacles within a radius of 2.0*min_obstacle_dist.
    // 参考obstacle_association_force_inclusion_factor，但是在优化过程中，超过[value]*min_obstacle_dist的倍数，所有的障碍都会被忽略。首先处理Obstacle_association_force_inclusion_factor。
    double obstacle_association_cutoff_factor; //!< See obstacle_association_force_inclusion_factor, but beyond a multiple of [value]*min_obstacle_dist all obstacles are ignored during optimization. obstacle_association_force_inclusion_factor is processed first.
    std::string costmap_converter_plugin; //!< Define a plugin name of the costmap_converter package (costmap cells are converted to points/lines/polygons)
    bool costmap_converter_spin_thread; //!< If \c true, the costmap converter invokes its callback queue in a different thread
    int costmap_converter_rate; //!< The rate that defines how often the costmap_converter plugin processes the current costmap (the value should not be much higher than the costmap update rate)
  } obstacles; //!< Obstacle related parameters


  //! Optimization related parameters
  struct Optimization
  {
    // 每次外部循环迭代中调用的实际求解器迭代次数
    int no_inner_iterations; //!< Number of solver iterations called in each outerloop iteration
    // 每次外部循环迭代都会根据所需的时间分辨率dt_ref自动调整轨迹的大小，
    // 并调用内部优化器（执行no_inner_iterations）。 因此，每个计划周期中求解程序迭代的总数是两个值的乘积。
    int no_outer_iterations; //!< Each outerloop iteration automatically resizes the trajectory and invokes the internal optimizer with no_inner_iterations
    // 激活优化器
    bool optimization_activate; //!< Activate the optimization
    // 打印详细信息
    bool optimization_verbose; //!< Print verbose information
    // 为硬约束近似的惩罚函数添加一个小的安全裕度
    // 打个比方,以速度为例,假设最大速度为5.0,penalty_epsilon为0.5,那么如果现在速度是5.4,就不会惩罚,超过5.5就要惩罚
    double penalty_epsilon; //!< Add a small safety margin to penalty functions for hard-constraint approximations

    double weight_max_vel_x; //!< Optimization weight for satisfying the maximum allowed translational velocity
    double weight_max_vel_y; //!< Optimization weight for satisfying the maximum allowed strafing velocity (in use only for holonomic robots)
    double weight_max_vel_theta; //!< Optimization weight for satisfying the maximum allowed angular velocity
    double weight_acc_lim_x; //!< Optimization weight for satisfying the maximum allowed translational acceleration
    double weight_acc_lim_y; //!< Optimization weight for satisfying the maximum allowed strafing acceleration (in use only for holonomic robots)
    double weight_acc_lim_theta; //!< Optimization weight for satisfying the maximum allowed angular acceleration
    // 满足非完整运动学的优化权值
    double weight_kinematics_nh; //!< Optimization weight for satisfying the non-holonomic kinematics
    // 优化重量，迫使机器人只选择正向方向(正转。速度，只有diffdrive机器人)
    double weight_kinematics_forward_drive; //!< Optimization weight for forcing the robot to choose only forward directions (positive transl. velocities, only diffdrive robot)
    double weight_kinematics_turning_radius; //!< Optimization weight for enforcing a minimum turning radius (carlike robots)
    double weight_optimaltime; //!< Optimization weight for contracting the trajectory w.r.t. transition time
    double weight_shortest_path; //!< Optimization weight for contracting the trajectory w.r.t. path length
    double weight_obstacle; //!< Optimization weight for satisfying a minimum separation from obstacles
    double weight_inflation; //!< Optimization weight for the inflation penalty (should be small)
    double weight_dynamic_obstacle; //!< Optimization weight for satisfying a minimum separation from dynamic obstacles
    double weight_dynamic_obstacle_inflation; //!< Optimization weight for the inflation penalty of dynamic obstacles (should be small)
    double weight_viapoint; //!< Optimization weight for minimizing the distance to via-points
    double weight_prefer_rotdir; //!< Optimization weight for preferring a specific turning direction (-> currently only activated if an oscillation is detected, see 'oscillation_recovery'
    // 一些特殊的权重(当前为'weight_obstacle')在每个外部TEB迭代中重复地按这个因子进行缩放(weight_new = weight_old*factor);
    // 迭代地增加权值，而不是预先设置一个巨大的值，可以使底层优化问题的数值条件更好。
    double weight_adapt_factor; //!< Some special weights (currently 'weight_obstacle') are repeatedly scaled by this factor in each outer TEB iteration (weight_new = weight_old*factor); Increasing weights iteratively instead of setting a huge value a-priori leads to better numerical conditions of the underlying optimization problem.
    // 非线性障碍代价指数(cost = linear_cost * obstacle_cost_exponent)。设置为1禁用非线性成本(默认)
    double obstacle_cost_exponent; //!< Exponent for nonlinear obstacle cost (cost = linear_cost * obstacle_cost_exponent). Set to 1 to disable nonlinear cost (default)
  } optim; //!< Optimization related parameters


  struct HomotopyClasses
  {
    bool enable_homotopy_class_planning; //!< Activate homotopy class planning (Requires much more resources that simple planning, since multiple trajectories are optimized at once).
    bool enable_multithreading; //!< Activate multiple threading for planning multiple trajectories in parallel.
    // 如果为真，则使用简单的左右方法（通过左侧或右侧的每个障碍物）来探索独特的轨迹以生成路径，
    // 否则在起点和目标之间的指定区域随机采样可能的路线图。
    // 这个在论文中会进行体现
    // 默认为假,表示随机采样
    bool simple_exploration; //!< If true, distinctive trajectories are explored using a simple left-right approach (pass each obstacle on the left or right side) for path generation, otherwise sample possible roadmaps randomly in a specified region between start and goal.
    int max_number_classes; //!< Specify the maximum number of allowed alternative homotopy classes (limits computational effort)
    double selection_cost_hysteresis; //!< Specify how much trajectory cost must a new candidate have w.r.t. a previously selected trajectory in order to be selected (selection if new_cost < old_cost*factor).
    double selection_prefer_initial_plan; //!< Specify a cost reduction in the interval (0,1) for the trajectory in the equivalence class of the initial plan.
    double selection_obst_cost_scale; //!< Extra scaling of obstacle cost terms just for selecting the 'best' candidate.
    double selection_viapoint_cost_scale; //!< Extra scaling of via-point cost terms just for selecting the 'best' candidate.
    bool selection_alternative_time_cost; //!< If true, time cost is replaced by the total transition time.
    double switching_blocking_period; //!< Specify a time duration in seconds that needs to be expired before a switch to new equivalence class is allowed

    int roadmap_graph_no_samples; //! < Specify the number of samples generated for creating the roadmap graph, if simple_exploration is turend off.
    double roadmap_graph_area_width; //!< Random keypoints/waypoints are sampled in a rectangular region between start and goal. Specify the width of that region in meters.
    double roadmap_graph_area_length_scale; //!< The length of the rectangular region is determined by the distance between start and goal. This parameter further scales the distance such that the geometric center remains equal!
    double h_signature_prescaler; //!< Scale number of obstacle value in order to allow huge number of obstacles. Do not choose it extremly low, otherwise obstacles cannot be distinguished from each other (0.2<H<=1).
    double h_signature_threshold; //!< Two h-signatures are assumed to be equal, if both the difference of real parts and complex parts are below the specified threshold.

    double obstacle_keypoint_offset; //!< If simple_exploration is turned on, this parameter determines the distance on the left and right side of the obstacle at which a new keypoint will be cretead (in addition to min_obstacle_dist).
    // 指定障碍航向和目标航向之间的归一化标量积的值，以便将它们（障碍物）考虑在内进行探索 [0,1]
    double obstacle_heading_threshold; //!< Specify the value of the normalized scalar product between obstacle heading and goal heading in order to take them (obstacles) into account for exploration [0,1]

    bool viapoints_all_candidates; //!< If true, all trajectories of different topologies are attached to the current set of via-points, otherwise only the trajectory sharing the same one as the initial/global plan.

    bool visualize_hc_graph; //!< Visualize the graph that is created for exploring new homotopy classes.
    double visualize_with_time_as_z_axis_scale; //!< If this value is bigger than 0, the trajectory and obstacles are visualized in 3d using the time as the z-axis scaled by this value. Most useful for dynamic obstacles.
    // 如果启用，规划器将丢弃相对于最佳计划向后绕行的计划
    bool delete_detours_backwards; //!< If enabled, the planner will discard the plans detouring backwards with respect to the best plan
    // 如果一个计划的开始方向与最佳计划的差异超过这个范围，则该计划被认为是绕道而行
    double detours_orientation_tolerance; //!< A plan is considered a detour if its start orientation differs more than this from the best plan
    // 用于计算计划起始方向的向量的长度
    double length_start_orientation_vector; //!< Length of the vector used to compute the start orientation of a plan
    // 如果 他们的执行时间/最佳teb的执行时间 > max_ratio_detours_duration_best_duration，则绕道被丢弃
    double max_ratio_detours_duration_best_duration; //!< Detours are discarted if their execution time / the execution time of the best teb is > this
  } hcp;

  //! Recovery/backup related parameters
  struct Recovery
  {
    // 允许规划器在自动检测到问题时缩小预测域临时(50%)。
    bool shrink_horizon_backup; //!< Allows the planner to shrink the horizon temporary (50%) in case of automatically detected issues.
    // 指定减少视界的最小持续时间，以防检测到不可行的轨迹。
    // 就是两个正常视界之间的最小时间
    double shrink_horizon_min_duration; //!< Specify minimum duration for the reduced horizon in case an infeasible trajectory is detected.
    bool oscillation_recovery; //!< Try to detect and resolve oscillations between multiple solutions in the same equivalence class (robot frequently switches between left/right/forward/backwards)
    // 平均归一化线速度的阈值:如果振荡_v_eps和振荡_omega_eps未同时超过，则检测到可能的振荡
    double oscillation_v_eps; //!< Threshold for the average normalized linear velocity: if oscillation_v_eps and oscillation_omega_eps are not exceeded both, a possible oscillation is detected
    double oscillation_omega_eps; //!< Threshold for the average normalized angular velocity: if oscillation_v_eps and oscillation_omega_eps are not exceeded both, a possible oscillation is detected
    // 震荡的最小允许时间
    double oscillation_recovery_min_duration; //!< Minumum duration [sec] for which the recovery mode is activated after an oscillation is detected.
    double oscillation_filter_duration; //!< Filter length/duration [sec] for the detection of oscillations
  } recovery; //!< Parameters related to recovery and backup strategies


  /**
  * @brief Construct the TebConfig using default values.
  * @warning If the \b rosparam server or/and \b dynamic_reconfigure (rqt_reconfigure) node are used,
  *	     the default variables will be overwritten: \n
  *	     E.g. if \e base_local_planner is utilized as plugin for the navigation stack, the initialize() method will register a
  * 	     dynamic_reconfigure server. A subset (not all but most) of the parameters are considered for dynamic modifications.
  * 	     All parameters considered by the dynamic_reconfigure server (and their \b default values) are
  * 	     set in \e PROJECT_SRC/cfg/TebLocalPlannerReconfigure.cfg. \n
  * 	     In addition the rosparam server can be queried to get parameters e.g. defiend in a launch file.
  * 	     The plugin source (or a possible binary source) can call loadRosParamFromNodeHandle() to update the parameters.
  * 	     In \e summary, default parameters are loaded in the following order (the right one overrides the left ones): \n
  * 		<b>TebConfig Constructor defaults << dynamic_reconfigure defaults << rosparam server defaults</b>
  */
  TebConfig()
  {

    odom_topic = "odom";
    map_frame = "odom";

    // Trajectory

    trajectory.teb_autosize = true;
    trajectory.dt_ref = 0.3;
    trajectory.dt_hysteresis = 0.1;
    trajectory.min_samples = 3;
    trajectory.max_samples = 500;
    trajectory.global_plan_overwrite_orientation = true;
    trajectory.allow_init_with_backwards_motion = false;
    trajectory.global_plan_viapoint_sep = -1;
    trajectory.via_points_ordered = false;
    trajectory.max_global_plan_lookahead_dist = 1;
    trajectory.global_plan_prune_distance = 1;
    trajectory.exact_arc_length = false;
    trajectory.force_reinit_new_goal_dist = 1;
    trajectory.force_reinit_new_goal_angular = 0.5 * M_PI;
    trajectory.feasibility_check_no_poses = 5;
    trajectory.publish_feedback = false;
    trajectory.min_resolution_collision_check_angular = M_PI;
    trajectory.control_look_ahead_poses = 1;
    
    // Robot

    robot.max_vel_x = 0.4;
    robot.max_vel_x_backwards = 0.2;
    robot.max_vel_y = 0.0;
    robot.max_vel_theta = 0.3;
    robot.acc_lim_x = 0.5;
    robot.acc_lim_y = 0.5;
    robot.acc_lim_theta = 0.5;
    robot.min_turning_radius = 0;
    robot.wheelbase = 1.0;
    robot.cmd_angle_instead_rotvel = false;
    robot.is_footprint_dynamic = false;

    // GoalTolerance

    goal_tolerance.xy_goal_tolerance = 0.2;
    goal_tolerance.yaw_goal_tolerance = 0.2;
    goal_tolerance.free_goal_vel = false;
    goal_tolerance.complete_global_plan = true;

    // Obstacles

    obstacles.min_obstacle_dist = 0.5;
    obstacles.inflation_dist = 0.6;
    obstacles.dynamic_obstacle_inflation_dist = 0.6;
    obstacles.include_dynamic_obstacles = true;
    obstacles.include_costmap_obstacles = true;
    obstacles.costmap_obstacles_behind_robot_dist = 1.5;
    obstacles.obstacle_poses_affected = 25;
    obstacles.legacy_obstacle_association = false;
    obstacles.obstacle_association_force_inclusion_factor = 1.5;
    obstacles.obstacle_association_cutoff_factor = 5;
    obstacles.costmap_converter_plugin = "";
    obstacles.costmap_converter_spin_thread = true;
    obstacles.costmap_converter_rate = 5;

    // Optimization

    optim.no_inner_iterations = 5;
    optim.no_outer_iterations = 4;
    optim.optimization_activate = true;
    optim.optimization_verbose = false;
    optim.penalty_epsilon = 0.1;
    optim.weight_max_vel_x = 2; //1
    optim.weight_max_vel_y = 2;
    optim.weight_max_vel_theta = 1;
    optim.weight_acc_lim_x = 1;
    optim.weight_acc_lim_y = 1;
    optim.weight_acc_lim_theta = 1;
    optim.weight_kinematics_nh = 1000;
    optim.weight_kinematics_forward_drive = 1;
    optim.weight_kinematics_turning_radius = 1;
    optim.weight_optimaltime = 1;
    optim.weight_shortest_path = 0;
    optim.weight_obstacle = 50;
    optim.weight_inflation = 0.1;
    optim.weight_dynamic_obstacle = 50;
    optim.weight_dynamic_obstacle_inflation = 0.1;
    optim.weight_viapoint = 1;
    optim.weight_prefer_rotdir = 50;

    optim.weight_adapt_factor = 2.0;
    optim.obstacle_cost_exponent = 1.0;

    // Homotopy Class Planner

    hcp.enable_homotopy_class_planning = true;
    hcp.enable_multithreading = true;
    hcp.simple_exploration = false;
    hcp.max_number_classes = 5;
    hcp.selection_cost_hysteresis = 1.0;
    hcp.selection_prefer_initial_plan = 0.95;
    hcp.selection_obst_cost_scale = 100.0;
    hcp.selection_viapoint_cost_scale = 1.0;
    hcp.selection_alternative_time_cost = false;

    hcp.obstacle_keypoint_offset = 0.1;
    hcp.obstacle_heading_threshold = 0.45;
    hcp.roadmap_graph_no_samples = 15;
    hcp.roadmap_graph_area_width = 6; // [m]
    hcp.roadmap_graph_area_length_scale = 1.0;
    hcp.h_signature_prescaler = 1;
    hcp.h_signature_threshold = 0.1;
    hcp.switching_blocking_period = 0.0;

    hcp.viapoints_all_candidates = true;

    hcp.visualize_hc_graph = false;
    hcp.visualize_with_time_as_z_axis_scale = 0.0;
    hcp.delete_detours_backwards = true;
    hcp.detours_orientation_tolerance = M_PI / 2.0;
    hcp.length_start_orientation_vector = 0.4;
    hcp.max_ratio_detours_duration_best_duration = 3.0;

    // Recovery

    recovery.shrink_horizon_backup = true;
    recovery.shrink_horizon_min_duration = 10;
    recovery.oscillation_recovery = true;
    recovery.oscillation_v_eps = 0.1;
    recovery.oscillation_omega_eps = 0.1;
    recovery.oscillation_recovery_min_duration = 10;
    recovery.oscillation_filter_duration = 10;


  }

  /**
   * @brief Load parmeters from the ros param server.
   * @param nh const reference to the local ros::NodeHandle
   */
  void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

  /**
   * @brief Reconfigure parameters from the dynamic_reconfigure config.
   * Change parameters dynamically (e.g. with <c>rosrun rqt_reconfigure rqt_reconfigure</c>).
   * A reconfigure server needs to be instantiated that calls this method in it's callback.
   * In case of the plugin \e teb_local_planner default values are defined
   * in \e PROJECT_SRC/cfg/TebLocalPlannerReconfigure.cfg.
   * @param cfg Config class autogenerated by dynamic_reconfigure according to the cfg-file mentioned above.
   */
  void reconfigure(TebLocalPlannerReconfigureConfig& cfg);

  /**
   * @brief Check parameters and print warnings in case of discrepancies
   *
   * Call this method whenever parameters are changed using public interfaces to inform the user
   * about some improper uses.
   */
  void checkParameters() const;

  /**
   * @brief Check if some deprecated parameters are found and print warnings
   * @param nh const reference to the local ros::NodeHandle
   */
  void checkDeprecated(const ros::NodeHandle& nh) const;

  /**
   * @brief Return the internal config mutex
   */
  boost::mutex& configMutex() {return config_mutex_;}

private:
  boost::mutex config_mutex_; //!< Mutex for config accesses and changes

};


} // namespace teb_local_planner

#endif
