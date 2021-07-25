#include <teb_local_planner/optimal_planner.h>
#include <map>
#include <limits>


namespace teb_local_planner
{

// ============== Implementation ===================
// teb优化器
TebOptimalPlanner::TebOptimalPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), cost_(HUGE_VAL), prefer_rotdir_(RotType::none),
                                         robot_model_(new PointRobotFootprint()), initialized_(false), optimized_(false)
{    
}

  //初始化普通的规划器
  //obstacles_是一个空的 存放所有相关障碍物的容器(见障碍物)
  //robot_model存储着机器人的形状信息 用于优化的机器人形状模型的共享指针(可选)
  //visualization_创建可视化实例
  //via_points_在局部轨迹优化过程中需要考虑的节点容器
  //planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_)); 
TebOptimalPlanner::TebOptimalPlanner(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{    
  initialize(cfg, obstacles, robot_model, visual, via_points);
}

TebOptimalPlanner::~TebOptimalPlanner()
{
  clearGraph();
  // free dynamically allocated memory
  //if (optimizer_) 
  //  g2o::Factory::destroy();
  //g2o::OptimizationAlgorithmFactory::destroy();
  //g2o::HyperGraphActionLibrary::destroy();
}
//就是初始化求解器 开始和目标速度,还有就是将输入值换成这个文件的全局变量
void TebOptimalPlanner::initialize(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{    
  // init optimizer (set solver and block ordering settings)
  // 设置求解器,线性求解->块求解->求解器
  optimizer_ = initOptimizer();
  
  cfg_ = &cfg;
  obstacles_ = obstacles;
  // robot_model机器人当前的轮廓,我们使用的圆,圆的参数就是只有圆的半径
  // getRobotFootprintFromParamServer(nh);
  // 这个函数得到机器人半径,也就是机器人轮廓
  robot_model_ = robot_model;
  // 机器人需要通过的点
  // updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);
  // 上面这个函数进行运行的
  via_points_ = via_points;
  //HUGE_VAL这个值不知道在哪了
  cost_ = HUGE_VAL;
  // 存储是否在优化中选择一个特定的初始旋转(可能在机器人振荡时被激活)
  // 这个值是用来判断是否震荡的
  prefer_rotdir_ = RotType::none;
  //注册一个Tebvisualization类来启用可视化例程(例如，发布本地计划和姿态序列)
  setVisualization(visual);
  //存储开始姿势时的初始速度
  vel_start_.first = true;
  vel_start_.second.linear.x = 0;
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x = 0;
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = 0;
  initialized_ = true;
}

// 设置显示的对象
void TebOptimalPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}

void TebOptimalPlanner::visualize()
{
  if (!visualization_)
    return;
 
  visualization_->publishLocalPlanAndPoses(teb_);
  
  if (teb_.sizePoses() > 0)
    visualization_->publishRobotFootprintModel(teb_.Pose(0), *robot_model_);
  
  if (cfg_->trajectory.publish_feedback)
    visualization_->publishFeedbackMessage(*this, *obstacles_);
 
}


/*
 * registers custom vertices and edges in g2o framework
 将为TEB定义的顶点和边注册到g2o::Factory。例如，这允许用户将内部图形导出到文本文件。
 */
void TebOptimalPlanner::registerG2OTypes()
{
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
  factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);
  //以时间间隔dt最小作为目标。（保证时间最短轨迹）
  factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
  factory->registerType("EDGE_SHORTEST_PATH", new g2o::HyperGraphElementCreator<EdgeShortestPath>);
  //以实际速度和限定最大速度经过惩罚函数的输出作为目标函数，顶点为相邻的位姿和时间间隔。（限制实际速度不超过最大，程序中引入了sigmoid函数决定速度的符号（根据论文内的说法，引入此函数是因为优化算法只能求解连续函数））。
  factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>); 
  //与EdgeVelocity的区别在于，ds是机器人本体坐标系下的，然后去求速度在本体坐标系下的表示。其余思路一致。
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>);
  factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
  factory->registerType("EDGE_ACCELERATION_START", new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
  factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>);
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>);
  factory->registerType("EDGE_KINEMATICS_CARLIKE", new g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>);
  //以障碍物与待优化位姿（顶点）的距离经过惩罚函数后的输出作为目标（保证离障碍物大于一定距离）
  factory->registerType("EDGE_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeObstacle>); 
  factory->registerType("EDGE_INFLATED_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInflatedObstacle>);
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeDynamicObstacle>);
  //以指定的经过点位置与待优化位姿（顶点）的误差作为目标。
  factory->registerType("EDGE_VIA_POINT", new g2o::HyperGraphElementCreator<EdgeViaPoint>);
  factory->registerType("EDGE_PREFER_ROTDIR", new g2o::HyperGraphElementCreator<EdgePreferRotDir>);
  return;
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 * 设置求解器,线性求解->块求解->求解器
 */
boost::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static boost::once_flag flag = BOOST_ONCE_INIT;
  // registerG2OTypes调用一次这个函数
  // registerG2OTypes将为TEB定义的顶点和边注册到g2o::Factory。例如，这允许用户将内部图形导出到文本文件。
  boost::call_once(&registerG2OTypes, flag);  

  // allocating the optimizer

  // 第1步：创建一个线性求解器LinearSolver
  TEBLinearSolver* linearSolver = new TEBLinearSolver(); // see typedef in optimization.h
  linearSolver->setBlockOrdering(true);
  // 第2步：创建BlockSolver。并用上面定义的线性求解器初始化
  TEBBlockSolver* blockSolver = new TEBBlockSolver(linearSolver);
  // 第3步：创建总求解器solver。并从GN, LM, DogLeg 中选一个，再用上述块求解器BlockSolver初始化
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
  // 第4步：创建终极大boss 稀疏优化器（SparseOptimizer）
  boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
  optimizer->setAlgorithm(solver);
  // 开启多线程计算
  optimizer->initMultiThreading(); // required for >Eigen 3.1
  
  return optimizer;
}

//compute_cost_afterwards这个不用
//optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
//  compute_cost_afterwards = false, obst_cost_scale = 1.0,  viapoint_cost_scale = 1.0,  alternative_time_cost = false
bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                    double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  if (cfg_->optim.optimization_activate==false) 
    return false;
  
  bool success = false;
  //只要最后一次优化成功完成，此变量为\c true
  optimized_ = false;
  
  double weight_multiplier = 1.0;

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
  //                (which leads to better results in terms of x-y-t homotopy planning).
  //                 however, we have not tested this mode intensively yet, so we keep
  //                 the legacy fast mode as default until we finish our tests.
  /*TODO(roesmann):
  我们引入了支持动态障碍的非快速模式(在x-y-t同伦规划方面效果更好)。
  但是，我们还没有对该模式进行密集的测试，
  所以在完成测试之前，我们将保留传统的快速模式作为默认模式。*/
  //就是没办法用,下面根本没办法用
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;
  /* 它由两个嵌套循环组成:
   *      外部循环通过调用TimedElasticBand:: autoResize()来根据时间分辨率调整轨迹。
   *      然后调用组成内层循环的内部方法optimizeGraph()。
   *      内部循环调用求解器(g2o框架，resp)。稀疏Levenberg-Marquardt)，并迭代指定数量的优化调用(\c迭代内部循环)。
   * */
  for(int i=0; i<iterations_outerloop; ++i)
  {
    // 启用自动调整轨迹w.r.t到时间分辨率(推荐)
    if (cfg_->trajectory.teb_autosize)
    {
      // dt_ref规划轨迹的时间分辨率,运行过程中会根据实际情况调整。局部路径规划的解析度。该值越小,运动越缓慢。默认 0.3。
      // dt_hysteresis根据当前时间分辨率(dt)自动调整大小的滞后:通常为dt_ref的10%
      // 根据时间分辨率调整轨迹。能够保证最终的整个teb路径两个点之间的时间差都在指定的dt_ref范围内。
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);

    }
    //这个函数就是添加约束
    success = buildGraph(weight_multiplier);
    if (!success) 
    {
        clearGraph();
        return false;
    }
    // 然后调用组成内层循环的内部方法optimizeGraph()。
    //这个函数就是利用g20优化器进行优化,优化次数是no_iterations(执行optimize(no_iterations))
    success = optimizeGraph(iterations_innerloop, false);
    if (!success) 
    {
        clearGraph();
        return false;
    }
    optimized_ = true;
    //compute_cost_afterwards不用这个，所以不用管
    if (compute_cost_afterwards && i==iterations_outerloop-1) // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
      
    clearGraph();
    // 一些特殊的权重(当前为'weight_obstacle')在每个外部TEB迭代中重复地按这个因子进行缩放(weight_new = weight_old*factor);
    // 迭代地增加权值，而不是预先设置一个巨大的值，可以使底层优化问题的数值条件更好。
    // 就是障碍物的信息越来越大,越来越注重障碍物的信息
    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  return true;
}
// 设置机器人的起始速度,一般这个速度就是机器人当前的速度
// 这个会作为加速度边中的参数使用
void TebOptimalPlanner::setVelocityStart(const geometry_msgs::Twist& vel_start)
{
  vel_start_.first = true;
  vel_start_.second.linear.x = vel_start.linear.x;
  vel_start_.second.linear.y = vel_start.linear.y;
  vel_start_.second.angular.z = vel_start.angular.z;
}

void TebOptimalPlanner::setVelocityGoal(const geometry_msgs::Twist& vel_goal)
{
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}


//planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
//这里的initial_plan是经过处理的，就是局部路径规划的点
// 更新pose_vec_和timediff_vec_,并且进行执行optimizeTEB()
bool TebOptimalPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{    
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  // 没有被初始化,那么就进去
  if (!teb_.isInit())
  {
    // init trajectory
    //局部规划第一次运行
    //这个函数的就是将initial_plan放进到pose_vec_和timediff_vec_，这个函数就做一次，直到initial_plan被重新赋值（就是该段的initial_plan到达了本段的目标）
    // global_plan_overwrite_orientation覆盖全局规划器提供的局部子目标的方向(因为它们通常只提供一个 2D 路径),默认 true。
    // 如果为真 那么根据当前的目标点和未来的目标点,就算出合适的目标角度
    // min_samples最小样本数（始终大于2）
    // 如果为真，底层轨迹可能会被初始化为反向运动，以防目标在局部costmap中的起始位置之后(只有当机器人配备了后方传感器时，才建议这样做)
    // 函数的作用就是将局部路径规划的起点放进pose_vec_,然后将后面的点轮着放到pose_vec_,并且将后面的点到局部路径规划的上一个点的行驶的最短时间也轮着放进timediff_vec_
    teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
  } 
  else // warm start
  {
    // 在运动过程中起点和终点是实时更新的
    PoseSE2 start_(initial_plan.front().pose);
    PoseSE2 goal_(initial_plan.back().pose);
    //actual warm start!之后先检查goal_是否变化很大，
    //如果变化不大则使用上一次teb的路径作为初值。
    // force_reinit_new_goal_dist 如果先前的目标更新的间隔大于指定的值(以米为单位)，则重新初始化轨迹(跳过热启动)
    if (teb_.sizePoses()>0
        && (goal_.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
      // teb第一次初始化后，后续每次仅需更新起点和终点即可、
      //不懂下面的注释是什么意思
      //该部分比较重要是因为路径被带走的问题，当有动态障碍物穿越teb时，A*会发生变化，
      //在根据前视距离对A*的全局路径进行截取时，
      //使用的是两个点之间的距离的累加。当A*不更新时会导致终点始终不变化，
      //导致使用上次的teb进行更新，始终向一个方向更新，导致路径被带走。

      //这个函数的作用就是删除之前走过的节点，然后用新的开始和目标（目标还是哪个目标）去更新两个固定的节点
      // 这个就相当于实时的更新局部路径规划点
      // update TEB进行简单的裁剪作为新的初值
      teb_.updateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples); // update TEB
    else // goal too far away -> reinit
    {
      //如果goal变化很大则使用A*的路径进行作为初值。
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      // 清空节点
      teb_.clearTimedElasticBand();
      // 函数的作用就是将局部路径规划的起点放进pose_vec_,然后将后面的点轮着放到pose_vec_,并且将后面的点到局部路径规划的上一个点的行驶的最短时间也轮着放进timediff_vec_
      teb_.initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, true, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  //start_vel就是开始的速度
  if (start_vel)
    // 设置机器人的起始的速度
    setVelocityStart(*start_vel);
    //如果最后的速度没有限制,就是在加速度边上对其没有限制
    //那么就调用这个函数
    //就是将vel_goal_.first = false
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
  
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}

// start_vel这个初始速度是需要设置的,在建图的时候会使用到
bool TebOptimalPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  PoseSE2 start_(start);
  PoseSE2 goal_(goal);
  return plan(start_, goal_, start_vel);
}
// start_vel这个初始速度是需要设置的,在建图的时候会使用到
bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{	
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (!teb_.isInit())
  {
    // init trajectory
    ////添加了n个位姿点和n-1个时间点
    teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion); // 0 intermediate samples, but dt=1 -> autoResize will add more samples before calling first optimization
  }
  else // warm start
  {
    if (teb_.sizePoses() > 0
        && (goal.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
      //但是这个函数的作用就是删除之前走过的节点，然后用新的开始和目标去更新两个固定的节点
      teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples);
    else // goal too far away -> reinit
    {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel)
    //一般这个速度就是机器人当前的速度
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    // 我们只需重新激活并使用之前设置的速度(如果没有任何修改，应该为零)
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
      
  // now optimize
  ////compute_cost_afterwards这个不用
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}

//这个函数就是添加约束
// 如果本身有数据就返回false,然后执行clearGraph
bool TebOptimalPlanner::buildGraph(double weight_multiplier)
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }
  
  // add TEB vertices
  //增加图的顶点，轨迹的路径点和dt为顶点
  // add TEB vertices 添加节点，一种是位姿节点，一种是时间差节点
  AddTEBVertices();
  
  // add Edges (local cost functions)
  //已经修改了将轨迹姿势与优化障碍联系起来的策略
  //通过将此参数设置为true来切换到旧/先前策略。 
  //true旧策略：对于每个障碍，找到最近的TEB姿势; 
  //false新策略：对于每个teb姿势，只找到“相关”的障碍
  //添加静态障碍物约束，不确定因为我不考虑障碍物
  if (cfg_->obstacles.legacy_obstacle_association)
    // 先不考虑这个 
    AddEdgesObstaclesLegacy(weight_multiplier);
  else
    //把障碍物信息加到图搜索中
    // 根据cfg_->obstacles.inflation_dist与cfg_->obstacles.min_obstacle_dist大小进行判断到底是加入EdgeInflatedObstacle还是EdgeObstacle
    // 只考虑 cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor内的障碍物
    // 和不远处最近的左右两边的障碍物
    AddEdgesObstacles(weight_multiplier);


  //添加动态障碍物约束
  // 这个是动态障碍物相关的边
  //  计算_measurement以当前速度运行t时间段后与bandpt的距离
  //  计算误差值,如果路径点距离障碍物小于cfg_->obstacles.min_obstacle_dist+cfg_->optim.penalty_epsilon,那么就给惩罚,也就是error[0]=正值,否则为0
  // 同理路径点距离障碍物小于cfg_->obstacles.dynamic_obstacle_inflation_dist也要进行判断
  if (cfg_->obstacles.include_dynamic_obstacles)
    AddEdgesDynamicObstacles();
  
  //添加全局路径约束, 取决于参数 global_plan_viapoint_sep
  // 一元边 与通过点相关的边
  // cost便是离当前路径点最近的通过点与当前点最近的距离
  AddEdgesViaPoints();//添加全局路径约束
  // 速度边,超过设定的速度就会进行惩罚,注意误差是两个值,一个是线速度一个是角速度
  AddEdgesVelocity();//添加速度约束
  // 加速度边:计算的是从当前速度到想要的速度中间的加速度是否满足要求,不满足要求的话,就给上惩罚
  AddEdgesAcceleration();//添加加速度约束
  // 一元边,表示时间节点时间越长,那么误差越大
  AddEdgesTimeOptimal();	//添加时间约束
  // 二元边 两个位姿节点之间的距离越小,那么他的误差越小,这就是为什么之前要把局部路径规划点进行重新分割
  // 分割成在dt_ref时间以最快速度运行得到的间隔点
  AddEdgesShortestPath();//添加最短路径约束
  // 采用了差速模型
  //增加动力学约束
  if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
    //  定义了差分驱动移动机器人满足非完整运动学的代价函数。
    //  有两个误差值,第一个是非完整数学模型的误差值,看论文(Trajectory modification considering dynamic constraints of autonomous robots)
    //  第二个是正向运动,如果是负的,表示是反向的,要惩罚
    AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
  else
    AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.

  //增加方向约束，是经常向左还是经常向右，应该是这个意思
  // 这个函数就是如果_measurement=1(preferLeft()),那么要做的就是惩罚右转的方向
  AddEdgesPreferRotDir();
    
  return true;  
}
//这个函数就是利用g20优化器进行优化,优化次数是no_iterations(执行optimize(no_iterations))
bool TebOptimalPlanner::optimizeGraph(int no_iterations,bool clear_after)
{
  // 表示最大速度是否合法
  if (cfg_->robot.max_vel_x<0.01)
  {
    ROS_WARN("optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;	
  }
  // 都是不合法的东西
  if (!teb_.isInit() || teb_.sizePoses() < cfg_->trajectory.min_samples)
  {
    ROS_WARN("optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
    if (clear_after) clearGraph();
    return false;	
  }
  //g20轨迹优化器指针optimizer_
  // 打印详细信息
  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  //初始化整个图结构
  optimizer_->initializeOptimization();
  //给定图的当前配置和存储在类实例中的当前设置，启动一次优化运行。
  // 进行no_iterations次优化
  // 优化完后,得到的结果是更改后的teb_图,就是图中的位姿节点和时间节点都发生了改变(使error最小)
  // 所以计算速度的时候也是通过前几个节点进行计算的
  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if(!iter)
  {
	ROS_ERROR("optimizeGraph(): Optimization failed! iter=%i", iter);
	return false;
  }

  if (clear_after) clearGraph();	
    
  return true;
}

void TebOptimalPlanner::clearGraph()
{
  // clear optimizer states
  if (optimizer_)
  {
    //optimizer.edges().clear(); // optimizer.clear deletes edges!!! Therefore do not run optimizer.edges().clear()
    optimizer_->vertices().clear();  // neccessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}


//将timedElasticBand类型中的位姿顶点序列、时间顶点序列依次加入图中。
// add TEB vertices 添加节点，一种是位姿节点，一种是时间差节点
void TebOptimalPlanner::AddTEBVertices()
{
  // add vertices to graph
  ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // used for vertices ids
  //就是按照id_counter顺序将位姿节点和时间节点放进到g20轨迹优化器中

  for (int i=0; i<teb_.sizePoses(); ++i)
  {
    //teb_是TimedElasticBand类指针
    //teb_已经在plan（就是在initTrajectoryToGoal或者下面的update函数中已经进行初始化/更新）进行赋值处理了
    teb_.PoseVertex(i)->setId(id_counter++);
    //optimizer_是g20轨迹优化器指针
    optimizer_->addVertex(teb_.PoseVertex(i));
    if (teb_.sizeTimeDiffs()!=0 && i<teb_.sizeTimeDiffs())
    {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));
    }
  } 
}

//把障碍物信息加到图搜索中
// 根据cfg_->obstacles.inflation_dist与cfg_->obstacles.min_obstacle_dist大小进行判断到底是加入EdgeInflatedObstacle还是EdgeObstacle
// 只考虑 cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor内的障碍物
// 和不远处最近的左右两边的障碍物
void TebOptimalPlanner::AddEdgesObstacles(double weight_multiplier)
{
  // 如果权重是0,那么就不用管它
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr )
    return; // if weight equals zero skip adding edges!
    
  // 是否膨胀半径大于距离障碍物的最小距离
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
  // 包含与障碍物保持最小距离的优化权重(默认50)和膨胀惩罚的优化权重（应该很小）。
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;
    
  // iterate all teb points (skip first and last)
  // 迭代所有teb点(跳过第一个和最后一个)
  // 然后找到距离某一个节点比较近(min_obstacle_dist*obstacle_association_force_inclusion_factor)的障碍物或者边
  for (int i=1; i < teb_.sizePoses()-1; ++i)
  {    
      double left_min_dist = std::numeric_limits<double>::max();
      double right_min_dist = std::numeric_limits<double>::max();
      Obstacle* left_obstacle = nullptr;
      Obstacle* right_obstacle = nullptr;
      
      std::vector<Obstacle*> relevant_obstacles;
      // 返回这个点的theta的向量
      const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();
      
      // iterate obstacles
      //就是迭代所有的障碍物
      //obst代表其中一个障碍物
      //obstacles_ 注意 在机器人后面,并且超过了costmap_obstacles_behind_robot_dist的障碍物不予考虑

      // 找到最小距离是满足下面的点,加入到relevant_obstacles(相关障碍物)
      // 障碍物离的不是很近,但是呢,也离得不是很远,所以进行另外的处理,就是找到离得最近的左右两边障碍物
      for (const ObstaclePtr& obst : *obstacles_)
      {
        // we handle dynamic obstacles differently below
        //下面我们将以不同的方式处理动态障碍
        if(cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic())
          continue;

          // calculate distance to robot model
          //计算路径点处机器人轮廓到障碍物的最小距离
          double dist = robot_model_->calculateDistance(teb_.Pose(i), obst.get());
          
          // force considering obstacle if really close to the current pose
          //如果最小距离是满足下面的要求的话,那么就加入到relevant_obstacles(相关障碍物)
        if (dist < cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor)
          {
              relevant_obstacles.push_back(obst.get());
              continue;
          }
          // cut-off distance
          // 距离超过下面的距离时,就不用考虑了
          if (dist > cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_cutoff_factor)
            continue;
          
          // 进行到下面这些,说明障碍物离的不是很近,但是呢,也离得不是很远,所以进行另外的处理,就是找到离得最近的左右两边障碍物
          // determine side (left or right) and assign obstacle if closer than the previous one
          // 确定边(左或右)，如果距离较近，则分配障碍
          // getCentroid得到障碍物的质心坐标
          // 表示障碍物在这个点的左边,因为叉乘为正
          if (cross2d(pose_orient, obst->getCentroid()) > 0) // left
          {
              // 更新最小左边障碍物
              if (dist < left_min_dist)
              {
                  left_min_dist = dist;
                  left_obstacle = obst.get();
              }
          }
          else
          {
              // 更新右边最小障碍物
              if (dist < right_min_dist)
              {
                  right_min_dist = dist;
                  right_obstacle = obst.get();
              }
          }
      }   
      
      // create obstacle edges
      //创造障碍边缘
      // 如果左边有障碍物
      if (left_obstacle)
      {
            // 表示cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist,那么就要考虑膨胀的影响了
            if (inflated)
            {
                // 这个是一个一元边
                EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
                //将障碍物和这个节点连接成一条边
                //这个只需要一个节点就可以了，就是本节点
                // 设置边的节点(就是路径中的第i个路径点),这个是一元边,所以就是设置为0
                dist_bandpt_obst->setVertex(0,teb_.PoseVertex(i));
                // information_inflated包含与障碍物保持最小距离的优化权重(默认50)和膨胀惩罚的优化权重（应该很小）。
                dist_bandpt_obst->setInformation(information_inflated);
                // 设置信息,注意这里面的观测值就是上面判断出来离机器人最近的左边的障碍物(障碍物离的不是很近,但是呢,也离得不是很远,所以进行另外的处理得到的障碍物信息)
                dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), left_obstacle);
                // 把边加进去
                optimizer_->addEdge(dist_bandpt_obst);
            }
            else
            {
                // 注意,当cfg_->obstacles.inflation_dist < cfg_->obstacles.min_obstacle_dist时候执行的是这个边
                // 计算误差值,如果路径点距离障碍物小于cfg_->obstacles.min_obstacle_dist+cfg_->optim.penalty_epsilon,那么就给惩罚,也就是error[0]=正值,否则为0
                EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
                dist_bandpt_obst->setVertex(0,teb_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information);
                dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), left_obstacle);
                optimizer_->addEdge(dist_bandpt_obst);
            }
      }
      //创造障碍边缘
      // 如果右边有障碍物
      if (right_obstacle)
      {
            // 如果考虑膨胀
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
                dist_bandpt_obst->setVertex(0,teb_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information_inflated);
                dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), right_obstacle);
                optimizer_->addEdge(dist_bandpt_obst);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
                dist_bandpt_obst->setVertex(0,teb_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information);
                dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), right_obstacle);
                optimizer_->addEdge(dist_bandpt_obst);
            }   
      }
      // 遍历离路径点很近的障碍物
      for (const Obstacle* obst : relevant_obstacles)
      {
            // 如果考虑膨胀
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
                dist_bandpt_obst->setVertex(0,teb_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information_inflated);
                dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst);
                optimizer_->addEdge(dist_bandpt_obst);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
                dist_bandpt_obst->setVertex(0,teb_.PoseVertex(i));
                dist_bandpt_obst->setInformation(information);
                dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst);
                optimizer_->addEdge(dist_bandpt_obst);
            }   
      }
  }  
        
}

// 先不用考虑这个
void TebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information; 
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
    
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;
  
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;
    
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (cfg_->obstacles.include_dynamic_obstacles && (*obst)->isDynamic()) // we handle dynamic obstacles differently below
      continue; 
    
    int index;
    
    if (cfg_->obstacles.obstacle_poses_affected >= teb_.sizePoses())
      index =  teb_.sizePoses() / 2;
    else
      index = teb_.findClosestTrajectoryPose(*(obst->get()));
     
    
    // check if obstacle is outside index-range between start and goal
    if ( (index <= 1) || (index > teb_.sizePoses()-2) ) // start and goal are fixed and findNearestBandpoint finds first or last conf if intersection point is outside the range
	    continue; 
        
    if (inflated)
    {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }

    for (int neighbourIdx=0; neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected/2); neighbourIdx++)
    {
      if (index+neighbourIdx < teb_.sizePoses())
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information_inflated);
                dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_r = new EdgeObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information);
                dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
      }
      if ( index - neighbourIdx >= 0) // needs to be casted to int to allow negative values
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information_inflated);
                dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_l = new EdgeObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information);
                dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
      }
    } 
    
  }
}

// 这个是动态障碍物相关的边
//  计算_measurement以当前速度运行t时间段后与bandpt的距离
//  计算误差值,如果路径点距离障碍物小于cfg_->obstacles.min_obstacle_dist+cfg_->optim.penalty_epsilon,那么就给惩罚,也就是error[0]=正值,否则为0
// 同理路径点距离障碍物小于cfg_->obstacles.dynamic_obstacle_inflation_dist也要进行判断
void TebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier)
{
  // 权重为0,就不用考虑
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==NULL )
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1,1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0,1) = information(1,0) = 0;
  // 遍历所有的障碍物
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    // 检查障碍物是否为动态障碍物,如果是就往下走
    if (!(*obst)->isDynamic())
      continue;

    // Skip first and last pose, as they are fixed
    double time = teb_.TimeDiff(0);
    for (int i=1; i < teb_.sizePoses() - 1; ++i)
    {
      // 这个是动态障碍物相关的边
      //  计算_measurement以当前速度运行t时间段后与bandpt的距离
      //  计算误差值,如果路径点距离障碍物小于cfg_->obstacles.min_obstacle_dist+cfg_->optim.penalty_epsilon,那么就给惩罚,也就是error[0]=正值,否则为0
      // 同理路径点距离障碍物小于cfg_->obstacles.dynamic_obstacle_inflation_dist也要进行判断
      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(time);
      // 与那个节点相连接
      dynobst_edge->setVertex(0,teb_.PoseVertex(i));
      // 将信息加进去
      dynobst_edge->setInformation(information);
      // 进行一个一个的遍历(是动态障碍物)
      dynobst_edge->setParameters(*cfg_, robot_model_.get(), obst->get());
      optimizer_->addEdge(dynobst_edge);
      // 这个时间是需要累加的,因为这个循环是遍历所有的路径点,进入下一个时间点的话,时间也是需要累加,这样障碍物才可以动态的移动到下一个位置
      time += teb_.TimeDiff(i); // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".
    }
  }
}


// 一元边 与通过点相关的边
// cost便是离当前路径点最近的通过点与当前点最近的距离
void TebOptimalPlanner::AddEdgesViaPoints()
{
  if (cfg_->optim.weight_viapoint==0 || via_points_==NULL || via_points_->empty() )
    return; // if weight equals zero skip adding edges!

  int start_pose_idx = 0;
  
  int n = teb_.sizePoses();
  //我们没有任何自由度来到达通过点
  if (n<3) // we do not have any degrees of freedom for reaching via-points
    return;
  
  for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
  {
    // 找到轨道上离给定参考点最近的点的距离,并且返回这个点的索引
    int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
    // 这个判断可以减少计算量,但是可能引发错误
    if (cfg_->trajectory.via_points_ordered)
      start_pose_idx = index+2; // skip a point to have a DOF inbetween for further via-points
     
    // check if point conicides with goal or is located behind it
    //检查point是否与goal重合或位于goal后面
    if ( index > n-2 ) 
      index = n-2; // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    // 与起点重合或者在起点的前面
    if ( index < 1)
    {
      if (cfg_->trajectory.via_points_ordered)
      {
        index = 1; // try to connect the via point with the second (and non-fixed) pose. It is likely that autoresize adds new poses inbetween later.
      }
      else
      {
        ROS_DEBUG("TebOptimalPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
        continue; // skip via points really close or behind the current robot pose
      }
    }
    // 上面两个if都是为了让点在局部路径点里面

    Eigen::Matrix<double,1,1> information;
    // 信息矩阵,存储的是权重值
    information.fill(cfg_->optim.weight_viapoint);
    // setViaPoint这个函数是在homotopy_class_planner文件中才会使用
    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0,teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->setParameters(*cfg_, &(*vp_it));
    optimizer_->addEdge(edge_viapoint);   
  }
}

// 速度边,超过设定的速度(注意这里要考虑安全裕度)就会进行惩罚,注意误差是两个值,一个是线速度一个是角速度
void TebOptimalPlanner::AddEdgesVelocity()
{
  // 表示是非全向机器人
  if (cfg_->robot.max_vel_y == 0) // non-holonomic robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!


    int n = teb_.sizePoses();
    // 跟最大角速度和线速度有关的权重值
    Eigen::Matrix<double,2,2> information;
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;

    for (int i=0; i < n - 1; ++i)
    {
      // 速度边,超过设定的速度就会进行惩罚,注意误差是两个值,一个是线速度一个是角速度
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  }
  else // holonomic-robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_y==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!
      
    int n = teb_.sizePoses();
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_y;
    information(2,2) = cfg_->optim.weight_max_vel_theta;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    } 
    
  }
}

// 加速度边:计算的是从当前速度到想要的速度中间的加速度是否满足要求,不满足要求的话,就给上惩罚
void TebOptimalPlanner::AddEdgesAcceleration()
{
  if (cfg_->optim.weight_acc_lim_x==0  && cfg_->optim.weight_acc_lim_theta==0) 
    return; // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();  
  // 非全向底盘
  if (cfg_->robot.max_vel_y == 0 || cfg_->robot.acc_lim_y == 0) // non-holonomic robot
  {
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    // 检查是否应该考虑初始速度,一般这个速度就是机器人当前的速度
    if (vel_start_.first)
    {
      // 加速度边:计算的是从当前速度到想要的速度中间的加速度是否满足要求,不满足要求的话,就给上惩罚
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      // 这个是五元边:三个节点和两个时间点,通过这个信息计算加速度,如果超过就惩罚
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    // 这个是通过free_goal_vel这个变量设置的
    // 如果free_goal_vel为True,那么这个first就是false,就不用对终点速度进行加速度限制
    if (vel_goal_.first)
    {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));//这个是终点的上一个点
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));//这个就是终点
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
  else // holonomic robot
  {
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_y;
    information(2,2) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
}


// 一元边,表示时间节点时间越长,那么误差越大
void TebOptimalPlanner::AddEdgesTimeOptimal()
{
  if (cfg_->optim.weight_optimaltime==0) 
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (int i=0; i < teb_.sizeTimeDiffs(); ++i)
  {
    // 一元边,表示时间节点时间越长,那么误差越大
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0,teb_.TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}
// 二元边 两个位姿节点之间的距离越小,那么他的误差越小,这就是为什么之前要把局部路径规划点进行重新分割
// 分割成在dt_ref时间以最快速度运行得到的间隔点
void TebOptimalPlanner::AddEdgesShortestPath()
{
  if (cfg_->optim.weight_shortest_path==0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  for (int i=0; i < teb_.sizePoses()-1; ++i)
  {
    // 二元边 两个位姿节点之间的距离越小,那么他的误差越小,这就是为什么之前要把局部路径规划点进行重新分割
    // 分割成在dt_ref时间以最快速度运行得到的间隔点
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0,teb_.PoseVertex(i));
    shortest_path_edge->setVertex(1,teb_.PoseVertex(i+1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}


//添加所有边缘(局部代价函数)以满足差动驱动机器人的运动学约束
//  定义了差分驱动移动机器人满足非完整运动学的代价函数。
//  有两个误差值,第一个是非完整数学模型的误差值,看论文(Trajectory modification considering dynamic constraints of autonomous robots)
//  第二个是正向运动,如果是负的,表示是反向的,要惩罚
void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!
  
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  // 满足非完整运动学的优化权值
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  // 优化重量，迫使机器人只选择正向方向(正转。速度，只有diffdrive机器人)
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    //  定义了差分驱动移动机器人满足非完整运动学的代价函数。
    //  有两个误差值,第一个是非完整数学模型的误差值,看论文(Trajectory modification considering dynamic constraints of autonomous robots)
    //  第二个是正向运动,如果是负的,表示是反向的,要惩罚
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }	 
}
// 不用这个
void TebOptimalPlanner::AddEdgesKinematicsCarlike()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_turning_radius==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }  
}

//添加所有的边(局部代价函数)来选择一个特定的转向方向(通过惩罚另一个)
// 这个函数就是如果_measurement=1(preferLeft()),那么要做的就是惩罚右转的方向
void TebOptimalPlanner::AddEdgesPreferRotDir()
{
  //TODO(roesmann): Note, these edges can result in odd predictions, in particular
  //                we can observe a substantional mismatch between open- and closed-loop planning
  //                leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation recovery:
  //                Activating the edge for a short time period might not be crucial and
  //                could move the robot to a new oscillation-free state.
  //                This needs to be analyzed in more detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir==0)
    return; // if weight equals zero skip adding edges!
  // 当prefer_rotdir_ == RotType::none就直接退出
  // 存储是否在优化中选择一个特定的初始旋转(可能在机器人振荡时被激活)
  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left)
  {
    ROS_WARN("TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,1,1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);
  
  for (int i=0; i < teb_.sizePoses()-1 && i < 3; ++i) // currently: apply to first 3 rotations
  {
    // 这个函数就是如果_measurement=1(preferLeft()),那么要做的就是惩罚右转的方向
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0,teb_.PoseVertex(i));
    rotdir_edge->setVertex(1,teb_.PoseVertex(i+1));      
    rotdir_edge->setInformation(information_rotdir);
    
    if (prefer_rotdir_ == RotType::left)
        rotdir_edge->preferLeft();
    else if (prefer_rotdir_ == RotType::right)
        rotdir_edge->preferRight();
    
    optimizer_->addEdge(rotdir_edge);
  }
}
// 没使用这个
void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{ 
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty())
  {
    // here the graph is build again, for time efficiency make sure to call this function 
    // between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph();	
    optimizer_->initializeOptimization();
  }
  else
  {
    graph_exist_flag = true;
  }
  
  optimizer_->computeInitialGuess();
  
  cost_ = 0;

  if (alternative_time_cost)
  {
    cost_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
    // since we are using an AutoResize Function with hysteresis.
  }
  
  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = optimizer_->activeEdges().begin(); it!= optimizer_->activeEdges().end(); it++)
  {
    double cur_cost = (*it)->chi2();

    if (dynamic_cast<EdgeObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr)
    {
      cur_cost *= obst_cost_scale;
    }
    else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr)
    {
      cur_cost *= viapoint_cost_scale;
    }
    else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr && alternative_time_cost)
    {
      continue; // skip these edges if alternative_time_cost is active
    }
    cost_ += cur_cost;
  }

  // delete temporary created graph
  if (!graph_exist_flag) 
    clearGraph();
}

// 根据图,计算速度
void TebOptimalPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const
{
  if (dt == 0)
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }
  
  Eigen::Vector2d deltaS = pose2.position() - pose1.position();
  // 非全向机器人
  // 计算线速度
  if (cfg_->robot.max_vel_y == 0) // nonholonomic robot
  {
    Eigen::Vector2d conf1dir( cos(pose1.theta()), sin(pose1.theta()) );
    // translational velocity
    double dir = deltaS.dot(conf1dir);
    vx = (double) g2o::sign(dir) * deltaS.norm()/dt;
    vy = 0;
  }
  else // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double p1_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;    
  }
  
  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  // 计算角速度
  omega = orientdiff/dt;
}
// look_ahead_poses什么意思呢?就是计算好图之后,位姿节点和时间节点都是会更新的,那么计算速度就是计算前几个(control_look_ahead_poses)位姿节点和时间节点
// 默认control_look_ahead_poses为1
// 根据图,计算速度
bool TebOptimalPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
{
  if (teb_.sizePoses()<2)
  {
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
  look_ahead_poses = std::max(1, std::min(look_ahead_poses, teb_.sizePoses() - 1));
  double dt = 0.0;
  for(int counter = 0; counter < look_ahead_poses; ++counter)
  {
    dt += teb_.TimeDiff(counter);
    // 累计时间不能过长,就是dt_ref * look_ahead_poses就可以
    if(dt >= cfg_->trajectory.dt_ref * look_ahead_poses)  // TODO: change to look-ahead time? Refine trajectory?
    {
        look_ahead_poses = counter + 1;
        break;
    }
  }
  if (dt<=0)
  {	
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
	  
  // Get velocity from the first two configurations
  // 根据图,计算速度
  extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);
  return true;
}

void TebOptimalPlanner::getVelocityProfile(std::vector<geometry_msgs::Twist>& velocity_profile) const
{
  int n = teb_.sizePoses();
  velocity_profile.resize( n+1 );

  // start velocity 
  velocity_profile.front().linear.z = 0;
  velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;  
  velocity_profile.front().linear.x = vel_start_.second.linear.x;
  velocity_profile.front().linear.y = vel_start_.second.linear.y;
  velocity_profile.front().angular.z = vel_start_.second.angular.z;
  
  for (int i=1; i<n; ++i)
  {
    velocity_profile[i].linear.z = 0;
    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
  }
  
  // goal velocity
  velocity_profile.back().linear.z = 0;
  velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;  
  velocity_profile.back().linear.x = vel_goal_.second.linear.x;
  velocity_profile.back().linear.y = vel_goal_.second.linear.y;
  velocity_profile.back().angular.z = vel_goal_.second.angular.z;
}

void TebOptimalPlanner::getFullTrajectory(std::vector<TrajectoryPointMsg>& trajectory) const
{
  int n = teb_.sizePoses();
  
  trajectory.resize(n);
  
  if (n == 0)
    return;
     
  double curr_time = 0;
  
  // start
  TrajectoryPointMsg& start = trajectory.front();
  teb_.Pose(0).toPoseMsg(start.pose);
  start.velocity.linear.z = 0;
  start.velocity.angular.x = start.velocity.angular.y = 0;
  start.velocity.linear.x = vel_start_.second.linear.x;
  start.velocity.linear.y = vel_start_.second.linear.y;
  start.velocity.angular.z = vel_start_.second.angular.z;
  start.time_from_start.fromSec(curr_time);
  
  curr_time += teb_.TimeDiff(0);
  
  // intermediate points
  for (int i=1; i < n-1; ++i)
  {
    TrajectoryPointMsg& point = trajectory[i];
    teb_.Pose(i).toPoseMsg(point.pose);
    point.velocity.linear.z = 0;
    point.velocity.angular.x = point.velocity.angular.y = 0;
    double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), vel1_x, vel1_y, omega1);
    extractVelocity(teb_.Pose(i), teb_.Pose(i+1), teb_.TimeDiff(i), vel2_x, vel2_y, omega2);
    point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
    point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
    point.velocity.angular.z = 0.5*(omega1+omega2);    
    point.time_from_start.fromSec(curr_time);
    
    curr_time += teb_.TimeDiff(i);
  }
  
  // goal
  TrajectoryPointMsg& goal = trajectory.back();
  teb_.BackPose().toPoseMsg(goal.pose);
  goal.velocity.linear.z = 0;
  goal.velocity.angular.x = goal.velocity.angular.y = 0;
  goal.velocity.linear.x = vel_goal_.second.linear.x;
  goal.velocity.linear.y = vel_goal_.second.linear.y;
  goal.velocity.angular.z = vel_goal_.second.angular.z;
  goal.time_from_start.fromSec(curr_time);
}

// 检查轨迹的冲突情况
// footprint_spec机器人的轮廓
// look_ahead_idx指定每个采样间隔应在预测计划的哪个位置进行可行性检查。检测姿态在规划路径的可行性的时间间隔
// 检查两个姿态之间的距离是否大于机器人半径或方向差值大于指定的阈值，并在这种情况下进行插值。
// 这个为什么进行差值呢,因为图中可能两个位姿节点之间的距离变大,障碍物位于中间
// 所以要做的就是插值,在两个节点之间插值,然后判断是否和障碍物相撞
bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
{
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses() - 1;
  // look_ahead_idx表示要向前方看多远(这段范围进行判断)
  for (int i=0; i <= look_ahead_idx; ++i)
  {           
    // 检查机器人在这个点的时候障碍物是否和机器人碰撞
    if ( costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius) < 0 )
    {
      if (visualization_)
      {
        visualization_->publishInfeasibleRobotPose(teb().Pose(i), *robot_model_);
      }
      return false;
    }
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold and interpolates in that case.
    // 检查两个姿态之间的距离是否大于机器人半径或方向差值大于指定的阈值，并在这种情况下进行插值。
    // (如果障碍物将两个连续的poses分开，两个连续poses之间的中心可能与障碍物重合;-)!
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
    if (i<look_ahead_idx)
    {
      // 两个点的角度
      double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) -
                                              g2o::normalize_theta(teb().Pose(i).theta()));
      // 两个点的距离差
      Eigen::Vector2d delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();
      // 检查两个姿态之间的距离是否大于机器人半径或方向差值大于指定的阈值，并在这种情况下进行插值。
      // 这个为什么进行差值呢,因为图中可能两个位姿节点之间的距离变大,障碍物位于中间
      // 所以要做的就是插值,在两个节点之间插值,然后判断是否和障碍物相撞
      if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
      {
        // 插值数目
        int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular), 
                                            std::ceil(delta_dist.norm() / inscribed_radius)) - 1;
        // 从起步点开始进行插值
        PoseSE2 intermediate_pose = teb().Pose(i);
        for(int step = 0; step < n_additional_samples; ++step)
        {
          // 位姿累加,然后下面进行判断
          intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
          intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() + 
                                                           delta_rot / (n_additional_samples + 1.0));
          if ( costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
            footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
          {
            if (visualization_) 
            {
              visualization_->publishInfeasibleRobotPose(intermediate_pose, *robot_model_);
            }
            return false;
          }
        }
      }
    }
  }
  return true;
}

} // namespace teb_local_planner
