// HomotopyClassPlanner 是一种同时优化多个轨迹的方法，由于目标函数的非凸性会生成一系列最优的候选轨迹，最终在备选局部解的集合中寻求总体最佳候选轨迹。
#include <teb_local_planner/homotopy_class_planner.h>

#include <limits>

namespace teb_local_planner
{

HomotopyClassPlanner::HomotopyClassPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), robot_model_(new PointRobotFootprint()), initial_plan_(NULL), initialized_(false)
{
}
//初始化普通的规划器
//obstacles_是一个空的 存放所有相关障碍物的容器(见障碍物)
//robot_model存储着机器人的形状信息 用于优化的机器人形状模型的共享指针(可选)
//visualization_创建可视化实例
//via_points_在局部轨迹优化过程中需要考虑的节点容器
//planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_)); 
HomotopyClassPlanner::HomotopyClassPlanner(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model,
                                           TebVisualizationPtr visual, const ViaPointContainer* via_points) : initial_plan_(NULL)
{
  initialize(cfg, obstacles, robot_model, visual, via_points);
}

HomotopyClassPlanner::~HomotopyClassPlanner()
{
}

void HomotopyClassPlanner::initialize(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model,
                                      TebVisualizationPtr visual, const ViaPointContainer* via_points)
{
  cfg_ = &cfg;
  obstacles_ = obstacles;
  via_points_ = via_points;
  robot_model_ = robot_model;
  // 如果为真，则使用简单的左右方法（通过左侧或右侧的每个障碍物）来探索独特的轨迹以生成路径，
  // 否则在起点和目标之间的指定区域随机采样可能的路线图。
  // 这个在论文中会进行体现
  // 默认为假,表示随机采样
  if (cfg_->hcp.simple_exploration)
    graph_search_ = boost::shared_ptr<GraphSearchInterface>(new lrKeyPointGraph(*cfg_, this));
  else
    graph_search_ = boost::shared_ptr<GraphSearchInterface>(new ProbRoadmapGraph(*cfg_, this));

  initialized_ = true;

  setVisualization(visual);
}

// 可视化判断符
void HomotopyClassPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}


//planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
//这里的initial_plan是经过处理的，就是局部路径规划的点
//free_goal_vel表示 如果最后的速度没有限制,就是在加速度边上对其没有限制
bool HomotopyClassPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");

  // store initial plan for further initializations (must be valid for the lifetime of this object or clearPlanner() is called!)
  // 把全局中的局部路径规划点给传进来
  initial_plan_ = &initial_plan;

  PoseSE2 start(initial_plan.front().pose);
  PoseSE2 goal(initial_plan.back().pose);

  return plan(start, goal, start_vel, free_goal_vel);
}


bool HomotopyClassPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  PoseSE2 start_pose(start);
  PoseSE2 goal_pose(goal);
  return plan(start_pose, goal_pose, start_vel, free_goal_vel);
}
// 这个是实际运行的函数
bool HomotopyClassPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");

  // Update old TEBs with new start, goal and velocity
  // 第一次执行，这个是进去之后不进行任何执行的
  // 之后这个函数的作用的就是更新各个轨迹优化器的长度，并且设置新的start和goal
  updateAllTEBs(&start, &goal, start_vel);

  // Init new TEBs based on newly explored homotopy classes
  // min_obstacle_dist最小障碍物距离
  exploreEquivalenceClassesAndInitTebs(start, goal, cfg_->obstacles.min_obstacle_dist, start_vel);
  // update via-points if activated
  updateReferenceTrajectoryViaPoints(cfg_->hcp.viapoints_all_candidates);
  // Optimize all trajectories in alternative homotopy classes
  optimizeAllTEBs(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
  // Select which candidate (based on alternative homotopy classes) should be used
  selectBestTeb();

  initial_plan_ = nullptr; // clear pointer to any previous initial plan (any previous plan is useless regarding the h-signature);
  return true;
}

bool HomotopyClassPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
{
  TebOptimalPlannerConstPtr best_teb = bestTeb();
  if (!best_teb)
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }

  return best_teb->getVelocityCommand(vx, vy, omega, look_ahead_poses);
}




void HomotopyClassPlanner::visualize()
{
  if (visualization_)
  {
    // Visualize graph
    if (cfg_->hcp.visualize_hc_graph && graph_search_)
      visualization_->publishGraph(graph_search_->graph_);

    // Visualize active tebs as marker
    visualization_->publishTebContainer(tebs_);

    // Visualize best teb and feedback message if desired
    TebOptimalPlannerConstPtr best_teb = bestTeb();
    if (best_teb)
    {
      visualization_->publishLocalPlanAndPoses(best_teb->teb());

      if (best_teb->teb().sizePoses() > 0) //TODO maybe store current pose (start) within plan method as class field.
        visualization_->publishRobotFootprintModel(best_teb->teb().Pose(0), *robot_model_);

      // feedback message
      if (cfg_->trajectory.publish_feedback)
      {
        int best_idx = bestTebIdx();
        if (best_idx>=0)
          visualization_->publishFeedbackMessage(tebs_, (unsigned int) best_idx, *obstacles_);
      }
    }
  }
  else ROS_DEBUG("Ignoring HomotopyClassPlanner::visualize() call, since no visualization class was instantiated before.");
}



bool HomotopyClassPlanner::hasEquivalenceClass(const EquivalenceClassPtr& eq_class) const
{
  // iterate existing h-signatures and check if there is an existing H-Signature similar the candidate
  for (const std::pair<EquivalenceClassPtr, bool>& eqrel : equivalence_classes_)
  {
     if (eq_class->isEqual(*eqrel.first))
        return true; // Found! Homotopy class already exists, therefore nothing added
  }
  return false;
}
// 这个就是判断一条轨迹的eq_class是否在equivalence_classes_,如果没有就加进去返回true,如果已经在里面了,就返回false
bool HomotopyClassPlanner::addEquivalenceClassIfNew(const EquivalenceClassPtr& eq_class, bool lock)
{
  if (!eq_class)
    return false;

  if (!eq_class->isValid())
  {
    ROS_WARN("HomotopyClassPlanner: Ignoring invalid H-signature");
    return false;
  }

  if (hasEquivalenceClass(eq_class))
    return false;

  // Homotopy class not found -> Add to class-list, return that the h-signature is new
  equivalence_classes_.push_back(std::make_pair(eq_class,lock));
  return true;
}
// 第一次也是进行什么都执行不了
// 利用了start和goal更新轨迹优化器中的轨迹，并且判断是否同伦，如果同伦就删除tebs_中同伦的轨迹
// 这样得到的tebs_都是不同伦的轨迹，并且最后执行deletePlansDetouringBackwards函数
// deletePlansDetouringBackwards这个函数将大于orient_threshold、没有被优化执行时间/最佳teb的执行时间 > max_ratio_detours_duration_best_duration的轨迹优化器都删除
void HomotopyClassPlanner::renewAndAnalyzeOldTebs(bool delete_detours)
{
  // clear old h-signatures (since they could be changed due to new obstacle positions.
  equivalence_classes_.clear();

  // Adding the equivalence class of the latest best_teb_ first
  // 先添加最新的best_teb_的equivalence类,这个是判断best_teb_是否在tebs_里面
  TebOptPlannerContainer::iterator it_best_teb = best_teb_ ? std::find(tebs_.begin(), tebs_.end(), best_teb_) : tebs_.end();
  // 表示在里面
  bool has_best_teb = it_best_teb != tebs_.end();
  if (has_best_teb)
  {
    // 将最后一个最好的 teb 放在容器的开头
    std::iter_swap(tebs_.begin(), it_best_teb);  // Putting the last best teb at the beginning of the container
    // best_teb_->teb()这个就是返回轨迹优化器中优化的轨迹
    // calculateEquivalenceClass这个根据论文中的计算H-signature
    // addEquivalenceClassIfNew这个就是判断一条轨迹的eq_class是否在equivalence_classes_,如果没有就加进去返回true,如果已经在里面了,就返回false
    addEquivalenceClassIfNew(calculateEquivalenceClass(best_teb_->teb().poses().begin(),
      best_teb_->teb().poses().end(), getCplxFromVertexPosePtr , obstacles_,
      best_teb_->teb().timediffs().begin(), best_teb_->teb().timediffs().end()));
  }
  // Collect h-signatures for all existing TEBs and store them together with the corresponding iterator / pointer:
//   typedef std::list< std::pair<TebOptPlannerContainer::iterator, std::complex<long double> > > TebCandidateType;
//   TebCandidateType teb_candidates;

  // get new homotopy classes and delete multiple TEBs per homotopy class. Skips the best teb if available (added before).
  // 如果has_best_teb是真,表示第一个是最优的,之前也遍历过,所以现在不用进行遍历了,就直接略过
  TebOptPlannerContainer::iterator it_teb = has_best_teb ? std::next(tebs_.begin(), 1) : tebs_.begin();
  while(it_teb != tebs_.end())
  {
    // calculate equivalence class for the current candidate
    // calculateEquivalenceClass这个根据论文中的计算H-signature,计算当前候选轨迹的equivalence class
    EquivalenceClassPtr equivalence_class = calculateEquivalenceClass(it_teb->get()->teb().poses().begin(), it_teb->get()->teb().poses().end(), getCplxFromVertexPosePtr , obstacles_,
                                                                      it_teb->get()->teb().timediffs().begin(), it_teb->get()->teb().timediffs().end());

//     teb_candidates.push_back(std::make_pair(it_teb,H));

    // WORKAROUND until the commented code below works
    // Here we do not compare cost values. Just first come first serve...
    // 这个就是判断一条轨迹的eq_class是否在equivalence_classes_,如果没有就加进去返回true,如果已经在里面了,就返回false
    bool new_flag = addEquivalenceClassIfNew(equivalence_class);
    // 进去的话表示已经在里面了
    if (!new_flag)
    {
      // 如果在里面的话就把轨迹优化器集合中的相应的轨迹优化删除
      // 这样迭代下去,剩下的就是不同伦的轨迹了
      it_teb = tebs_.erase(it_teb);
      // 直接略过,it_teb不增加,因为已经删除了,所以指针本身就会改变了
      continue;
    }

    ++it_teb;
  }
  // 如果启用，规划器将丢弃相对于最佳计划向后绕行的计划
  // detours_orientation_tolerance如果一个计划的开始方向与最佳计划的差异超过这个范围，则该计划被认为是绕道而行
  // length_start_orientation_vector用于计算计划起始方向的向量的长度
  // 这个函数将大于orient_threshold、没有被优化执行时间/最佳teb的执行时间 > max_ratio_detours_duration_best_duration的轨迹优化器都删除
  if(delete_detours)
    deletePlansDetouringBackwards(cfg_->hcp.detours_orientation_tolerance, cfg_->hcp.length_start_orientation_vector);

  // Find multiple candidates and delete the one with higher cost
  // TODO: this code needs to be adpated. Erasing tebs from the teb container_ could make iteratores stored in the candidate list invalid!
//   TebCandidateType::reverse_iterator cand_i = teb_candidates.rbegin();
//   int test_idx = 0;
//   while (cand_i != teb_candidates.rend())
//   {
//
//     TebCandidateType::reverse_iterator cand_j = std::find_if(boost::next(cand_i),teb_candidates.rend(), boost::bind(compareH,_1,cand_i->second));
//     if (cand_j != teb_candidates.rend() && cand_j != cand_i)
//     {
//         TebOptimalPlannerPtr pt1 = *(cand_j->first);
//         TebOptimalPlannerPtr pt2 = *(cand_i->first);
//         assert(pt1);
//         assert(pt2);
//       if ( cand_j->first->get()->getCurrentCost().sum() > cand_i->first->get()->getCurrentCost().sum() )
//       {
// 	// found one that has higher cost, therefore erase cand_j
// 	tebs_.erase(cand_j->first);
// 	teb_candidates.erase(cand_j);
//       }
//       else   // otherwise erase cand_i
//       {
// 	tebs_.erase(cand_i->first);
// 	cand_i = teb_candidates.erase(cand_i);
//       }
//     }
//     else
//     {
//         ROS_WARN_STREAM("increase cand_i");
//         ++cand_i;
//     }
//   }

  // now add the h-signatures to the internal lookup-table (but only if there is no existing duplicate)
//   for (TebCandidateType::iterator cand=teb_candidates.begin(); cand!=teb_candidates.end(); ++cand)
//   {
//     bool new_flag = addNewHSignatureIfNew(cand->second, cfg_->hcp.h_signature_threshold);
//     if (!new_flag)
//     {
// //       ROS_ERROR_STREAM("getAndFilterHomotopyClassesTEB() - This schould not be happen.");
//       tebs_.erase(cand->first);
//     }
//   }

}

void HomotopyClassPlanner::updateReferenceTrajectoryViaPoints(bool all_trajectories)
{
  if ( (!all_trajectories && !initial_plan_) || !via_points_ || via_points_->empty() || cfg_->optim.weight_viapoint <= 0)
    return;

  if(equivalence_classes_.size() < tebs_.size())
  {
    ROS_ERROR("HomotopyClassPlanner::updateReferenceTrajectoryWithViaPoints(): Number of h-signatures does not match number of trajectories.");
    return;
  }

  if (all_trajectories)
  {
    // enable via-points for all tebs
    for (std::size_t i=0; i < equivalence_classes_.size(); ++i)
    {
        tebs_[i]->setViaPoints(via_points_);
    }
  }
  else
  {
    // enable via-points for teb in the same hommotopy class as the initial_plan and deactivate it for all other ones
    for (std::size_t i=0; i < equivalence_classes_.size(); ++i)
    {
      if(initial_plan_eq_class_->isEqual(*equivalence_classes_[i].first))
        tebs_[i]->setViaPoints(via_points_);
      else
        tebs_[i]->setViaPoints(NULL);
    }
  }
}

// dist_to_obst表示距离最小障碍物的距离
void HomotopyClassPlanner::exploreEquivalenceClassesAndInitTebs(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, const geometry_msgs::Twist* start_vel)
{
  // first process old trajectories
  // 第一次也是进行什么都执行不了
  // 利用了start和goal更新轨迹优化器中的轨迹，并且判断是否同伦，如果同伦就删除tebs_中同伦的轨迹
  // 这样得到的tebs_都是不同伦的轨迹，并且最后执行deletePlansDetouringBackwards函数
  // deletePlansDetouringBackwards这个函数将大于orient_threshold、没有被优化执行时间/最佳teb的执行时间 > max_ratio_detours_duration_best_duration的轨迹优化器都删除
  renewAndAnalyzeOldTebs(cfg_->hcp.delete_detours_backwards);

  // inject initial plan if available and not yet captured
  // 判断是否得到初始的轨迹
  if (initial_plan_)
  {
    // 传入的是初始的局部路劲规划轨迹和现在机器人的速度
    // 根据传进来路径点得到轨迹优化器，并且计算H-signature，如果已经在equivalence_classes_里面了，就返回空指针
    // 如果没有在里面就放进去并且返回这个轨迹优化器的指针
    initial_plan_teb_ = addAndInitNewTeb(*initial_plan_, start_vel);
  }
  else
  {
    initial_plan_teb_.reset();
    // 检查initial_plan_teb_是否仍包含在tebs_中,如果在,就返回这个指针,不再的话返回空指针
    initial_plan_teb_ = getInitialPlanTEB(); // this method searches for initial_plan_eq_class_ in the teb container (-> if !initial_plan_teb_)
  }

  // now explore new homotopy classes and initialize tebs if new ones are found. The appropriate createGraph method is chosen via polymorphism.
  // 现在探索新的同伦类并在找到新的类时初始化tebs。通过多态选择合适的createGraph方法。
  // 指定障碍航向和目标航向之间的归一化标量积的值，以便将它们（障碍物）考虑在内进行探索 [0,1]
  graph_search_->createGraph(start,goal,dist_to_obst,cfg_->hcp.obstacle_heading_threshold, start_vel);
}


TebOptimalPlannerPtr HomotopyClassPlanner::addAndInitNewTeb(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_velocity)
{
  if(tebs_.size() >= cfg_->hcp.max_number_classes)
    return TebOptimalPlannerPtr();
  TebOptimalPlannerPtr candidate =  TebOptimalPlannerPtr( new TebOptimalPlanner(*cfg_, obstacles_, robot_model_, visualization_));

  candidate->teb().initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);

  if (start_velocity)
    candidate->setVelocityStart(*start_velocity);

  EquivalenceClassPtr H = calculateEquivalenceClass(candidate->teb().poses().begin(), candidate->teb().poses().end(), getCplxFromVertexPosePtr, obstacles_,
                                                    candidate->teb().timediffs().begin(), candidate->teb().timediffs().end());

  if(addEquivalenceClassIfNew(H))
  {
    tebs_.push_back(candidate);
    return tebs_.back();
  }

  // If the candidate constitutes no new equivalence class, return a null pointer
  return TebOptimalPlannerPtr();
}

// 传入的是初始的局部路劲规划轨迹和现在机器人的速度
// 根据传进来路径点得到轨迹优化器，并且计算H-signature，如果已经在equivalence_classes_里面了，就返回空指针
// 如果没有在里面就放进去并且返回这个轨迹优化器的指针
TebOptimalPlannerPtr HomotopyClassPlanner::addAndInitNewTeb(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_velocity)
{
  // 轨迹优化器中的单个轨迹优化器的数目已经超过，直接运行TebOptimalPlannerPtr()
  if(tebs_.size() >= cfg_->hcp.max_number_classes)
    return TebOptimalPlannerPtr();
  // 创建一个单个轨迹优化器 
  TebOptimalPlannerPtr candidate = TebOptimalPlannerPtr( new TebOptimalPlanner(*cfg_, obstacles_, robot_model_, visualization_));
  // 函数的作用就是将局部路径规划的起点放进pose_vec_,然后将后面的点轮着放到pose_vec_,并且将后面的点到局部路径规划的上一个点的行驶的最短时间也轮着放进timediff_vec_
  candidate->teb().initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x,
    cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
  // 设置机器人的起始速度,一般这个速度就是机器人当前的速度
  // 这个会作为加速度边中的参数使用
  if (start_velocity)
    candidate->setVelocityStart(*start_velocity);

  // store the h signature of the initial plan to enable searching a matching teb later.
  // 存储初始计划的H-signature，以便稍后搜索匹配的 teb。
  // calculateEquivalenceClass这个根据论文中的计算H-signature
  initial_plan_eq_class_ = calculateEquivalenceClass(candidate->teb().poses().begin(), candidate->teb().poses().end(), getCplxFromVertexPosePtr, obstacles_,
                                                     candidate->teb().timediffs().begin(), candidate->teb().timediffs().end());
  // 这个就是判断一条轨迹的eq_class是否在equivalence_classes_,
  // 如果没有就加进去返回true,如果已经在里面了,就返回false
  if(addEquivalenceClassIfNew(initial_plan_eq_class_, true)) // also prevent candidate from deletion
  {
    tebs_.push_back(candidate);
    // 第一次返回的就是初始构造的单个轨迹优化器指针
    // 如果是之后的就返回最近的一个指针
    return tebs_.back();
  }

  // If the candidate constitutes no new equivalence class, return a null pointer
  // 如果候选者不构成新的等价类，则返回空指针
  return TebOptimalPlannerPtr();
}
// 第一次执行，这个是进去之后不进行任何执行的
// 之后这个函数的作用的就是更新各个轨迹优化器的长度，并且设置新的start和goal
void HomotopyClassPlanner::updateAllTEBs(const PoseSE2* start, const PoseSE2* goal, const geometry_msgs::Twist* start_velocity)
{
  // If new goal is too far away, clear all existing trajectories to let them reinitialize later.
  // Since all Tebs are sharing the same fixed goal pose, just take the first candidate:
  // 如果新目标太远，清除所有现有轨迹，让它们稍后重新初始化。
  // 由于所有 Tebs 共享相同的固定目标姿势，只需取第一个候选者：

  // tebs_不是空,需要做的就是清空 
  if (!tebs_.empty()
      && ((goal->position() - tebs_.front()->teb().BackPose().position()).norm() >= cfg_->trajectory.force_reinit_new_goal_dist
        || fabs(g2o::normalize_theta(goal->theta() - tebs_.front()->teb().BackPose().theta())) >= cfg_->trajectory.force_reinit_new_goal_angular))
  {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      tebs_.clear();
      equivalence_classes_.clear();
  }

  // hot-start from previous solutions
  // 从之前的解进行热启动,刚开始这个就是空,所以不用管它
  for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
  {
    // 这个函数就是找到离new_start最近的点(最多找到十个点)
    // 找到后删除大于的点,并且将new_start和new_goal作为新的起点和终点
    it_teb->get()->teb().updateAndPruneTEB(*start, *goal);
    if (start_velocity)
      it_teb->get()->setVelocityStart(*start_velocity);
  }
}


void HomotopyClassPlanner::optimizeAllTEBs(int iter_innerloop, int iter_outerloop)
{
  // optimize TEBs in parallel since they are independend of each other
  if (cfg_->hcp.enable_multithreading)
  {
    // Must prevent .join_all() from throwing exception if interruption was
    // requested, as this can lead to multiple threads operating on the same
    // TEB, which leads to SIGSEGV
    boost::this_thread::disable_interruption di;

    boost::thread_group teb_threads;
    for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
    {
      teb_threads.create_thread( boost::bind(&TebOptimalPlanner::optimizeTEB, it_teb->get(), iter_innerloop, iter_outerloop,
                                             true, cfg_->hcp.selection_obst_cost_scale, cfg_->hcp.selection_viapoint_cost_scale,
                                             cfg_->hcp.selection_alternative_time_cost) );
    }
    teb_threads.join_all();
  }
  else
  {
    for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
    {
      it_teb->get()->optimizeTEB(iter_innerloop,iter_outerloop, true, cfg_->hcp.selection_obst_cost_scale,
                                 cfg_->hcp.selection_viapoint_cost_scale, cfg_->hcp.selection_alternative_time_cost); // compute cost as well inside optimizeTEB (last argument = true)
    }
  }
}
// 检查initial_plan_teb_是否仍包含在tebs_中,如果在,就返回这个指针,不再的话返回空指针
TebOptimalPlannerPtr HomotopyClassPlanner::getInitialPlanTEB()
{
    // first check stored teb object
    if (initial_plan_teb_)
    {
        // check if the teb is still part of the teb container
        // 检查 teb 是否仍然是 teb 容器的一部分
        if ( std::find(tebs_.begin(), tebs_.end(), initial_plan_teb_ ) != tebs_.end() )
            return initial_plan_teb_;
        else
        {
            initial_plan_teb_.reset(); // reset pointer for next call
            ROS_DEBUG("initial teb not found, trying to find a match according to the cached equivalence class");
        }
    }

    // reset the locked state for equivalence classes // TODO: this might be adapted if not only the plan containing the initial plan is locked!
    // 如果不仅包含初始计划的计划被锁定，他可能会被调整！
    for (int i=0; i<equivalence_classes_.size(); ++i)
    {
        equivalence_classes_[i].second = false;
    }

    // otherwise check if the stored reference equivalence class exist in the list of known classes
    // 否则检查存储的reference equivalence类是否存在于已知类列表中
    if (initial_plan_eq_class_ && initial_plan_eq_class_->isValid())
    {
         if (equivalence_classes_.size() == tebs_.size())
         {
            for (int i=0; i<equivalence_classes_.size(); ++i)
            {
                if (equivalence_classes_[i].first->isEqual(*initial_plan_eq_class_))
                {
                    equivalence_classes_[i].second = true;
                    return tebs_[i];
                }
            }
         }
         else
             ROS_ERROR("HomotopyClassPlanner::getInitialPlanTEB(): number of equivalence classes (%lu) and number of trajectories (%lu) does not match.", equivalence_classes_.size(), tebs_.size());
    }
    else
        ROS_DEBUG("HomotopyClassPlanner::getInitialPlanTEB(): initial TEB not found in the set of available trajectories.");

    return TebOptimalPlannerPtr();
}

TebOptimalPlannerPtr HomotopyClassPlanner::selectBestTeb()
{
    double min_cost = std::numeric_limits<double>::max(); // maximum cost
    double min_cost_last_best = std::numeric_limits<double>::max();
    double min_cost_initial_plan_teb = std::numeric_limits<double>::max();
    // 检查initial_plan_teb_是否仍包含在tebs_中,如果在,就返回这个指针,不再的话返回空指针
    TebOptimalPlannerPtr initial_plan_teb = getInitialPlanTEB();

    // check if last best_teb is still a valid candidate
    if (best_teb_ && std::find(tebs_.begin(), tebs_.end(), best_teb_) != tebs_.end())
    {
        // get cost of this candidate
        min_cost_last_best = best_teb_->getCurrentCost() * cfg_->hcp.selection_cost_hysteresis; // small hysteresis
        last_best_teb_ = best_teb_;
    }
    else
    {
      last_best_teb_.reset();
    }

    if (initial_plan_teb) // the validity was already checked in getInitialPlanTEB()
    {
        // get cost of this candidate
        min_cost_initial_plan_teb = initial_plan_teb->getCurrentCost() * cfg_->hcp.selection_prefer_initial_plan; // small hysteresis
    }


    best_teb_.reset(); // reset pointer

    for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
    {
        // check if the related TEB leaves the local costmap region
//      if (tebs_.size()>1 && !(*it_teb)->teb().isTrajectoryInsideRegion(20, -1, 1))
//      {
//          ROS_INFO("HomotopyClassPlanner::selectBestTeb(): skipping trajectories that are not inside the local costmap");
//          continue;
//      }

        double teb_cost;

        if (*it_teb == last_best_teb_)
            teb_cost = min_cost_last_best; // skip already known cost value of the last best_teb
        else if (*it_teb == initial_plan_teb)
            teb_cost = min_cost_initial_plan_teb;
        else
            teb_cost = it_teb->get()->getCurrentCost();

        if (teb_cost < min_cost)
        {
          // check if this candidate is currently not selected
          best_teb_ = *it_teb;
          min_cost = teb_cost;
        }
     }


  // in case we haven't found any teb due to some previous checks, investigate list again
//   if (!best_teb_ && !tebs_.empty())
//   {
//       ROS_DEBUG("all " << tebs_.size() << " tebs rejected previously");
//       if (tebs_.size()==1)
//         best_teb_ = tebs_.front();
//       else // if multiple TEBs are available:
//       {
//           // try to use the one that relates to the initial plan
//           TebOptimalPlannerPtr initial_plan_teb = getInitialPlanTEB();
//           if (initial_plan_teb)
//               best_teb_ = initial_plan_teb;
//           else
//           {
//              // now compute the cost for the rest (we haven't computed it before)
//              for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
//              {
//                 double teb_cost = it_teb->get()->getCurrentCost();
//                 if (teb_cost < min_cost)
//                 {
//                     // check if this candidate is currently not selected
//                     best_teb_ = *it_teb;
//                     min_cost = teb_cost;
//                 }
//              }
//           }
//       }
//   }

    // check if we are allowed to change
    if (last_best_teb_ && best_teb_ != last_best_teb_)
    {
      ros::Time now = ros::Time::now();
      if ((now-last_eq_class_switching_time_).toSec() > cfg_->hcp.switching_blocking_period)
      {
        last_eq_class_switching_time_ = now;
      }
      else
      {
        ROS_DEBUG("HomotopyClassPlanner::selectBestTeb(): Switching equivalence classes blocked (check parameter switching_blocking_period.");
        // block switching, so revert best_teb_
        best_teb_ = last_best_teb_;
      }

    }


    return best_teb_;
}

int HomotopyClassPlanner::bestTebIdx() const
{
  if (tebs_.size() == 1)
    return 0;

  if (!best_teb_)
    return -1;

  int idx = 0;
  for (TebOptPlannerContainer::const_iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb, ++idx)
  {
    if (*it_teb == best_teb_)
      return idx;
  }
  return -1;
}

bool HomotopyClassPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                                double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
{
  bool feasible = false;
  while(ros::ok() && !feasible && tebs_.size() > 0)
  {
    TebOptimalPlannerPtr best = findBestTeb();
    if (!best)
    {
      ROS_ERROR("Couldn't retrieve the best plan");
      return false;
    }
    feasible = best->isTrajectoryFeasible(costmap_model, footprint_spec, inscribed_radius, circumscribed_radius, look_ahead_idx);
    if(!feasible)
    {
      removeTeb(best);
      if(last_best_teb_ && (last_best_teb_ == best)) // Same plan as before.
        return feasible;                             // Not failing could result in oscillations between trajectories.
    }
  }
  return feasible;
}

TebOptimalPlannerPtr HomotopyClassPlanner::findBestTeb()
{
  if(tebs_.empty())
    return TebOptimalPlannerPtr();
  if (!best_teb_ || std::find(tebs_.begin(), tebs_.end(), best_teb_) == tebs_.end())
    best_teb_ = selectBestTeb();
  return best_teb_;
}

TebOptPlannerContainer::iterator HomotopyClassPlanner::removeTeb(TebOptimalPlannerPtr& teb)
{
  TebOptPlannerContainer::iterator return_iterator = tebs_.end();
  if(equivalence_classes_.size() != tebs_.size())
  {
      ROS_ERROR("removeTeb: size of eq classes != size of tebs");
      return return_iterator;
  }
  auto it_eq_classes = equivalence_classes_.begin();
  for(auto it = tebs_.begin(); it != tebs_.end(); ++it)
  {
    if(*it == teb)
    {
      return_iterator = tebs_.erase(it);
      equivalence_classes_.erase(it_eq_classes);
      break;
    }
    ++it_eq_classes;
  }
  return return_iterator;
}

void HomotopyClassPlanner::setPreferredTurningDir(RotType dir)
{
  // set preferred turning dir for all TEBs
  for (TebOptPlannerContainer::const_iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
  {
    (*it_teb)->setPreferredTurningDir(dir);
  }
}

void HomotopyClassPlanner::computeCurrentCost(std::vector<double>& cost, double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
  {
    it_teb->get()->computeCurrentCost(cost, obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
  }
}
// 规划器将丢弃相对于最佳计划向后绕行的计划
// orient_threshold如果一个计划的开始方向与最佳计划的差异超过这个范围，则该计划被认为是绕道而行
// len_orientation_vector用于计算计划起始方向的向量的长度
// 这个函数将大于orient_threshold、没有被优化执行时间/最佳teb的执行时间 > max_ratio_detours_duration_best_duration的轨迹优化器都删除
void HomotopyClassPlanner::deletePlansDetouringBackwards(const double orient_threshold,
  const double len_orientation_vector)
{
  // 表示tebs_尺寸太小，或者轨迹不合法
  if (tebs_.size() < 2 || !best_teb_ || std::find(tebs_.begin(), tebs_.end(), best_teb_) == tebs_.end() ||
    best_teb_->teb().sizePoses() < 2)
  {
    return;  // A moving direction wasn't chosen yet
  }
  double current_movement_orientation;
  // getSumOfAllTimeDiffs求的轨迹点中所有时间的和
  double best_plan_duration = std::max(best_teb_->teb().getSumOfAllTimeDiffs(), 1.0);
  // computeStartOrientation这个函数的作用就是找到距离起始点len_orientation_vector远的点与起始点的atan2角是多少
  // 如果找到就返回真，找不到就返回假 
  if(!computeStartOrientation(best_teb_, len_orientation_vector, current_movement_orientation))
    return;  // The plan is shorter than len_orientation_vector
  for(auto it_teb = tebs_.begin(); it_teb != tebs_.end();)
  {
    // 最好的轨迹被计算过了，所以不用再进行一次计算
    if(*it_teb == best_teb_)  // The current plan should not be considered a detour
    {
      ++it_teb;
      continue;
    }
    // 这个轨迹的优化器太短了，直接删除（认为不合法）
    if((*it_teb)->teb().sizePoses() < 2)
    {
      ROS_DEBUG("Discarding a plan with less than 2 poses");
      it_teb = removeTeb(*it_teb);
      // 这个是不用+1的，因为已经删除了，那么这个指针是自动更新的
      continue;
    }
    double plan_orientation;
    if(!computeStartOrientation(*it_teb, len_orientation_vector, plan_orientation))
    {
      ROS_DEBUG("Failed to compute the start orientation for one of the tebs, likely close to the target");
      it_teb = removeTeb(*it_teb);
      continue;
    }
    // 如果大于orient_threshold，那么就删除
    if(fabs(g2o::normalize_theta(plan_orientation - current_movement_orientation)) > orient_threshold)
    {
      it_teb = removeTeb(*it_teb);  // Plan detouring backwards
      continue;
    }
    // 如果没有被优化，也删除
    if(!it_teb->get()->isOptimized())
    {
      ROS_DEBUG("Removing a teb because it's not optimized");
      it_teb = removeTeb(*it_teb);  // Deletes tebs that cannot be optimized (last optim call failed)
      continue;
    }
    // 如果 他们的执行时间/最佳teb的执行时间 > max_ratio_detours_duration_best_duration，则绕道被丢弃
    if(it_teb->get()->teb().getSumOfAllTimeDiffs() / best_plan_duration > cfg_->hcp.max_ratio_detours_duration_best_duration)
    {
      ROS_DEBUG("Removing a teb because it's duration is much longer than that of the best teb");
      it_teb = removeTeb(*it_teb);
      continue;
    }
    ++it_teb;
  }
}
// plan就是轨迹优化器中的一个轨迹优化器
// len_orientation_vector用于计算从其实点到另外一个点的最长长度
// orientation表示要进行赋值的
// 这个函数的作用就是找到距离起始点len_orientation_vector远的点与起始点的atan2角是多少
// 如果找到就返回真，找不到就返回假 
bool HomotopyClassPlanner::computeStartOrientation(const TebOptimalPlannerPtr plan, const double len_orientation_vector,
  double& orientation)
{
  VertexPose start_pose = plan->teb().Pose(0);
  bool second_pose_found = false;
  Eigen::Vector2d start_vector;
  for(auto& pose : plan->teb().poses())
  {
    start_vector = start_pose.position() - pose->position();
    if(start_vector.norm() > len_orientation_vector)
    {
      second_pose_found = true;
      break;
    }
  }
  if(!second_pose_found)  // The current plan is too short to make assumptions on the start orientation
    return false;
  orientation = std::atan2(start_vector[1], start_vector[0]);
  return true;
}


} // end namespace
