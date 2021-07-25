#include <teb_local_planner/timed_elastic_band.h>


namespace teb_local_planner
{

// 初始化
TimedElasticBand::TimedElasticBand()
{		
}

TimedElasticBand::~TimedElasticBand()
{
  ROS_DEBUG("Destructor Timed_Elastic_Band...");
  clearTimedElasticBand();
}
// 将位置压到pose_vec_
// 默认fixed为false,在轨迹优化过程中标记固定或不固定的姿态(对TebOptimalPlanner很重要)
void TimedElasticBand::addPose(const PoseSE2& pose, bool fixed)
{
  VertexPose* pose_vertex = new VertexPose(pose, fixed);
  pose_vec_.push_back( pose_vertex );
  return;
}

void TimedElasticBand::addPose(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, bool fixed)
{
  VertexPose* pose_vertex = new VertexPose(position, theta, fixed);
  pose_vec_.push_back( pose_vertex );
  return;
}

 void TimedElasticBand::addPose(double x, double y, double theta, bool fixed)
{
  VertexPose* pose_vertex = new VertexPose(x, y, theta, fixed);
  pose_vec_.push_back( pose_vertex );
  return;
}
// 把当前这个点到终点的最短时间放进去
void TimedElasticBand::addTimeDiff(double dt, bool fixed)
{
  VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt, fixed);
  timediff_vec_.push_back( timediff_vertex );
  return;
}


void TimedElasticBand::addPoseAndTimeDiff(double x, double y, double angle, double dt)
{
  if (sizePoses() != sizeTimeDiffs())
  {
    addPose(x,y,angle,false);
    addTimeDiff(dt,false);
  }
  else 
    ROS_ERROR("Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
  return;
}


// 把当前点和当前点到终点的最短时间分别放到pose_vec_和timediff_vec_
void TimedElasticBand::addPoseAndTimeDiff(const PoseSE2& pose, double dt)
{
  // 所以正常情况下,两个长度是不相等的
  // 因为多一个起点的位姿
  if (sizePoses() != sizeTimeDiffs())
  {
    // 将位置压到pose_vec_
    // 默认fixed为false,在轨迹优化过程中标记固定或不固定的姿态(对TebOptimalPlanner很重要)
    addPose(pose,false);
    // 把当前这个点到终点的最短时间放进去
    addTimeDiff(dt,false);
  } else
    // 方法addPoseAndTimeDiff:先添加一个姿势。Timediff描述了上次配置文件和给定配置文件之间的时间差
    ROS_ERROR("Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
  return;
}

void TimedElasticBand::addPoseAndTimeDiff(const Eigen::Ref<const Eigen::Vector2d>& position, double theta, double dt)
{
  if (sizePoses() != sizeTimeDiffs())
  {
    addPose(position, theta,false);
    addTimeDiff(dt,false);
  } else 
    ROS_ERROR("Method addPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf");
  return;
}


void TimedElasticBand::deletePose(int index)
{
  ROS_ASSERT(index<pose_vec_.size());
  delete pose_vec_.at(index);
  pose_vec_.erase(pose_vec_.begin()+index);
}
// 从1开始删除,把起点位姿保存
void TimedElasticBand::deletePoses(int index, int number)
{
  ROS_ASSERT(index+number<=(int)pose_vec_.size());
  for (int i = index; i<index+number; ++i)
    delete pose_vec_.at(i);
  // 这个是从1开始删除
  pose_vec_.erase(pose_vec_.begin()+index, pose_vec_.begin()+index+number);
}
// 从0开始删除
void TimedElasticBand::deleteTimeDiff(int index)
{
  ROS_ASSERT(index<(int)timediff_vec_.size());
  delete timediff_vec_.at(index);
  // 这个是从0开始删除
  timediff_vec_.erase(timediff_vec_.begin()+index);
}

void TimedElasticBand::deleteTimeDiffs(int index, int number)
{
  ROS_ASSERT(index+number<=timediff_vec_.size());
  for (int i = index; i<index+number; ++i)
    delete timediff_vec_.at(i);
  timediff_vec_.erase(timediff_vec_.begin()+index, timediff_vec_.begin()+index+number);
}

void TimedElasticBand::insertPose(int index, const PoseSE2& pose)
{
  VertexPose* pose_vertex = new VertexPose(pose);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}
// 在index前面插入一个值
void TimedElasticBand::insertPose(int index, const Eigen::Ref<const Eigen::Vector2d>& position, double theta)
{
  VertexPose* pose_vertex = new VertexPose(position, theta);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}

void TimedElasticBand::insertPose(int index, double x, double y, double theta)
{
  VertexPose* pose_vertex = new VertexPose(x, y, theta);
  pose_vec_.insert(pose_vec_.begin()+index, pose_vertex);
}

void TimedElasticBand::insertTimeDiff(int index, double dt)
{
  VertexTimeDiff* timediff_vertex = new VertexTimeDiff(dt);
  timediff_vec_.insert(timediff_vec_.begin()+index, timediff_vertex);
}

// 清空节点
void TimedElasticBand::clearTimedElasticBand()
{
  for (PoseSequence::iterator pose_it = pose_vec_.begin(); pose_it != pose_vec_.end(); ++pose_it)
    delete *pose_it;
  pose_vec_.clear();
  
  for (TimeDiffSequence::iterator dt_it = timediff_vec_.begin(); dt_it != timediff_vec_.end(); ++dt_it)
    delete *dt_it;
  timediff_vec_.clear();
}


void TimedElasticBand::setPoseVertexFixed(int index, bool status)
{
  ROS_ASSERT(index<sizePoses());
  //pose_vec_就是存储可优化姿态顶点序列的内部容器
  //status就是这个是固定点还是灵活的点
  pose_vec_.at(index)->setFixed(status);   
}

void TimedElasticBand::setTimeDiffVertexFixed(int index, bool status)
{
  ROS_ASSERT(index<sizeTimeDiffs());
  timediff_vec_.at(index)->setFixed(status);
}
// dt_ref规划轨迹的时间分辨率,运行过程中会根据实际情况调整。局部路径规划的解析度。该值越小,运动越缓慢。默认 0.3。
// dt_hysteresis根据当前时间分辨率(dt)自动调整大小的滞后:通常为dt_ref的10%
// 根据时间分辨率调整轨迹。能够保证最终的整个teb路径两个点之间的时间差都在指定的dt_ref范围内。
void TimedElasticBand::autoResize(double dt_ref, double dt_hysteresis, int min_samples, int max_samples, bool fast_mode)
{  
  // 判断是否合法
  ROS_ASSERT(sizeTimeDiffs() == 0 || sizeTimeDiffs() + 1 == sizePoses());
  /// iterate through all TEB states and add/remove states!
  bool modified = true;
  // 实际上，它应该是while()，但我们想确保不被一些振荡卡住，因此Max 100重复。
  // 正如注释中所说本来需要不断的对teb路径进行autoresize的，但是为了避免陷入死循环这里只进行100次的调整。  
  // 整体思路：当两个位姿之间的时间差大于dt_ref时就在两个位姿之间插入一个点，插入一个timediff，此时两个点之间的  
  // 时间差肯定比dt_ref要小了，第二次进行调整的时候就会在此之间删除一个点，这时两个点之间的时间差可能就会变大，  
  // 第三次进来再插入一个点，两个点之间的距离可能就满足要求了。  
  // 如此往复的增加和删除，能够保证最终的整个teb路径两个点之间的时间差都在指定的dt_ref范围内。
  for (int rep = 0; rep < 100 && modified; ++rep) // actually it should be while(), but we want to make sure to not get stuck in some oscillation, hence max 100 repitions.
  {
    modified = false;

    for(int i=0; i < sizeTimeDiffs(); ++i) // TimeDiff connects Point(i) with Point(i+1)
    {
      // sizeTimeDiffs表示还可以往里面加
      // 注意TimeDiff(i)是存储的是Pose(i),Pose(i+1)之间的最短时间
      if(TimeDiff(i) > dt_ref + dt_hysteresis && sizeTimeDiffs()<max_samples)
      {
        //ROS_DEBUG("teb_local_planner: autoResize() inserting new bandpoint i=%u, #TimeDiffs=%lu",i,sizeTimeDiffs());

        double newtime = 0.5*TimeDiff(i);
        // 中间插一个点代表需要加一段时间
        TimeDiff(i) = newtime;
        // 在index前面插入一个值
        insertPose(i+1, PoseSE2::average(Pose(i),Pose(i+1)) );
        insertTimeDiff(i+1,newtime);

        modified = true;
      }
      // 只有当size大于min_samples时才删除样本。sizeTimeDiffs()表示还是可以删除的
      else if(TimeDiff(i) < dt_ref - dt_hysteresis && sizeTimeDiffs()>min_samples) // only remove samples if size is larger than min_samples.
      {
        //ROS_DEBUG("teb_local_planner: autoResize() deleting bandpoint i=%u, #TimeDiffs=%lu",i,sizeTimeDiffs());

        if(i < ((int)sizeTimeDiffs()-1))
        {
          TimeDiff(i+1) = TimeDiff(i+1) + TimeDiff(i);
          deleteTimeDiff(i);
          deletePose(i+1);
        }
        else
        { // last motion should be adjusted, shift time to the interval before
          TimeDiff(i-1) += TimeDiff(i);
          deleteTimeDiff(i);
          deletePose(i);
        }

        modified = true;
      }
    }
    if (fast_mode) break;
  }
}

// 求的轨迹点中所有时间的和
double TimedElasticBand::getSumOfAllTimeDiffs() const
{
  double time = 0;

  for(TimeDiffSequence::const_iterator dt_it = timediff_vec_.begin(); dt_it != timediff_vec_.end(); ++dt_it)
  {
      time += (*dt_it)->dt();
  }
  return time;
}

double TimedElasticBand::getSumOfTimeDiffsUpToIdx(int index) const
{
  ROS_ASSERT(index<=timediff_vec_.size());

  double time = 0;

  for(int i = 0; i < index; ++i)
  {
    time += timediff_vec_.at(i)->dt();
  }

  return time;
}

double TimedElasticBand::getAccumulatedDistance() const
{
  double dist = 0;

  for(int i=1; i<sizePoses(); ++i)
  {
      dist += (Pose(i).position() - Pose(i-1).position()).norm();
  }
  return dist;
}


  /* teb_.initTrajectoryToGoal(initial_plan, 
                              cfg_->robot.max_vel_x, 
                              cfg_->trajectory.global_plan_overwrite_orientation, 
                              cfg_->trajectory.min_samples, 
                              cfg_->trajectory.allow_init_with_backwards_motion);
  */
//添加了n个位姿点和n-1个时间点
bool TimedElasticBand::initTrajectoryToGoal(const PoseSE2& start, const PoseSE2& goal, double diststep, double max_vel_x, int min_samples, bool guess_backwards_motion)
{
  if (!isInit())
  {   
    addPose(start); // add starting point
    //是一个固定的约束，true表示固定的点
    setPoseVertexFixed(0,true); // StartConf is a fixed constraint during optimization


    double timestep = 0.1;
    //diststep 两个连续位姿之间的欧几里得距离(如果o，即使样本最小也不插入中间样本)
    if (diststep!=0)
    {
      Eigen::Vector2d point_to_goal = goal.position()-start.position();
      double dir_to_goal = std::atan2(point_to_goal[1],point_to_goal[0]); // direction to goal
      double dx = diststep*std::cos(dir_to_goal);
      double dy = diststep*std::sin(dir_to_goal);
      double orient_init = dir_to_goal;
      // check if the goal is behind the start pose (w.r.t. start orientation)
      if (guess_backwards_motion && point_to_goal.dot(start.orientationUnitVec()) < 0) 
        orient_init = g2o::normalize_theta(orient_init+M_PI);
      // TODO: timestep ~ max_vel_x_backwards for backwards motions
      
      double dist_to_goal = point_to_goal.norm();
      //no_steps_d这个就是将start与goal之间分多少段
      double no_steps_d = dist_to_goal/std::abs(diststep); // ignore negative values
      //把他转化成int类型，进行下面的循环
      unsigned int no_steps = (unsigned int) std::floor(no_steps_d);

      if (max_vel_x > 0) timestep = diststep / max_vel_x;
      
      for (unsigned int i=1; i<=no_steps; i++) // start with 1! starting point had index 0
      {
        if (i==no_steps && no_steps_d==(float) no_steps) 
            break; // if last conf (depending on stepsize) is equal to goal conf -> leave loop
        //这些位姿和时间都是灵活的点
        addPoseAndTimeDiff(start.x()+i*dx,start.y()+i*dy,orient_init,timestep);
      }

    }
    
    // if number of samples is not larger than min_samples, insert manually
    //如果样本数量不大于最小样本，手动插入
    if ( sizePoses() < min_samples-1 )
    {
      ROS_DEBUG("initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...");
      while (sizePoses() < min_samples-1) // subtract goal point that will be added later
      {
        // simple strategy: interpolate between the current pose and the goal
        PoseSE2 intermediate_pose = PoseSE2::average(BackPose(), goal);
        if (max_vel_x > 0) timestep = (intermediate_pose.position()-BackPose().position()).norm()/max_vel_x;
        addPoseAndTimeDiff( intermediate_pose, timestep ); // let the optimier correct the timestep (TODO: better initialization
      }
    }
    
    // add goal
    if (max_vel_x > 0) timestep = (goal.position()-BackPose().position()).norm()/max_vel_x;
    //将固定点的位姿和时间加进去
    addPoseAndTimeDiff(goal,timestep); // add goal point

    setPoseVertexFixed(sizePoses()-1,true); // GoalConf is a fixed constraint during optimization	
  }
  else // size!=0
  {
    ROS_WARN("Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
    ROS_WARN("Number of TEB configurations: %d, Number of TEB timediffs: %d",(unsigned int) sizePoses(),(unsigned int) sizeTimeDiffs());
    return false;
  }
  return true;
}

/*initTrajectoryToGoal(initial_plan, 
                        cfg_->robot.max_vel_x, 
                        cfg_->trajectory.global_plan_overwrite_orientation, 
                        cfg_->trajectory.min_samples, 
                        cfg_->trajectory.allow_init_with_backwards_motion);*/
    // max_vel_x机器人允许的最大的速度
    // estimate_orient覆盖全局规划器提供的局部子目标的方向(因为它们通常只提供一个 2D 路径),默认 true。
    // 如果为真 那么根据当前的目标点和未来的目标点,就算出合适的目标角度
    // min_samples最小样本数（始终大于2）
    // guess_backwards_motion如果为真，底层轨迹可能会被初始化为反向运动，以防目标在局部costmap中的起始位置之后(只有当机器人配备了后方传感器时，才建议这样做)
// 函数的作用就是将局部路径规划的起点放进pose_vec_,然后将后面的点轮着放到pose_vec_,并且将后面的点到局部路径规划的上一个点的行驶的最短时间也轮着放进timediff_vec_
bool TimedElasticBand::initTrajectoryToGoal(const std::vector<geometry_msgs::PoseStamped>& plan, double max_vel_x, bool estimate_orient, int min_samples, bool guess_backwards_motion)
{
  // 每一次初始化都会进入,就是每一次更新局部路径
  if (!isInit())
  {
    //设置局部路径的开始和目标
    PoseSE2 start(plan.front().pose);
    PoseSE2 goal(plan.back().pose);

    double dt = 0.1;
    // 将位置压到pose_vec_
    // 默认fixed为false,在轨迹优化过程中标记固定或不固定的姿态(对TebOptimalPlanner很重要)
    addPose(start); // add starting point with given orientation
    // 设置为固定点
    setPoseVertexFixed(0,true); // StartConf is a fixed constraint during optimization
    // 就是判断机器人是否允许后退
    bool backwards = false;
    // check if the goal is behind the start pose (w.r.t. start orientation)
    //guess_backwards_motion 如果目标航向指向机器人的后面（就是允许后退），允许初始化向后定向的轨迹
    if (guess_backwards_motion && (goal.position()-start.position()).dot(start.orientationUnitVec()) < 0) 
        backwards = true;
    // TODO: dt ~ max_vel_x_backwards for backwards motions
    // 从第二个点开始往里面加第一个点不用计算时间
    for (int i=1; i<(int)plan.size()-1; ++i)
    {
        double yaw;
        if (estimate_orient)
        {
            // get yaw from the orientation of the distance vector between pose_{i+1} and pose_{i}
            double dx = plan[i+1].pose.position.x - plan[i].pose.position.x;
            double dy = plan[i+1].pose.position.y - plan[i].pose.position.y;
            yaw = std::atan2(dy,dx);
            if (backwards)
                yaw = g2o::normalize_theta(yaw+M_PI);
        }
        else 
        {
            yaw = tf::getYaw(plan[i].pose.orientation);
        }
        // 中间的点
        PoseSE2 intermediate_pose(plan[i].pose.position.x, plan[i].pose.position.y, yaw);
        // 如果有最大速度,那么就是最大速度行驶的情况下,到当前点的距离pose_vec_最后一个点的最段时间是dt
        if (max_vel_x > 0) dt = (intermediate_pose.position()-BackPose().position()).norm()/max_vel_x;
        //这个就是将中间点加进去
        // 把中间点和中间点到上一个点的最短时间分别放到pose_vec_和timediff_vec_
        addPoseAndTimeDiff(intermediate_pose, dt);
    }
    
    // if number of samples is not larger than min_samples, insert manually
    //如果样本数量不大于最小样本，手动插入
    // 这个先不用考虑
    if ( sizePoses() < min_samples-1 )
    {
      ROS_DEBUG("initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...");
      while (sizePoses() < min_samples-1) // subtract goal point that will be added later
      {
        // simple strategy: interpolate between the current pose and the goal
        PoseSE2 intermediate_pose = PoseSE2::average(BackPose(), goal);
        if (max_vel_x > 0) dt = (intermediate_pose.position()-BackPose().position()).norm()/max_vel_x;
        addPoseAndTimeDiff( intermediate_pose, dt ); // let the optimier correct the timestep (TODO: better initialization
      }
    }
    
    // Now add final state with given orientation
    if (max_vel_x > 0) dt = (goal.position()-BackPose().position()).norm()/max_vel_x;
    //把最后的目标点加进去
    addPoseAndTimeDiff(goal, dt);
    // 把最后的目标点也设置为固定
    setPoseVertexFixed(sizePoses()-1,true); // GoalConf is a fixed constraint during optimization
  }
  else // size!=0
  {
    ROS_WARN("Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!");
    ROS_WARN("Number of TEB configurations: %d, Number of TEB timediffs: %d", sizePoses(), sizeTimeDiffs());
    return false;
  }
  
  return true;
}

// 找到轨道上离给定参考点最近的点的距离,并且返回这个点的索引
int TimedElasticBand::findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_point, double* distance, int begin_idx) const
{
  std::vector<double> dist_vec; // TODO: improve! efficiency
  // 开辟路径点长度的空间
  dist_vec.reserve(sizePoses());
  
  int n = sizePoses();
  
  // calc distances
  // 从begin_idx开始搜索
  for (int i = begin_idx; i < n; i++)
  {
    Eigen::Vector2d diff = ref_point - Pose(i).position();
    dist_vec.push_back(diff.norm());
  }
  
  if (dist_vec.empty())
    return -1;
  
  // find minimum
  int index_min = 0;
  // 寻找最近的点
  double last_value = dist_vec.at(0);
  for (int i=1; i < (int)dist_vec.size(); i++)
  {
    if (dist_vec.at(i) < last_value)
    {
      last_value = dist_vec.at(i);
      index_min = i;
    }
  }
  if (distance)
    *distance = last_value;
  return begin_idx+index_min;
}


int TimedElasticBand::findClosestTrajectoryPose(const Eigen::Ref<const Eigen::Vector2d>& ref_line_start, const Eigen::Ref<const Eigen::Vector2d>& ref_line_end, double* distance) const
{
  std::vector<double> dist_vec; // TODO: improve! efficiency
  dist_vec.reserve(sizePoses());

  int n = sizePoses();
  
  // calc distances  
  for (int i = 0; i < n; i++)
  {
    Eigen::Vector2d point = Pose(i).position();
    double diff = distance_point_to_segment_2d(point, ref_line_start, ref_line_end);
    dist_vec.push_back(diff);
  }
  
  if (dist_vec.empty())
    return -1;
  
  // find minimum
  int index_min = 0;

  double last_value = dist_vec.at(0);
  for (int i=1; i < (int)dist_vec.size(); i++)
  {
    if (dist_vec.at(i) < last_value)
    {
      last_value = dist_vec.at(i);
      index_min = i;
    }
  }
  if (distance)
    *distance = last_value;
  return index_min; // return index, because it's equal to the vertex, which represents this bandpoint
}

int TimedElasticBand::findClosestTrajectoryPose(const Point2dContainer& vertices, double* distance) const
{
  if (vertices.empty())
    return 0;
  else if (vertices.size() == 1)
    return findClosestTrajectoryPose(vertices.front());
  else if (vertices.size() == 2)
    return findClosestTrajectoryPose(vertices.front(), vertices.back());
  
  std::vector<double> dist_vec; // TODO: improve! efficiency
  dist_vec.reserve(sizePoses());
  
  int n = sizePoses();
  
  // calc distances  
  for (int i = 0; i < n; i++)
  {
    Eigen::Vector2d point = Pose(i).position();
    double diff = HUGE_VAL;
    for (int j = 0; j < (int) vertices.size()-1; ++j)
    {
       diff = std::min(diff, distance_point_to_segment_2d(point, vertices[j], vertices[j+1]));
    }
    diff = std::min(diff, distance_point_to_segment_2d(point, vertices.back(), vertices.front()));
    dist_vec.push_back(diff);
  }

  if (dist_vec.empty())
    return -1;

  // find minimum
  int index_min = 0;

  double last_value = dist_vec.at(0);
  for (int i=1; i < (int)dist_vec.size(); i++)
  {
    if (dist_vec.at(i) < last_value)
    {
      last_value = dist_vec.at(i);
      index_min = i;
    }
  }
  if (distance)
    *distance = last_value;
  return index_min; // return index, because it's equal to the vertex, which represents this bandpoint
}


int TimedElasticBand::findClosestTrajectoryPose(const Obstacle& obstacle, double* distance) const
{
  const PointObstacle* pobst = dynamic_cast<const PointObstacle*>(&obstacle);
  if (pobst)
    return findClosestTrajectoryPose(pobst->position(), distance);
  
  const LineObstacle* lobst = dynamic_cast<const LineObstacle*>(&obstacle);
  if (lobst)
    return findClosestTrajectoryPose(lobst->start(), lobst->end(), distance);
  
  const PolygonObstacle* polyobst = dynamic_cast<const PolygonObstacle*>(&obstacle);
  if (polyobst)
    return findClosestTrajectoryPose(polyobst->vertices(), distance);
  
  return findClosestTrajectoryPose(obstacle.getCentroid(), distance);  
}

  /*teb_.updateAndPruneTEB(start, 
                           goal, 
                           cfg_->trajectory.min_samples);*/
// 这个函数就是找到离new_start最近的点(最多找到十个点)
// 找到后删除大于的点,并且将new_start和new_goal作为新的起点和终点
void TimedElasticBand::updateAndPruneTEB(boost::optional<const PoseSE2&> new_start, boost::optional<const PoseSE2&> new_goal, int min_samples)
{
  // first and simple approach: change only start confs (and virtual start conf for inital velocity)
  // TEST if optimizer can handle this "hard" placement

  if (new_start && sizePoses()>0)
  {    
    // find nearest state (using l2-norm) in order to prune the trajectory
    // 找到最近的状态(使用l2-norm)，以修剪轨迹
    // (remove already passed states)
    // dist_cache表示新的起点(这个新的起点就是离机器人最近的点)和上一个起点的差值
    double dist_cache = (new_start->position()- Pose(0).position()).norm();
    double dist;
    // 要向前看多少个点
    int lookahead = std::min<int>( sizePoses()-min_samples, 10); // satisfy min_samples, otherwise max 10 samples
    // 就是离机器人最近的点
    int nearest_idx = 0;
    // 就这样一直循环知道找到机器人最近的点,注意最多看10个点或者更少
    for (int i = 1; i<=lookahead; ++i)
    {
      dist = (new_start->position()- Pose(i).position()).norm();
      // 这个便是机器人在第一个点和第二个点中间
      if (dist<dist_cache)
      {
        dist_cache = dist;
        nearest_idx = i;
      }
      else break;
    }
    
    // prune trajectory at the beginning (and extrapolate sequences at the end if the horizon is fixed)
    // 在开始时修剪轨迹(如果域是固定的，在结束时外推序列)
    if (nearest_idx>0)
    {
      // nearest_idx is equal to the number of samples to be removed (since it counts from 0 ;-) )
      // nearest_idx等于要移除的样本数目(因为它从0计数0)
      // WARNING delete starting at pose 1, and overwrite the original pose(0) with new_start, since Pose(0) is fixed during optimization!
      //警告删除pose1开始，并用new_start覆盖原来的姿势(0)，因为姿势(0)在优化期间是固定的!
      // 从1开始删除,把起点位姿保存
      deletePoses(1, nearest_idx);  // delete first states such that the closest state is the new first one
      // 从0开始删除
      deleteTimeDiffs(1, nearest_idx); // delete corresponding time differences
    }
    
    // update start
    // 更换pose_vec_中的值
    Pose(0) = *new_start;
  }
  
  if (new_goal && sizePoses()>0)
  {
    // 更换pose_vec_末端中的值
    BackPose() = *new_goal;
  }
};


bool TimedElasticBand::isTrajectoryInsideRegion(double radius, double max_dist_behind_robot, int skip_poses)
{
    if (sizePoses()<=0)
        return true;
    
    double radius_sq = radius*radius;
    double max_dist_behind_robot_sq = max_dist_behind_robot*max_dist_behind_robot;
    Eigen::Vector2d robot_orient = Pose(0).orientationUnitVec();
    
    for (int i=1; i<sizePoses(); i=i+skip_poses+1)
    {
        Eigen::Vector2d dist_vec = Pose(i).position()-Pose(0).position();
        double dist_sq = dist_vec.squaredNorm();
        
        if (dist_sq > radius_sq)
        {
            ROS_INFO("outside robot");
            return false;
        }
        
        // check behind the robot with a different distance, if specified (or >=0)
        if (max_dist_behind_robot >= 0 && dist_vec.dot(robot_orient) < 0 && dist_sq > max_dist_behind_robot_sq)
        {
            ROS_INFO("outside robot behind");
            return false;
        }
        
    }
    return true;
}




} // namespace teb_local_planner
