// 判断机器人是否在震荡
#ifndef RECOVERY_BEHAVIORS_H__
#define RECOVERY_BEHAVIORS_H__


#include <boost/circular_buffer.hpp>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace teb_local_planner
{


/**
 * @class FailureDetector
 * @brief This class implements methods in order to detect if the robot got stucked or is oscillating
 * 
 * The StuckDetector analyzes the last N commanded velocities in order to detect whether the robot
 * might got stucked or oscillates between left/right/forward/backwards motions.
 */  
class FailureDetector
{
public:

  /**
   * @brief Default constructor
   */
  FailureDetector() {}  
  
  /**
   * @brief destructor.
   */
  ~FailureDetector() {}
  
  /**
   * @brief Set buffer length (measurement history)
   * @param length number of measurements to be kept
   */
  void setBufferLength(int length) {buffer_.set_capacity(length);}
  
  /**
   * @brief Add a new twist measurement to the internal buffer and compute a new decision
   * @param twist geometry_msgs::Twist velocity information
   * @param v_max maximum forward translational velocity
   * @param v_backwards_max maximum backward translational velocity
   * @param omega_max maximum angular velocity 
   * @param v_eps Threshold for the average normalized linear velocity in (0,1) that must not be exceded (e.g. 0.1)
   * @param omega_eps Threshold for the average normalized angular velocity in (0,1) that must not be exceded (e.g. 0.1)
   */
  void update(const geometry_msgs::Twist& twist, double v_max, double v_backwards_max, double omega_max, double v_eps, double omega_eps);
  
  /**
   * @brief Check if the robot got stucked
   * 
   * This call does not compute the actual decision,
   * since it is computed within each update() invocation.
   * @return true if the robot got stucked, false otherwise.
   */
  bool isOscillating() const;
  
  /**
   * @brief Clear the current internal state
   * 
   * This call also resets the internal buffer
   */
  void clear();
       
protected:
    
    /** Variables to be monitored */
    struct VelMeasurement
    {
        double v = 0;
        double omega = 0; //ω
    };
    
    /**
     * @brief Detect if the robot got stucked based on the current buffer content
     * 
     * Afterwards the status might be checked using gotStucked();
     * @param v_eps Threshold for the average normalized linear velocity in (0,1) that must not be exceded (e.g. 0.1)
     * @param omega_eps Threshold for the average normalized angular velocity in (0,1) that must not be exceded (e.g. 0.1)
     * @return true if the robot got stucked, false otherwise
     */
    bool detect(double v_eps, double omega_eps);
  
private:
    // 其capcity是固定的，不像标准容器中vector或list，他们的capcity是会根据策略动态增长的。当容量满了以后，插入一个元素时，会在容器的开头或结尾处删除一个元素。至于是头部还是尾部取决已加入的位置。
    // 大小为oscillation_filter_duration*controller_frequency
    boost::circular_buffer<VelMeasurement> buffer_; //!< Circular buffer to store the last measurements @see setBufferLength
    bool oscillating_ = false; //!< Current state: true if robot is oscillating
                
};


} // namespace teb_local_planner

#endif /* RECOVERY_BEHAVIORS_H__ */
