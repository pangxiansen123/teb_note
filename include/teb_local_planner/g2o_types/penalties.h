// 惩罚函数
#ifndef PENALTIES_H
#define PENALTIES_H

#include <cmath>
#include <Eigen/Core>
#include <g2o/stuff/misc.h>

namespace teb_local_planner
{

/**
 * 超过速度范围(加上额外的安全裕度),就会惩罚
 * @brief Linear penalty function for bounding \c var to the interval \f$ -a < var < a \f$
 * @param var The scalar that should be bounded
 * @param a lower and upper absolute bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see penaltyBoundToIntervalDerivative
 * @return Penalty / cost value that is nonzero if the constraint is not satisfied
 */
inline double penaltyBoundToInterval(const double& var,const double& a,const double& epsilon)
{
  if (var < -a+epsilon)
  {
    return (-var - (a - epsilon));
  }
  if (var <= a-epsilon)
  {
    return 0.;
  }
  else
  {
    return (var - (a - epsilon));
  }
}

/**
 * 超过速度范围(加上额外的安全裕度),就会惩罚
 * @brief  Linear penalty function for bounding var to the interval  a < var < b
 * @param var The scalar that should be bounded
 * @param a lower bound
 * @param b upper bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see penaltyBoundToIntervalDerivative
 * @return Penalty / cost value that is nonzero if the constraint is not satisfied
 */
inline double penaltyBoundToInterval(const double& var,const double& a, const double& b, const double& epsilon)
{
  // 超出范围
  if (var < a+epsilon)
  {
    return (-var + (a + epsilon));
  }
  // 正向判断是否超过范围
  if (var <= b-epsilon)
  {
    return 0.;
  }
  else
  {
    return (var - (b - epsilon));
  }
}


/**
 * 表示一个线性惩罚:当var大于a+epsilon,不惩罚,当小于的时候线性惩罚
 * @brief Linear penalty function for bounding \c var from below: \f$ a < var \f$
 * @param var The scalar that should be bounded
 * @param a lower bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see penaltyBoundFromBelowDerivative
 * @return Penalty / cost value that is nonzero if the constraint is not satisfied
 */
inline double penaltyBoundFromBelow(const double& var, const double& a,const double& epsilon)
{
  if (var >= a+epsilon)
  {
    return 0.;
  }
  else
  {
    return (-var + (a+epsilon));
  }
}

/**
 * @brief Derivative of the linear penalty function for bounding \c var to the interval \f$ -a < var < a \f$
 * @param var The scalar that should be bounded
 * @param a lower and upper absolute bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see penaltyBoundToInterval
 * @return Derivative of the penalty function w.r.t. \c var
 */
inline double penaltyBoundToIntervalDerivative(const double& var,const double& a, const double& epsilon)
{
  if (var < -a+epsilon)
  {
    return -1;
  }
  if (var <= a-epsilon)
  {
    return 0.;
  }
  else
  {
    return 1;		
  }
}

/**
 * @brief Derivative of the linear penalty function for bounding \c var to the interval \f$ a < var < b \f$
 * @param var The scalar that should be bounded
 * @param a lower bound
 * @param b upper bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see penaltyBoundToInterval
 * @return Derivative of the penalty function w.r.t. \c var
 */
inline double penaltyBoundToIntervalDerivative(const double& var,const double& a, const double& b, const double& epsilon)
{
  if (var < a+epsilon)
  {
    return -1;
  }
  if (var <= b-epsilon)
  {
    return 0.;
  }
  else
  {
    return 1;		
  }
}


/**
 * @brief Derivative of the linear penalty function for bounding \c var from below: \f$ a < var \f$
 * @param var The scalar that should be bounded
 * @param a lower bound
 * @param epsilon safty margin (move bound to the interior of the interval)
 * @see penaltyBoundFromBelow
 * @return Derivative of the penalty function w.r.t. \c var
 */
inline double penaltyBoundFromBelowDerivative(const double& var, const double& a,const double& epsilon)
{
  if (var >= a+epsilon)
  {
    return 0.;
  }
  else
  {
    return -1;
  }
}


} // namespace teb_local_planner


#endif // PENALTIES_H
