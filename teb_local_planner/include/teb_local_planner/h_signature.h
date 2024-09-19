/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Christoph Rösmann, Franz Albers
 *********************************************************************/

#ifndef H_SIGNATURE_H_
#define H_SIGNATURE_H_

#include <teb_local_planner/equivalence_relations.h>
#include <teb_local_planner/misc.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/timed_elastic_band.h>

#include <ros/ros.h>
#include <math.h>
#include <algorithm>
#include <functional>
#include <vector>
#include <iterator>


namespace teb_local_planner
{

/**
 * @brief The H-signature defines an equivalence relation based on homology in terms of complex calculus.
 *
 * The H-Signature depends on the obstacle configuration and can be utilized
 * to check whether two trajectores belong to the same homology class.
 * Refer to: \n
 * 	- S. Bhattacharya et al.: Search-based Path Planning with Homotopy Class Constraints, AAAI, 2010
 */
class HSignature : public EquivalenceClass
{

public:

    /**
    * @brief Constructor accepting a TebConfig
    * @param cfg TebConfig storing some user configuration options
    */
    HSignature(const TebConfig& cfg) : cfg_(&cfg) {}


   /**
    * @brief Calculate the H-Signature of a path
    *
    * The implemented function accepts generic path descriptions that are restricted to the following structure: \n
    * The path is composed of points T and is represented by a std::vector< T > or similar type (std::list, std::deque, ...). \n
    * Provide a unary function with the following signature <c> std::complex< long double > (const T& point_type) </c>
    * that returns a complex value for the position (Re(*)=x, Im(*)=y).
     *
    * T could also be a pointer type, if the passed function also accepts a const T* point_Type.
    *
    * @param path_start Iterator to the first element in the path
    * @param path_end Iterator to the last element in the path
    * @param obstacles obstacle container
    * @param fun_cplx_point function accepting the dereference iterator type and that returns the position as complex number.
    * @tparam BidirIter Bidirectional iterator type
    * @tparam Fun function of the form std::complex< long double > (const T& point_type)
    */
    template<typename BidirIter, typename Fun>
    void calculateHSignature(BidirIter path_start, BidirIter path_end, Fun fun_cplx_point, const ObstContainer* obstacles)
    {
        if (obstacles->empty())
        {
            hsignature_ = std::complex<double>(0,0);
            return;
        }


        ROS_ASSERT_MSG(cfg_->hcp.h_signature_prescaler>0.1 && cfg_->hcp.h_signature_prescaler<=1, "Only a prescaler on the interval (0.1,1] ist allowed.");

        // guess values for f0
        // paper proposes a+b=N-1 && |a-b|<=1, 1...N obstacles
        int m = std::max( (int)obstacles->size()-1, 5 );  // for only a few obstacles we need a min threshold in order to get significantly high H-Signatures

        int a = (int) std::ceil(double(m)/2.0);
        int b = m-a;

        std::advance(path_end, -1); // reduce path_end by 1 (since we check line segments between those path points

        typedef std::complex<long double> cplx;
        // guess map size (only a really really coarse guess is required
        // use distance from start to goal as distance to each direction
        // TODO: one could move the map determination outside this function, since it remains constant for the whole planning interval
        cplx start = fun_cplx_point(*path_start);
        cplx end = fun_cplx_point(*path_end); // path_end points to the last point now after calling std::advance before
        cplx delta = end-start;
        cplx normal(-delta.imag(), delta.real());
        cplx map_bottom_left;
        cplx map_top_right;
        if (std::abs(delta) < 3.0)
        { // set minimum bound on distance (we do not want to have numerical instabilities) and 3.0 performs fine...
            map_bottom_left = start + cplx(0, -3);
            map_top_right = start + cplx(3, 3);
        }
        else
        {
            map_bottom_left = start - normal;
            map_top_right = start + delta + normal;
        }

        hsignature_ = 0; // reset local signature

        std::vector<double> imag_proposals(5);

        // iterate path
        while(path_start != path_end)
        {
            cplx z1 = fun_cplx_point(*path_start);
            cplx z2 = fun_cplx_point(*std::next(path_start));

            for (std::size_t l=0; l<obstacles->size(); ++l) // iterate all obstacles
            {
                cplx obst_l = obstacles->at(l)->getCentroidCplx();
                //cplx f0 = (long double) prescaler * std::pow(obst_l-map_bottom_left,a) * std::pow(obst_l-map_top_right,b);
                cplx f0 = (long double) cfg_->hcp.h_signature_prescaler * (long double)a*(obst_l-map_bottom_left) * (long double)b*(obst_l-map_top_right);

                // denum contains product with all obstacles exepct j==l
                cplx Al = f0;
                for (std::size_t j=0; j<obstacles->size(); ++j)
                {
                    if (j==l)
                        continue;
                    cplx obst_j = obstacles->at(j)->getCentroidCplx();
                    cplx diff = obst_l - obst_j;
                    //if (diff.real()!=0 || diff.imag()!=0)
                    if (std::abs(diff)<0.05) // skip really close obstacles
                        continue;
                     else
                        Al /= diff;
                }
                // compute log value
                double diff2 = std::abs(z2-obst_l);
                double diff1 = std::abs(z1-obst_l);
                if (diff2 == 0 || diff1 == 0)
                    continue;
                double log_real = std::log(diff2)-std::log(diff1);
                // complex ln has more than one solution -> choose minimum abs angle -> paper
                double arg_diff = std::arg(z2-obst_l)-std::arg(z1-obst_l);
                imag_proposals.at(0) = arg_diff;
                imag_proposals.at(1) = arg_diff+2*M_PI;
                imag_proposals.at(2) = arg_diff-2*M_PI;
                imag_proposals.at(3) = arg_diff+4*M_PI;
                imag_proposals.at(4) = arg_diff-4*M_PI;
                // NOTE:经过下面这两行的测试发现，H-signature中实际起作用的是imag这个角度差，如果把他置0，则所有的同伦类都是同一个
                // NOTE: 不同轨迹的H-signature的核心差别，其实也是来自于这个min
                double log_imag = *std::min_element(imag_proposals.begin(),imag_proposals.end(),smaller_than_abs);
                // log_real = 0;
                // log_imag = 0;
                cplx log_value(log_real,log_imag);
                //cplx log_value = std::log(z2-obst_l)-std::log(z1-obst_l); // the principal solution doesn't seem to work
                hsignature_ += Al*log_value;
            }
            ++path_start;
        }
    }


   /**
    * @brief Check if two candidate classes are equivalent
    * @param other The other equivalence class to test with
    */
    virtual bool isEqual(const EquivalenceClass& other) const
    {
        const HSignature* hother = dynamic_cast<const HSignature*>(&other); // TODO: better architecture without dynamic_cast
        if (hother)
        {
            double diff_real = std::abs(hother->hsignature_.real() - hsignature_.real());
            double diff_imag = std::abs(hother->hsignature_.imag() - hsignature_.imag());
            if (diff_real<=cfg_->hcp.h_signature_threshold && diff_imag<=cfg_->hcp.h_signature_threshold)
                return true; // Found! Homotopy class already exists, therefore nothing added
        }
        else
            ROS_ERROR("Cannot compare HSignature equivalence classes with types other than HSignature.");

        return false;
    }

   /**
    * @brief Check if the equivalence value is detected correctly
    * @return Returns false, if the equivalence class detection failed, e.g. if nan- or inf values occur.
    */
    virtual bool isValid() const
    {
        return std::isfinite(hsignature_.real()) && std::isfinite(hsignature_.imag());
    }

    /**
     * @brief Check if the trajectory is non-looping around an obstacle.
     * @return Returns always true, as this cannot be detected by this kind of H-Signature.
     */
    virtual bool isReasonable() const
    {
      return true;
    }

    /**
     * @brief Get the current value of the h-signature (read-only)
     * @return h-signature in complex-number format
     */
     const std::complex<long double>& value() const {return hsignature_;}


private:

    const TebConfig* cfg_;
    std::complex<long double> hsignature_;
};





/**
 * @brief The H-signature in three dimensions (here: x-y-t) defines an equivalence relation based on homology using theorems from electro magnetism.
 *
 * The H-Signature depends on the obstacle configuration and can be utilized
 * to check whether two trajectores belong to the same homology class.
 * Refer to: \n
 * 	- S. Bhattacharya et al.: Identification and Representation of Homotopy Classes of Trajectories for Search-based Path Planning in 3D, 2011
 */
class HSignature3d : public EquivalenceClass
{
public:
    /**
    * @brief Constructor accepting a TebConfig
    * @param cfg TebConfig storing some user configuration options
    */
    HSignature3d(const TebConfig& cfg) : cfg_(&cfg) {}


   /**
    * @brief Calculate the H-Signature of a path
    *
    * The implemented function accepts generic path descriptions that are restricted to the following structure: \n
    * The path is composed of points T and is represented by a std::vector< T > or similar type (std::list, std::deque, ...). \n
    * Provide a unary function with the following signature <c> std::complex< long double > (const T& point_type) </c>
    * that returns a complex value for the position (Re(*)=x, Im(*)=y).
    *
    * T could also be a pointer type, if the passed function also accepts a const T* point_Type.
    *
    * @param path_start Iterator to the first element in the path
    * @param path_end Iterator to the last element in the path
    * @param obstacles obstacle container
    * @param fun_cplx_point function accepting the dereference iterator type and that returns the position as complex number.
    * @tparam BidirIter Bidirectional iterator type
    * @tparam Fun function of the form std::complex< long double > (const T& point_type)
    */
    template<typename BidirIter, typename Fun>
    void calculateHSignature(BidirIter path_start, BidirIter path_end, Fun fun_cplx_point, const ObstContainer* obstacles,
                             boost::optional<TimeDiffSequence::iterator> timediff_start, boost::optional<TimeDiffSequence::iterator> timediff_end, bool is_xyt = true)
    {
      // 输入依次是：(路径的起点指针、路径的终点指针、如何解析指针的函数、障碍物指针、时间差序列的起始指针、时间差序列的结束指针)
      // hsignature3d是由于一系列值组成的，每个障碍物一个值
      hsignature3d_.resize(obstacles->size());
      std::advance(path_end, -1); // reduce path_end by 1 (since we check line segments between those path points)

      constexpr int num_int_steps_per_segment = 10;

      for (std::size_t l = 0; l < obstacles->size(); ++l) // iterate all obstacles
      {
        double H = 0;
        double transition_time = 0;
        double next_transition_time = 0;
        // 用下面两个iter去遍历path和timediff
        BidirIter path_iter;
        TimeDiffSequence::iterator timediff_iter;

        // **** 从这里开始 是和障碍物相关的，实际上是 假设障碍物按照当前速度运动，t=120s后，障碍物的xyt差值ds_sq_norm ****
        // s1=(障碍物的当前x、障碍物的当前y，时间t=0)
        Eigen::Vector3d s1 (obstacles->at(l)->getCentroid()(0), obstacles->at(l)->getCentroid()(1), 0);
        double t = 120; // some large value for defining the end point of the obstacle/"conductor" model
        // s2=(障碍物的当前x、障碍物的当前y，时间t=120) （这里做过修改啦，初版本应该是t时刻的xy）
        Eigen::Vector3d s2;
        // zqy 把动态障碍物当做静态障碍物处理！！
        if (is_xyt)
          obstacles->at(l)->predictCentroidConstantVelocity(t, s2.head(2));
        else
          obstacles->at(l)->predictCentroidConstantVelocity(0.0, s2.head(2));
        s2[2] = t;
        // 未来xyt-当前xyt
        Eigen::Vector3d ds = s2 - s1;
        double ds_sq_norm = ds.squaredNorm(); // by definition not zero as t > 0 (3rd component)
        // **** 障碍物计算结束 ****

        // 遍历path和timediff
        for (path_iter = path_start, timediff_iter = timediff_start.get(); path_iter != path_end; ++path_iter, ++timediff_iter)
        {
          // **** 从这里开始 是和相邻的两轨迹点相关的。计算direction_vec=(下一个waypoint和当前waypoint的x差值，y差值，从当前waypoint到下一个的时间 - 从上一个到当前的时间) ****
          // 当前路径点和下个路径点
          std::complex<long double> z1 = fun_cplx_point(*path_iter);
          std::complex<long double> z2 = fun_cplx_point(*std::next(path_iter));

          transition_time = next_transition_time;
          // next_transition_time=机器人按照自大速度走 所消耗的时间
          if (timediff_start == boost::none || timediff_end == boost::none) // if no time information is provided yet, approximate transition time
            next_transition_time += std::abs(z2 - z1) / cfg_->robot.max_vel_x; // Approximate the time, if no time is known
          else // otherwise use the time information from the teb trajectory
          {
            // 这里判断的是指针的距离 而不是欧氏距离/插值
            if (std::distance(path_iter, path_end) != std::distance(timediff_iter, timediff_end.get()))
              ROS_ERROR("Size of poses and timediff vectors does not match. This is a bug.");
            next_transition_time += (*timediff_iter)->dt();
          }

          // 两个waypoint的(x差值, y差值, dt差值)
          Eigen::Vector3d direction_vec;
          direction_vec[0] = z2.real() - z1.real();
          direction_vec[1] = z2.imag() - z1.imag();
          direction_vec[2] = next_transition_time - transition_time;
          // 太小了 直接跳过
          if(direction_vec.norm() < 1e-15)  // Coincident poses
            continue;
          // r=（当前时刻waypoint的x，当前时刻waypointy，时间戳）
          Eigen::Vector3d r(z1.real(), z1.imag(), transition_time);
          // 将路径点差值离散为10份
          Eigen::Vector3d dl = 1.0/static_cast<double>(num_int_steps_per_segment) * direction_vec; // Integrate with multiple steps between each pose
          // **** 计算direction_vec结束，并将其离散了num_int_steps_per_segment=10份 ****

          // **** 计算H-Signature开始，考虑ds，p1，p2的三角关系 ****
          Eigen::Vector3d p1, p2, d, phi;
          for (int i = 0; i < num_int_steps_per_segment; ++i, r += dl)
          {
            // 对于一个waypoint点r，当前障碍物点s1，未来障碍物点s2，分别计算差值
            p1 = s1 - r;
            p2 = s2 - r;
            // 三个叉乘 ds=障碍物点的差值，p1是waypoint到障碍物点s1，p2是waypoint到障碍物点s2；ds_sq_norm是障碍物差的长度
            d = (ds.cross(p1.cross(p2))) / ds_sq_norm;
            // phi = 1 / d的长度 * (d与p2的叉积/p2的长度 - d与p1的叉积/p1的长度)。
            phi = 1.0 / d.squaredNorm() * ((d.cross(p2) / p2.norm()) - (d.cross(p1) / p1.norm()));
            // 累加H-Signature。所以这里其实就是 对每个障碍物算一个H，这个H是traj的所有waypoint的加和。
            H += phi.dot(dl);
          }
          // **** 计算H-Signature结束 ****
        }
        // normalize to 1
        hsignature3d_.at(l) = H/(4.0*M_PI);
      }
      // for (int i=0; i<hsignature3d_.size(); i++)
      //   std::cout<<hsignature3d_.at(i)<<",";
      // std::cout<<std::endl;
    }

    /**
     * @brief Check if two candidate classes are equivalent
     *
     * If the absolute value of the H-Signature is equal or greater than 1, a loop (in x-y) around the obstacle is indicated.
     * Positive H-Signature: Obstacle lies on the left hand side of the planned trajectory
     * Negative H-Signature: Obstacle lies on the right hand side of the planned trajectory
     * H-Signature equals zero: Obstacle lies in infinity, has no influence on trajectory
     *
     * @param other The other equivalence class to test with
     */
    virtual bool isEqual(const EquivalenceClass& other) const
    {
      const HSignature3d* hother = dynamic_cast<const HSignature3d*>(&other); // TODO: better architecture without dynamic_cast
      if (hother)
      {
        if (hsignature3d_.size() == hother->hsignature3d_.size())
        {
          for(size_t i = 0; i < hsignature3d_.size(); ++i)
          {
            // If the H-Signature for one obstacle is below this threshold, that obstacle is far away from the planned trajectory,
            // and therefore ignored in the homotopy class planning
            if (std::abs(hother->hsignature3d_.at(i)) < cfg_->hcp.h_signature_threshold || std::abs(hsignature3d_.at(i)) < cfg_->hcp.h_signature_threshold)
              continue;

            if (boost::math::sign(hother->hsignature3d_.at(i)) != boost::math::sign(hsignature3d_.at(i)))
              return false; // Signatures are not equal, new homotopy class
          }
        return true; // Found! Homotopy class already exists, therefore nothing added
        }
      }
      else
          ROS_ERROR("Cannot compare HSignature3d equivalence classes with types other than HSignature3d.");

      return false;
    }

    virtual bool isEqual(const std::vector<double>& Hsignature3d_values) const
    {
      if (hsignature3d_.size() == Hsignature3d_values.size())
      {
        for(size_t i = 0; i < hsignature3d_.size(); ++i)
        {
          // If the H-Signature for one obstacle is below this threshold, that obstacle is far away from the planned trajectory,
          // and therefore ignored in the homotopy class planning
          if (std::abs(Hsignature3d_values.at(i)) < cfg_->hcp.h_signature_threshold || std::abs(hsignature3d_.at(i)) < cfg_->hcp.h_signature_threshold)
            continue;
          
          if (boost::math::sign(Hsignature3d_values.at(i)) != boost::math::sign(hsignature3d_.at(i)))
            return false; // Signatures are not equal, new homotopy class
        }
      return true; // Found! Homotopy class already exists, therefore nothing added
      }
      else
        ROS_ERROR("Cannot compare HSignature3d equivalence classes with different size %d vs %d.", hsignature3d_.size(), Hsignature3d_values.size());

      return false;
    }

   virtual std::vector<double> getSignature3DValue()const {return hsignature3d_;};

    /**
     * @brief Check if the equivalence value is detected correctly
     * @return Returns false, if the equivalence class detection failed, e.g. if nan- or inf values occur.
     */
     virtual bool isValid() const
     {
        for(const double& value : hsignature3d_)
        {
          if (!std::isfinite(value))
            return false;
        }
        return true;
     }

    /**
     * @brief Check if the trajectory is non-looping around any obstacle. Values greater than 1 indicate a looping trajectory.
     * @return Returns false, if the trajectory loops around an obstacle
     */
    virtual bool isReasonable() const
    {
      for(const double& value : hsignature3d_)
      {
        if (value > 1.0)
          return false;
      }
      return true;
    }

    /**
     * @brief Get the current h-signature (read-only)
     * @return h-signature in complex-number format
     */
     const std::vector<double>& values() const {return hsignature3d_;}

private:
    const TebConfig* cfg_;
    std::vector<double> hsignature3d_;
};


} // namespace teb_local_planner


#endif /* H_SIGNATURE_H_ */
