#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

double g_updateRate, g_simulationFactor;
std::string g_worldFrame, g_robotFrame;
geometry_msgs::Twist g_currentTwist;
tf::Transform g_currentPose;
boost::shared_ptr<tf::TransformBroadcaster> g_transformBroadcaster;
boost::mutex mutex_vel;
boost::mutex mutex_pos;
int robot_mode;
bool is_reset_pose;
bool pause_env;
ros::ServiceServer srv_pause_simulation_;
ros::ServiceServer srv_unpause_simulation_;


/// Simulates robot motion of a differential-drive robot with translational and
/// rotational velocities as input
/// These are provided in the form of a geometry_msgs::Twist, e.g. by
/// turtlebot_teleop/turtlebot_teleop_key.
/// The resulting robot position is published as a TF transform from world -->
/// base_footprint frame.

bool onPauseSimulation(std_srvs::Empty::Request& request,
                                  std_srvs::Empty::Response& response) {
  pause_env = true;
  return true;
}

bool onUnpauseSimulation(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response) {
  pause_env = false;
  return true;
}


void updateLoop() {
  ros::Rate rate(g_updateRate);
  const double dt = g_simulationFactor / g_updateRate;

  while (true) {
    if (pause_env){
      rate.sleep();
      continue;
    }

    // Get current pose
    double x = g_currentPose.getOrigin().x();
    double y = g_currentPose.getOrigin().y();
    double theta = tf::getYaw(g_currentPose.getRotation());

    // Get requested translational and rotational velocity
    double v, omega;
    {
      boost::mutex::scoped_lock lock(mutex_vel);
      v = g_currentTwist.linear.x;
      omega = g_currentTwist.angular.z;
    }

    // 看是否有initial_pose发生
    if (is_reset_pose){
      is_reset_pose = false;
    }
    else{
      // Simulate robot movement
      x += cos(theta) * v * dt;
      y += sin(theta) * v * dt;
      theta += omega * dt;

      // Update pose
      g_currentPose.getOrigin().setX(x);
      g_currentPose.getOrigin().setY(y);
      g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, theta));
    }

    // Broadcast transform
    g_transformBroadcaster->sendTransform(tf::StampedTransform(
        g_currentPose, ros::Time::now(), g_worldFrame, g_robotFrame));

    rate.sleep();
  }
}

void onTwistReceived(const geometry_msgs::Twist::ConstPtr& twist) {
  boost::mutex::scoped_lock lock(mutex_vel);
  g_currentTwist = *twist;
}

void resetPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose) {
  boost::mutex::scoped_lock lock(mutex_pos);
  g_currentPose.setOrigin(tf::Vector3(pose->pose.pose.position.x,
                                     pose->pose.pose.position.y, 0));
  g_currentPose.setRotation(tf::Quaternion(pose->pose.pose.orientation.x,
                                          pose->pose.pose.orientation.y,
                                          pose->pose.pose.orientation.z,
                                          pose->pose.pose.orientation.w));
  is_reset_pose = true;
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "simulate_diff_drive_robot");
  ros::NodeHandle nodeHandle("");
  ros::NodeHandle privateHandle("~");
  privateHandle.param<int>("robot_mode", robot_mode, 0);
  //如果机器人用social force，那就直接用simulation.cpp中的tf转换就好
  if (robot_mode==1){
    exit(0);
  }
  is_reset_pose = false;
  pause_env = false;
  srv_pause_simulation_ = nodeHandle.advertiseService(
      "/simulate_diff_drive_robot/pause_simulation", onPauseSimulation);
  srv_unpause_simulation_ = nodeHandle.advertiseService(
      "/simulate_diff_drive_robot/unpause_simulation", onUnpauseSimulation);

  // Process parameters
  privateHandle.param<std::string>("world_frame", g_worldFrame, "odom");
  privateHandle.param<std::string>("robot_frame", g_robotFrame,
                                   "base_link");

  privateHandle.param<double>("/pedsim_simulator/simulation_factor", g_simulationFactor,
                              1.0);  // set to e.g. 2.0 for 2x speed
  privateHandle.param<double>("/pedsim_simulator/update_rate", g_updateRate, 25.0);  // in Hz

  double initialX = 0.0, initialY = 0.0, initialTheta = 0.0;
  privateHandle.param<double>("pose_initial_x", initialX, 0.0);
  privateHandle.param<double>("pose_initial_y", initialY, 0.0);
  privateHandle.param<double>("pose_initial_theta", initialTheta, 0.0);

  g_currentPose.getOrigin().setX(initialX);
  g_currentPose.getOrigin().setY(initialY);
  g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, initialTheta));

  // Create ROS subscriber and TF broadcaster
  g_transformBroadcaster.reset(new tf::TransformBroadcaster());
  ros::Subscriber twistSubscriber =
      nodeHandle.subscribe<geometry_msgs::Twist>("/cmd_vel", 3, onTwistReceived);
  
  // subscriber，接收topic名为"/initialpose"，类型为“geometry_msgs/PoseWithCovarianceStamped”，回调函数为resetPose
  ros::Subscriber initialPoseSubscriber = nodeHandle.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, resetPose);

  // Run
  boost::thread updateThread(updateLoop);
  ros::spin();
}
