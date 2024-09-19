/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
* \author Sven Wehner <mail@svenwehner.de>
*/

#include <pedsim_simulator/agentstatemachine.h>
#include <pedsim_simulator/config.h>
#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/waypoint.h>
#include <pedsim_simulator/force/force.h>
#include <pedsim_simulator/scene.h>
#include <pedsim_simulator/waypointplanner/waypointplanner.h>
#include <tf/tf.h>

Agent::Agent() {
  // initialize
  Ped::Tagent::setType(Ped::Tagent::ADULT);
  Ped::Tagent::setForceFactorObstacle(CONFIG.forceObstacle);
  forceSigmaObstacle = CONFIG.sigmaObstacle;
  Ped::Tagent::setForceFactorSocial(CONFIG.forceSocial);
  // waypoints
  currentDestination = nullptr;
  waypointplanner = nullptr;
  // state machine
  stateMachine = new AgentStateMachine(this);
  // group
  group = nullptr;
}

Agent::~Agent() {
  // clean up
  foreach (Force* currentForce, forces) { delete currentForce; }
}


/// Calculates the desired force. Same as in lib, but adds graphical
/// representation
Ped::Tvector Agent::desiredForce() {
  Ped::Tvector force;
  if (!disabledForces.contains("Desired")) force = Tagent::desiredForce();

  // inform users
  emit desiredForceChanged(force.x, force.y);

  return force;
}

/// Calculates the social force. Same as in lib, but adds graphical
/// representation
Ped::Tvector Agent::socialForce() const {
  Ped::Tvector force;
  if (!disabledForces.contains("Social")) force = Tagent::socialForce();

  // inform users
  emit socialForceChanged(force.x, force.y);

  return force;
}

/// Calculates the obstacle force. Same as in lib, but adds graphical
/// representation
Ped::Tvector Agent::obstacleForce() const {
  Ped::Tvector force;
  if (!disabledForces.contains("Obstacle")) force = Tagent::obstacleForce();

  // inform users
  emit obstacleForceChanged(force.x, force.y);

  return force;
}

Ped::Tvector Agent::myForce(Ped::Tvector desired) const {
  // run additional forces
  Ped::Tvector forceValue;
  foreach (Force* force, forces) {
    // skip disabled forces
    if (disabledForces.contains(force->getName())) {
      // update graphical representation
      emit additionalForceChanged(force->getName(), 0, 0);
      continue;
    }

    // add force to the total force
    Ped::Tvector currentForce = force->getForce(desired);
    // → sanity checks
    if (!currentForce.isValid()) {
      ROS_DEBUG("Invalid Force: %s", force->getName().toStdString().c_str());
      currentForce = Ped::Tvector();
    }
    forceValue += currentForce;

    // update graphical representation
    emit additionalForceChanged(force->getName(), currentForce.x,
                                currentForce.y);
  }

  // inform users
  emit myForceChanged(forceValue.x, forceValue.y);

  return forceValue;
}

Ped::Twaypoint* Agent::getCurrentDestination() const {
  return currentDestination;
}

Ped::Twaypoint* Agent::updateDestination() {
  // assign new destination
  if (!destinations.isEmpty()) {
    if (currentDestination != nullptr) {
      // cycle through destinations
      Waypoint* previousDestination = destinations.takeFirst();
      destinations.append(previousDestination);
    }
    currentDestination = destinations.first();
  }

  return currentDestination;
}

void Agent::updateState() {
  // check state
  stateMachine->doStateTransition();
}

void Agent::move(double h) {
  if (getType() == Ped::Tagent::ROBOT) {
    if (CONFIG.robot_mode == RobotMode::TELEOPERATION) {
      // NOTE: Moving is now done by setting x, y position directly in
      // simulator.cpp
      // Robot's vx, vy will still be set for the social force model to work
      // properly wrt. other agents.

      // FIXME: This is a very hacky way of making the robot "move" (=update
      // position in hash tree) without actually moving it
      const double vx = getvx();
      const double vy = getvy();

      setvx(0);
      setvy(0);
      Ped::Tagent::move(h);
      setvx(vx);
      setvy(vy);
    } else if (CONFIG.robot_mode == RobotMode::CONTROLLED) {
      // just like robotmode::teleoperation
      const double vx = getvx();
      const double vy = getvy();
      setvx(0);
      setvy(0);
      // Ped::Tagent::move(h);
      setvx(vx);
      setvy(vy);
      // if (SCENE.getTime() >= CONFIG.robot_wait_time) {
      //   Ped::Tagent::move(h);
      // }
    } else if (CONFIG.robot_mode == RobotMode::SOCIAL_DRIVE) {
      Ped::Tagent::setForceFactorSocial(CONFIG.forceSocial * 2.0);
      Ped::Tagent::setForceFactorObstacle(35);
      Ped::Tagent::setForceFactorDesired(4.2);

      Ped::Tagent::setVmax(0.3);
      Ped::Tagent::SetRadius(0.4);
      Ped::Tagent::move(h);
    }
  } else {
    if (getType() == Ped::Tagent::ELDER) {
      // Old people slow!
      Ped::Tagent::setVmax(0.3);
      Ped::Tagent::setForceFactorDesired(0.5);
    }
    if (CONFIG.person_mode == PersonMode::SOCIAL_DRIVE){
      Ped::Tagent::move(h);
    }
  }



  // inform users
  emit positionChanged(getx(), gety());
  emit velocityChanged(getvx(), getvy());
  emit accelerationChanged(getax(), getay());
}

const QList<Waypoint*>& Agent::getWaypoints() const { return destinations; }

bool Agent::setWaypoints(const QList<Waypoint*>& waypointsIn) {
  destinations = waypointsIn;
  return true;
}

bool Agent::addWaypoint(Waypoint* waypointIn) {
  destinations.append(waypointIn);
  return true;
}

bool Agent::removeWaypoint(Waypoint* waypointIn) {
  const int removeCount = destinations.removeAll(waypointIn);

  return (removeCount > 0);
}

bool Agent::needNewDestination() const {
  if (waypointplanner == nullptr)
    return (!destinations.isEmpty());
  else {
    // ask waypoint planner
    return waypointplanner->hasCompletedDestination();
  }
}

Ped::Twaypoint* Agent::getCurrentWaypoint() const {
  // sanity checks
  if (waypointplanner == nullptr) return nullptr;

  // ask waypoint planner
  return waypointplanner->getCurrentWaypoint();
}

bool Agent::isInGroup() const { return (group != nullptr); }

AgentGroup* Agent::getGroup() const { return group; }

void Agent::setGroup(AgentGroup* groupIn) { group = groupIn; }

bool Agent::addForce(Force* forceIn) {
  forces.append(forceIn);

  // inform users
  emit forceAdded(forceIn->getName());

  // report success
  return true;
}

bool Agent::removeForce(Force* forceIn) {
  int removeCount = forces.removeAll(forceIn);

  // inform users
  emit forceRemoved(forceIn->getName());

  // report success if a Behavior has been removed
  return (removeCount >= 1);
}

bool Agent::setTopic(){
  this->topicName = "/people"+to_string(this->id)+"/keyboard";
  this->GazetopicName="/people"+to_string(this->id)+"/keyboard"+"/gaze";
  return true;
}

bool Agent::setSubscriber(ros::NodeHandle nh){
  this->nh_ = nh;
  this->sub_action_ = this->nh_.subscribe<geometry_msgs::Twist>(this->topicName, 1, boost::bind(&Agent::controlCallback, this,_1));
  this->sub_state_ = this->nh_.subscribe<pedsim_msgs::TrackedPersons>("/persons_recorded", 1, boost::bind(&Agent::stateCallback, this,_1));
  this->sub_gaze_ =this->nh_.subscribe<geometry_msgs::Twist>(this->GazetopicName,1,boost::bind(&Agent::gazeCallback,this,_1));
  return true;
}
bool Agent::setPubTopic(){
  // ROS_INFO_STREAM("topic name set");
  this->topicPubGazeDirName="/people"+to_string(this->id)+"/gaze/direction";
  return true;
}
bool Agent::setPublisher(ros::NodeHandle nh){
  this->nh_=nh;
  this->pub_gaze_dir_=this->nh_.advertise<visualization_msgs::Marker>(this->topicPubGazeDirName,1,true);
  this->pub_gaze_tar_=this->nh_.advertise<visualization_msgs::Marker>(this->topicPubGazeTarName,1,true);
  
  return true;
}



void Agent::gazePublish(const visualization_msgs::Marker dir_msg,const visualization_msgs::Marker tar_msg) const{

  this->pub_gaze_dir_.publish(dir_msg);
  this->pub_gaze_tar_.publish(tar_msg);
}

void Agent::controlCallback(const geometry_msgs::Twist::ConstPtr& msg){
  float vDot = msg->linear.x;
  float wDot = msg->angular.z; 
  this->actionFromTopic = Ped::Tvector(vDot,wDot,0);
}

void Agent::stateCallback(const pedsim_msgs::TrackedPersons::ConstPtr& msg){
  for (auto person:msg->tracks){
    if (person.track_id == this->id){
      tf::Quaternion quat;
      tf::quaternionMsgToTF(person.pose.pose.orientation, quat);
      double yaw, pitch, roll;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      this->stateFromTopic = Ped::Tvector(person.pose.pose.position.x, person.pose.pose.position.y, yaw, person.twist.twist.linear.x, person.twist.twist.linear.y);
    }
  }
}

void Agent::gazeCallback(const geometry_msgs::Twist::ConstPtr& msg){
  

  float yawRot= msg->angular.z; // for gaze target move along with x axis
  float pitchRot = msg->angular.y; // for gaze target move along with y axis
  float rollRot=msg->angular.x; // for gaze target move along with z axis
  float mode=msg->linear.x;

  this->gazeFromTopic=Ped::Tvector(yawRot,pitchRot,rollRot,mode);
}

AgentStateMachine* Agent::getStateMachine() const { return stateMachine; }

WaypointPlanner* Agent::getWaypointPlanner() const { return waypointplanner; }

void Agent::setWaypointPlanner(WaypointPlanner* plannerIn) {
  waypointplanner = plannerIn;
}

QList<const Agent*> Agent::getNeighbors() const {
  // upcast neighbors
  QList<const Agent*> output;
  for (const Ped::Tagent* neighbor : neighbors) {
    const Agent* upNeighbor = dynamic_cast<const Agent*>(neighbor);
    if (upNeighbor != nullptr) output.append(upNeighbor);
  }

  return output;
}

void Agent::disableForce(const QString& forceNameIn) {
  // disable force by adding it to the list of disabled forces
  disabledForces.append(forceNameIn);
}

void Agent::enableAllForces() {
  // remove all forces from disabled list
  disabledForces.clear();
}

void Agent::setPosition(double xIn, double yIn) {
  // call super class' method
  Ped::Tagent::setPosition(xIn, yIn);

  // inform users
  emit positionChanged(xIn, yIn);
}

void Agent::setX(double xIn) { setPosition(xIn, gety()); }

void Agent::setY(double yIn) { setPosition(getx(), yIn); }

void Agent::setType(Ped::Tagent::AgentType typeIn) {
  // call super class' method
  Ped::Tagent::setType(typeIn);

  // inform users
  emit typeChanged(typeIn);
}

Ped::Tvector Agent::getDesiredDirection() const { return desiredforce; }

Ped::Tvector Agent::getWalkingDirection() const { return v; }

Ped::Tvector Agent::getSocialForce() const { return socialforce; }

Ped::Tvector Agent::getObstacleForce() const { return obstacleforce; }

Ped::Tvector Agent::getMyForce() const { return myforce; }

QPointF Agent::getVisiblePosition() const { return QPointF(getx(), gety()); }

void Agent::setVisiblePosition(const QPointF& positionIn) {
  // check and apply new position
  if (positionIn != getVisiblePosition())
    setPosition(positionIn.x(), positionIn.y());
}

QString Agent::toString() const {
  return tr("Agent %1 (@%2,%3)").arg(getId()).arg(getx()).arg(gety());
}
