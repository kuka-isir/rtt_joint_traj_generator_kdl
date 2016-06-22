
#include <iostream>
#include <map>

#include <Eigen/Dense>

#include <rtt/Component.hpp>

#include <kdl/tree.hpp>

#include <kdl_parser/kdl_parser.hpp>

#include <rtt_rosclock/rtt_rosclock.h>
#include <rtt_rosparam/rosparam.h>
#include <rtt_roscomm/rtt_rostopic.h>

#include <rtt_ros_kdl_tools/tools.hpp>
#include <rtt_joint_traj_generator_kdl/rtt_joint_traj_generator_kdl.h>

using namespace RTT;

JointTrajGeneratorKDL::JointTrajGeneratorKDL(std::string const& name) :
  TaskContext(name)
  // Working variables
  ,n_dof_(0)
{
  this->addProperty("trap_max_vels",trap_max_vels_).doc("Maximum velocities for trap generation.");
  this->addProperty("trap_max_accs",trap_max_accs_).doc("Maximum acceperations for trap generation.");
  this->addProperty("position_tolerance",position_tolerance_).doc("Maximum position error.");
  this->addProperty("velocity_smoothing_factor",velocity_smoothing_factor_).doc("Exponential smoothing factor to use when estimating veolocity from finite differences.");

  // Configure data ports
  this->ports()->addPort("joint_position_in", joint_position_in_);
  this->ports()->addPort("joint_velocity_in", joint_velocity_in_);
  this->ports()->addPort("joint_position_cmd_in", joint_position_cmd_eig_in_);
  this->ports()->addPort("joint_position_out", joint_position_out_)
    .doc("Output port: nx1 vector of joint positions. (n joints)");
  this->ports()->addPort("joint_velocity_out", joint_velocity_out_)
    .doc("Output port: nx1 vector of joint velocities. (n joints)");

  // ROS ports
  this->ports()->addPort("joint_position_cmd_ros_in", joint_position_cmd_ros_in_);
  this->ports()->addPort("joint_state_desired_out", joint_state_desired_out_);

}

bool JointTrajGeneratorKDL::configureHook()
{
  // Initialize kinematics (KDL tree, KDL chain, and #DOF)
  n_dof_ = rtt_ros_kdl_tools::getNumberOfJointsFromROSParamURDF();
  if(!n_dof_)
  {
    RTT::log(RTT::Error) << "Could not initialize robot kinematics!" << RTT::endlog();
    return false;
  }

  // ROS topics
  if(!joint_position_cmd_ros_in_.createStream( rtt_roscomm::topic( this->getName() + "/joint_position_cmd"))
     || !joint_state_desired_out_.createStream(rtt_roscomm::topic( this->getName() + "/joint_state_desired")))
  {
    RTT::log(RTT::Error) << "ROS Topics could not be streamed..." <<RTT::endlog();
    return false;
  }

  // ROS parameters
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam =
    this->getProvider<rtt_rosparam::ROSParam>("rosparam");


  // Resize IO vectors
  trap_max_vels_.resize(n_dof_);
  trap_max_vels_.setConstant(0.2);
  trap_max_accs_.resize(n_dof_);
  trap_max_accs_.setConstant(0.2);
  position_tolerance_.resize(n_dof_);
  position_tolerance_.setConstant(0.1);
  joint_position_.setZero(n_dof_);
  joint_position_last_.setZero(n_dof_);
  joint_position_cmd_.setZero(n_dof_);
  joint_position_sample_.setZero(n_dof_);
  joint_velocity_.setZero(n_dof_);
  joint_velocity_raw_.setZero(n_dof_);
  joint_velocity_sample_.setZero(n_dof_);

  trajectory_start_times_.resize(n_dof_);
  trajectory_end_times_.resize(n_dof_);

  joint_position_out_.setDataSample(joint_position_sample_);
  joint_velocity_out_.setDataSample(joint_velocity_sample_);

  velocity_smoothing_factor_ = 0.95;

  if(rosparam)
  {
    rosparam->getParam(getName()+"/" +"trap_max_vels","trap_max_vels");
    rosparam->getParam(getName()+"/" +"trap_max_accs","trap_max_accs");
    rosparam->getParam(getName()+"/" +"position_tolerance","position_tolerance");
    rosparam->getParam(getName()+"/" +"velocity_smoothing_factor","velocity_smoothing_factor");
  }else{
    RTT::log(RTT::Warning) << "Could not load rosparam" << RTT::endlog();
  }

  // Create trajectory generators
  trajectories_.resize(n_dof_);
  for(unsigned i=0; i<n_dof_; i++){
    trajectory_start_times_[i] = 0.0;
    trajectory_end_times_[i] = 0.0;
    trajectories_[i] = KDL::VelocityProfile_Trap(trap_max_vels_[i], trap_max_accs_[i]);
  }

  return true;
}

bool JointTrajGeneratorKDL::startHook()
{
  // Reset the last position flag
  has_last_position_data_ = false;

  return true;
}

void JointTrajGeneratorKDL::updateHook()
{
  // Read in the current joint positions & velocities
  bool new_position_data = (joint_position_in_.readNewest(joint_position_) == RTT::NewData);
  bool new_velocity_data = (joint_velocity_in_.readNewest(joint_velocity_raw_) == RTT::NewData);

  // If we don't get any position update, we don't write any new data to the ports
  if(!new_position_data) {
    return;
  }

  // Get the current and the time since the last update
  static ros::Time last = rtt_rosclock::host_now();
  ros::Time now = rtt_rosclock::host_now();

  double period = (now - last).toSec();
  double time = now.toSec();

  // Check the minimum requirements to compute the control command
  if(new_velocity_data || has_last_position_data_) {
    // Trust a supplied velocity, or compute it from an exponentially-smothed finite difference
    if(new_velocity_data) {
      // Trust the velocity input
      joint_velocity_ = joint_velocity_raw_;
    } else {
      // Estimate the joint velocity if we don't get a joint velocity estimate
      joint_velocity_ =
        (velocity_smoothing_factor_*(joint_position_ - joint_position_last_)/period)
        + (1-velocity_smoothing_factor_)*(joint_velocity_);
    }

    // Read in any newly commanded joint positions
    RTT::FlowStatus rtt_cmd = joint_position_cmd_eig_in_.readNewest( joint_position_cmd_ );
    RTT::FlowStatus ros_cmd = joint_position_cmd_ros_in_.readNewest( joint_position_cmd_ros_ );

    if(rtt_cmd == RTT::NoData && ros_cmd == RTT::NoData)
    {
      // Do nothing if we don't have any desired positions
      //return;
      static bool has_first_pos = false;
      if(!has_first_pos)
      {
        log(Debug) << "Initializing to current position "<<joint_position_.transpose()<<endlog();
        joint_position_cmd_ = joint_position_;
        has_first_pos = true;
        rtt_cmd = RTT::NewData;
      }
    }
    if(rtt_cmd == RTT::NewData || ros_cmd == RTT::NewData) {
      // ROS command overrides RTT command if it's new
      if(ros_cmd == RTT::NewData && rtt_cmd != RTT::NewData) {
        RTT::log(RTT::Debug) << "New ROS trajectory point "<<joint_position_cmd_ros_ << RTT::endlog();
        if(joint_position_cmd_ros_.positions.size() == joint_position_cmd_.size())
            joint_position_cmd_ = Eigen::VectorXd::Map(
                joint_position_cmd_ros_.positions.data(),
                joint_position_cmd_ros_.positions.size());
      }
      for(unsigned i=0; i<n_dof_; i++) {
        // Check to make sure the last trajectory has completed in this degree-of-freedom

        if(time > trajectory_end_times_[i]) {
          // Set the 1-dof trajectory profile
          // This will compute a trajectory subject to the velocity and
          // acceleration limits, but assume it's starting with zero velocity.
          trajectories_[i].SetProfile(joint_position_[i], joint_position_cmd_[i]);
          trajectory_start_times_[i] = time;
          trajectory_end_times_[i] = trajectory_start_times_[i] + trajectories_[i].Duration();
          RTT::log(RTT::Debug) << "New SetProfile Duration : " <<trajectories_[i].Duration() <<RTT::endlog();
        }
      }
    }



    // Sample the trajectory
    for(unsigned i=0; i<n_dof_; i++) {
      // Set final position commands
      if(time > trajectory_end_times_[i]) {
        joint_position_sample_[i] = joint_position_cmd_[i];
        joint_velocity_sample_[i] = 0.0;
      } else {
        joint_position_sample_[i] = trajectories_[i].Pos(time - trajectory_start_times_[i]);
        joint_velocity_sample_[i] = trajectories_[i].Vel(time - trajectory_start_times_[i]);
      }

      // Check tolerance
      if(fabs(joint_position_sample_[i] - joint_position_[i]) > position_tolerance_[i]) {
        RTT::log(RTT::Debug) << "Exceeded tolerance in joint "<<i
            <<" : sample: "<<joint_position_sample_[i]
            <<" curr:"<<joint_position_[i]
            <<" tolerance:"<<position_tolerance_[i]
            << RTT::endlog();
        trajectories_[i].SetProfile(joint_position_[i], joint_position_cmd_[i]);
        trajectory_start_times_[i] = time;
        trajectory_end_times_[i] = trajectory_start_times_[i] + trajectories_[i].Duration();
      }
    }



  }

  // RTT::log(RTT::Debug) << "joint_position_sample_ : "<<joint_position_sample_.transpose() << RTT::endlog();
  // RTT::log(RTT::Debug) << "position_tolerance_ : "<<position_tolerance_.transpose() << RTT::endlog();
  // RTT::log(RTT::Debug) << "joint_position_ : "<<joint_position_.transpose() << RTT::endlog();

  // Send instantaneous joint position and velocity commands
  joint_position_out_.write(joint_position_sample_);
  joint_velocity_out_.write(joint_velocity_sample_);

  // Publish debug traj to ros
  joint_state_desired_.header.stamp = rtt_rosclock::host_now();
  joint_state_desired_.position.resize(n_dof_);
  joint_state_desired_.velocity.resize(n_dof_);
  std::copy(joint_position_sample_.data(), joint_position_sample_.data() + n_dof_, joint_state_desired_.position.begin());
  std::copy(joint_velocity_sample_.data(), joint_velocity_sample_.data() + n_dof_, joint_state_desired_.velocity.begin());
  joint_state_desired_out_.write(joint_state_desired_);

  // Save the last joint position
  joint_position_last_ = joint_position_;
  has_last_position_data_ = true;
}

void JointTrajGeneratorKDL::stopHook()
{
  // Clear data buffers (this will make them return OldData if nothing new is written to them)
  joint_position_in_.clear();
  joint_velocity_in_.clear();
}

void JointTrajGeneratorKDL::cleanupHook()
{
}


ORO_CREATE_COMPONENT(JointTrajGeneratorKDL)
