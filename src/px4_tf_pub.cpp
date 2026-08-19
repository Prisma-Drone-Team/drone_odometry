#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <Eigen/Core>
#include "utils.h"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

// using namespace px4_ros_com::frame_transforms; //TODO remove

using std::placeholders::_1;

class Px4TfPublisher : public rclcpp::Node
{
  private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_out_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr px4_odometry_out_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr test_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr companion_odometry_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_visual_odometry_pub_;

    rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr trajectory_setpoint_ros_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_ros_sub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;  

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // params
    std::string px4_odom_frame_id_; //output odometry topic and tf tfame name
    std::string vio_desired_parent_frame_id_; //input vio parent is trasformed loocking up to this frame and vio parent , should coincide with px4_odom_frame_id_
    bool publish_tf_;
    bool feed_twist_to_px4_;
    bool odom_child_is_not_base_link_;
    bool odom_parent_is_not_odom_;
    // When false, skip publishing to /fmu/in/vehicle_visual_odometry.
    // Set to false in sewer_exploration so flight_odometry_filter is the sole PX4 VIO source.
    bool relay_odometry_;


  public:
    Px4TfPublisher(): Node("px4_tf_pub"){

      //read params
      this->declare_parameter<string>("px4_odom_frame_id", "odom"); //tf_pub_parent_frame (/odom if not slam, /map if slam) TBD
      px4_odom_frame_id_ = this->get_parameter("px4_odom_frame_id").as_string();

      this->declare_parameter<string>("vio_desired_parent_frame_id", px4_odom_frame_id_); // used to lookup transoform in for parent vio offset should coincide with px4_odom_frame_id_
      vio_desired_parent_frame_id_ = this->get_parameter("vio_desired_parent_frame_id").as_string();

      this->declare_parameter<bool>("publish_tf", false);
      publish_tf_ = this->get_parameter("publish_tf").as_bool();

      this->declare_parameter<bool>("feed_twist_to_px4", true);
      feed_twist_to_px4_ = this->get_parameter("feed_twist_to_px4").as_bool();

      this->declare_parameter<bool>("odom_child_is_not_base_link", false);
      odom_child_is_not_base_link_ = this->get_parameter("odom_child_is_not_base_link").as_bool();
      
      this->declare_parameter<bool>("odom_parent_is_not_odom", false);
      odom_parent_is_not_odom_ = this->get_parameter("odom_parent_is_not_odom").as_bool();

      this->declare_parameter<bool>("relay_odometry", true);
      relay_odometry_ = this->get_parameter("relay_odometry").as_bool();
      if (!relay_odometry_) {
        RCLCPP_INFO(this->get_logger(),
          "relay_odometry=false: /fmu/in/vehicle_visual_odometry will NOT be published by this node."
          " Ensure another node (e.g. flight_odometry_filter) is the sole PX4 VIO source.");
      }

      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

      // px4 odometry from autopilot, to be relied into ros and published as tf
      vehicle_odometry_out_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&Px4TfPublisher::px4_odom_out_cb, this, _1));
      px4_odometry_out_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/px4/odometry/out", qos);

       // ros odometry from companion pc, to be relied to px4
      companion_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", qos, std::bind(&Px4TfPublisher::companion_odom_cb, this, _1));
      vehicle_visual_odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", qos); // /fmu/in/vehicle_mocap_odometry or /fmu/in/vehicle_visual_odometry
      
      trajectory_setpoint_ros_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>("/px4/trajectory_setpoint_enu", qos, std::bind(&Px4TfPublisher::trajectory_setpoint_ros_cb, this, _1));
      cmd_vel_ros_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/px4/cmd_vel", qos, std::bind(&Px4TfPublisher::cmd_vel_ros_cb, this, _1));
      trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos);
      offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos);
      // vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos);

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      
      /* TODO
        1) add tf broadcaster
        2) add parameters for : tf_pub_parent_frame (/odom if not slam, /map if slam TBD)
        
      */
      

      test_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry_rotated", qos); //TODO remove
    }

  private:

    void cmd_vel_ros_cb(const geometry_msgs::msg::Twist::UniquePtr msg_enu){
      px4_msgs::msg::TrajectorySetpoint msg_ned;
      Eigen::Vector3d x_enu;
      Eigen::Vector3d x_ned;

      msg_ned.position[0] = NAN;
      msg_ned.position[1] = NAN;
      msg_ned.position[2] = NAN;

      x_enu << msg_enu->linear.x, msg_enu->linear.y, msg_enu->linear.z;
      x_ned = utilities::R_ned_enu*x_enu;
      msg_ned.velocity[0] = x_ned.x();
      msg_ned.velocity[1] = x_ned.y();
      msg_ned.velocity[2] = x_ned.z();

      msg_ned.acceleration[0] = NAN;
      msg_ned.acceleration[1] = NAN;
      msg_ned.acceleration[2] = NAN;

      msg_ned.yaw = NAN;
      
      msg_ned.yawspeed = -msg_enu->angular.z;; //:)

      msg_ned.timestamp = this->get_clock()->now().nanoseconds() / 1000;
      trajectory_setpoint_pub_->publish(msg_ned);

      // control mode 
      px4_msgs::msg::OffboardControlMode msg_ctrl_mode;
      msg_ctrl_mode.position = false;
      msg_ctrl_mode.velocity = true;
      msg_ctrl_mode.acceleration = false;
      msg_ctrl_mode.attitude = false;
      msg_ctrl_mode.body_rate = true;
      msg_ctrl_mode.timestamp = this->get_clock()->now().nanoseconds() / 1000;
      offboard_control_mode_pub_->publish(msg_ctrl_mode);
    }

    void trajectory_setpoint_ros_cb(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::UniquePtr msg_enu){
      //setpoint
      px4_msgs::msg::TrajectorySetpoint msg_ned;
      Eigen::Vector3d x_enu;
      Eigen::Vector3d x_ned;

      x_enu << msg_enu->transforms[0].translation.x, msg_enu->transforms[0].translation.y, msg_enu->transforms[0].translation.z;
      x_ned = utilities::R_ned_enu*x_enu;
      msg_ned.position[0] = x_ned.x();
      msg_ned.position[1] = x_ned.y();
      msg_ned.position[2] = x_ned.z();

      x_enu << msg_enu->velocities[0].linear.x, msg_enu->velocities[0].linear.y, msg_enu->velocities[0].linear.z;
      x_ned = utilities::R_ned_enu*x_enu;
      msg_ned.velocity[0] = x_ned.x();
      msg_ned.velocity[1] = x_ned.y();
      msg_ned.velocity[2] = x_ned.z();

      x_enu << msg_enu->accelerations[0].linear.x, msg_enu->accelerations[0].linear.y, msg_enu->accelerations[0].linear.z;
      x_ned = utilities::R_ned_enu*x_enu;
      msg_ned.acceleration[0] = x_ned.x();
      msg_ned.acceleration[1] = x_ned.y();
      msg_ned.acceleration[2] = x_ned.z();

      // x_enu << msg_enu->jerk[0], msg_enu->jerk[1], msg_enu->jerk[2];
      // x_ned = utilities::R_ned_enu*x_enu;
      // msg_ned.jerk[0] = x_ned.x();
      // msg_ned.jerk[1] = x_ned.y();
      // msg_ned.jerk[2] = x_ned.z();

      // Eigen::Matrix3d R_enu_flu = utilities::XYZ2R(Eigen::Vector3d(0, msg_enu->yaw, 0));
      Eigen::Vector4d q_enu( msg_enu->transforms[0].rotation.w, msg_enu->transforms[0].rotation.x, msg_enu->transforms[0].rotation.y, msg_enu->transforms[0].rotation.z);
      Eigen::Matrix3d R_enu_flu = utilities::QuatToMat(q_enu);
      Eigen::Matrix3d R_ned_frd = utilities::R_ned_enu*R_enu_flu*utilities::R_flu_frd;
      Eigen::Vector3d eta_ned_frd = utilities::R2XYZ(R_ned_frd);
      msg_ned.yaw = eta_ned_frd.z();
      // std::cout<<"\n-----\n"<<"q_enu= "<<q_enu<<"\neta_ned= "<<eta_ned_frd<<"\n";


      msg_ned.yawspeed = -msg_enu->velocities[0].angular.z;; //:)

      msg_ned.timestamp = this->get_clock()->now().nanoseconds() / 1000;
      trajectory_setpoint_pub_->publish(msg_ned);

      // control mode 
      px4_msgs::msg::OffboardControlMode msg_ctrl_mode;
      msg_ctrl_mode.position = true;
      msg_ctrl_mode.velocity = true;
      msg_ctrl_mode.acceleration = true;
      msg_ctrl_mode.attitude = true;
      msg_ctrl_mode.body_rate = true;
      msg_ctrl_mode.timestamp = this->get_clock()->now().nanoseconds() / 1000;
      offboard_control_mode_pub_->publish(msg_ctrl_mode);

    }

    void companion_odom_cb(const nav_msgs::msg::Odometry::UniquePtr msg){

      // std::cout << "============================="     							<< std::endl;    //TEST
      // std::cout << "\n\nRECEIVED Companion ODOMETRY  DATA"   					<< std::endl;  //TEST
      // std::cout << "frame: "         				<< msg->header.header.frame 			<< std::endl;

      /*TODO:
        1) convert from odom child frame ( eg cam frame) to base link :
              N.B. not useful if passed in robot localization
              tf loockup for sensor frame to base link, only first time if static, to get T
              rotate twist and position and cov ecc
        2) add displacement from /map to odom parent frame to account for slam correction or similar
              tf loockup for map to sensor odom frame link to get T
              rotate and translate position
              rotate velocity
        3) transform to px4 coordinate and fill message:
              check for timestamp conversion
              choose in wich frame publish, if 1 1 as px4 odom in
              raotate everithing             
              check if feed_twist_to_px4_ param
      */
      
      nav_msgs::msg::Odometry odom = *msg;

      if (odom_child_is_not_base_link_){
        odom = rotate_ros_odom_child(odom); //change from child to /base_link
        // test_pub_->publish(odom);
      }

      if (odom_parent_is_not_odom_){
        odom = rotate_ros_odom_parent(odom); //change wrt parent to wrt /odom
        // test_pub_->publish(odom);
      }

      px4_msgs::msg::VehicleOdometry px4_odom = transform_ros_odometry_to_px4(odom);
      if (relay_odometry_) {
        vehicle_visual_odometry_pub_->publish(px4_odom);
      }

      
      //todo first_odom = true;
    }

    px4_msgs::msg::VehicleOdometry transform_ros_odometry_to_px4(const  nav_msgs::msg::Odometry& ros_odom) { //custom
      px4_msgs::msg::VehicleOdometry px4_odom;

      px4_odom.pose_frame = px4_odom.POSE_FRAME_NED;
      //position
      Eigen::Vector3d p_enu(ros_odom.pose.pose.position.x,ros_odom.pose.pose.position.y, ros_odom.pose.pose.position.z);
      Eigen::Vector3d p_ned = utilities::R_ned_enu*p_enu;
      px4_odom.position[0] = p_ned.x();
      px4_odom.position[1] = p_ned.y();
      px4_odom.position[2] = p_ned.z();

      //attitude
      Eigen::Vector4d q_enu_flu(ros_odom.pose.pose.orientation.w, ros_odom.pose.pose.orientation.x, ros_odom.pose.pose.orientation.y, ros_odom.pose.pose.orientation.z);
      Eigen::Matrix3d R_enu_flu = utilities::QuatToMat(q_enu_flu);
      Eigen::Matrix3d R_ned_frd = utilities::R_ned_enu*R_enu_flu*utilities::R_flu_frd;
      Eigen::Vector4d q_ned_frd = utilities::rot2quat(R_ned_frd);
      q_ned_frd = q_ned_frd/q_ned_frd.norm();
      px4_odom.q[0] = q_ned_frd[0]; 
      px4_odom.q[1] = q_ned_frd[1];
      px4_odom.q[2] = q_ned_frd[2];
      px4_odom.q[3] = q_ned_frd[3];


      if(feed_twist_to_px4_){
        px4_odom.velocity_frame = px4_odom.VELOCITY_FRAME_BODY_FRD;
        //velocity
        Eigen::Vector3d v_flu(ros_odom.twist.twist.linear.x, ros_odom.twist.twist.linear.y, ros_odom.twist.twist.linear.z);
        // Eigen::Vector3d v_flu = R_ned_frd*utilities::R_flu_frd*v_ned; //if VELOCITY_FRAME_NED
        Eigen::Vector3d v_frd = utilities::R_frd_flu*v_flu;
        px4_odom.velocity[0] = v_frd.x();
        px4_odom.velocity[1] = v_frd.y();
        px4_odom.velocity[2] = v_frd.z();

        //angular velocity
        Eigen::Vector3d w_flu(ros_odom.twist.twist.angular.x, ros_odom.twist.twist.angular.y, ros_odom.twist.twist.angular.z);
        Eigen::Vector3d w_frd = utilities::R_frd_flu*w_flu;
        px4_odom.angular_velocity[0] = w_frd.x();
        px4_odom.angular_velocity[1] = w_frd.y();
        px4_odom.angular_velocity[2] = w_frd.z();
      }
      else{
        px4_odom.velocity_frame =2 ;
        px4_odom.velocity[0] = NAN;
        px4_odom.velocity[1] = NAN;
        px4_odom.velocity[2] = NAN;
        px4_odom.angular_velocity[0] = NAN;
        px4_odom.angular_velocity[1] = NAN;
        px4_odom.angular_velocity[2] = NAN;
      }
      
      //covariance
      utilities::Matrix6d cov_pos_ros = utilities::cov_to_mat(ros_odom.pose.covariance);
      utilities::Matrix6d cov_vel_ros = utilities::cov_to_mat(ros_odom.twist.covariance);

      Eigen::Matrix3d cov_pos_enu = cov_pos_ros.block(0,0,3,3);
      Eigen::Matrix3d cov_rot_enu = cov_pos_ros.block(3,3,3,3);
      Eigen::Matrix3d cov_vel_flu = cov_vel_ros.block(0,0,3,3);

      Eigen::Matrix3d R = utilities::R_ned_enu; //pos
      Eigen::Matrix3d cov_pos_ned = R*cov_pos_enu*R.transpose();

      R = utilities::R_frd_flu; //vel 
      // R = R_ned_frd*utilities::R_flu_frd; //if VELOCITY_FRAME_NED
      Eigen::Matrix3d cov_vel_frd = R*cov_vel_flu*R.transpose();

      R = utilities::R_ned_enu*R_enu_flu*utilities::R_flu_frd;
      Eigen::Matrix3d cov_rot_ned = R*cov_rot_enu*R.transpose();

      px4_odom.position_variance[0] = cov_pos_ned(0,0);
      px4_odom.position_variance[1] = cov_pos_ned(1,1);
      px4_odom.position_variance[2] = cov_pos_ned(2,2);

      px4_odom.orientation_variance[0] = cov_rot_ned(0,0);
      px4_odom.orientation_variance[1] = cov_rot_ned(1,1);
      px4_odom.orientation_variance[2] = cov_rot_ned(2,2);

      if(feed_twist_to_px4_){
        px4_odom.velocity_variance[0] = cov_vel_frd(0,0);
        px4_odom.velocity_variance[1] = cov_vel_frd(1,1);
        px4_odom.velocity_variance[2] = cov_vel_frd(2,2);
      }
      else{
        px4_odom.velocity_variance[0] = NAN;
        px4_odom.velocity_variance[1] = NAN;
        px4_odom.velocity_variance[2] = NAN;
      }
      

      // Header
      // It seems that time is updated automatically on PX4 side
      // after v1.14
      // px4_odom.timestamp = 0; //static_cast<uint64_t>(msg.header.stamp.sec*1e6) + static_cast<uint64_t>(msg.header.stamp.nanosec/1e3);
      // px4_odom.timestamp_sample = 0;

      //from vinsco
      px4_odom.timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
       
      return px4_odom;
    }

    nav_msgs::msg::Odometry rotate_ros_odom_child(const  nav_msgs::msg::Odometry & ros_odom){
      //rotate ros odom from odom child frame to /base_link, leave parent frame unchanged
      nav_msgs::msg::Odometry ros_odom_out;

      Eigen::Matrix4d T_b_c;
      
      try {
        // auto Tf_b_c = tf_buffer_->lookupTransform("base_link", ros_odom.child_frame_id , tf2::TimePointZero);
        auto Tf_b_c = tf_buffer_->lookupTransform("base_link", ros_odom.child_frame_id, rclcpp::Time(0),100ms);
        
        T_b_c = utilities::T_from_tf(Tf_b_c); // TODO make global and do only until first odom
        // cout<<"get TF base_link ->child : \n"<<T_b_c<<"\n\n";
        
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      }
      

      utilities::Vector6d v_c_c = utilities::twits_from_odom(ros_odom);
      Eigen::Matrix4d T_o_c = utilities::T_from_odom(ros_odom);

      utilities::Vector6d v_b_b = utilities::rotate_twist(v_c_c,T_b_c);

      Eigen::Matrix4d T_c_b = utilities::T_inverse(T_b_c);
      Eigen::Matrix4d T_o_b = T_o_c*T_c_b;   
      
      utilities::Matrix6d cov_vel_c = utilities::cov_to_mat(ros_odom.pose.covariance);
      utilities::Matrix6d cov_vel_b = utilities::rotate_twist_cov(cov_vel_c, T_b_c);

      utilities::Matrix6d cov_pos_c = utilities::cov_to_mat(ros_odom.pose.covariance);
      utilities::Matrix6d cov_pos_b = utilities::rotate_pose_cov(cov_pos_c, T_c_b);

      utilities::odom_from_tf_and_twist(ros_odom_out, T_o_b, v_b_b);

      utilities::mat_to_cov(ros_odom_out.twist.covariance, cov_vel_b );
      utilities::mat_to_cov(ros_odom_out.pose.covariance, cov_pos_b );


      // Header
      ros_odom_out.header.stamp = this->get_clock()->now();
      ros_odom_out.header.frame_id = ros_odom.header.frame_id;
      ros_odom_out.child_frame_id = "base_link"; 

      return ros_odom_out;
    }

    nav_msgs::msg::Odometry rotate_ros_odom_parent(const  nav_msgs::msg::Odometry & ros_odom){
      //rotate ros odom parent frame to add offset from slam /map ->/odom , rotate also pose cov expressing in map frame, 
      //twist in wrt body frame so it will be not changed
      nav_msgs::msg::Odometry ros_odom_out = ros_odom;

      Eigen::Matrix4d T_m_o;
      
      try {
        auto Tf_m_o = tf_buffer_->lookupTransform(vio_desired_parent_frame_id_, ros_odom.header.frame_id, rclcpp::Time(0),100ms);
        
        T_m_o = utilities::T_from_tf(Tf_m_o); // TODO make global and do only until first odom
        // cout<<"get TF base_link ->child : \n"<<T_b_c<<"\n\n";
        
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      }

      if(T_m_o == Eigen::Matrix4d::Identity()){
        RCLCPP_WARN(this->get_logger(), "tf %s->%s is Identity",vio_desired_parent_frame_id_.c_str(),ros_odom.header.frame_id.c_str());
      }

      
      Eigen::Matrix4d T_o_c = utilities::T_from_odom(ros_odom);

      Eigen::Matrix4d T_m_c = T_m_o*T_o_c;   
      utilities::Matrix6d cov_pos_o = utilities::cov_to_mat(ros_odom.pose.covariance);
      utilities::Matrix6d cov_pos_m = utilities::rotate_pose_cov(cov_pos_o, T_m_o);

      utilities::odom_from_tf(ros_odom_out, T_m_c);
      utilities::mat_to_cov(ros_odom_out.pose.covariance, cov_pos_m );

      // Header
      ros_odom_out.header.stamp = this->get_clock()->now();
      ros_odom_out.header.frame_id = vio_desired_parent_frame_id_;
      ros_odom_out.child_frame_id = ros_odom.child_frame_id; 

      return ros_odom_out;
    }

       
    void px4_odom_out_cb(const px4_msgs::msg::VehicleOdometry::UniquePtr msg){
      
      // std::cout << "============================="     							<< std::endl; //TEST
      // std::cout << "\n\nRECEIVED Vehicle ODOMETRY  DATA"   					<< std::endl; //TEST
      // std::cout << "ts: "         				<< msg->timestamp   			<< std::endl; //TEST

      px4_msgs::msg::VehicleOdometry msgdata = *msg;
      nav_msgs::msg::Odometry ros_odom = transform_px4_odometry_to_ros(msgdata);
      geometry_msgs::msg::TransformStamped tf_odom;

      // nav_msgs::msg::Odometry ros_odom_cov = overwrite_covariance(ros_odom); //TEST
      // px4_odometry_out_pub_->publish(ros_odom_cov); //TEST

      px4_odometry_out_pub_->publish(ros_odom); 
      if(publish_tf_){
        utilities::tf_from_odom(tf_odom,ros_odom);
        tf_broadcaster_->sendTransform(tf_odom);
      }
      
      /* TODO 
        1) publish tf carrefully selecting frames
              if /odom -> /base_link check that is not the same of the position given in input to px4, that is actually /map -> /baselink if achived point 2) in convert_from_ros()
              otherwise the needing is to publish /map->/baselink   and leave slam publishing /map->/odom wihouth complete that branch of tf tree (will not be any /odom ->/baselink)
      */
       
    }


    nav_msgs::msg::Odometry transform_px4_odometry_to_ros(const  px4_msgs::msg::VehicleOdometry& px4_odom) { //custom
      nav_msgs::msg::Odometry ros_odom;

      //if frame convention is non NED for both, is needed co compute other transform equations
      if(px4_odom.pose_frame!=px4_odom.POSE_FRAME_NED || px4_odom.velocity_frame!=px4_odom.VELOCITY_FRAME_NED){
        RCLCPP_ERROR(this->get_logger(), "wrong frame convention in odom from px4, cannot transform");
        return ros_odom;
      }

      //position
      Eigen::Vector3d p_ned(px4_odom.position[0], px4_odom.position[1], px4_odom.position[2]);
      Eigen::Vector3d p_enu = utilities::R_enu_ned*p_ned;
      ros_odom.pose.pose.position.x = p_enu.x();
      ros_odom.pose.pose.position.y = p_enu.y();
      ros_odom.pose.pose.position.z = p_enu.z();

      //attitude
      Eigen::Vector4d q_ned_frd(px4_odom.q[0], px4_odom.q[1], px4_odom.q[2], px4_odom.q[3]);
      Eigen::Matrix3d R_ned_frd = utilities::QuatToMat(q_ned_frd);
      Eigen::Matrix3d R_enu_flu = utilities::R_enu_ned*R_ned_frd*utilities::R_frd_flu;
      Eigen::Vector4d q_enu_flu = utilities::rot2quat(R_enu_flu);
      q_enu_flu = q_enu_flu/q_enu_flu.norm();
      ros_odom.pose.pose.orientation.w = q_enu_flu[0];
      ros_odom.pose.pose.orientation.x = q_enu_flu[1];
      ros_odom.pose.pose.orientation.y = q_enu_flu[2];
      ros_odom.pose.pose.orientation.z = q_enu_flu[3];

      //velocity
      Eigen::Vector3d v_ned(px4_odom.velocity[0], px4_odom.velocity[1], px4_odom.velocity[2]);
      Eigen::Vector3d v_flu = utilities::R_flu_frd*R_ned_frd.transpose()*v_ned;
      ros_odom.twist.twist.linear.x = v_flu.x();
      ros_odom.twist.twist.linear.y = v_flu.y();
      ros_odom.twist.twist.linear.z = v_flu.z();

      //angular velocity
      Eigen::Vector3d w_frd(px4_odom.angular_velocity[0], px4_odom.angular_velocity[1], px4_odom.angular_velocity[2]);
      Eigen::Vector3d w_flu = utilities::R_flu_frd*w_frd;
      ros_odom.twist.twist.angular.x = w_flu.x();
      ros_odom.twist.twist.angular.y = w_flu.y();
      ros_odom.twist.twist.angular.z = w_flu.z();

      //covariance
      Eigen::Matrix3d cov_pos_ned = Eigen::Vector3d(px4_odom.position_variance[0], px4_odom.position_variance[1], px4_odom.position_variance[2]).asDiagonal();
      Eigen::Matrix3d cov_vel_ned = Eigen::Vector3d(px4_odom.velocity_variance[0], px4_odom.velocity_variance[1], px4_odom.velocity_variance[2]).asDiagonal();
      Eigen::Matrix3d cov_rot_ned = Eigen::Vector3d(px4_odom.orientation_variance[0], px4_odom.orientation_variance[1], px4_odom.orientation_variance[2]).asDiagonal();
      Eigen::Matrix3d R = utilities::R_enu_ned;
      Eigen::Matrix3d cov_pos_enu = R*cov_pos_ned*R.transpose();
      R = utilities::R_flu_frd*R_ned_frd.transpose();
      Eigen::Matrix3d cov_vel_flu = R*cov_vel_ned*R.transpose();
      R = utilities::R_enu_ned*R_ned_frd*utilities::R_frd_flu;
      Eigen::Matrix3d cov_rot_enu = R*cov_rot_ned*R.transpose();
      double ang_vel_cov = 1e-6;
      ros_odom.pose.covariance = {
          cov_pos_enu(0,0), cov_pos_enu(0,1), cov_pos_enu(0,2), 0., 0., 0.,
          cov_pos_enu(1,0), cov_pos_enu(1,1), cov_pos_enu(1,2), 0., 0., 0.,
          cov_pos_enu(2,0), cov_pos_enu(2,1), cov_pos_enu(2,2), 0., 0., 0.,
          0., 0., 0., cov_rot_enu(0,0), cov_rot_enu(0,1), cov_rot_enu(0,2),
          0., 0., 0., cov_rot_enu(1,0), cov_rot_enu(1,1), cov_rot_enu(1,2),
          0., 0., 0., cov_rot_enu(2,0), cov_rot_enu(2,1), cov_rot_enu(2,2)
      };
      ros_odom.twist.covariance = {
          cov_vel_flu(0,0), cov_vel_flu(0,1), cov_vel_flu(0,2), 0., 0., 0.,
          cov_vel_flu(1,0), cov_vel_flu(1,1), cov_vel_flu(1,2), 0., 0., 0.,
          cov_vel_flu(2,0), cov_vel_flu(2,1), cov_vel_flu(2,2), 0., 0., 0.,
          0., 0., 0., ang_vel_cov, 0.,          0.,
          0., 0., 0., 0.,          ang_vel_cov, 0.,
          0., 0., 0., 0.,          0.,          ang_vel_cov
      };

      // Header
      // ros_odom.header.stamp = rclcpp::Time(px4_odom.timestamp); // TODO check timing conversion stuff
      ros_odom.header.stamp =this->get_clock()->now();
      ros_odom.header.frame_id = px4_odom_frame_id_;
      ros_odom.child_frame_id = "base_link";   

      return ros_odom;
    }

    

    nav_msgs::msg::Odometry overwrite_covariance(nav_msgs::msg::Odometry ros_odom){
      double cov_lin = 0.1;
      double cov_ang = 0.001;

      ros_odom.pose.covariance = {
          cov_lin, 0., 0., 0., 0., 0.,
          0., cov_lin, 0., 0., 0., 0.,
          0., 0., cov_lin, 0., 0., 0.,
          0., 0., 0., cov_ang, 0., 0.,
          0., 0., 0., 0., cov_ang, 0.,
          0., 0., 0., 0., 0.,cov_ang
      };
      ros_odom.twist.covariance = {
          cov_lin, 0., 0., 0., 0., 0.,
          0., cov_lin, 0., 0., 0., 0.,
          0., 0., cov_lin, 0., 0., 0.,
          0., 0., 0., cov_ang, 0., 0.,
          0., 0., 0., 0., cov_ang, 0.,
          0., 0., 0., 0., 0., cov_ang
      };
      return ros_odom;
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4TfPublisher>());
  rclcpp::shutdown();
  return 0;
}