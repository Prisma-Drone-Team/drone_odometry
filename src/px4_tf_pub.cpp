#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "px4_ros_com/frame_transforms.h"
 #include "nav_msgs/msg/odometry.hpp"
 #include <Eigen/Core>
 #include "utils.h"

 using namespace px4_ros_com::frame_transforms; 

using std::placeholders::_1;

class Px4TfPublisher : public rclcpp::Node
{
  public:
    Px4TfPublisher(): Node("px4_tf_pub"){
      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

      // px4 odometry from autopilot, to be relied into ros and published as tf
      vehicle_odometry_out_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&Px4TfPublisher::px4_odom_out_cb, this, _1));
      px4_odometry_out_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/px4/odometry/out", qos);

       // ros odometry from companion pc, to be relied to px4
      companion_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", qos, std::bind(&Px4TfPublisher::companion_odom_cb, this, _1));
      vehicle_visual_odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", qos); // /fmu/in/vehicle_mocap_odometry or /fmu/in/vehicle_visual_odometry
      
     
      

      // test_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/px4/odometry_custom", qos);
    }

  private:

    void companion_odom_cb(const nav_msgs::msg::Odometry::UniquePtr msg){

      std::cout << "============================="     							<< std::endl;
      std::cout << "\n\nRECEIVED Companion ODOMETRY  DATA"   					<< std::endl;
      // std::cout << "frame: "         				<< msg->header.header.frame 			<< std::endl;

      
      px4_msgs::msg::VehicleOdometry px4_odom = convert_from_ros(*msg);

      vehicle_visual_odometry_pub_->publish(px4_odom); 
      
    }

    px4_msgs::msg::VehicleOdometry convert_from_ros(const  nav_msgs::msg::Odometry & ros_odom){

      px4_msgs::msg::VehicleOdometry px4_odom;

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

      */

      return px4_odom;
    }


    
    void px4_odom_out_cb(const px4_msgs::msg::VehicleOdometry::UniquePtr msg){
      
      std::cout << "============================="     							<< std::endl;
      std::cout << "\n\nRECEIVED Vehicle ODOMETRY  DATA"   					<< std::endl;
      std::cout << "ts: "         				<< msg->timestamp   			<< std::endl;

      px4_msgs::msg::VehicleOdometry msgdata = *msg;
      nav_msgs::msg::Odometry ros_odom = transform_px4_odometry_to_ros(msgdata);



      px4_odometry_out_pub_->publish(ros_odom); 

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
      Eigen::Vector3d position_enu = utilities::R_enu_ned*p_ned;
      ros_odom.pose.pose.position.x = position_enu.x();
      ros_odom.pose.pose.position.y = position_enu.y();
      ros_odom.pose.pose.position.z = position_enu.z();

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
      ros_odom.header.stamp = rclcpp::Time(px4_odom.timestamp); // TODO check timing conversion stuff
      ros_odom.header.frame_id = "map";
      ros_odom.child_frame_id = "base_link";   

      return ros_odom;
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_out_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr px4_odometry_out_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr test_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr companion_odometry_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_visual_odometry_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4TfPublisher>());
  rclcpp::shutdown();
  return 0;
}