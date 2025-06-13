#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "px4_ros_com/frame_transforms.h"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Core>
#include "utils.h"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace px4_ros_com::frame_transforms; //TODO remove

using std::placeholders::_1;

class Px4TfPublisher : public rclcpp::Node
{
  public:
    Px4TfPublisher(): Node("px4_tf_pub"){

      //read params
      this->declare_parameter<string>("px4_odom_frame_id", "odom"); //tf_pub_parent_frame (/odom if not slam, /map if slam) TBD
      px4_odom_frame_id_ = this->get_parameter("px4_odom_frame_id").as_string();

      this->declare_parameter<bool>("publish_tf", false);
      publish_tf_ = this->get_parameter("publish_tf").as_bool();

      this->declare_parameter<bool>("feed_twist_to_px4", true);
      feed_twist_to_px4_ = this->get_parameter("feed_twist_to_px4").as_bool();
      

      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

      // px4 odometry from autopilot, to be relied into ros and published as tf
      vehicle_odometry_out_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos, std::bind(&Px4TfPublisher::px4_odom_out_cb, this, _1));
      px4_odometry_out_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/px4/odometry/out", qos);

       // ros odometry from companion pc, to be relied to px4
      companion_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/zed/zed_node/odom", qos, std::bind(&Px4TfPublisher::companion_odom_cb, this, _1));
      vehicle_visual_odometry_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", qos); // /fmu/in/vehicle_mocap_odometry or /fmu/in/vehicle_visual_odometry
      
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      
      /* TODO
        1) add tf broadcaster
        2) add parameters for : tf_pub_parent_frame (/odom if not slam, /map if slam TBD)
        
      */
      

      test_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry_rotated", qos);
    }

  private:

    void companion_odom_cb(const nav_msgs::msg::Odometry::UniquePtr msg){

      std::cout << "============================="     							<< std::endl;
      std::cout << "\n\nRECEIVED Companion ODOMETRY  DATA"   					<< std::endl;
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
      

      nav_msgs::msg::Odometry odom_rot = rotate_ros_odom_child(*msg); //change from  chilt to base_link


      nav_msgs::msg::Odometry odom_map = rotate_ros_odom_parent(odom_rot); //change wrt odom to wrt map

      test_pub_->publish(odom_rot);

      // px4_msgs::msg::VehicleOdometry px4_odom = convert_from_ros(*odom_rot);
      // vehicle_visual_odometry_pub_->publish(px4_odom); 


      
      //todo first_odom = true;
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
      nav_msgs::msg::Odometry ros_odom_out;

      Eigen::Matrix4d T_m_o;
      
      try {
        auto Tf_m_o = tf_buffer_->lookupTransform("map", ros_odom.header.frame_id, rclcpp::Time(0),100ms);
        
        T_m_o = utilities::T_from_tf(Tf_m_o); // TODO make global and do only until first odom
        // cout<<"get TF base_link ->child : \n"<<T_b_c<<"\n\n";
        
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      }

      if(T_m_o == Eigen::Matrix4d::Identity()){
        RCLCPP_WARN(this->get_logger(), "tf map->odom is Identity");
      }

      
      Eigen::Matrix4d T_o_c = utilities::T_from_odom(ros_odom);

      Eigen::Matrix4d T_m_c = T_m_o*T_o_c;   
      utilities::Matrix6d cov_pos_o = utilities::cov_to_mat(ros_odom.pose.covariance);
      utilities::Matrix6d cov_pos_m = utilities::rotate_pose_cov(cov_pos_o, T_m_o);

      utilities::odom_from_tf(ros_odom_out, T_m_c);
      utilities::mat_to_cov(ros_odom_out.pose.covariance, cov_pos_m );

      // Header
      ros_odom_out.header.stamp = this->get_clock()->now();
      ros_odom_out.header.frame_id = "map";
      ros_odom_out.child_frame_id = ros_odom.child_frame_id; 

      return ros_odom_out;
    }

    // px4_msgs::msg::VehicleOdometry convert_from_ros(const  nav_msgs::msg::Odometry & ros_odom){

    //   px4_msgs::msg::VehicleOdometry px4_odom;

  

    //     //1)

    //   if(ros_odom.child_frame_id != "base_link"){ //&& !first_odom_
    //     try {
    //       auto Tf_b_c = tf_buffer_->lookupTransform("base_link", ros_odom.child_frame_id, tf2::TimePointZero);
    //       Eigen::Matrix4d T_b_c = utilities::T_from_tf(Tf_b_c); // TODO make global and do only until first odom
          
    //     } catch (tf2::TransformException &ex) {
    //       RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    //     }
    //   }

    //   utilities::Vector6d v_c_c = utilities::twits_from_odom(ros_odom);
    //   Eigen::Matrix4d T_o_c = utilities::T_from_odom(ros_odom);

    //   utilities::Vector6d v_b_b = utilities::rotate_twist(v_c_c,T_b_c);

    //   Eigen::Matrix4d T_o_b = T_o_c*T_b_c;

    //   return px4_odom;
    // }


    
    void px4_odom_out_cb(const px4_msgs::msg::VehicleOdometry::UniquePtr msg){
      
      std::cout << "============================="     							<< std::endl; //TEST
      std::cout << "\n\nRECEIVED Vehicle ODOMETRY  DATA"   					<< std::endl; //TEST
      std::cout << "ts: "         				<< msg->timestamp   			<< std::endl; //TEST

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

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_out_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr px4_odometry_out_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr test_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr companion_odometry_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_visual_odometry_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    //params
    std::string px4_odom_frame_id_;
    bool publish_tf_;
    bool feed_twist_to_px4_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4TfPublisher>());
  rclcpp::shutdown();
  return 0;
}