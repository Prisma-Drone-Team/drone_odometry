#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "px4_ros_com/frame_transforms.h"
#include "nav_msgs/msg/odometry.hpp"
#include <Eigen/Core>
#include "utils.h"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


#include <sophus/se3.hpp>


using std::placeholders::_1;

class OdomDerivative : public rclcpp::Node
{
  public:
    OdomDerivative(): Node("odom_derivative"){

        this->declare_parameter<string>("odom_topic", "/zed/zed_node/odom"); 
        odom_topic_ = this->get_parameter("odom_topic").as_string();

        if(odom_topic_[0] != '/') odom_topic_= "/" + odom_topic_;

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, qos, std::bind(&OdomDerivative::odom_cb, this, _1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(std::string(odom_topic_+"_derivative"), qos);
        
        first_odom_ = false;
        last_t_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        T_ = T_old_ = Eigen::Matrix4d::Identity();
    }

  private:

    void odom_cb(const nav_msgs::msg::Odometry::UniquePtr msg){

        rclcpp::Time t_(msg->header.stamp);
        T_ = utilities::T_from_odom(*msg);

        RCLCPP_INFO(this->get_logger(), "got odometry at %f", t_.seconds());
        if(first_odom_){
            double dt = (t_ - last_t_).seconds();
            double f = 1/dt;
            RCLCPP_INFO(this->get_logger(), "T = %f     f=%f", dt,f);

            Sophus::SE3d T = Sophus::SE3d::fitToSE3(T_);
            Sophus::SE3d T_old = Sophus::SE3d::fitToSE3(T_old_);

            Sophus::SE3d delta_SE3 = T_old.inverse() * T;
            utilities::Vector6d xi_body = delta_SE3.log() / dt; //twist body

            utilities::Matrix6d Ad_T = T.Adj();
            utilities::Vector6d xi_spatial = Ad_T * xi_body;   //twist fixed

            msg->twist.twist.linear.x = xi_body[0];
            msg->twist.twist.linear.y = xi_body[1];
            msg->twist.twist.linear.z = xi_body[2];
            msg->twist.twist.angular.x = xi_body[3];
            msg->twist.twist.angular.y = xi_body[4];
            msg->twist.twist.angular.z = xi_body[5];

            odom_pub_->publish(*msg);  

        }
        last_t_=t_;
        T_old_ = T_;

        
        
        // nav_msgs::msg::Odometry odom_out;
        

        first_odom_ = true;
    }

    // void odom_twist_from_pos_derivative(const nav_msgs::msg::Odometry odom){

    //     Eigen::Matrix4d T_o_c = utilities::T_from_odom(odom);


        

    // }
    
    

    //global
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    bool first_odom_;

    rclcpp::Time last_t_;
    Eigen::Matrix4d T_, T_old_;
    

    //params
    std::string odom_topic_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomDerivative>());
  rclcpp::shutdown();
  return 0;
}