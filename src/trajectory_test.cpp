#include <memory>

#include "rclcpp/rclcpp.hpp"
// #include "boost/thread.hpp"
#include <thread>
#include "std_msgs/msg/string.hpp"
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp"

#include <Eigen/Core>
#include "utils.h"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


using std::placeholders::_1;

class TrajectoryTest : public rclcpp::Node
{
  private:
        
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr trajectory_setpoint_ros_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr sp_pub_timer_;

    
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint traj_msg_;

    utilities::Vector6d x_m_w_; /*measured pose in world frame*/
    //trajectory planner
    Eigen::Vector4d ref_traj_pos_; /*x y z yaw*/ //TODO add pitch
    Eigen::Vector4d ref_traj_vel_;
    Eigen::Vector4d ref_traj_acc_;
    utilities::Vector6d traj_goal_pose_; /*measured pose in world frame*/
   
    bool first_odom_pose_;
    bool offboard_enabled_;

    double traj_goal_time_;
    bool new_traj_;
    bool new_traj_is_takeoff_;
    bool traj_ended_;
    bool abort_trajectory_;
    bool traj_aborted_;

    double par_vel_angular_;
    double par_vel_linear_;
    double par_t_min_traj_;

    std::thread traj_planner_th;
    std::thread user_menu_th_;

  public:
    TrajectoryTest(): Node("trajectory_test"){

      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/px4/odometry/out", qos, std::bind(&TrajectoryTest::odom_cb, this, _1));
      trajectory_setpoint_ros_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>("/px4/trajectory_setpoint_enu", qos);

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
      //TODO wait here for first odom ? 
      sp_pub_timer_ = this->create_wall_timer(20ms, std::bind(&TrajectoryTest::sp_pub_timer_cb, this)); //loop @ 50 Hz

      //prepare message structure
      geometry_msgs::msg::Transform transform;
      geometry_msgs::msg::Twist velocity;
      geometry_msgs::msg::Twist acceleration;
      traj_msg_.transforms.push_back(transform);
      traj_msg_.velocities.push_back(velocity);
      traj_msg_.accelerations.push_back(acceleration);

    //   first_odom_pose_ = false;
    //   abort_trajectory_ = false;

      par_vel_angular_ = 0.1; //TODO should be params
      par_vel_linear_ = 0.3;
      par_t_min_traj_ = 0.1;

    //   boost::thread traj_plan_t( &TrajectoryTest::traj_planner_task, this);

      traj_planner_th = std::thread(&TrajectoryTest::traj_planner_task, this);

      user_menu_th_ = std::thread(&TrajectoryTest::user_menu_task, this);

    //   user_menu_task
      
    }

  private:

    void odom_cb(nav_msgs::msg::Odometry::UniquePtr msg){

        Eigen::Vector3d rpy = utilities::R2XYZ ( utilities::QuatToMat ( Eigen::Vector4d( msg->pose.pose.orientation.w,  msg->pose.pose.orientation.x,  msg->pose.pose.orientation.y,  msg->pose.pose.orientation.z) ) );
        x_m_w_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,rpy(0),rpy(1),rpy(2);
        first_odom_pose_ = true;

    }

    void sp_pub_timer_cb(){ //loop @ 50 Hz
      
        traj_msg_.transforms[0].translation.x = ref_traj_pos_(0); 
        traj_msg_.transforms[0].translation.y = ref_traj_pos_(1); 
        traj_msg_.transforms[0].translation.z = ref_traj_pos_(2);
            
        Eigen::Vector4d ref_q = utilities::rot2quat(utilities::XYZ2R(Eigen::Vector3d(0.0, 0.0, ref_traj_pos_(3))));

        traj_msg_.transforms[0].rotation.w = ref_q(0);
        traj_msg_.transforms[0].rotation.x = ref_q(1);
        traj_msg_.transforms[0].rotation.y = ref_q(2);
        traj_msg_.transforms[0].rotation.z = ref_q(3);

        traj_msg_.velocities[0].linear.x = ref_traj_vel_(0); 
        traj_msg_.velocities[0].linear.y = ref_traj_vel_(1); 
        traj_msg_.velocities[0].linear.z = ref_traj_vel_(2);
        traj_msg_.velocities[0].angular.z = ref_traj_vel_(3);

        traj_msg_.accelerations[0].linear.x = ref_traj_acc_(0); 
        traj_msg_.accelerations[0].linear.y = ref_traj_acc_(1); 
        traj_msg_.accelerations[0].linear.z = ref_traj_acc_(2);


        // if(_open_x_pos_loop){
        //     traj_msg_.transforms[0].translation.x = _x_m_w(0);
        //     //TBD
        // }


      trajectory_setpoint_ros_pub_->publish(traj_msg_);

    }


    void traj_compute(Eigen::Vector4d p_i, Eigen::Vector4d p_f,double T,double vel_i_ff =0.0, double acc_i_ff =0.0){
        rclcpp::Rate rate(100.0);
        double dt = 1.0/100.0;
        Eigen::Matrix<double,6,6> A;
        Vector4d e = p_f-p_i;
        e(3) = utilities::angleError(p_f(3),p_i(3));
        double s_f = e.norm(); //arclength;
        Eigen::VectorXd b(6);
        b << 0.0, vel_i_ff, acc_i_ff, s_f, 0.0, 0.0; //qi qi_d qi_dd qf qf_d qf_dd : initial final pos vel acc
        A << 0,           0,           0,          0,        0,  1,
             0,           0,           0,          0,        1,  0,
             0,           0,           0,          1,        0,  0,
             pow(T,5),    pow(T,4),    pow(T,3),   pow(T,2), T,  1,
             5*pow(T,4),  4*pow(T,3),  3*pow(T,2), 2*T,      1,  0,
             20*pow(T,3), 12*pow(T,2), 6*T,        1,        0,  0;
     
        Eigen::VectorXd x = A.inverse()*b;
        double s, s_d, s_dd;
        double t = 0.0;
        int N = T/dt;
        int i =0;
        // double t_0 =ros::Time::now().toSec();
        while(rclcpp::ok() && i <=N && !abort_trajectory_){
            //t = ros::Time::now().toSec()-t_0;
            s    = x(0)*pow(t,5)    +x(1)*pow(t,4)    +x(2)*pow(t,3)   +x(3)*pow(t,2) +x(4)*t +x(5);
            s_d  = x(0)*5*pow(t,4)  +x(1)*4*pow(t,3)  +x(2)*3*pow(t,2) +x(3)*2*t      +x(4);
            s_dd = x(0)*20*pow(t,3) +x(1)*12*pow(t,2) +x(2)*6*t        +x(3);
            
            ref_traj_pos_ = p_i + s*e/s_f;
            ref_traj_vel_ = s_d*e/s_f;
            ref_traj_acc_ = s_dd*e/s_f;

            i++;
            t+=dt;
            rate.sleep();
        }   
        if(abort_trajectory_){
            RCLCPP_ERROR(this->get_logger(), "trajectory aborted");
            ref_traj_vel_<<0,0,0,0;
            ref_traj_acc_<<0,0,0,0;
            traj_aborted_ = true;
        }
    }


    void traj_planner_task(void){

        rclcpp::Rate rate(50.0); //NB prima era 1Hz
        while(! first_odom_pose_){
            rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "[traj_planner_th] got first odom pose");
    
        traj_aborted_ = false;
    
        while(! offboard_enabled_){
            traj_goal_pose_ = x_m_w_;
            traj_goal_time_ = 0;
            ref_traj_pos_ <<x_m_w_(0), x_m_w_(1), x_m_w_(2), x_m_w_(5);
            ref_traj_vel_ <<0,0,0,0;
            ref_traj_acc_ <<0,0,0,0;
            rate.sleep();
        }

        new_traj_ = false;
        traj_ended_ = true;
        traj_aborted_ = false;
    
        while(rclcpp::ok()){
    
            if(! offboard_enabled_){ //TODO
                traj_goal_pose_ = x_m_w_;
                traj_goal_time_ = 0;
                ref_traj_pos_ <<x_m_w_(0), x_m_w_(1), x_m_w_(2), x_m_w_(5);
                ref_traj_vel_ <<0,0,0,0;
                ref_traj_acc_ <<0,0,0,0;
                rate.sleep();
            }
            
            
            if(new_traj_){
                new_traj_ = false;
                traj_ended_ = false;
                abort_trajectory_ = false;
                traj_aborted_ = false;
                Eigen::Vector4d start_point;
                double vel_ff = 0.00;
                double acc_ff = 0.00;
                //start_point <<_x_m_w(0), _x_m_w(1), _x_m_w(2), _x_m_w(5);
                start_point = ref_traj_pos_;  //TODO test run from prev trajectory end ( while in offboard)
                
                traj_compute(start_point, Eigen::Vector4d(traj_goal_pose_(0),traj_goal_pose_(1),traj_goal_pose_(2),traj_goal_pose_(5)), traj_goal_time_,vel_ff, acc_ff);
                traj_ended_ = true;
                
            }
            rate.sleep();
        }
    
    }

    int plan_new_trajectory( Eigen::Vector3d pos_goal, double yaw_goal,double vel_linear){
        if(traj_ended_){
            Eigen::Vector3d pos_now;
            //pos_now <<_x_m_w(0), _x_m_w(1), _x_m_w(2);
            pos_now <<ref_traj_pos_(0),ref_traj_pos_(1),ref_traj_pos_(2);
            //double yaw_now =  _x_m_w(5);
            double yaw_now = ref_traj_pos_(3);
            double delta_t_lin = (pos_goal - pos_now).norm()/vel_linear;
            //double delta_t_ang = fabs(yaw_goal -yaw_now)/_approach_vel_angular;  
            double delta_t_ang = fabs(utilities::angleError(yaw_goal,yaw_now))/par_vel_angular_; 
            //cout<<"linear time: "<<delta_t_lin<<endl;
            //cout<<"ang time: "<< delta_t_ang<<endl;
            traj_goal_time_ = std::max(par_t_min_traj_,( delta_t_lin > delta_t_ang) ? delta_t_lin : delta_t_ang);
    
            //cout<<"traj time: "<< _traj_goal_time<<endl;
            RCLCPP_INFO(this->get_logger(),"traj time: %f",traj_goal_time_);
            traj_goal_pose_(0) = pos_goal(0);
            traj_goal_pose_(1) = pos_goal(1);
            traj_goal_pose_(2) = pos_goal(2);
            traj_goal_pose_(5) = yaw_goal;
            new_traj_ = true;
            traj_ended_ = false;
            abort_trajectory_ = false;
            traj_aborted_ = false;
            return 1;
        }
        else return -1;
    }


    void go_offboard(){   //TODO call px4 msgs
        RCLCPP_INFO(this->get_logger(),"going offboard");
        offboard_enabled_= true;
    }

    void user_menu_task() {
        rclcpp::Rate rate(1);  // Frequenza 1 Hz
        char input = 0;
    
        while ( rclcpp::ok()) {
            // Stampa il menu
            std::cout << "\n=== MENU PRINCIPALE ===" << std::endl;
            std::cout << "g: Inserisci nuovo goal (x, y, z, yaw)" << std::endl;
            std::cout << "o: Attiva modalità offboard" << std::endl;
            std::cout << "q: Esci dal programma" << std::endl;
            std::cout << "Scelta: ";
            std::cout.flush();  // Forza l'output
    
            // Lettura input non bloccante
            if (std::cin.peek() != EOF) {
                std::cin >> input;
                
                switch(input) {
                    case 'g': {
                        Eigen::Vector3d xyz_goal;
                        double yaw_goal;
                        std::cout << "Inserisci x y z yaw: ";
                        if (std::cin >> xyz_goal[0] >> xyz_goal[1] >> xyz_goal[2] >> yaw_goal) {
                            std::cout << "Nuovo goal impostato: "
                                      << xyz_goal[0] << ", "
                                      << xyz_goal[1] << ", "
                                      << xyz_goal[2] << ", "
                                      << yaw_goal << std::endl;
    
                            plan_new_trajectory(xyz_goal,yaw_goal,par_vel_linear_);
                        } else {
                            std::cin.clear();  // Reset errori di input
                            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                            std::cout << "Input non valido!" << std::endl;
                        }
                        break;
                    }
                    case 'o':
                        std::cout << "going offboard..." << std::endl;
                        go_offboard();
                        break;
                    case 'q':
                        std::cout << "Uscita richiesta..." << std::endl;
                        break;
                    default:
                        std::cout << "Comando non riconosciuto!" << std::endl;
                        break;
                }
                
                // Pulisce il buffer di input
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            }
            
            rate.sleep();  // Mantiene la frequenza 1Hz
        }
    }
   

};






int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryTest>());
  rclcpp::shutdown();
  return 0;
}


/*

void traj_control::sp_publisher_task(void)
{
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);
    double dt = 1.0f/50.0f;
    
    mavros_msgs::PositionTarget ptarget;
    mavros_msgs::DebugValue debug_value_msg;

    debug_value_msg.type = mavros_msgs::DebugValue::TYPE_NAMED_VALUE_FLOAT;//3
    debug_value_msg.value_float = 0.0f;

    
   
    uint16_t mask_tracking =0;
    // uint16_t mask_position = 
    //     //mavros_msgs::PositionTarget::IGNORE_PX |
    //     //mavros_msgs::PositionTarget::IGNORE_PY |
    //     //mavros_msgs::PositionTarget::IGNORE_PZ ;
    //     mavros_msgs::PositionTarget::IGNORE_VX |
    //     mavros_msgs::PositionTarget::IGNORE_VY |
    //     mavros_msgs::PositionTarget::IGNORE_VZ |
    //     mavros_msgs::PositionTarget::IGNORE_AFX |
    //     mavros_msgs::PositionTarget::IGNORE_AFY |
    //     mavros_msgs::PositionTarget::IGNORE_AFZ ;//|
    //     //mavros_msgs::PositionTarget::FORCE |
    //     mavros_msgs::PositionTarget::IGNORE_YAW_RATE; // |

    ptarget.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    ptarget.type_mask = mask_tracking;

    //set local position feedback rate at 50hz
    if(_set_local_position_freq){
        mavros_msgs::MessageInterval msg_int;
        msg_int.request.message_id =32; //LOCAL_POSITION_NED (32)
        msg_int.request.message_rate =50.0f;
        if(_mavros_msginterval_client.call(msg_int) && msg_int.response.success){
            ROS_INFO("set mavros local_position at 50 hz");
        }
        else ROS_ERROR(" failed set mavros local_position at 50 hz");
    }
    

    while(! _first_meas_pose){
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
     
            
         publish mavros setpoint 
        ptarget.header.stamp = ros::Time::now();
        ptarget.position.x = _ref_traj_pos(0); //trajectory position
        ptarget.position.y = _ref_traj_pos(1);
        ptarget.position.z = _ref_traj_pos(2); 

        ptarget.velocity.x = _ref_traj_vel(0);//trajectory velocity
        ptarget.velocity.y = _ref_traj_vel(1);
        ptarget.velocity.z = _ref_traj_vel(2);

        ptarget.acceleration_or_force.x = _ref_traj_acc(0);//trajectory aceleration
        ptarget.acceleration_or_force.y = _ref_traj_acc(1);
        ptarget.acceleration_or_force.z = _ref_traj_acc(2);

        ptarget.yaw = _ref_traj_pos(3);      //trajectory position
        ptarget.yaw_rate = _ref_traj_vel(3); //trajectory velocity
        debug_value_msg.value_float = _ref_traj_pitch;


        if(_open_y_pos_loop){
            ptarget.position.y = _x_m_w(1);
            
        }
        if(_open_x_pos_loop){
            ptarget.position.x = _x_m_w(0); //current fb
            
        }
        if(_ena_Delta_z){
            ptarget.position.z = _x_m_w(2)+_delta_z_open; 
            ptarget.velocity.z = _perching_vel_linear;
        }
        if(_open_yaw_pos_loop){
            ptarget.yaw = _x_m_w(5); //current fb
        }

        // if(_open_pitch_pos_loop && _offboard_enabled){
        //     debug_value_msg.value_float = _x_m_w(4); //current fb
        // }
        // else{
        //     debug_value_msg.value_float = 0.0f;
        // }

        // debug_value_msg.value_float = _x_m_w(4); //current fb

        if(_mandatory_armed && !_mavros_state.armed){
            ROS_ERROR("detect disarm while mandatory_armed, arm...");
            arm();
        }
        
        _pitch_sp_pub.publish(debug_value_msg);
        _local_pos_pub.publish(ptarget);

        rate.sleep();      
    }
}



* TRAJCETORY PLANNER


void traj_control::traj_planner_task(void){

    ros::Rate rate(50.0); //NB prima era 1Hz
    while(! _first_meas_pose){
        ros::spinOnce();
        rate.sleep();
    }

    _traj_aborted = false;

    while(! _offboard_enabled){
        _traj_goal_pose = _x_m_w;
        _traj_goal_time = 0;
        _ref_traj_pos <<_x_m_w(0), _x_m_w(1), _x_m_w(2), _x_m_w(5);
        _ref_traj_vel<<0,0,0,0;
        _ref_traj_acc<<0,0,0,0;
        rate.sleep();
    }
    _new_traj = false;
    _new_traj_is_takeoff = false;
    _traj_ended = true;
    _traj_aborted = false;

    while(ros::ok()){

        if(! _offboard_enabled){ //TODO
            _traj_goal_pose = _x_m_w;
            _traj_goal_time = 0;
            _ref_traj_pos <<_x_m_w(0), _x_m_w(1), _x_m_w(2), _x_m_w(5);
            _ref_traj_vel<<0,0,0,0;
            _ref_traj_acc<<0,0,0,0;
            rate.sleep();
        }
        
        
        if(_new_traj){
            _new_traj = false;
            _traj_ended = false;
            _abort_trajectory = false;
            _traj_aborted = false;
            Eigen::Vector4d start_point;
            double vel_ff = 0.00;
            double acc_ff = 0.00;
            //start_point <<_x_m_w(0), _x_m_w(1), _x_m_w(2), _x_m_w(5);
            start_point = _ref_traj_pos;  //TODO test run from prev trajectory end ( while in offboard)
            if(_new_traj_is_takeoff){
                //start_point <<_x_m_w(0), _x_m_w(1), _x_m_w(2)+_takeoff_disc_offset, _x_m_w(5); //vecchia impl

                start_point <<_ref_traj_pos(0), _ref_traj_pos(1), _ref_traj_pos(2)+_takeoff_disc_offset, _ref_traj_pos(3); //TODO check
                vel_ff = _vel_linear_takeoff_ff;
                acc_ff = _acc_linear_takeoff_ff;

                //takeoff_compute();

                traj_compute(start_point,
                         Eigen::Vector4d(_traj_goal_pose(0),_traj_goal_pose(1),_traj_goal_pose(2),_traj_goal_pose(5)),
                          _traj_goal_time,vel_ff, acc_ff);
                   _traj_ended = true;

                _new_traj_is_takeoff = false;
                _traj_ended = true;
                
            }
            else{
                traj_compute(start_point,
                         Eigen::Vector4d(_traj_goal_pose(0),_traj_goal_pose(1),_traj_goal_pose(2),_traj_goal_pose(5)),
                          _traj_goal_time,vel_ff, acc_ff);
                   _traj_ended = true;
            }
            
        }
        rate.sleep();
    }

}

void traj_control::takeoff_compute(){ //just step setpoint and ff of
    ros::Rate rate(100.0);

    _ref_traj_pos(2)+= _takeoff_pipe_height;
    _ref_traj_vel(2) = _vel_linear_takeoff_ff;
    _ref_traj_acc(2) = _acc_linear_takeoff_ff;
    double e =0.0;
    bool arrived = false;
    while(ros::ok() && !arrived && ! _abort_trajectory){
        
        e = fabs(_ref_traj_pos(2) - _x_m_w(2));
        // ROS_INFO("takeoff loop, e= %f",e);

        if(e<= 0.03){
             arrived = true;
             ROS_INFO("takeoff arrived");
        }
   
        if(_abort_trajectory){
            _ref_traj_pos(2)  =  _x_m_w(2); //should be impl in trajectory ? may no
            _ref_traj_vel<<0,0,0,0;
            _ref_traj_acc<<0,0,0,0;
        }
     
        rate.sleep();
    }   
    if(_abort_trajectory){
        ROS_ERROR("trajectory aborted");
        _ref_traj_vel<<0,0,0,0;
        _ref_traj_acc<<0,0,0,0;
        _traj_aborted = true;
        
    }
}


void traj_control::traj_compute(Eigen::Vector4d p_i, Eigen::Vector4d p_f,double T,double vel_i_ff =0.0, double acc_i_ff =0.0){
    ros::Rate rate(100.0);
    double dt = 1.0/100.0;
    Eigen::Matrix<double,6,6> A;
    Vector4d e = p_f-p_i;
    e(3) = utilities::angleError(p_f(3),p_i(3));
    double s_f = e.norm(); //arclength;
    Eigen::VectorXd b(6);
    b << 0.0, vel_i_ff, acc_i_ff, s_f, 0.0, 0.0; //qi qi_d qi_dd qf qf_d qf_dd : initial final pos vel acc
    A << 0,           0,           0,          0,        0,  1,
         0,           0,           0,          0,        1,  0,
         0,           0,           0,          1,        0,  0,
         pow(T,5),    pow(T,4),    pow(T,3),   pow(T,2), T,  1,
         5*pow(T,4),  4*pow(T,3),  3*pow(T,2), 2*T,      1,  0,
         20*pow(T,3), 12*pow(T,2), 6*T,        1,        0,  0;
 
    Eigen::VectorXd x = A.inverse()*b;
    double s, s_d, s_dd;
    double t = 0.0;
    int N = T/dt;
    int i =0;
    double t_0 =ros::Time::now().toSec();
    while(ros::ok() && i <=N && !_abort_trajectory){
        //t = ros::Time::now().toSec()-t_0;
        s    = x(0)*pow(t,5)    +x(1)*pow(t,4)    +x(2)*pow(t,3)   +x(3)*pow(t,2) +x(4)*t +x(5);
        s_d  = x(0)*5*pow(t,4)  +x(1)*4*pow(t,3)  +x(2)*3*pow(t,2) +x(3)*2*t      +x(4);
        s_dd = x(0)*20*pow(t,3) +x(1)*12*pow(t,2) +x(2)*6*t        +x(3);
        
        _ref_traj_pos = p_i + s*e/s_f;
        if(_abort_trajectory){
            _ref_traj_vel<<0,0,0,0;
            _ref_traj_acc<<0,0,0,0;
        }
        _ref_traj_vel = s_d*e/s_f;
        _ref_traj_acc = s_dd*e/s_f;
        i++;
        t+=dt;
        rate.sleep();
    }   
    if(_abort_trajectory){
        ROS_ERROR("trajectory aborted");
        _ref_traj_vel<<0,0,0,0;
        _ref_traj_acc<<0,0,0,0;
        _traj_aborted = true;
        
    }
}


int traj_control::plan_new_trajectory( Eigen::Vector3d pos_goal, double yaw_goal,double vel_linear){
    if(_traj_ended ){
        Eigen::Vector3d pos_now;
        //pos_now <<_x_m_w(0), _x_m_w(1), _x_m_w(2);
        pos_now <<_ref_traj_pos(0),_ref_traj_pos(1),_ref_traj_pos(2);
        //double yaw_now =  _x_m_w(5);
        double yaw_now = _ref_traj_pos(3);
        double delta_t_lin = (pos_goal - pos_now).norm()/vel_linear;
        //double delta_t_ang = fabs(yaw_goal -yaw_now)/_approach_vel_angular;  
        double delta_t_ang = fabs(utilities::angleError(yaw_goal,yaw_now))/_approach_vel_angular; 
        //cout<<"linear time: "<<delta_t_lin<<endl;
        //cout<<"ang time: "<< delta_t_ang<<endl;
        _traj_goal_time = std::max(_t_min_traj,( delta_t_lin > delta_t_ang) ? delta_t_lin : delta_t_ang);

        //cout<<"traj time: "<< _traj_goal_time<<endl;
        ROS_INFO("traj time: %f",_traj_goal_time);
        _traj_goal_pose(0) = pos_goal(0);
        _traj_goal_pose(1) = pos_goal(1);
        _traj_goal_pose(2) = pos_goal(2);
        _traj_goal_pose(5) = yaw_goal;
        _new_traj = true;
        _traj_ended= false;
        _abort_trajectory = false;
        _traj_aborted = false;
        return 1;
    }
    else return -1;

}

int traj_control::plan_new_takeoff_up(double height, double vel_linear){ //takeoff quando telecamera giu, sali su !!
    if(_traj_ended ){
        // Eigen::Vector3d pos_now;
        // Eigen::Vector3d pos_goal;
        // pos_goal<<_ref_traj_pos(0), _ref_traj_pos(1), _ref_traj_pos(2)+height;
        // pos_now <<_ref_traj_pos(0), _ref_traj_pos(1), _ref_traj_pos(2);
        // double yaw_now =  _ref_traj_pos(3);
        // double delta_t_lin = (pos_goal - pos_now).norm()/vel_linear;
        // _traj_goal_time =  delta_t_lin ;
        // //cout<<"traj time: "<< _traj_goal_time<<endl;
        // ROS_INFO("traj time: %f",_traj_goal_time);
        // _traj_goal_pose(0) = pos_goal(0);
        // _traj_goal_pose(1) = pos_goal(1);
        // _traj_goal_pose(2) = pos_goal(2);
        // _traj_goal_pose(5) = yaw_now;
        _new_traj = true;
        _new_traj_is_takeoff = true;
        _traj_ended= false;
        _abort_trajectory = false;
        _traj_aborted = false;
        return 1;
    }
    else return -1;

}

*/