
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <cascadePID.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>

using namespace std;
using namespace Eigen;

ros::Publisher control_RPM_pub;
ros::Publisher control_pub;

Vector3d pos_des,vel_des,acc_des;
double yaw_des = 0.0;
cascadePID quad_PID(20);

ros::Time t_init,t2;

nav_msgs::Odometry current_odom, last_odom;
int first_odom_flag = 0;
void Odom_callback(const nav_msgs::Odometry& odom)
{
    if(first_odom_flag == 1)
    {
        last_odom = current_odom;
    }else{
        first_odom_flag = 1;
    }
    current_odom = odom;
    ROS_INFO("current odom received");
}

geometry_msgs::PoseStamped pose_cmd;
int control_flag = 0;
//void cmd_callback(const geometry_msgs::PoseStamped& msg)
// {
//     pose_cmd = msg;
//     control_flag = 1;
// }

void fuel_position_cmd_callback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd) {
  pos_des = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  vel_des = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  acc_des = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);

  yaw_des = cmd->yaw;
}

void run_control(const ros::TimerEvent& event)
{
        if(first_odom_flag == 1) 
        {

        // ROS_INFO("start running controller");

        //run controller
        Vector3d current_euler;
        //获取无人机当前状态：方向、位置、速度
        Eigen::Quaterniond quaternion_temp(current_odom.pose.pose.orientation.w, current_odom.pose.pose.orientation.x\
                                        , current_odom.pose.pose.orientation.y, current_odom.pose.pose.orientation.z);
        Vector3d current_pos, current_vel;
        current_pos << current_odom.pose.pose.position.x, current_odom.pose.pose.position.y, current_odom.pose.pose.position.z;
        current_vel << current_odom.twist.twist.linear.x, current_odom.twist.twist.linear.y, current_odom.twist.twist.linear.z;
        //将观测到的状态，前馈输入转换到无人机的当前状态，作为真实观测量，输入到控制回路中
        quad_PID.FeedbackInput(current_pos, current_vel, quaternion_temp);
        // ROS_INFO("current_pos = %f,%f,%f",current_pos(0),current_pos(1),current_pos(2));
        Eigen::Vector3d eulerAngle_des;
        eulerAngle_des << 0.1,0.1,0.0;
        // quad_PID.setAngledes(eulerAngle_des);

        // ROS_INFO("set des angle");
        //看哪种形式，control_flag=1代表普通cmd控制，如果control_flag == 1为false，代表规划器(FUEL)输出的轨迹点控制-->设定值
        if(control_flag == 1)
        {
            pos_des << pose_cmd.pose.position.x, pose_cmd.pose.position.y, pose_cmd.pose.position.z;
            
            Eigen::Quaterniond q_des(pose_cmd.pose.orientation.w, pose_cmd.pose.orientation.x\
                                        , pose_cmd.pose.orientation.y, pose_cmd.pose.orientation.z);
            Eigen::Matrix3d R_des;
            R_des = q_des.matrix();
            Eigen::Vector3d x_body;
            x_body << 1,0,0;
            x_body = R_des * x_body;

            yaw_des = atan2(x_body(1),x_body(0));//pose_cmd.pose.orientation.z
            t2 = ros::Time::now();
            // yaw_des = 3.1415926;//20.8 * (t2-t_init).toSec()
        }
        
        quad_PID.setOdomdes(pos_des,vel_des,acc_des, yaw_des);//将规划器输出控制指令转换为控制器设定值

        // ROS_INFO("set des odom");

        quad_PID.RunController();

        Eigen::Vector3d Torque_des;
        
        Torque_des = quad_PID.getTorquedes();
        ROS_INFO("Torque_des = %f,%f,%f",Torque_des(0),Torque_des(1),Torque_des(2));
        Vector4d RPM_output;
        RPM_output = quad_PID.getRPMoutputs();
        // ROS_INFO("RPM_output = %f,%f,%f,%f",RPM_output(0),RPM_output(1),RPM_output(2),RPM_output(3));

        std_msgs::Float32MultiArray rpm_array;
        float rpm_data[4];
        rpm_data[0] = RPM_output(0);
        rpm_data[1] = RPM_output(1);
        rpm_data[2] = RPM_output(2);
        rpm_data[3] = RPM_output(3);
        // rpm_array.data = rpm_data;
        rpm_array.data.clear();
        rpm_array.data.push_back(RPM_output(0));
        rpm_array.data.push_back(RPM_output(1));
        rpm_array.data.push_back(RPM_output(2));
        rpm_array.data.push_back(RPM_output(3));

        control_RPM_pub.publish(rpm_array);
        }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cascade_PID");

    ros::NodeHandle n("~");


    double init_x, init_y, init_z;
    double init_yaw;
    double controller_rate,angle_stable_time,damping_ratio;
    int drone_id;
    std::string quad_name;
    n.param("init_state_x", init_x, 0.0);
    n.param("init_state_y", init_y, 0.0);
    n.param("init_state_z", init_z, 1.0);
    n.param("init_state_yaw", init_yaw, 1.0);
    init_yaw = init_yaw / 180.0 * M_PI;
    yaw_des = init_yaw;
    n.param("controller_rate", controller_rate, 200.0);
    n.param("quadrotor_name", quad_name, std::string("quadrotor"));
    n.param("angle_stable_time", angle_stable_time, 0.1);
    n.param("damping_ratio", damping_ratio, 0.7);
    n.param("drone_id", drone_id, 0);

    control_RPM_pub = n.advertise<std_msgs::Float32MultiArray>("cmd_RPM", 100);//发布控制指令到仿真无人机--->旋翼转速控制改为官方位置控制

    control_pub = n.advertise<nav_msgs::Odometry>("position_control",100);//---->待续

    ros::Subscriber odom_sub = n.subscribe("odom", 100, Odom_callback,
                                           ros::TransportHints().tcpNoDelay());

    // ros::Subscriber cmd_sub = n.subscribe("cmd_pose", 100, cmd_callback,
    //                                       ros::TransportHints().tcpNoDelay());

    ros::Subscriber position_cmd_sub_ = n.subscribe("position_cmd", 10, fuel_position_cmd_callback,
                                                    ros::TransportHints().tcpNoDelay());

    quad_PID.setdroneid(drone_id);

    ros::Timer controller_timer = n.createTimer(ros::Duration(1.0/controller_rate), run_control);//控制器时间对象

    Matrix3d Internal_mat;
    Internal_mat << 2.64e-3,0,0,
                    0,2.64e-3,0,
                    0,0,4.96e-3;

    double arm_length = 0.22;
    double k_F = 8.98132e-9;
    k_F = 3.0*k_F;
    double mass = 1.9;

    // cascadePID quad_PID(controller_rate);
    quad_PID.setrate(controller_rate);
    quad_PID.setInternal(mass, Internal_mat, arm_length, k_F);
    quad_PID.setParam(angle_stable_time,damping_ratio);

    ros::Rate rate(controller_rate);
    rate.sleep();

        pos_des << init_x,init_y,init_z;
        vel_des << 0,0,0;
        acc_des << 0,0,0;

    t_init = ros::Time::now();

    ros::spin();

    return 0;
  
}