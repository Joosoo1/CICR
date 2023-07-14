#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <ros/package.h>

using namespace std;
using namespace Eigen;

class cascadePID
{
private:
    double dt;
    Vector3d Pos_des;
    Vector3d Vel_des;
    Vector3d Acc_des;
    Vector3d Angle_Kd,Angle_Kp,Angle_Ki;
    Vector3d Torque_des;
    Vector3d euler_angle,last_euler_angle, angle_des, last_angle_des;
    Vector3d current_pos, current_vel, current_acc;
    Eigen::Quaterniond q;
    Matrix3d R_body2world;
    double Yaw_des, Yaw_cur;
    int angle_in_flag = 0;
    int angledes_in_flag = 0;
    Vector3d Angle_error, Angle_last_error, d_Angle_error, Pos_error, Vel_error;
    int first_control_flag = 0;
    double angle_wn;
    Matrix3d Internal_mat;
    double mass = 1.9;
    double arm_length;
    double k_F = 8.98132e-9;//fixed parameter for calculate motor drag from motor speed
    double k_T = 0.07 * (3 * 0.062) * k_F; //fixed parameter for calculate motor torque from motor speed
    Matrix4d Mixermatrix_rpm2torque;
    MatrixXd motor_pos;
    Vector4d RPM_output, RPM_output_last;
    Vector3d PID_POS_Z, PID_POS_XY;
    double min_rpm = 2000;//10500
    double max_rpm = 35000;//35000
    double g = 9.81;
    double z_intergral_val = 0;
    double z_intergral_limit = 2;
    double RPM_change_limit = 1000;
    double Torque_limit = 20;
    double Torque_limit_yaw = 0.1;
    std::ofstream myfile;
    std::string pkg_path;
    int droneid = 0;

public:
    cascadePID(double control_rate);
    ~cascadePID();
    void setdroneid(int id);
    void setrate(double control_rate);
    void setParam(double stable_time, double damping_ratio);
    void setInternal(double m, Matrix3d Internal_mat, double arm, double kF);
    void FeedbackInput(Vector3d pos_fb, Vector3d vel_fb, Eigen::Quaterniond q_in);
    void setAngledes(Vector3d angle);
    void setOdomdes(Vector3d P_des, Vector3d V_des, Vector3d A_des, double Y_des);
    void RunController();
    Vector3d getTorquedes();
    Vector4d getRPMoutputs();
    void Quat2EulerAngle(const Quaterniond& q_input, double& roll, double& pitch, double& yaw);
};

cascadePID::cascadePID(double control_rate)
{
    dt = 1.0/control_rate;  // 计算采样时间
    RPM_output << 0,0,0,0;  // 初始化电机转速输出为0
    RPM_output_last << 0,0,0,0;  // 初始化上一时刻的电机转速输出为0
    RPM_change_limit = 1000.0*(dt/0.005);  // 计算电机转速变化限制
}


cascadePID::~cascadePID()
{
    myfile.close();
}

void cascadePID::setdroneid(int id){
    droneid = id;  // 设置飞行器的ID

    pkg_path = ros::package::getPath("cascadePID");  // 获取包的路径
    pkg_path.append("/data/log_" + std::to_string(droneid) + ".txt");  // 构建日志文件路径，使用飞行器ID作为后缀
    std::cout << "\nFound pkg_path = " << pkg_path << std::endl;  // 输出包的路径到控制台

    myfile.open(pkg_path.c_str(), std::ios_base::out);  // 打开日志文件，以输出模式写入
}
//记录每个无人机的日志状态到pkg/data/log_.txt

void cascadePID::setrate(double control_rate)
{
    dt = 1.0/control_rate;
}

void cascadePID::Quat2EulerAngle(const Quaterniond& q_input, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q_input.w() * q_input.x() + q_input.y() * q_input.z());
    double cosr_cosp = +1.0 - 2.0 * (q_input.x() * q_input.x() + q_input.y() * q_input.y());
    roll = atan2(sinr_cosp, cosr_cosp);  // 使用反正切函数计算roll角

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q_input.w() * q_input.y() - q_input.z() * q_input.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);  // 如果超出范围，设置为90度
    else
        pitch = asin(sinp);  // 使用反正弦函数计算pitch角

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q_input.w() * q_input.z() + q_input.x() * q_input.y());
    double cosy_cosp = +1.0 - 2.0 * (q_input.y() * q_input.y() + q_input.z() * q_input.z());
    yaw = atan2(siny_cosp, cosy_cosp);  // 使用反正切函数计算yaw角
}


void cascadePID::setInternal(double m, Matrix3d I, double arm, double kF)
{
    Internal_mat = I;  // 设置内部参数，惯性矩阵
    arm_length = arm;  // 设置臂长
    k_F = kF;  // 设置转速与力之间的比例系数
    k_T = 0.07 * (3 * 0.062) * k_F;  // 计算转速与扭矩之间的比例系数，根据公式0.07 * (3 * 0.062) * k_F

    motor_pos.resize(4,3);
    motor_pos << arm_length*sqrt(2)/2.0, -arm_length*sqrt(2)/2.0, 0,  // 设置电机的位置，采用平面对角布局
                -arm_length*sqrt(2)/2.0, arm_length*sqrt(2)/2.0, 0,
                 arm_length*sqrt(2)/2.0, arm_length*sqrt(2)/2.0, 0,
                -arm_length*sqrt(2)/2.0,-arm_length*sqrt(2)/2.0, 0;

    Vector3d z_body;
    z_body << 0,0,1;
    MatrixXd Mixermatrix_rpm2torque_noForce;
    Mixermatrix_rpm2torque_noForce.resize(3,4);

    for(int i = 0; i < 4; i++)
    {
        Vector3d temp;
        temp = motor_pos.row(i);
        // torque_temp = torque_temp+ temp.cross(F_motor(i)*z_body);
        Mixermatrix_rpm2torque_noForce.col(i) = k_F* temp.cross(z_body);  // 计算每个电机产生的扭矩，并存储在转换矩阵中
    }

    // Vector3d a = k_F* motor_pos.row(0).transpose().cross(z_body) + k_T* z_body;
    Mixermatrix_rpm2torque_noForce.col(0) = Mixermatrix_rpm2torque_noForce.col(0) - k_T* z_body;  // 考虑电机产生的扭矩对总力的影响
    Mixermatrix_rpm2torque_noForce.col(1) = Mixermatrix_rpm2torque_noForce.col(1) - k_T* z_body;
    Mixermatrix_rpm2torque_noForce.col(2) = Mixermatrix_rpm2torque_noForce.col(2) + k_T* z_body;
    Mixermatrix_rpm2torque_noForce.col(3) = Mixermatrix_rpm2torque_noForce.col(3) + k_T* z_body;
    Mixermatrix_rpm2torque.row(0) << k_F, k_F, k_F, k_F;
    Mixermatrix_rpm2torque.block<3,4>(1,0) = Mixermatrix_rpm2torque_noForce;  // 构建完整的转换矩阵

    std::cout << Mixermatrix_rpm2torque <<std::endl;  // 输出转换矩阵
}

//角速度自然频率代表了系统对输入信号的响应速度，阻尼比与控制系统的稳定性和超调量密切相关，将增益自然频率和阻尼比挂钩，可以使稳定性和性能之间取得一个平衡
void cascadePID::setParam(double stable_time, double damping_ratio)
{
    angle_wn = 4.6/(damping_ratio*stable_time);  // 根据稳定时间和阻尼比计算角速度的自然频率(wn为希腊字母"omega"的小写)
    Angle_Kd << 2*damping_ratio*angle_wn, 2*damping_ratio*angle_wn, 2*damping_ratio*angle_wn;  // 设置角速度的微分增益
    Angle_Kp << angle_wn*angle_wn, angle_wn*angle_wn, angle_wn*angle_wn;  // 设置角速度的比例增益
    Angle_Kp(2) = 1*Angle_Kp(2);  // 调整角速度在z轴上的比例增益
    Angle_Kd(2) = 1*Angle_Kd(2);  // 调整角速度在z轴上的微分增益
}//这里特指姿态控制

//将观测量前馈到控制器输入
void cascadePID::FeedbackInput(Vector3d pos_fb, Vector3d vel_fb, Eigen::Quaterniond q_in)
{
    q = q_in;  // 将传入的四元数赋值给成员变量q
    R_body2world = q_in.matrix();  // 从四元数计算出机体坐标系到世界坐标系的旋转矩阵
    current_pos = pos_fb;  // 设置当前位置的反馈值
    current_vel = vel_fb;  // 设置当前速度的反馈值
    Eigen::Vector3d angle = q_in.matrix().eulerAngles(0,1,2);  // 从旋转矩阵中获取机体坐标系的欧拉角（滚转、俯仰、偏航角）

    // 如果角度输入标志为1，则表示之前已经有角度输入
    if(angle_in_flag == 1)
    {
        last_euler_angle = euler_angle;  // 将当前的欧拉角保存为上一次的欧拉角
    }else{
        angle_in_flag=1;  // 如果角度输入标志为0，则将其设置为1，并将上一次的欧拉角初始化为0
        last_euler_angle << 0,0,0;
    }
    euler_angle = angle;  // 更新当前的欧拉角

    // 计算偏航角
    Eigen::Vector3d x_body;
    x_body << 1,0,0;
    x_body = R_body2world*x_body;  // 将机体坐标系中的x轴单位向量转换到世界坐标系中
    Yaw_cur = atan2(x_body(1),x_body(0));  // 计算偏航角，atan2函数返回[-π, π]范围内的角度值
}


void cascadePID::setAngledes(Vector3d angle)
{
    if(angledes_in_flag == 1)
    {
        last_angle_des = angle_des;
    }else{
        angledes_in_flag=1;
        last_angle_des << 0,0,0;
    }
    angle_des = angle;
}

//设置setpoint
void cascadePID::setOdomdes(Vector3d P_des, Vector3d V_des, Vector3d A_des, double Y_des)
{
    Pos_des = P_des;
    Vel_des = V_des;
    Acc_des = A_des;
    Yaw_des = Y_des;
}

void cascadePID::RunController()
{
    // 高度控制器参数
    double t_z_stable = 2.0;  // 稳定时间
    double damping_ratio_z = 0.8;  // 阻尼比
    double wn_z = 4.6 / (damping_ratio_z * t_z_stable);  // 自然频率
    double KP_Z = wn_z * wn_z;  // 高度比例增益
    //double KP_Z = 1.5;  // 高度比例增益
    double KI_Z = 0;  // 高度积分增益（设为0）
    double KD_Z = wn_z * 2 * damping_ratio_z;  // 高度微分增益
    //double KD_Z = 1.5;  // 高度微分增益
    //cout << "Kp2=" << KP_Z << " " << "Kv2" << KD_Z << endl;

    // XY平面位置控制器参数
    double t_xy_stable = 3.0;  // 稳定时间
    double damping_ratio_xy = 0.90;  // 阻尼比
    double wn_xy = 4.6 / (damping_ratio_xy * t_xy_stable);  // 自然频率
    double KP_XY = wn_xy * wn_xy;  // xy平面比例增益
    //double KP_XY = 1.5;  // xy平面比例增益
    double KD_XY = wn_xy * 2 * damping_ratio_xy;  // xy平面微分增益
    //double KD_XY = 1.5;  // xy平面微分增益
    //cout << "Kp0=" << KP_XY << " " << "Kv0" << KD_XY << endl;

    // 误差计算
    Pos_error = Pos_des - current_pos;  // 位置误差
    Vel_error = Vel_des - current_vel;  // 速度误差

    // 最大误差限制
    double max_pos_error = 1 * 10.0;  // 最大位置误差
    double max_vel_error = 1 * 10.0;  // 最大速度误差
    //限制误差在(-max,max)
    Pos_error(0) = std::max(std::min(Pos_error(0), max_pos_error), -max_pos_error);  // 限制x轴位置误差
    Pos_error(1) = std::max(std::min(Pos_error(1), max_pos_error), -max_pos_error);  // 限制y轴位置误差
    Pos_error(2) = std::max(std::min(Pos_error(2), max_pos_error), -max_pos_error);  // 限制z轴位置误差
    Vel_error(0) = std::max(std::min(Vel_error(0), max_vel_error), -max_vel_error);  // 限制x轴速度误差
    Vel_error(1) = std::max(std::min(Vel_error(1), max_vel_error), -max_vel_error);  // 限制y轴速度误差
    Vel_error(2) = std::max(std::min(Vel_error(2), max_vel_error), -max_vel_error);  // 限制z轴速度误差

    // z轴位置误差积分项
    //z_intergral_val = z_intergral_val + Pos_error(2) * dt;

    z_intergral_val += Pos_error(2) * dt;

    if (z_intergral_val > z_intergral_limit)
    {
        z_intergral_val = z_intergral_limit;  // 限制积分项
    }
    else if (z_intergral_val < -z_intergral_limit)
    {
        z_intergral_val = -z_intergral_limit;  // 限制积分项
    }

    // z轴的期望加速度，高度PID控制，但此时KI_z为0，相当于PD控制，PID可以实现更加准确控制精度，相对于PD控制来讲
    double a_des_z = (KP_Z * Pos_error(2) + KD_Z * Vel_error(2) + KI_Z * z_intergral_val + Acc_des(2));

    // 限制z轴的期望加速度
    if (a_des_z > 4 * g)
    {
        a_des_z = 4 * g;
    }
    else if (a_des_z < (-0.8 * g))
    {
        a_des_z = -0.8 * g;
    }

    // 期望总推力
    double F_des_z = mass * (a_des_z + g);//计算所需推力，世界坐标系z轴方向
    Vector3d z_body;
    z_body << 0, 0, 1;
    double current_tilt_angle = acos((R_body2world * z_body).dot(z_body));//计算在机体Z轴方向上的投影角度。机体坐标系z轴与垂直朝上方向之间的夹角
    double F_des = F_des_z / cos(current_tilt_angle);//得到实际推力，机体坐标系下

    // 限制期望总推力
    F_des = std::max(std::min(F_des, mass * 4 * g), 0.0);

    // XY平面位置控制器 PD控制，更易于实现轨迹的快速跟踪
    double acc_x_des = (KP_XY * Pos_error(0) + KD_XY * Vel_error(0) + Acc_des(0));  // 期望x轴加速度
    double acc_y_des = (KP_XY * Pos_error(1) + KD_XY * Vel_error(1) + Acc_des(1));  // 期望y轴加速度

    // 限制期望加速度在x和y轴上的范围
    acc_x_des = std::max(std::min(acc_x_des, 10.0), -10.0);
    acc_y_des = std::max(std::min(acc_y_des, 10.0), -10.0);

    // 根据期望加速度计算期望姿态
    Eigen::Vector3d new_z_body, rotation_vec;
    new_z_body << mass * acc_x_des, mass * acc_y_des, F_des_z;//三轴的力
    new_z_body.normalize();//归一化处理
    Matrix3d new_z_body_inv, R_des, I;

    // 计算期望的旋转矩阵（R_des）
    if (new_z_body.dot(z_body) == 1.0)  // 如果期望姿态与当前姿态平行
    {
        R_des << 1, 0, 0,
                 0, 1, 0,
                 0, 0, 1;
    }
    else  // 如果期望姿态与当前姿态不平行
    {
        rotation_vec = z_body.cross(new_z_body);//将z轴转到新的期望方向的旋转向量
        rotation_vec.normalize();

        new_z_body_inv << 0, -rotation_vec(2), rotation_vec(1),
                          rotation_vec(2), 0, -rotation_vec(0),
                          -rotation_vec(1), rotation_vec(0), 0;//由旋转向量建立反对称矩阵

        double rotation_angle = acos(z_body.dot(new_z_body));//计算z轴所需的旋转角度
        rotation_angle = std::max(std::min(rotation_angle, 0.35 * 1.5), -0.35 * 1.5);  // 限制旋转角度

        I << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;

        R_des = I + sin(rotation_angle) * new_z_body_inv + (1 - cos(rotation_angle)) * new_z_body_inv * new_z_body_inv;
        //根据罗德里格斯公式计算得到旋转矩阵，z轴的旋转
    }

    angle_des = R_des.eulerAngles(0, 1, 2);  // 期望的滚转角、俯仰角和偏航角

    double angle_limit = 30.0 / 180.0 * 3.1415926;  // 角度限制

    Matrix3d R_z;
    R_z << cos(Yaw_des), -sin(Yaw_des), 0,
           sin(Yaw_des), cos(Yaw_des), 0,
           0, 0, 1;
        //得到绕Z轴旋转的旋转矩阵
    // 误差状态
    Matrix3d R_diff;
    R_diff = R_body2world.transpose() * (R_des * R_z);//姿态误差
    //观测量得到的旋转矩阵转置乘以期望旋转矩阵，代表了当前姿态与期望姿态之间的误差


    Eigen::AngleAxisd rotation_vec_des(R_diff);  // 期望的旋转矢量，得到旋转轴以及旋转角度

    if (first_control_flag == 0)
    {
        d_Angle_error << 0, 0, 0;
        first_control_flag = 1;
    }
    else
    {
        d_Angle_error = (rotation_vec_des.axis() * rotation_vec_des.angle() - Angle_last_error) / dt;//角度误差的导数
    }
    Angle_last_error = rotation_vec_des.axis() * rotation_vec_des.angle();

    // 期望力矩--->姿态控制PD控制
    Torque_des(0) = (Angle_Kp(0) * rotation_vec_des.axis()(0) * rotation_vec_des.angle() + Angle_Kd(0) * d_Angle_error(0));
    Torque_des(1) = (Angle_Kp(1) * rotation_vec_des.axis()(1) * rotation_vec_des.angle() + Angle_Kd(1) * d_Angle_error(1));
    Torque_des(2) = (Angle_Kp(2) * rotation_vec_des.axis()(2) * rotation_vec_des.angle() + Angle_Kd(2) * d_Angle_error(2));
    Torque_des = Internal_mat * Torque_des;

    for (int i = 0; i < 2; i++)
    {
        if (Torque_des(i) > Torque_limit)
        {
            Torque_des(i) = Torque_limit;
        }
        else if (Torque_des(i) < -Torque_limit)
        {
            Torque_des(i) = -Torque_limit;
        }
    }

    if (Torque_des(2) > Torque_limit_yaw)
    {
        Torque_des(2) = Torque_limit_yaw;
    }
    else if (Torque_des(2) < -Torque_limit_yaw)
    {
        Torque_des(2) = -Torque_limit_yaw;
    }

    // 混合器
    for (int i = 0; i < 4; i++)
    {
        RPM_output(i) = F_des / (4.0 * k_F)
                        + Torque_des(0) / (4.0 * Mixermatrix_rpm2torque(1, i))
                        + Torque_des(1) / (4.0 * Mixermatrix_rpm2torque(2, i))
                        + Torque_des(2) / (4.0 * Mixermatrix_rpm2torque(3, i));
                        //rpm2torque 转速到力矩的映射

        if (RPM_output(i) < 0)
        {
            RPM_output(i) = 0;
        }
        else
        {
            RPM_output(i) = sqrt(RPM_output(i));
        }
    }

    // 限制输出RPM，RPM：电机转速
    for (int i = 0; i < 4; i++)
    {
        if ((RPM_output(i) - RPM_output_last(i)) > RPM_change_limit)
        {
            RPM_output(i) = RPM_output_last(i) + RPM_change_limit;
        }
        else if ((RPM_output(i) - RPM_output_last(i)) < -RPM_change_limit)
        {
            RPM_output(i) = RPM_output_last(i) - RPM_change_limit;
        }

        if (RPM_output(i) > max_rpm)
        {
            RPM_output(i) = max_rpm;
        }
        else if (RPM_output(i) < min_rpm)
        {
            RPM_output(i) = min_rpm;
        }
    }
    RPM_output_last = RPM_output;

    // 记录日志
    myfile << Pos_des(0) << " " << Pos_des(1) << " " << Pos_des(2) << " "
            << current_pos(0) << " " << current_pos(1) << " " << current_pos(2) << " "
            << Vel_des(0) << " " << Vel_des(1) << " " << Vel_des(2) << " "
            << current_vel(0) << " " << current_vel(1) << " " << current_vel(2) << " "
            << angle_des(0) << " " << angle_des(1) << " " << Yaw_des << " "
            << euler_angle(0) << " " << euler_angle(1) << " " << Yaw_cur << " "
            << Angle_error(0) << " " << Angle_error(1) << " " << Angle_error(2) << " "
            << Acc_des(0) << " " << Acc_des(1) << " " << Acc_des(2) << " "
            << Torque_des(0) << " " << Torque_des(1) << " " << Torque_des(2) << " "
            << new_z_body(0) << " " << new_z_body(1) << " " << new_z_body(2) << " "
            << RPM_output(0) << " " << RPM_output(1) << " " << RPM_output(2) << " " 
            << RPM_output(3) << " " << KP_Z << " " << KP_XY << " " << KD_Z << " " << " "
            << KD_XY <<std::endl;
}

Vector3d cascadePID::getTorquedes()
{
    return Torque_des;
}

Vector4d cascadePID::getRPMoutputs()
{
    return RPM_output;//RPM是转速
}