#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <fstream>
#include "tf/transform_datatypes.h"
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sys/select.h>
#include <queue>
#include <geometry_msgs/Pose2D.h>
#include "geometry_msgs/PoseStamped.h"
#include "kkswarm_msgs/kkSwarmPose.h"
#include "kkswarm_msgs/kkSwarmState.h"
#include <cmath>
#include <stdio.h>
#define N 5
#define ABS(x) ((x > 0) ? (x) : (-x))
#define pi 3.14159
#define Min(x, y) (x > y) ? y : x

using namespace std;
using namespace Eigen;

typedef struct
{
    /*****安排过度过程*******/
    float x1;    //跟踪微分器状态量
    float x2;    //跟踪微分器状态量微分项
    float r;     //时间尺度
    float h;     //ADRC系统积分时间
    uint16_t N0; //跟踪微分器解决速度超调h0=N*h

    float h0;
    float fh; //最速微分加速度跟踪量
} Fhan_Data;

typedef struct
{
    /*****安排过度过程*******/
    float x;    //状态
    float x_f;  //滤波后的状态
    float dx_f; //滤波导数
    float T_1;  //滤波常数
    float tk;   //采样时间
} Filter_Data;

int flag_pd0, flag_pd1, flag_pd2;      //启动标志
int flag_sta;                          //圆形轨迹跟踪:1    编队切换:0
float x_0 = 0, y_0 = 0, phi_0 = 0.000; //小车0的位置和姿态
float theta1, theta2, thetae;          //姿态连续性变换
float p = 0, p1 = 0;                   //虚拟控制律
float alpha, alpha1;
float v_2, w_2;     //小车的线速度和角速度
float xd_2, yd_2;   //期望位置
float dxd_2, dyd_2; //期望位置导数
float xe_2, ye_2;   //位置误差
float m1, m2;
float c1 = 1, c2 = 1, c3 = 2; //位置控制增益
//float c1=0.4,c2=0.4,c3=0.8; //位置控制增益
float k = 0, k1 = 0, k2 = 9;        //姿态连续性
float t;                            //时间，在所有小车启动时开始计时
float vmax, wmax, v_a_max, w_a_max; //速度和角速度限制
float e;                            //横向距离
float x_1, y_1, phi_1;              //小车1的位置和姿态
float x_2, y_2, phi_2;              //小车2的位置和姿态
float thd_2, dthd_2;                //小车2的正切角
float v_0, w_0;
float xe_0, ye_0;
float xd_0, yd_0;
float dxd_0, dyd_0;
float xd;

float wd_0, vd_0, dphid_0, phid_0;
/************编队控制****************/
Filter_Data deltax_20, deltay_20, deltax_21, deltay_21, deltax_22, deltay_22; //队形距离变量
float px_20, px_21, px_22, py_20, py_21, py_22;
float kp_0 = 0.5, kp_1 = 0.5, kp_2 = 0.5, c_2 = 0.6, k_2 = 0.6;
float sumx_2, sumy_2, ux_2, uy_2, e_th, s_2, xite_2 = 0.5, del = 0.1, kk = 10, sats_2;
float delta_tri_x0; //队形控制变量
float delta_tri_y0;
float delta_tri_x1;
float delta_tri_y1;
float delta_tri_x2;
float delta_tri_y2;
float vx_r, vy_r; //期望速度
float vs_0, ws_0, vs_dot_0, ws_dot_0;
float zex_dot_0, zey_dot_0, zex_0, zey_0, zeyaw_dot_0, zeyaw_0;
float ts;

Fhan_Data RCa;
Fhan_Data Rxd;
Fhan_Data Ryd;
Fhan_Data RCv;
Fhan_Data RCw;
Fhan_Data RCthd_2;
Filter_Data test1;
Fhan_Data test2;

// void Float_to_Byte(float f,unsigned char byte[])  ;

// void Float_to_Byte(float f,unsigned char byte[])
// {
//     FloatLongType fl;
//     fl.fdata=f;
//     byte[0]=(unsigned char)fl.ldata;
//     byte[1]=(unsigned char)(fl.ldata>>8);
//     byte[2]=(unsigned char)(fl.ldata>>16);
//     byte[3]=(unsigned char)(fl.ldata>>24);
// }

int16_t Sign_TD(float Input)
{
    int16_t output = 0;
    if (Input > 1E-6)
        output = 1;
    else if (Input < -1E-6)
        output = -1;
    else
        output = 0;
    return output;
}

int16_t Fsg_TD(float x, float d)
{
    int16_t output = 0;
    output = (Sign_TD(x + d) - Sign_TD(x - d)) / 2;
    return output;
}

void ADRC_Init(Fhan_Data *fhan_Input)
{
    fhan_Input->r = 2.5;
    fhan_Input->h = 0.05;
    fhan_Input->N0 = 5;
}

void FilterOne_Init(Filter_Data *filter_Input, float xDat)
{
    filter_Input->x = xDat;
    filter_Input->x_f = xDat;
    filter_Input->dx_f = 0;
    filter_Input->T_1 = 1;
    filter_Input->tk = 0.01;
}

void ADRC_Init1(Fhan_Data *fhan_Input)
{
    fhan_Input->r = 1000;
    fhan_Input->h = 0.2;
    fhan_Input->N0 = 5;
}

void ADRC_Init2(Fhan_Data *fhan_Input)
{
    fhan_Input->r = 700;
    fhan_Input->h = 0.2;
    fhan_Input->N0 = 5;
}

void forma0Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    flag_pd0 = (int)msg->linear.y;
    ROS_WARN("flag_pd0:%d,flag_pd1:%d", flag_pd0, flag_pd1);
}

void forma1Callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    flag_pd1 = (int)msg->linear.y;
    ROS_WARN("flag_pd0:%d,flag_pd1:%d", flag_pd0, flag_pd1);
}

// T265反馈位姿信息
void camera0Callback(const nav_msgs::Odometry::ConstPtr &cameramsg)
{
    tf::Quaternion quat;
    double roll, pitch, yaw;

    tf::quaternionMsgToTF(cameramsg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换

    Eigen::Vector3f ea(yaw, pitch, roll);

    phi_0 = yaw;
    x_0 = cameramsg->pose.pose.position.x;
    y_0 = cameramsg->pose.pose.position.y;

    ROS_WARN("x_0:%0.6f,y_0:%0.6f,phi_0:%0.6f", x_0, y_0, phi_0);
}

// T265反馈位姿信息: vehicle1
void camera1Callback(const nav_msgs::Odometry::ConstPtr &cameramsg)
{
    tf::Quaternion quat;
    double roll, pitch, yaw;

    tf::quaternionMsgToTF(cameramsg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换

    Eigen::Vector3f ea(yaw, pitch, roll);

    phi_1 = yaw;
    x_1 = cameramsg->pose.pose.position.x;
    y_1 = cameramsg->pose.pose.position.y - 0.8;
    // y_1=cameramsg->pose.pose.position.y;

    ROS_WARN("x_1:%0.6f,y_1:%0.6f,phi_1:%0.6f", x_1, y_1, phi_1);
}

// T265反馈位姿信息: vehicle2
void camera2Callback(const nav_msgs::Odometry::ConstPtr &cameramsg)
{
    tf::Quaternion quat;
    double roll, pitch, yaw;

    Eigen::Vector3f ea(yaw, pitch, roll);

    phi_2 = yaw;
    x_2 = cameramsg->pose.pose.position.x;
    y_2 = cameramsg->pose.pose.position.y - 0.3;
    //y_2=cameramsg->pose.pose.position.y;

    ROS_WARN("x_2:%0.6f,y_2:%0.6f,phi_2:%0.6f", x_2, y_2, phi_2);
}

void pose0Callback(const nav_msgs::Odometry::ConstPtr &pose0msg)
{
    tf::Quaternion quat;
    double roll, pitch, yaw;

    tf::quaternionMsgToTF(pose0msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换

    Eigen::Vector3f ea(yaw, pitch, roll);
    phi_0 = yaw + 3.14156926 / 2;
    x_0 = pose0msg->pose.pose.position.x;
    y_0 = pose0msg->pose.pose.position.y;
}

void car0poseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose0msg)
{
    tf::Quaternion quat;
    double roll, pitch, yaw;

    tf::quaternionMsgToTF(pose0msg->pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换

    Eigen::Vector3f ea(yaw, pitch, roll);
    phi_0 = yaw + 3.14156926 / 2;
    x_0 = pose0msg->pose.position.x;
    y_0 = pose0msg->pose.position.y;
}

void Filter_one(Filter_Data *filter_Input, float expect_Dat) //安排ADRC过度过程
{
    filter_Input->x = expect_Dat;
    filter_Input->dx_f = (filter_Input->x - filter_Input->x_f) / filter_Input->T_1;
    filter_Input->x_f = filter_Input->x_f + filter_Input->dx_f * filter_Input->tk;
}

//ADRC最速跟踪微分器TD，改进的算法fhan
void Fhan_TD(Fhan_Data *fhan_Input, float expect_rcDat) //安排ADRC过度过程
{
    float d = 0, a0 = 0, y = 0, a1 = 0, a2 = 0, a = 0;
    float x1_delta = 0;                                  //RC状态跟踪误差项
    x1_delta = fhan_Input->x1 - expect_rcDat;            //用x1-v(k)替代x1得到离散更新公式
    fhan_Input->h0 = fhan_Input->N0 * fhan_Input->h;     //用h0替代h，解决最速跟踪微分器速度超调问题
    d = fhan_Input->r * fhan_Input->h0 * fhan_Input->h0; //d=rh^2;
    a0 = fhan_Input->h0 * fhan_Input->x2;                //a0=h*x2
    y = x1_delta + a0;                                   //y=x1+a0
    a1 = sqrt(d * (d + 8 * ABS(y)));                     //a1=sqrt(d*(d+8*ABS(y))])
    a2 = a0 + Sign_TD(y) * (a1 - d) / 2;                 //a2=a0+sign(y)*(a1-d)/2;
    a = (a0 + y) * Fsg_TD(y, d) + a2 * (1 - Fsg_TD(y, d));
    fhan_Input->fh = -fhan_Input->r * (a / d) * Fsg_TD(a, d) - fhan_Input->r * Sign_TD(a) * (1 - Fsg_TD(a, d)); //得到最速微分加速度跟踪量
    fhan_Input->x1 += fhan_Input->h * fhan_Input->x2;                                                           //跟新最速跟踪状态量x1，遥控器需要的信号
    fhan_Input->x2 += fhan_Input->h * fhan_Input->fh;                                                           //跟新最速跟踪状态量微分x2
}

// 饱和函数
double satur(double ut, double lim)
{
    float u;

    if (ut > lim)
        u = lim;
    else if (ut < -lim)
        u = -lim;
    else
        u = ut;

    return u;
}

float satd_fuc(float ut, float lim)
{
    float u;
    if (abs(ut) > abs(lim))
        u = lim;
    else
        u = ut;

    return u;
}

float sat_fuc(float ut, float lim)
{
    float u;
    u = max(-lim, min(lim, ut));
    return u;
}

int main(int argc, char *argv[])
{
    //ROS节点初始化phi_0
    ros::init(argc, argv, "tracking_circle2_kk");

    ros::NodeHandle n;

    // 创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
    ros::Publisher odom_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    ros::Subscriber pose_sub = n.subscribe("/kcg0_t265/odom/sample", 10, camera0Callback);
    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    ros::Subscriber pose_sub1 = n.subscribe("/kcg1_t265/odom/sample", 10, camera1Callback);
    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    // ros::Subscriber pose_sub2 = n.subscribe("/kcg2_t265/odom/sample", 10, camera2Callback);
    //创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数 forma0Callback
    ros::Subscriber vel_sub0 = n.subscribe("/kcg_0/cmd_vel", 10, forma0Callback);
    //创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数 forma1Callback
    ros::Subscriber vel_sub1 = n.subscribe("/kcg_1/cmd_vel", 10, forma1Callback);
    // 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    ros::Subscriber pose_sub2 = n.subscribe("/vrpn_client_node/car0/pose", 10, car0poseCallback);

    //创建一个Subscriber，订阅名为/robot_0/pose的topic，注册回调函数 forma1Callback
    ros::Subscriber pose_sub0 = n.subscribe("/robot_1/pose", 10, pose0Callback);
    // 创建一个Publisher，发布名为/robot_0/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
    ros::Publisher odom_pub0 = n.advertise<geometry_msgs::Twist>("/robot_1/cmd_vel", 10);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose2D>("/expect_pose", 10);
    ros::Publisher pose_pub1 = n.advertise<geometry_msgs::Pose2D>("/actual_pose", 10);
    ros::Publisher pose_pub2 = n.advertise<geometry_msgs::Pose2D>("/pose_err", 10);

    //设置循环频率
    ros::Rate loop_rate(100);

    flag_pd0 = 1;
    flag_pd1 = 1;
    flag_pd2 = 0;
    flag_sta = 1;

    // xd_0 = 0.789;
    // yd_0 = 0.28;
    // phid_0 = 0;
    xd_0 = 1.15;
    yd_0 = 0.87;
    phid_0 = 0;
    //设置角速度和线速度的最大值
    vmax = 1.5;
    wmax = 1.5;
    v_a_max = 0.4;
    w_a_max = 0.4;
    ts = 0.01;

    ADRC_Init(&RCa);
    ADRC_Init1(&Rxd);
    ADRC_Init1(&Ryd);
    ADRC_Init(&RCthd_2);
    ADRC_Init2(&RCv);
    ADRC_Init2(&RCw);
    //   FilterOne_Init(&test1);
    //  ADRC_Init2(&test2);

    //队形指令初始化
    FilterOne_Init(&deltax_20, 0);
    FilterOne_Init(&deltax_21, 0);
    FilterOne_Init(&deltax_22, 0);
    FilterOne_Init(&deltay_20, -1.6);
    FilterOne_Init(&deltay_21, -0.8);
    FilterOne_Init(&deltay_22, 0);

    while (ros::ok())
    {
        ros::spinOnce();

        /*测试一阶低通滤波器
     if(t>10)
     {
         xd=1;
     }
     elseROS_WARN("x_0:%0.6f,y_0:%0.6f,phi_0:%0.6f,theta2:%0.6f", x_0,y_0,phi_0,theta2);
//     Filter_one(&test1,xd);
     Fhan_TD(&test2,xd);
     ROS_WARN("test1.x:%0.6f,test1.x_f:%0.6f", test2.x1,test2.x2);
     **************************/

        if (flag_pd0 && flag_pd1)
        {
            if (flag_sta == 1)
            {

                // xd_0=0.7*sin(3.1415926*t/16)+0.67;
                // yd_0=-0.7*cos(3.1415926*t/16)+1;
                // dxd_0=0.7*3.1415926/16*cos(3.1415926*t/16);
                // dyd_0=0.7*3.1415926/16*sin(3.1415926*t/16);
                vd_0 = 0.1;
                wd_0 = 0.3;
                dphid_0 = wd_0;
                phid_0 = phid_0 + dphid_0 * 0.01;
                dxd_0 = vd_0 * cos(phid_0);
                dyd_0 = vd_0 * sin(phid_0);
                xd_0 = xd_0 + dxd_0 * 0.01;
                yd_0 = yd_0 + dyd_0 * 0.01;

                xe_0 = xd_0 - x_0;
                ye_0 = yd_0 - y_0;

                //速度和加速度饱和
                vs_dot_0 = sat_fuc((sat_fuc(v_0, vmax) - vs_0) / ts, v_a_max);
                ws_dot_0 = sat_fuc((sat_fuc(w_0, wmax) - ws_0) / ts, w_a_max);

                vs_0 = vs_0 + vs_dot_0 * ts;
                ws_0 = ws_0 + ws_dot_0 * ts;

                zex_dot_0 = -10 * zex_0 + 1.2 * cos(alpha) * (v_0 - vs_0);
                zey_dot_0 = -10 * zey_0 + 1.2 * sin(alpha) * (v_0 - vs_0);
                zex_0 = zex_0 + zex_dot_0 * ts;
                zey_0 = zey_0 + zey_dot_0 * ts;

                zeyaw_dot_0 = -0.5 * zeyaw_0 + 1.2 * (w_0 - ws_0);
                zeyaw_0 = zeyaw_0 + zeyaw_dot_0 * ts;

                m1 = dxd_0 + c1 * xe_0 - 10 * zex_0;
                m2 = dyd_0 + c2 * ye_0 - 10 * zey_0;

                //线速度控制律
                v_0 = sqrt(pow(m1, 2) + pow(m2, 2));

                t = t + 0.01;

                if (fabs(xe_0) <= 0.04 && fabs(ye_0) <= 0.04)
                    k2 = 9; //abs(double x)返回双精度参数x的绝对值,当第一次跟踪上轨迹时，速度提升

                if (phi_0 - theta1 < -5)
                    k1 = k1 + 1;
                if (phi_0 - theta1 > 5)
                    k1 = k1 - 1;

                theta1 = phi_0;
                theta2 = phi_0 + 2 * 3.1415926 * k1;

                //虚拟控制律
                //  p=atan2(m2,m1)-atan2(ye_0,2);
                p = atan2(m2, m1);
                if (p * p1 < -0.8 * pi * pi)
                {
                    if (p < 0)
                    {
                        k = k + 1;
                    }
                    else
                    {
                        k = k - 1;
                    }
                }
                p1 = p;
                alpha = p + 2 * 3.1415926 * k; //

                /*******角速度控制律_1*******/
                // alpha=-0.5*t;
                Fhan_TD(&RCa, alpha);
                alpha = RCa.x1;
                alpha1 = RCa.x2;
                // alpha1=-0.2;
                w_0 = alpha1 + c3 * (alpha - theta2) - 0.5 * zeyaw_0;

                //  autual_v_l=

                ROS_WARN("xd_0:%0.6f,yd_0:%0.6f,alpha:%6f", xd_0, yd_0, alpha);
                ROS_WARN("x_0:%0.6f,y_0:%0.6f,theta2:%0.6f,phi_0:%0.6f", x_0, y_0, theta2, phi_0);
                //  ROS_WARN("v_0:%0.6f,w_0:%0.6f", v_0,w_0);
                ROS_WARN("dxd_0:%0.6f,dyd_0:%0.6f", dxd_0, dyd_0);
                ROS_WARN("xe_0:%0.6f,ye_0:%0.6f,phie_0:%6f", xe_0, ye_0, alpha - theta2);

                ofstream outfile; //记录数据
                outfile.open("/home/ris/testdata/test_0.txt", ios::app);
                // outfile<<"t:"<<t<<":x_0:"<<x_0<<":xd_0:"<<xd_0<< ":xe_0:"<<xe_0 <<":y_0:"<<y_0<<":yd_0:"<<yd_0<<":ye_0:"<< ye_0<<":phi_0:"<<theta2<<":phid_0:"<<alpha<<":phie_0:"<<alpha-phi_0<<endl; //记录数据

                outfile << "t:" << t << ":x_0:" << x_0 << ":xd_0:" << xd_0 << ":xe_0:" << xe_0 << ":y_0" << y_0 << ":yd_0:" << yd_0 << ":ye_0:" << ye_0 << ":phi_0:" << theta2 << ":phid_0:" << alpha << ":phie_0:" << alpha - phi_0 << ":v_d:" << vd_0 << ":w_d:" << wd_0 << ":v_0:" << v_0 << ":w_0:" << w_0 << endl; //记录数据
                outfile.close();
                /*****角速度控制律_2*******/
                //  thetae=alpha-theta2;
                //  e=Sign_TD(sin(thetae))*sqrt(pow(xe_0,2)+pow(ye_0,2));//横向距离
                //  w_0=thetae+atan2(50*e,v_0);	 //角速度控制器

                /*****角速度控制律_3*******/
                //  thetae=alpha-theta2;
                //  w_0=atan2(2*sin(thetae),v_0);  //角速度控制器
            }
            //     else if(flag_sta=2)
            //    {
            //          if(flag_pd2==1)
            //        {
            //              delta_tri_x0=0;
            //              delta_tri_y0=0;
            //              delta_tri_x1=0;
            //              delta_tri_y1=-0.8;
            //              delta_tri_x2=0;
            //              delta_tri_y2=-1.6;
            //              vx_r=0.1;
            //              vy_r=0;
            //        }
            //          if(flag_pd2==2)
            //        {
            //              delta_tri_x0=0;
            //              delta_tri_y0=0;
            //              delta_tri_x1=0.8;
            //              delta_tri_y1=-0.8;
            //              delta_tri_x2=0;
            //              delta_tri_y2=-1.6;
            //              vx_r=0.1;
            //              vy_r=0;
            //        }
            //         //线速度控制律
            //          px_20=x_2-x_0;
            //          py_20=y_2-y_0;
            //          px_21=x_2-x_1;
            //          py_21=y_2-y_1;
            //          px_22=0;
            //          py_22=0;

            //         //队形指令滤波：修改后
            //          Filter_one(&deltax_20,delta_tri_x2- delta_tri_x0);
            //          Filter_one(&deltax_21,delta_tri_x2- delta_tri_x1);
            //          Filter_one(&deltax_22,delta_tri_x2- delta_tri_x2);
            //          Filter_one(&deltay_20,delta_tri_y2- delta_tri_y0);
            //          Filter_one(&deltay_21,delta_tri_y2- delta_tri_y1);
            //          Filter_one(&deltay_22,delta_tri_y2- delta_tri_y2);

            //          sumx_2=kp_0*(px_20-deltax_20.x_f)+kp_1*(px_21-deltax_21.x_f)+kp_2*(px_22-deltax_22.x_f);
            //          sumy_2=kp_0*(py_20-deltay_20.x_f)+kp_1*(py_21-deltay_21.x_f)+kp_2*(py_22-deltay_22.x_f);
            //         ROS_WARN("deltax_20.x_f:%0.6f,deltax_21.x_f:%0.6f,deltax_22.x_f:%0.6f",deltax_20.x_f,deltax_21.x_f,deltax_22.x_f);
            //         ROS_WARN("deltay_20.x_f:%0.6f,deltay_21.x_f:%0.6f,deltay_22.x_f:%0.6f",deltay_20.x_f,deltay_21.x_f,deltay_22.x_f);

            //          ofstream outfile;     //记录数据
            //          outfile.open("/home/x/testdata/test_21.txt",ios::app);
            //          outfile<<"formation2:"<<"t:"<<t<<": x_0:"<<x_0<<" :y_0:"<< y_0<<" :phi_0:"<<phi_0<<" x_1:"<<x_1<<" :y_1:"<< y_1<<" :phi_1:"<<phi_1<<" x_2:"<<x_2<<" :y_2:"<< y_2<<" :phi_2:"<<phi_2<<" :theta2:"<<theta2<<" thetad:"<<thd_2<<endl; //记录数据
            //          outfile.close();

            //          //角度连续性变换
            //          if(phi_2-theta1<-5) k1=k1+1;
            //          if(phi_2-theta1>5)  k1=k1-1;

            //          theta1=phi_2;
            // 		 theta2=phi_2+2*3.1415926*k1;

            //          ux_2=vx_r-c_2*sumx_2;
            //          uy_2=vy_r-c_2*sumy_2;

            //          p=atan2(uy_2,ux_2+0.005);
            //          if (p*p1<-0.8*pi*pi)
            //          {
            //              if (p<0){k=k+1;}
            // 		           else {k=k-1;}
            //          }
            //          p1=p;
            //          thd_2=p+2*3.1415926*k;
            //        // thd_2=atan2(uy_2,ux_2);

            //         //角速度控制律
            //          Fhan_TD(&RCthd_2,thd_2);
            //          e_th=theta2- RCthd_2.x1;
            //          dthd_2=RCthd_2.x2;

            //          s_2=e_th;

            //          if(abs(s_2)>0.1)
            //              sats_2=Sign_TD(s_2);
            //          else
            //              sats_2=kk*s_2;

            //          if(flag_pd2==1||flag_pd2==2)
            //          {
            //              v_2=ux_2/cos(thd_2);
            //              //w_2=dthd_2-k_2*s_2-xite_2*sats_2;
            //              w_2=dthd_2-k_2*s_2;
            //          }
            //          else if(flag_pd2==0)
            //          {
            //              v_2=0;
            //              w_2=0;
            //          }

            //          t=t+0.01;

            //          if(t<20)
            //         {
            //             flag_pd2=1;
            //         }
            //         if(t>=20 && t<=40)
            //         {
            //              flag_pd2=2;
            //         }
            //         if(t>40)
            //         {
            //              flag_pd2=0;
            //         }
            //     }
            else
            {
                v_0 = 0;
                w_0 = 0;
            }
        }
        else
        {
            v_0 = 0;
            w_0 = 0;
        }

        Fhan_TD(&RCv, v_0);
        Fhan_TD(&RCw, w_0);

        RCv.x1 = satd_fuc(RCv.x1, vs_0);
        RCw.x1 = satd_fuc(RCw.x1, ws_0);



        ROS_WARN("v_0:%0.6f,w_0:%0.6f", v_0, w_0);

        // RCv.x1 = 0.1;
        // RCw.x1 = 0;

        //ROS_WARN("RCv.x1:%0.6f,RCw.x1:%0.6f", RCv.x1,RCw.x1);

        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = RCv.x1;
        vel_msg.linear.y = flag_pd2;
        vel_msg.angular.z = RCw.x1;
        odom_pub0.publish(vel_msg);

        // 发布实际消息
        geometry_msgs::Pose2D vel_err;
        vel_err.x = x_0;
        vel_err.y = y_0;
        vel_err.theta = theta2;
        pose_pub1.publish(vel_err);

        // 发布期望消息
        geometry_msgs::Pose2D vel_req;
        vel_req.x = xd_0;
        vel_req.y = yd_0;
        vel_req.theta = alpha;
        pose_pub.publish(vel_req);

        //发布跟踪误差
        geometry_msgs::Pose2D pos_err;
        pos_err.x = xe_0;
        pos_err.y = ye_0;
        pos_err.theta = alpha - theta2;
        pose_pub2.publish(pos_err);

        loop_rate.sleep();
    }

    return 0;
}
