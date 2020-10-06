#include <tf/tf.h>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <math.h>

ros::Publisher pub_cmd_;
ros::Rate loop_rate(100);
ros::Duration wait_time(1.0);
double cur_x, cur_y, cur_yaw;

void callback(const nav_msgs::Odometry::ConstPtr msg){
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll_, pitch_;
    m.getRPY(roll_, roll_, cur_yaw);

    cur_x = msg->pose.pose.position.x;
    cur_y = msg->pose.pose.position.y;
}

void stop();
void forward(double dist, bool end_stop=true);
void rotate(double dist, bool end_stop=true);
void curve(double dist, bool end_stop=true);
void boxMovement(bool end_stop=true);
void roundBox(bool end_stop=true);
void circle(bool end_stop=true);

int main(int argc, char **argv){
    ros::init(argc, argv, "ClosedLoopController_Node");
    ros::NodeHandle n;
    ros::Subscriber sub_odom_ = n.subscribe("/odom", 1, callback);
    pub_cmd_ = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    ros::Publisher pub_done_ = n.advertise<std_msgs::Empty>("/done", 1);

    ros::Duration(5.0).sleep();

    // Set the command below this line

    // Set the command above this line

    pub_done_.publish(std_msgs::Empty());

    return 0;
}

void stop(){
  geometry_msgs::Twist cmd;
  pub_cmd_.publish(cmd);
}

void forward(double dist, bool end_stop=true){
    tf::Matrix3x3 m;
    geometry_msgs::Twist cmd;
    double start_x, diff_x = 0.0;

    if(dist >= 0):
        cmd.linear.x = 0.2;
    else:
        cmd.linear.x = -0.2;
    m.setRPY(0,0,cur_yaw);
    start_x = abs(cur_x*m[0][0] + cur_y*m[0][1]);

    while(abs(diff_x) < abs(dist)){
        ros::spinOnce();
        pub_cmd_.publish(cmd);
        m.setRPY(0,0,cur_yaw);
        diff_x = abs(cur_x*m[0][0] + cur_y*m[0][1]) - start_x;
        loop_rate.sleep();
    }

    if(end_stop){
        stop();
        wait_time.sleep();
    }
}

void rotate(double dist, bool end_stop=true){
    geometry_msgs::Twist cmd;
    double start_yaw, diff_yaw = 0.0;

    cmd.angular.z = 0.5;
    dist_theta = dist*M_PI/180.0;
    start_yaw = abs(cur_yaw);

    while(diff_yaw < dist_theta){
        ros::spinOnce();
        diff_yaw += abs(abs(cur_yaw) - start_yaw);
        loop_rate.sleep();
    }

    if(end_stop){
        stop();
        wait_time.sleep();
    }
}

void curve(double dist, bool end_stop=true){
    geometry_msgs::Twist cmd;
    double start_yaw, diff_yaw = 0.0;

    cmd.linear.x = 0.2;
    cmd.angular.z = 0.4;
    dist_theta = dist*M_PI/180.0;
    start_yaw = abs(cur_yaw);

    while(diff_yaw < dist_theta){
        ros::spinOnce();
        diff_yaw += abs(abs(cur_yaw) - start_yaw);
        loop_rate.sleep();
    }

    if(end_stop){
        stop();
        wait_time.sleep();
    }
}

void boxMovement(double size=0.5, bool end_stop=true){
    forward(size, end_stop);
    rotate(90, end_stop);

    forward(size, end_stop);
    rotate(90, end_stop);

    forward(size, end_stop);
    rotate(90, end_stop);

    forward(size, end_stop);
    rotate(90, true);
}

void roundBox(double size=0.5, bool end_stop=true){
    forward(size, end_stop);
    curve(90, end_stop);

    forward(size, end_stop);
    curve(90, end_stop);

    forward(size, end_stop);
    curve(90, end_stop);

    forward(size, end_stop);
    curve(90, true);
}

void circle(bool end_stop=true){
    curve(90, end_stop);
    curve(90, end_stop);
    curve(90, end_stop);
    curve(90, true);
}