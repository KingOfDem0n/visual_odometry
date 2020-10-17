#include <tf/tf.h>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <assert.h>
#include <math.h>
#include <algorithm>

ros::Publisher pub_cmd_;
ros::Publisher update_rate;
int LOOP_HZ = 100;
ros::Duration wait_time;
double cur_x=0, cur_y=0, cur_yaw=0;

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
    update_rate.publish(std_msgs::Empty());
}

double angDiff(double theta1, double theta2);
void move2goal(float x_goal, float y_goal, float v_p=0.2, float w_p=1);
void move2pose(float x_goal, float y_goal, float theta_goal, float k_rho=0.2, float k_alpha=1, float k_beta=-0.5);
void stop();
void forward(double dist, bool end_stop=true);
void rotate(double dist, bool end_stop=true);
void curve(double dist, bool end_stop=true);
void boxMovement(double size=0.5, bool end_stop=true);
void roundBox(double size=0.5, bool end_stop=true);
void circle(bool end_stop=true);

int main(int argc, char **argv){
    ros::init(argc, argv, "ClosedLoopController_Node");
    ros::NodeHandle n;
    ros::Subscriber sub_odom_ = n.subscribe("/odom", 1, callback, ros::TransportHints().tcpNoDelay());
    pub_cmd_ = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    update_rate = n.advertise<std_msgs::Empty>("/Odom_update", 1);
    ros::Publisher pub_done_ = n.advertise<std_msgs::Empty>("/done", 1);
    wait_time = ros::Duration(1.0);

    //ros::Duration(5.0).sleep();

    // Set the command below this line
    move2pose(1,1,90);
    // move2goal(-1,-1);
    stop();
    // Set the command above this line

    ros::spinOnce();
    std::cout << "Odometry info: \n";
    std::cout << "\tX: " << cur_x << "\n";
    std::cout << "\tY: " << cur_y << "\n";
    std::cout << "\tTheta: " << cur_yaw << "\n";

    pub_done_.publish(std_msgs::Empty());

    return 0;
}

double angDiff(double theta1, double theta2){
  // Returns the difference between theta1 and theta2 in [-pi, pi) interval
  double d = theta1 - theta2;
  return fmod(d+M_PI, 2*M_PI) - M_PI;
}

void move2goal(float x_goal, float y_goal, float v_p, float w_p){
  ros::Rate loop_rate(LOOP_HZ);
  geometry_msgs::Twist cmd;

  while(sqrt(pow(x_goal-cur_x, 2) + pow(y_goal-cur_y, 2)) > 0.05){
    ros::spinOnce();
    cmd.linear.x = v_p*sqrt(pow(x_goal-cur_x, 2) + pow(y_goal-cur_y, 2));
    cmd.angular.z = w_p*angDiff(atan2(y_goal-cur_y, x_goal-cur_x), cur_yaw);
    //std::cout << cmd << std::endl;
    pub_cmd_.publish(cmd);
    loop_rate.sleep();
  }
}

void move2pose(float x_goal, float y_goal, float theta_goal, float k_rho, float k_alpha, float k_beta){
  assert(k_rho > 0 && k_beta < 0 && k_alpha > k_rho); //This is needed for a stable system

  ros::Rate loop_rate(LOOP_HZ);
  geometry_msgs::Twist cmd;

  ros::spinOnce();
  theta_goal = theta_goal*M_PI/180.0; //Convert from deg to rad
  double rho, alpha, beta;
  double diff_x = cur_x-x_goal, diff_y = cur_y-y_goal;
  double remaining;
  alpha = atan2(-diff_y, -diff_x) - cur_yaw;
  int direction = 1;

  // Decide if traveling backward
  if(alpha < -M_PI/2.0 || alpha > M_PI/2.0) {direction = -1;}

  while(sqrt(pow(diff_x, 2) + pow(diff_y, 2)) > 0.05 || fabs(angDiff(theta_goal, cur_yaw)) > 0.05){
    std::cout << "Diff: " << sqrt(pow(diff_x, 2) + pow(diff_y, 2)) << std::endl;
    std::cout << "Angle: " << fabs(angDiff(theta_goal, cur_yaw)) << std::endl;
    ros::spinOnce();
    diff_x = cur_x-x_goal;
    diff_y = cur_y-y_goal;
    rho = sqrt(pow(diff_x, 2) + pow(diff_y, 2));

    if(direction == -1){
      //alpha = atan2(diff_y, diff_x) - cur_yaw;
      //beta = -cur_yaw - alpha;
      beta = -atan2(diff_y, diff_x);
      alpha = -cur_yaw - beta;
    }
    else{
      //alpha = atan2(-diff_y, -diff_x) - cur_yaw;
      //beta = -cur_yaw - alpha;
      beta = -atan2(-diff_y, -diff_x);
      alpha = -cur_yaw - beta;
    }
    if(alpha > M_PI/2) {alpha = M_PI/2;}
    if(alpha < -M_PI/2) {alpha = -M_PI/2;}

    cmd.linear.x = direction*k_rho*rho;
    cmd.angular.z = atan(direction*(k_alpha*alpha + k_beta*(beta+theta_goal)));

    pub_cmd_.publish(cmd);
    loop_rate.sleep();
  }

}

// void followTrajectory(float x_goal, float y_goal, float followDist){
//   ros::Rate loop_rate(30);
//   geometry_msgs::Twist cmd;
//
//   // Update trajectory here somehow
//
//   ros::spinOnce();
//   double error = sqrt(pow(cur_x-x_goal, 2) + pow(cur_y-y_goal, 2)) - followDist;
//   double error_sum = 0, theta_star;
//
//   while(error > 0.05){
//     // Update trajectory here somehow
//     ros::spinOnce();
//     error = sqrt(pow(cur_x-x_goal, 2) + pow(cur_y-y_goal, 2)) - followDist;
//     error_sum += error;
//
//     cmd.linear.x = kv*error + ki*error_sum;
//     cmd.angular.z =kh*angDiff(atan2(y_goal-cur_y, x_goal-cur_x), cur_yaw);
//     //std::cout << cmd << std::endl;
//     pub_cmd_.publish(cmd);
//     loop_rate.sleep();
//   }
// }

void stop(){
  geometry_msgs::Twist cmd;
  pub_cmd_.publish(cmd);
}

void forward(double dist, bool end_stop){
    tf::Matrix3x3 m;
    ros::Rate loop_rate(LOOP_HZ);
    geometry_msgs::Twist cmd;
    double start_x, diff_x = 0.0;
    double slow_start = 0.2;

    if(dist >= 0)
        cmd.linear.x = 0.2;
    else
        cmd.linear.x = -0.2;
    ros::spinOnce();
    m.setRPY(0,0,cur_yaw);
    start_x = fabs(cur_x*m[0][0] + cur_y*m[0][1]);

    while(fabs(diff_x) < fabs(dist)){
        pub_cmd_.publish(cmd);
        ros::spinOnce();
        m.setRPY(0,0,cur_yaw);
        diff_x = fabs(cur_x*m[0][0] + cur_y*m[0][1]) - start_x;
        if(fabs(dist) - diff_x <= slow_start){
          cmd.linear.x = std::max(0.1, 0.2*(fabs(dist) - diff_x)/slow_start);
          if(dist < 0) cmd.linear.x *= -1;
        }
        loop_rate.sleep();
    }

    if(end_stop){
        stop();
        wait_time.sleep();
    }
}

void rotate(double dist, bool end_stop){
    ros::Rate loop_rate(LOOP_HZ);
    geometry_msgs::Twist cmd;
    double prev_yaw, dist_theta, diff_yaw = 0.0;
    double slow_start = 20*M_PI/180.0;

    if(dist >= 0)
        cmd.angular.z = 0.5;
    else
        cmd.angular.z = -0.5;

    dist_theta = fabs(dist*M_PI/180.0);
    ros::spinOnce();
    prev_yaw = fabs(cur_yaw);

    while(diff_yaw < dist_theta){
        pub_cmd_.publish(cmd);
        ros::spinOnce();
        diff_yaw += fabs(fabs(cur_yaw) - prev_yaw);
        prev_yaw = fabs(cur_yaw);

        // std::cout << "Current yaw: " << cur_yaw << "\n";
        // Slow down as we approach the end
        if(dist_theta - diff_yaw <= slow_start){
          cmd.angular.z = std::max(0.2, 0.5*(dist_theta - diff_yaw)/slow_start);
          if(dist < 0) cmd.angular.z *= -1;
        }
        loop_rate.sleep();
    }

    if(end_stop){
        stop();
        wait_time.sleep();
    }
}

void curve(double dist, bool end_stop){
    ros::Rate loop_rate(LOOP_HZ);
    geometry_msgs::Twist cmd;
    double prev_yaw, dist_theta, diff_yaw = 0.0;
    double slow_start = 20*M_PI/180.0;

    cmd.linear.x = 0.2;
    cmd.angular.z = 0.4;
    dist_theta = fabs(dist*M_PI/180.0);
    ros::spinOnce();
    prev_yaw = fabs(cur_yaw);

    while(diff_yaw < dist_theta){
      pub_cmd_.publish(cmd);
      ros::spinOnce();
      diff_yaw += fabs(fabs(cur_yaw) - prev_yaw);
      prev_yaw = fabs(cur_yaw);
      if(dist_theta - diff_yaw <= slow_start){
        cmd.linear.x = std::max(0.07, 0.2*(dist_theta - diff_yaw)/slow_start);
        cmd.angular.z = std::max(0.15, 0.4*(dist_theta - diff_yaw)/slow_start);
      }
      loop_rate.sleep();
    }

    if(end_stop){
        stop();
        wait_time.sleep();
    }
}

void boxMovement(double size, bool end_stop){
    forward(size, end_stop);
    rotate(90, end_stop);

    forward(size, end_stop);
    rotate(90, end_stop);

    forward(size, end_stop);
    rotate(90, end_stop);

    forward(size, end_stop);
    rotate(90, true);
}

void roundBox(double size, bool end_stop){
    forward(size, end_stop);
    curve(90, end_stop);

    forward(size, end_stop);
    curve(90, end_stop);

    forward(size, end_stop);
    curve(90, end_stop);

    forward(size, end_stop);
    curve(90, true);
}

void circle(bool end_stop){
    curve(90, end_stop);
    curve(90, end_stop);
    curve(90, end_stop);
    curve(90, true);
}
