#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"

#include <iostream>
#include <math.h>

void stop(ros::Publisher pub);
void forward(ros::Publisher pub, bool end_stop=true);
void rotate(ros::Publisher pub, bool end_stop=true);
void curve(ros::Publisher pub, bool end_stop=true);
void boxMovement(ros::Publisher pub, bool end_stop=true);
void roundBox(ros::Publisher pub, bool end_stop=true);
void circle(ros::Publisher pub, bool end_stop=true);

int main(int argc, char **argv){
  ros::init(argc, argv, "VO_controller");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
  ros::Publisher done = n.advertise<std_msgs::Empty>("/done", 1000);

  ros::Duration(5.0).sleep();

  rotate(pub);
  rotate(pub);
  //curve(pub);
  //boxMovement(pub);
  //roundBox(pub);
  //circle(pub);

  done.publish(std_msgs::Empty());

  stop(pub);
  ros::Duration(1.0).sleep();

  return 0;
}


void stop(ros::Publisher pub){
  geometry_msgs::Twist cmd;
  pub.publish(cmd);
}

void forward(ros::Publisher pub, bool end_stop){
  ros::WallTime start, end;
  start = ros::WallTime::now();
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.2;

  ros::Duration duration((0.5/cmd.linear.x)*1.35);
  std::cout << "Forward Duration: " << duration.toSec() << std::endl;
  pub.publish(cmd);
  duration.sleep();

  if(end_stop){
    stop(pub);
    ros::Duration(0.5).sleep();
  }
  end = ros::WallTime::now();
  std::cout << "Forward Execution time: " << (end-start).toSec() << std::endl;
}

void rotate(ros::Publisher pub, bool end_stop){
  ros::WallTime start, end;
  start = ros::WallTime::now();
  geometry_msgs::Twist cmd;
  cmd.angular.z = 0.5;

  ros::Duration duration(((M_PI/2.0)/cmd.angular.z)*2.1);
  std::cout << "Rotate Duration: " << duration.toSec() << std::endl;
  pub.publish(cmd);
  duration.sleep();

  if(end_stop){
    stop(pub);
    ros::Duration(0.5).sleep();
  }
  end = ros::WallTime::now();
  std::cout << "Rotate Execution time: " << (end-start).toSec() << std::endl;
}

void curve(ros::Publisher pub, bool end_stop){
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.2;
  cmd.angular.z = 0.4;

  ros::Duration duration(((M_PI/2.0)/cmd.angular.z)*1.4);
  pub.publish(cmd);
  duration.sleep();

  if(end_stop){
    stop(pub);
    ros::Duration(1.0).sleep();
  }
}

void boxMovement(ros::Publisher pub, bool end_stop){
  forward(pub, end_stop);
  rotate(pub, end_stop);

  forward(pub, end_stop);
  rotate(pub, end_stop);

  forward(pub, end_stop);
  rotate(pub, end_stop);

  forward(pub, end_stop);
  rotate(pub, true);
}

void roundBox(ros::Publisher pub, bool end_stop){
  ros::Duration wait_time(1.0);

  forward(pub, end_stop);
  wait_time.sleep();
  curve(pub, end_stop);
  wait_time.sleep();

  forward(pub, end_stop);
  wait_time.sleep();
  curve(pub, end_stop);
  wait_time.sleep();

  forward(pub, end_stop);
  wait_time.sleep();
  curve(pub, end_stop);
  wait_time.sleep();

  forward(pub, end_stop);
  wait_time.sleep();
  curve(pub, end_stop);
  wait_time.sleep();
}

void circle(ros::Publisher pub, bool end_stop){
  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.2;
  cmd.angular.z = 0.4;

  ros::Duration duration(((M_PI/2.0)/cmd.angular.z)*5);
  pub.publish(cmd);
  duration.sleep();

  if(end_stop)
    stop(pub);
}
