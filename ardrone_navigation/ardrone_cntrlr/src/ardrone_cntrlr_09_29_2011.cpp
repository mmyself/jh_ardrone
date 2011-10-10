#include <ros/ros.h>
#include <iostream>
#include <deque>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <time.h>

#include "ardrone_lclzr/ardrone_mean_state.h"
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>

#define PI 3.1415926535

#define __TRACKER_SKIP_CNT 3
#define _KP_ 0.006
#define _KI_ 0.06
#define _WT_ 0.006
#define __DEBUG_LEVEL_1

using namespace std;

deque<geometry_msgs::PoseStamped> planner_path;

double ardrone_lclzrx;
double ardrone_lclzry;
double ardrone_lclzrthz;
double ardrone_lclzrdt;

//PID controller variables
double tx;
double ty;
double tx_prev;
double ty_prev;

double kp;
double kv;
double ki;
double des_traj_wt;

bool lclzr_callback_occurred;
bool traj_track_mode;
bool traj_track_firstcall;

unsigned int cntr;

double drone_time;
double drone_time_last;

ros::Publisher cmdvel_pub;

int track_trajectory();

void append_onto_path(double xg, double yg)
{

  //dividing dist into 5cm steps
  double xd=(xg-ardrone_lclzrx);
  double yd=(yg-ardrone_lclzry);
  double dg=sqrt(xd*xd+yd*yd);
  unsigned int dsteps=(unsigned int)((dg/0.05)+0.5);
  double xinc=(xd/(dsteps*1.0));
  double yinc=(yd/(dsteps*1.0));

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/map";
  pose.pose.position.x = ardrone_lclzrx;
  pose.pose.position.y = ardrone_lclzry;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  planner_path.push_back(pose);
  
  for(unsigned int i=0; i<dsteps; i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "/map";
      pose.pose.position.x = (ardrone_lclzrx+xinc);
      pose.pose.position.y = (ardrone_lclzry+yinc);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      planner_path.push_back(pose);
    }

}

void path_callback(const nav_msgs::PathConstPtr& path)
{

  unsigned int path_size = planner_path.size();

  if(path_size>0)
    {
      for(unsigned int i=0; i<path_size; i++)
	planner_path.pop_front();
      planner_path.clear();
      cout << "ardrone_cntrlr. planner path cleared.\n";
    }

  unsigned int gpath_size = path->poses.size();
  
  if(gpath_size>0)
    {
      for(unsigned int i=0; i<2; i++)
	{
	  geometry_msgs::PoseStamped pose;
	  pose.header.stamp = ros::Time::now();
	  pose.header.frame_id = "/map";
	  pose.pose.position.x = ardrone_lclzrx;
	  pose.pose.position.y = ardrone_lclzry;
	  pose.pose.position.z = 0.0;
	  pose.pose.orientation.x = 0.0;
	  pose.pose.orientation.y = 0.0;
	  pose.pose.orientation.z = 0.0;
	  pose.pose.orientation.w = 1.0;
	  planner_path.push_back(pose);
	}
      
      for(unsigned int i=0; i<gpath_size; i++)
	{
	  geometry_msgs::PoseStamped pose;
	  pose = path->poses[i];
	  planner_path.push_back(pose);
	}
      cout << "path received. planner_path.size(): " << planner_path.size() << "\n";
    }
  else
    {
      cout << "path_callback. error. planner_path.size(): " << gpath_size << "\n";
    }

}

void lclzr_callback(const ardrone_lclzr::ardrone_mean_state& drone_pose)
{
  ardrone_lclzrx=drone_pose.x;
  ardrone_lclzry=drone_pose.y;
  ardrone_lclzrthz=drone_pose.thz;
  ardrone_lclzrdt=drone_pose.dt;
  
  if(!lclzr_callback_occurred)
    {
      lclzr_callback_occurred=true;      
    }
  else
    {
      drone_time+=ardrone_lclzrdt;
      track_trajectory();
    }
}

int track_trajectory()
{

  cntr++;

  if((cntr%__TRACKER_SKIP_CNT)!=0)
    {
      return 1;
    }
      
  cntr=0;

  if(!traj_track_mode)
    return 0;
  
  if(planner_path.size()==0)
    {
      cout << "trajectory is empty.\n\n";
      return 0;
    }

  if(traj_track_firstcall)
    {
      traj_track_firstcall=false;
      tx=ardrone_lclzrx;
      ty=ardrone_lclzry;
      drone_time_last=drone_time;
    }
  
  if(planner_path.size()>1)
    planner_path.pop_front();
    
  geometry_msgs::PoseStamped pose;
  pose=planner_path.front();

  tx_prev=tx;
  ty_prev=ty;

  tx=pose.pose.position.x;
  ty=pose.pose.position.y;

  double drone_dt=(drone_time-drone_time_last);
  drone_time_last=drone_time;

  double txdiff = (tx-tx_prev);
  double tydiff = (ty-ty_prev);

  double cthz = cos(ardrone_lclzrthz);
  double sthz = sin(ardrone_lclzrthz);

  double txdiff_drone = (cthz*txdiff+sthz*tydiff);
  double tydiff_drone = (-sthz*txdiff+cthz*tydiff);

  double xdot_des;
  double ydot_des;

  if(drone_dt > 0.01)
    {
      xdot_des=(txdiff_drone/drone_dt);
      ydot_des=(tydiff_drone/drone_dt);
    }
  else
    {
      xdot_des=0.0;
      ydot_des=0.0;
    }

  double xperror = (tx-ardrone_lclzrx);
  double yperror = (ty-ardrone_lclzry);

  double xperror_drone = (cthz*xperror+sthz*yperror);
  double yperror_drone = (-sthz*xperror+cthz*yperror);

  /*
  double ux = (xdot_des + kv*xperror_drone + kp*xperror_drone*drone_dt + ki*xperror_drone*drone_dt*drone_dt/2.0);
  double uy = (ydot_des + kv*yperror_drone + kp*yperror_drone*drone_dt + ki*yperror_drone*drone_dt*drone_dt/2.0);
  */

  double ux = (des_traj_wt*xdot_des + kv*xperror_drone + kp*xperror_drone*drone_dt);
  double uy = (des_traj_wt*ydot_des + kv*yperror_drone + kp*yperror_drone*drone_dt);

#ifdef __DEBUG_LEVEL_1
  cout << "planner_path.size(): " << planner_path.size() << "\n";
  cout << "tx, ty: " << tx << "," << ty << "\n";
  cout << "tx_prev, ty_prev: " << tx_prev << "," << ty_prev << "\n";
  cout << "lclzrx,lclzry,lclzrthz: " << ardrone_lclzrx << "," << ardrone_lclzry << "," << ardrone_lclzrthz << "\n";
  cout << "drone_dt: " << drone_dt << "\n";
  cout << "xdot_des,ydot_des: " << xdot_des << "," << ydot_des << "\n";
  cout << "xperror_drone,yperror_drone: " << xperror_drone << "," << yperror_drone << "\n";
  cout << "ux,uy: " << ux << "," << uy << "\n\n\n";
#endif

  //command translation
  geometry_msgs::Twist cmdtwist;
  cmdtwist.linear.x=ux;
  cmdtwist.linear.y=uy;
  cmdtwist.linear.z=0.0;
  cmdtwist.angular.x=0.0;
  cmdtwist.angular.y=0.0;
  cmdtwist.angular.z=0.0;
  cmdvel_pub.publish(cmdtwist);
  
  return 1;

}

void traj_tracking_callback(const std_msgs::Empty &msg)
{

  cout << "Inside trajectory tracking callback.\n\n";

  traj_track_mode=true;
  traj_track_firstcall=true;

  tx=0.0;
  ty=0.0;
  tx_prev=0.0;
  ty_prev=0.0;

  //send a zero twist command to the drone
  geometry_msgs::Twist cmdtwist;
  cmdtwist.linear.x=0.0;
  cmdtwist.linear.y=0.0;
  cmdtwist.linear.z=0.0;
  cmdtwist.angular.x=0.0;
  cmdtwist.angular.y=0.0;
  cmdtwist.angular.z=0.0;
  cmdvel_pub.publish(cmdtwist);

}

void stop_tracking_callback(const std_msgs::Empty &msg)
{

  cout << "Inside stop_tracking_callback.\n";
  traj_track_mode=false;

  tx=0.0;
  ty=0.0;
  tx_prev=0.0;
  ty_prev=0.0;

  //send a zero twist command to the drone
  geometry_msgs::Twist cmdtwist;
  cmdtwist.linear.x=0.0;
  cmdtwist.linear.y=0.0;
  cmdtwist.linear.z=0.0;
  cmdtwist.angular.x=0.0;
  cmdtwist.angular.y=0.0;
  cmdtwist.angular.z=0.0;
  cmdvel_pub.publish(cmdtwist);

  unsigned int path_size = planner_path.size();
  if(path_size>0)
    {
      for(unsigned int i=0; i<path_size; i++)
	planner_path.pop_front();
      planner_path.clear();
      cout << "ardrone_cntrlr. planner path cleared.\n";
    }

}

int teleop_relay(const geometry_msgs::TwistConstPtr &msg)
{
  traj_track_mode=false;

#ifdef __DEBUG_LEVEL_2
  cout << "teleop_relay. cmd_vel msg: \n";
  cout << "msg->linear.(x,y,z): " << msg->linear.x << "," << msg->linear.y << "," << msg->linear.z << "\n";
  cout << "msg->angular.(x,y,z): " << msg->angular.x << "," << msg->angular.y << "," << msg->angular.z << "\n\n";
#endif
  
  geometry_msgs::Twist cmdtwist;
  cmdtwist=(*msg);
  cmdvel_pub.publish(cmdtwist);

  return 1;
}

void cmd_vel_callback(const geometry_msgs::TwistConstPtr &msg)
{
  teleop_relay(msg);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ardrone_cntrlr");
  ros::NodeHandle n;

  cout << "Inside ardrone_cntrlr.\n";

  planner_path.clear();

  drone_time=0.0;
  drone_time_last=0.0;

  //PID controller variables
  tx=0.0;
  ty=0.0;
  tx_prev=0.0;
  ty_prev=0.0;

  kp=_KP_;
  kv=(2.0*sqrt(kp));
  ki=_KI_;
  
  lclzr_callback_occurred=false;
  ardrone_lclzrx=6.85;
  ardrone_lclzry=43.05;
  ardrone_lclzrthz=0.0;
  ardrone_lclzrdt=0.0;

  traj_track_mode=false;

  des_traj_wt=_WT_;
  cntr=0;

  ros::Subscriber lclzr_sub = n.subscribe("/ardrone_mean_state",1,&lclzr_callback);
  ros::Subscriber plan_sub = n.subscribe("/plan",1,&path_callback);
  cmdvel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",0);

  ros::Subscriber traj_tracking_sub = n.subscribe("/traj_tracking_start",1,&traj_tracking_callback);
  ros::Subscriber stop_tracking_sub = n.subscribe("/stop_tracking",1,&stop_tracking_callback);
  ros::Subscriber cmdvel_sub = n.subscribe("/ardrone_cntrlr/cmd_vel",1,&cmd_vel_callback);

  ros::Rate loop_rate(40);

  while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }

  return 1;

}




