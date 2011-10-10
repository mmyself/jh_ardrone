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

#define __DEBUG_LEVEL_1

using namespace std;

deque<geometry_msgs::PoseStamped> planner_path;

double ardrone_lclzrx;
double ardrone_lclzry;
double ardrone_lclzrthz;
double ardrone_lclzrdthz;
double ardrone_lclzrdt;

//PID controller variables
double derror;
double xperror;
double yperror;
double thzerror;
double derror_prev;
double xperror_prev;
double yperror_prev;
double thzerror_prev;
double derror_i;
double xperror_i;
double yperror_i;
double thzerror_i;
double kp;
double kpthz;
double kv;
double kvthz;
double ki;
double kithz;

bool lclzr_callback_occurred;
ros::Publisher cmdvel_pub;
bool traj_track_mode;
bool traj_track_new;
bool tracking_yaw;
bool traj_track_first_pose;
unsigned int gpath_size;
int track_trajectory();

void insert_into_path(double xg, double yg)
{
  /*
  unsigned int gsize=planner_path.size();
  for(unsigned int i=0; i<gsize; i++)
    {
      planner_path.pop_front();
    }
  planner_path.clear();
  */

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
  
  for(int i=0; i<dsteps; i++)
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

  gpath_size = path->poses.size();
  
  if(gpath_size>0)
    {
  
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



  ardrone_lclzrdthz=(drone_pose.thz-ardrone_lclzrthz);
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
      track_trajectory();
    }
}

int track_trajectory()
{
  if(!lclzr_callback_occurred)
    {
      cout << "No data from localizer yet.\n";
      return 0;
    }

  double targetx=0.0;
  double targety=0.0;

  bool dir_forward=true;
  bool do_not_track=false;
  bool last_pose=false;

  if(traj_track_mode)
    {
      if(planner_path.size()==0)
	{
	  cout << "planner tracker. 0. trajecotry is empty.\n";
	  return 0;
	}

#ifdef __DEBUG_LEVEL_1
      cout << "planner tracker. 1. planner_path.size(): " << planner_path.size() << "\n";
#endif

      if(planner_path.size()==1)
	{
	  last_pose=true;
	  geometry_msgs::PoseStamped temppose1;
	  temppose1 = planner_path.front();
	  targetx=temppose1.pose.position.x;
	  targety=temppose1.pose.position.y;
	}
      else if(planner_path.size()>1)
	{
	  geometry_msgs::PoseStamped temppose1;
	  planner_path.pop_front();
	  temppose1 = planner_path.front();
	  targetx=temppose1.pose.position.x;
	  targety=temppose1.pose.position.y;
	}
      
#ifdef __DEBUG_LEVEL_1
      cout << "planner tracker. 2. planner_path.size(): " << planner_path.size() << "\n";
      cout << "planner tracker. 3. targetx, targety: " << targetx << "," << targety << "\n";
      cout << "planner tracker. 4. lclzrx,lclzry: " << ardrone_lclzrx << "," << ardrone_lclzry << "\n";
      cout << "planner tracker. 5.a. lclzrthz: " << ardrone_lclzrthz << "\n";
#endif
      
      xperror_prev = xperror;
      yperror_prev = yperror;
      xperror = (targetx-ardrone_lclzrx);
      yperror = (targety-ardrone_lclzry);
      //double xperror_difft = ((xperror-xperror_prev)/ardrone_lclzrdt);
      //double yperror_difft = ((yperror-yperror_prev)/ardrone_lclzrdt);
      xperror_i+=xperror;
      yperror_i+=yperror;

      double thz = atan2(yperror,xperror);
      double thzerr=0.0;

#ifdef __DEBUG_LEVEL_1
      cout << "planner tracker. 6. xperror,yperror,thz: " << xperror << "," << yperror << "," << thz << "\n";
#endif
      
      if(fabs(thz)<0.001) thz=0.0;
      if(fabs(ardrone_lclzrthz)<0.001) ardrone_lclzrthz=0.0;
      if((thz>(PI/2)) && (ardrone_lclzrthz<(-PI/2)))
	{
	  double errval = (thz-ardrone_lclzrthz);
	  thzerr = (errval-(2*PI));
	}
      else if((thz<(-PI/2)) && (ardrone_lclzrthz>(PI/2)))
	{
	  double errval = (thz-ardrone_lclzrthz);
	  thzerr = (errval+(2*PI));
	}
      else
	{
	  thzerr = (thz-ardrone_lclzrthz);
	}

      thzerror_prev=thzerror;
      thzerror=thzerr;

      derror_prev=derror;
      derror=sqrt(xperror*xperror+yperror*yperror);

      double derror_difft=((derror-derror_prev)/ardrone_lclzrdt);
      double thzerror_difft=((thzerror-thzerror_prev)/ardrone_lclzrdt);

      derror_i+=derror;
      thzerror_i+=thzerror;

      double drone_velx;
      double drone_vely;
      double drone_velthz;

      if(last_pose || traj_track_first_pose)
	{
	  //track x and y independently
	  //dont track yaw angle
	  if(traj_track_new)
	    {
	      drone_velx = kp*xperror;
	      drone_vely = kp*yperror;
	      drone_velthz = 0.0;
	      traj_track_new=false;
	    }
	  else
	    {
	      drone_velx = (kp*xperror+kv*xperror_difft);
	      drone_vely = (kp*yperror+kv*yperror_difft);
	      drone_velthz = 0.0;
	    }
	  
	  //command translation
	  geometry_msgs::Twist cmdtwist;
	  cmdtwist.linear.x=drone_velx;
	  cmdtwist.linear.y=drone_vely;
	  cmdtwist.linear.z=0.0;
	  cmdtwist.angular.x=0.0;
	  cmdtwist.angular.y=0.0;
	  cmdtwist.angular.z=0.0;
	  cmdvel_pub.publish(cmdtwist);

	}
      else 
	{
	  if(traj_track_new)
	    {
	      //when you first start tracking a new point, there is no derivative term
	      drone_velx = kp*derror;
	      drone_vely = 0.0;
	      drone_velthz = kpthz*thzerror;
	      traj_track_new=false;
	    }
	  else
	    {
	      drone_velx = (kp*derror+kv*derror_difft);
	      drone_vely = 0.0;
	      drone_velthz = (kpthz*thzerror+kvthz*thzerror_difft);
	    }



#ifdef __DEBUG_LEVEL_1
	  cout << "planner tracker. 7. cmdtwist. velx,vely,velthz: " << drone_velx << "," << drone_vely << "," << drone_velthz << ", tracking_yaw: " << tracking_yaw << "\n\n";
#endif

	  //command translation
	  geometry_msgs::Twist cmdtwist;
	  if(tracking_yaw)
	    cmdtwist.linear.x=0.0;
	  else
	    cmdtwist.linear.x=drone_velx;
	  cmdtwist.linear.y=0.0;
	  cmdtwist.linear.z=0.0;
	  cmdtwist.angular.x=0.0;
	  cmdtwist.angular.y=0.0;
	  cmdtwist.angular.z=drone_velthz;
	  cmdvel_pub.publish(cmdtwist);
	}
      

    }      

  return 1;
}

void traj_tracking_callback(const std_msgs::Empty &msg)
{

  cout << "Inside trajectory tracking callback.\n\n";
  traj_track_mode=true;
  traj_track_first_pose=true;
  derror=0.0;
  thzerror=0.0;
  derror_prev=0.0;
  thzerror_prev=0.0;
  derror_i=0.0;
  thzerror_i=0.0;
  xperror=0.0;
  yperror=0.0;
  xperror_prev=0.0;
  yperror_prev=0.0;
  xperror_i=0.0;
  yperror_i=0.0;
  traj_track_new=true;

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
  traj_track_first_pose=true;

  derror=0.0;
  thzerror=0.0;
  derror_prev=0.0;
  thzerror_prev=0.0;
  derror_i=0.0;
  thzerror_i=0.0;
  xperror=0.0;
  yperror=0.0;
  xperror_prev=0.0;
  yperror_prev=0.0;
  xperror_i=0.0;
  yperror_i=0.0;

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

  //PID controller variables
  derror=0.0;
  thzerror=0.0;
  derror_prev=0.0;
  thzerror_prev=0.0;
  derror_i=0.0;
  thzerror_i=0.0;
  xperror=0.0;
  yperror=0.0;
  xperror_prev=0.0;
  yperror_prev=0.0;
  xperror_i=0.0;
  yperror_i=0.0;
  kp=0.1;
  kpthz=0.1;
  kv=0.0;
  kvthz=0.0;
  ki=0.0;
  kithz=0.0;
  
  lclzr_callback_occurred=false;

  ardrone_lclzrdthz=0.0;
  ardrone_lclzrx=6.85;
  ardrone_lclzry=43.05;
  ardrone_lclzrthz=0.0;
  ardrone_lclzrdt=0.0;
  gpath_size=0;

  traj_track_mode=false;
  traj_track_new=true;
  tracking_yaw=true;
  traj_track_first_pose=true;
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

  return 0;

}
