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
double ardrone_lclzrvx_droneframe;
double ardrone_lclzrvy_droneframe;
double ardrone_lclzrthz;

//PID controller variables
double xdes;
double ydes;
double xdes_p;
double ydes_p;
double xdes_dot;
double ydes_dot;
double xdes_dot_p;
double ydes_dot_p;
double xdes_ddot;
double ydes_ddot;
double thz;
double thz_p;
double thz_dot;
double thz_dot_p;
double vxdrone_map;
double vydrone_map;
double vxdrone_map_p;
double vydrone_map_p;

double xerr;
double yerr;
double xerr_p;
double yerr_p;
double xerr_i;
double yerr_i;
double thzerr;
double thzerr_p;

double kp;
double kv;
double ki;
double des_traj_wt;

bool traj_track_mode;
bool traj_track_firstcall;

unsigned int gcntr;
unsigned int cntr;

double drone_time;
double drone_time_last;

ros::Publisher cmdvel_pub;

int track_trajectory();

void getSkewSymMatrix(const btVector3& ang_vel, btMatrix3x3& ssmat)
{
  ssmat[0][0]=ssmat[1][1]=ssmat[2][2]=0.0;
  ssmat[0][1]=-ang_vel[2];
  ssmat[1][0]=ang_vel[2];
  ssmat[0][2]=ang_vel[1];
  ssmat[2][0]=-ang_vel[1];
  ssmat[1][2]=-ang_vel[0];
  ssmat[2][1]=ang_vel[0];
}

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
  ardrone_lclzrvx_droneframe=drone_pose.vx_droneframe;
  ardrone_lclzrvy_droneframe=drone_pose.vy_droneframe;
  drone_time+=drone_pose.dt;
  track_trajectory();
}

int track_trajectory()
{
  gcntr++;

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
      drone_time_last=drone_time;
    }

  double drone_dt=(drone_time-drone_time_last);
  drone_time_last=drone_time;

  thz_p=thz;
  thz=ardrone_lclzrthz;

  thz_dot_p=thz_dot;

  if((drone_dt>0.01)&&(thz_p!=-9999))
    {
      //calculate ardrone angular velocity about its z axis
      double thzdiff=0.0;
      if(fabs(thz)<0.001) thz=0.0;
      if(fabs(thz_p)<0.001) thz_p=0.0;
      if((thz>(PI/2)) && (thz_p<(-PI/2)))
	{
	  double thdiff = (thz-thz_p);
	  thzdiff = (thdiff-(2*PI));
	}
      else if((thz<(-PI/2)) && (thz_p>(PI/2)))
	{
	  double thdiff = (thz-thz_p);
	  thzdiff = (thdiff+(2*PI));
	}
      else
	{
	  thzdiff = (thz-thz_p);
	}
      thz_dot=(thzdiff/drone_dt);
    }

  double thz_ddot=0.0;

  if((drone_dt>0.01)&&(thz_dot_p!=-9999))
    {
      thz_ddot = ((thz_dot-thz_dot_p)/drone_dt);
    }

  if(planner_path.size()>1)
    planner_path.pop_front();
    
  geometry_msgs::PoseStamped pose;
  pose=planner_path.front();

  xdes_p = xdes;
  ydes_p = ydes;

  xdes = pose.pose.position.x;
  ydes = pose.pose.position.y;

  xerr_p = xerr;
  yerr_p = yerr;

  double xerr_map = (xdes-ardrone_lclzrx);
  double yerr_map = (ydes-ardrone_lclzry);

  thzerr_p = thzerr;

  double thz_des = atan2(yerr_map,xerr_map);

  if(fabs(thz_des)<0.001) thz_des=0.0;
  if(fabs(thz)<0.001) thz=0.0;
  if((thz_des>(PI/2)) && (thz<(-PI/2)))
    {
      double errval = (thz_des-thz);
      thzerr = (errval-(2*PI));
    }
  else if((thz_des<(-PI/2)) && (thz>(PI/2)))
    {
      double errval = (thz_des-thz);
      thzerr = (errval+(2*PI));
    }
  else
    {
      thzerr = (thz_des-thz);
    }

  double thzerr_dot=0.0;

  if((drone_dt>0.01)&&(thzerr_p!=-9999))
    {
      double thzdiff=0.0;
      if(fabs(thzerr)<0.001) thzerr=0.0;
      if(fabs(thzerr_p)<0.001) thzerr_p=0.0;
      if((thzerr>(PI/2)) && (thzerr_p<(-PI/2)))
	{
	  double errval = (thzerr-thzerr_p);
	  thzdiff = (errval-(2*PI));
	}
      else if((thzerr<(-PI/2)) && (thzerr_p>(PI/2)))
	{
	  double errval = (thzerr-thzerr_p);
	  thzdiff = (errval+(2*PI));
	}
      else
	{
	  thzdiff = (thzerr-thzerr_p);
	}
      thzerr_dot = (thzdiff/drone_dt);
    }

  double cthz = cos(ardrone_lclzrthz);
  double sthz = sin(ardrone_lclzrthz);

  xerr = (cthz*xerr_map+sthz*yerr_map);
  yerr = (-sthz*xerr_map+cthz*yerr_map);

  double xerr_dot=0.0;
  double yerr_dot=0.0;

  if((drone_dt>0.01)&&(xerr_p!=-9999))
    {
      xerr_dot = ((xerr-xerr_p)/drone_dt);
      yerr_dot = ((yerr-yerr_p)/drone_dt);
    }

  xdes_dot_p = xdes_dot;
  ydes_dot_p = ydes_dot;

  if((drone_dt>0.01)&&(xdes_p!=-9999))
    {
      xdes_dot = ((xdes-xdes_p)/drone_dt);
      ydes_dot = ((ydes-ydes_p)/drone_dt);
    }
  
  if((drone_dt>0.01)&&(xdes_dot_p!=-9999))
    {
      xdes_ddot = ((xdes_dot-xdes_dot_p)/drone_dt);
      ydes_ddot = ((ydes_dot-ydes_dot_p)/drone_dt);
    }

  double xdes_d_map=0.0;
  double ydes_d_map=0.0;
  double xdes_dd_map=0.0;
  double ydes_dd_map=0.0;
  double thd=0.0;
  double thdd=0.0;

  if(xdes_dot!=-9999)
    {
      xdes_d_map=xdes_dot;
      ydes_d_map=ydes_dot;
    }

  if(xdes_ddot!=-9999)
    {
      xdes_dd_map=xdes_ddot;
      ydes_dd_map=ydes_ddot;
    }

  if(thz_dot!=-9999)
    thd=thz_dot;

  thdd=thz_ddot;

  btVector3 angvel(0.0,0.0,thd);
  btMatrix3x3 thd_ss;
  getSkewSymMatrix(angvel,thd_ss);
  
  btVector3 angacc(0.0,0.0,thdd);
  btMatrix3x3 thdd_ss;
  getSkewSymMatrix(angacc,thdd_ss);

  double vx_dr = ardrone_lclzrvx_droneframe;
  double vy_dr = ardrone_lclzrvy_droneframe;

  vxdrone_map_p = vxdrone_map;
  vydrone_map_p = vydrone_map;

  vxdrone_map = (cthz*vx_dr-sthz*vy_dr);
  vydrone_map = (sthz*vx_dr+cthz*vy_dr);

  double axdrone_map=0.0;
  double aydrone_map=0.0;

  if((drone_dt>0.01)&&(vxdrone_map_p!=-9999))
    {
      axdrone_map = ((vxdrone_map-vxdrone_map_p)/drone_dt);
      aydrone_map = ((vydrone_map-vydrone_map_p)/drone_dt);
    }

  //compute velocity of desired x,y wrt drone in the drone frame
  btVector3 dr_vdes_to_dr(0.0,0.0,0.0);
  btVector3 m_vdes(xdes_d_map,ydes_d_map,0.0);
  btVector3 m_vdr(vxdrone_map,vydrone_map,0.0);
  btVector3 m_rp_to_q;



  /*
  double ux = (xdot_des + kv*xerror_drone + kp*xerror_drone*drone_dt + ki*xerror_drone*drone_dt*drone_dt/2.0);
  double uy = (ydot_des + kv*yerror_drone + kp*yerror_drone*drone_dt + ki*yerror_drone*drone_dt*drone_dt/2.0);
  */

  double ux = (des_traj_wt*xdot_des + kv*xerror_drone + kp*xerror_drone*drone_dt);
  double uy = (des_traj_wt*ydot_des + kv*yerror_drone + kp*yerror_drone*drone_dt);

#ifdef __DEBUG_LEVEL_1
  cout << "planner_path.size(): " << planner_path.size() << "\n";
  cout << "tx, ty: " << tx << "," << ty << "\n";
  cout << "tx_prev, ty_prev: " << tx_prev << "," << ty_prev << "\n";
  cout << "lclzrx,lclzry,lclzrthz: " << ardrone_lclzrx << "," << ardrone_lclzry << "," << ardrone_lclzrthz << "\n";
  cout << "drone_dt: " << drone_dt << "\n";
  cout << "xdot_des,ydot_des: " << xdot_des << "," << ydot_des << "\n";
  cout << "xerror_drone,yerror_drone: " << xerror_drone << "," << yerror_drone << "\n";
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

  xdes=0.0;
  ydes=0.0;
  xdes_p=0.0;
  ydes_p=0.0;
  xdes_dot=0.0;
  ydes_dot=0.0;
  xdes_dot_p=0.0;
  ydes_dot_p=0.0;
 
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

  xdes=0.0;
  ydes=0.0;
  xdes_p=0.0;
  ydes_p=0.0;
  xdes_dot=0.0;
  ydes_dot=0.0;
  xdes_dot_p=0.0;
  ydes_dot_p=0.0;

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

  xdes=-9999;
  ydes=-9999;
  xdes_p=-9999;
  ydes_p=-9999;
  xdes_dot=-9999;
  ydes_dot=-9999;
  xdes_dot_p=-9999;
  ydes_dot_p=-9999;
  thz=-9999;
  thz_p=-9999;
  thz_dot=-9999;
  thz_dot_p=-9999;
  vxdrone_map=-9999;
  vydrone_map=-9999;
  vxdrone_map_p=-9999;
  vydrone_map_p=-9999;

  xerr=-9999;
  yerr=-9999;
  xerr_p=-9999;
  yerr_p=-9999;
  xerr_i=-9999;
  yerr_i=-9999;
  thzerr=-9999;
  thzerr_p=-9999;

  kp=_KP_;
  kv=(2.0*sqrt(kp));
  ki=_KI_;
  
  ardrone_lclzrx=6.85;
  ardrone_lclzry=43.05;
  ardrone_lclzrthz=0.0;
  ardrone_lclzrvx_droneframe=0.0;
  ardrone_lclzrvy_droneframe=0.0;

  traj_track_mode=false;

  des_traj_wt=_WT_;

  gcntr=0;
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




