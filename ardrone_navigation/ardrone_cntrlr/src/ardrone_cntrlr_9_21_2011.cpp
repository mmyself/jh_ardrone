#include <ros/ros.h>
#include <iostream>
#include <deque>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

#include "ardrone_lclzr/ardrone_mean_state.h"
#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>

#define PI 3.1415926535

#define __DEBUG_LEVEL_1

#define __TEST1_CNTRLR_
using namespace std;

deque<geometry_msgs::PoseStamped> planner_path;
deque<geometry_msgs::PoseStamped> teleop_path;

double ardrone_lclzrx;
double ardrone_lclzry;
double ardrone_lclzrthz;
double ardrone_lclzrdthz;

//PID controller variables
double ex;
double ey;
double ethz;
double ex_prev;
double ey_prev;
double ethz_prev;
double eix;
double eiy;
double eithz;
double kpx;
double kpy;
double kpthz;
double kvx;
double kvy;
double kvthz;
double kix;
double kiy;
double kithz;

bool lclzr_callback_occurred;
unsigned int cntr;
ros::Publisher cmdvel_pub;
bool traj_track_mode;
bool teleop_mode;

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

  path_size = path->poses.size();
  
  if(path_size>0)
    {
  
      for(unsigned int i=0; i<path_size; i++)
	{
	  geometry_msgs::PoseStamped pose;
	  pose = path->poses[i];
	  planner_path.push_back(pose);
	}
      cout << "path received. planner_path.size(): " << planner_path.size() << "\n";
    }
  else
    {
      cout << "path_callback. error. planner_path.size(): " << path_size << "\n";
    }

  /*
  deque<geometry_msgs::PoseStamped>::iterator it=planner_path.begin();

  cout << "Inside path_callback.\n";
  unsigned int i=0;
  for(; it!=planner_path.end(); it++)
    {
      cout << "planner_path[" << i << "].pose : ";
      cout << (*it).pose.position.x << "," << (*it).pose.position.y << "\n";
      i++;
      planner_path.pop_front();
    }
  */
  
  /*
  cout << "Inside path_callback.\n";
  geometry_msgs::PoseStamped temppose;
  
  for(unsigned int i=0; i<path_size; i++)
    {
      cout << "pose[" << i << "] : ";
      temppose = planner_path.front();
      planner_path.pop_front();
      cout << temppose.pose.position.x << "," << temppose.pose.position.y << "\n";
    }
  */

}



void lclzr_callback(const ardrone_lclzr::ardrone_mean_state& drone_pose)
{

  lclzr_callback_occurred=true;

  ardrone_lclzrdthz=(drone_pose.thz-ardrone_lclzrthz);
  ardrone_lclzrx=drone_pose.x;
  ardrone_lclzry=drone_pose.y;
  ardrone_lclzrthz=drone_pose.thz;

  /*
  if(path_received)
    {
      ardrone_lclzrx=drone_pose.x;
      ardrone_lclzry=drone_pose.y;
      ardrone_lclzrthz=drone_pose.thz;
      
      if(first_lclzr_callback)
	{
	  cout << "Path received. Inside lclzr_callback first time.\n";
	  first_lclzr_callback=false;
	  geometry_msgs::PoseStamped temppose;
	  temppose.header.stamp=ros::Time();
	  temppose.header.frame_id="/map";
	  temppose.pose.position.x=ardrone_lclzrx;
	  temppose.pose.position.y=ardrone_lclzry;
	  temppose.pose.position.z=0.0;
	  btVector3 rotaxis(0,0,1);
	  btQuaternion qrot(rotaxis,ardrone_lclzrthz);
	  temppose.pose.orientation.x = qrot.x();
	  temppose.pose.orientation.y = qrot.y();
	  temppose.pose.orientation.z = qrot.z();
	  temppose.pose.orientation.w = qrot.w();
	  planner_path.push_front(temppose);
	}
    }
  */
}


int track_trajectory()
{
  if(!lclzr_callback_occurred)
    {
      cout << "No data from localizer yet.\n";
      return 0;
    }

  //cout << "Inside track_trajectory.\n";
  
  /*
  cntr++;

  if(cntr%30!=0)
    {
      return 1;
    }
  else
    {
      cntr=0;
    }
  */

  double targetx=0.0;
  double targety=0.0;

  bool dir_forward=true;

  if(traj_track_mode)
    {
      if(planner_path.size()==0)
	{
	  return 0;
	}

#ifdef __DEBUG_LEVEL_1
      cout << "planner tracker. 1. planner_path.size(): " << planner_path.size() << "\n";
#endif

      if(planner_path.size()==1)
	{
	  geometry_msgs::PoseStamped temppose;
	  temppose = planner_path.front();
	  targetx=temppose.pose.position.x;
	  targety=temppose.pose.position.y;
	  //there is only one pose left in the path
	  //so we dont have to rotate the drone about its yaw
	  dir_forward=true;
	}
      if(planner_path.size()>1)
	{
	  geometry_msgs::PoseStamped temppose,temppose2;
	  temppose = planner_path.front();
	  planner_path.pop_front();
	  targetx=temppose.pose.position.x;
	  targety=temppose.pose.position.y;

	  temppose2 = planner_path.front();
	  double x1 = temppose2.pose.position.x;
	  double y1 = temppose2.pose.position.y;
	  double xd = (x1-targetx);
	  double yd = (y1-targety);
	  double th1 = atan2(yd,xd);
	  if(fabs(th1)>2.0)
	    dir_forward=false;
	  
	  double xerr = (targetx-ardrone_lclzrx);
	  double yerr = (targety-ardrone_lclzry);
	  double thzerr = atan2(yerr,xerr);
	  double errdist = sqrt(xerr*xerr+yerr*yerr);

	  if(fabs(thzerr)<0.001) thzerr=0.0;
	  if(fabs(ardrone_lclzrthz)<0.001) ardrone_lclzrthz=0.0;
	  if((thzerr>(PI/2)) && (ardrone_lclzrthz<(-PI/2)))
	    {
	      double errval = (thzerr-ardrone_lclzrthz);
	      thzerr = (errval-(2*PI));
	    }
	  else if((thzerr<(-PI/2)) && (ardrone_lclzrthz>(PI/2)))
	    {
	      double errval = (thzerr-ardrone_lclzrthz);
	      thzerr = (errval+(2*PI));
	    }
	  else
	    {
	      thzerr = (thzerr-ardrone_lclzrthz);
	    }
	  
	  /*
	  if((fabs(thzerr)<(0.008726*3))&&(errdist<0.03))
	    {
	      planner_path.pop_front();
	      temppose = planner_path.front();
	      targetx=temppose.pose.position.x;
	      targety=temppose.pose.position.y;
	    }
	  */
	}

      /*
      double cthz = cos(ardrone_lclzrthz);
      double sthz = sin(ardrone_lclzrthz);
      
      double tx=((cthz*targetx)-(sthz*targety));
      double ty=((sthz*targetx)+(cthz*targety));
      
      ex_prev=ex;
      ey_prev=ey;
      ethz_prev=ethz;
      
      ex=(tx-ardrone_lclzrx);
      ey=(ty-ardrone_lclzry);

      ex_prev=ex;
      ey_prev=ey;
      ethz_prev=ethz;
      
      ex=(targetx-ardrone_lclzrx);
      ey=(targety-ardrone_lclzry);

      //target thz -> tthz
      double tthz=atan2(ey,ex);
      ethz=(tthz-ardrone_lclzrthz);
      */

#ifdef __DEBUG_LEVEL_1
      cout << "planner tracker. 2. planner_path.size(): " << planner_path.size() << "\n";
      cout << "planner tracker. 3. targetx, targety: " << targetx << "," << targety << "\n";
      cout << "planner tracker. 4. lclzrx,lclzry: " << ardrone_lclzrx << "," << ardrone_lclzry << "\n";
      cout << "planner tracker. 5. lclzrthz: " << ardrone_lclzrthz << "\n";
#endif

      double xerr = (targetx-ardrone_lclzrx);
      double yerr = (targety-ardrone_lclzry);
      double thzerr = atan2(yerr,xerr);

#ifdef __DEBUG_LEVEL_1
      cout << "planner tracker. 6. xerr,yerr,thzerr: " << xerr << "," << yerr << "," << thzerr << "\n";
#endif
      
      if(fabs(thzerr)<0.001) thzerr=0.0;
      if(fabs(ardrone_lclzrthz)<0.001) ardrone_lclzrthz=0.0;
      if((thzerr>(PI/2)) && (ardrone_lclzrthz<(-PI/2)))
	{
	  double errval = (thzerr-ardrone_lclzrthz);
	  thzerr = (errval-(2*PI));
	}
      else if((thzerr<(-PI/2)) && (ardrone_lclzrthz>(PI/2)))
	{
	  double errval = (thzerr-ardrone_lclzrthz);
	  thzerr = (errval+(2*PI));
	}
      else
	{
	  thzerr = (thzerr-ardrone_lclzrthz);
	}

      ethz_prev=ethz;
      ethz=thzerr;
      
      //note, have to map ex, ey from previous frame to current frame
      double cdthz = cos(ardrone_lclzrdthz);
      double sdthz = sin(ardrone_lclzrdthz);
      ex_prev=((cdthz*ex)+(sdthz*ey));
      ey_prev=((-sdthz*ex)+(cdthz*ey));

      //Map xerr and yerr to drone coordinate frame
      double cthz = cos(ardrone_lclzrthz);
      double sthz = sin(ardrone_lclzrthz);
      ex = ((cthz*xerr)-(sthz*yerr));
      ey = ((sthz*xerr)+(cthz*yerr));

      double drone_velx = kpx*ex;
      double drone_vely = kpy*ey;
      double drone_velthz = kpthz*ethz;

#ifdef __DEBUG_LEVEL_1
      cout << "planner tracker. 7. cmdtwist. velx,vely,velthz: " << drone_velx << "," << drone_vely << "," << drone_velthz << "\n\n";
#endif

      //sometimes the drone drifts forward and the point to servo to is behind it
      //to avoid rotating the drone backwards to face this point, dont track this point
      if(dir_forward && fabs(ethz)>2.0)
	{
	  ex=0.0;
	  ey=0.0;
	  ethz=0.0;
	  ex_prev=0.0;
	  ey_prev=0.0;
	  ethz_prev=0.0;
	}
      else 
	{
	  geometry_msgs::Twist cmdtwist;
	  cmdtwist.linear.x=drone_velx;
	  cmdtwist.linear.y=drone_vely;
	  cmdtwist.linear.z=0.0;
	  cmdtwist.angular.x=0.0;
	  cmdtwist.angular.y=0.0;
	  cmdtwist.angular.z=drone_velthz;
      
	  cmdvel_pub.publish(cmdtwist);
	}

    }      
  else if(teleop_mode)
    {

      if(teleop_path.size()==0)
	{
	  return 0;
	}

#ifdef __DEBUG_LEVEL_2
      cout << "teleop tracker. 1. teleop_path.size(): " << teleop_path.size() << "\n";
#endif

      if(teleop_path.size()==1)
	{
	  geometry_msgs::PoseStamped temppose;
	  temppose = teleop_path.front();
	  targetx=temppose.pose.position.x;
	  targety=temppose.pose.position.y;
	  
	}
      if(teleop_path.size()>1)
	{
	  geometry_msgs::PoseStamped temppose;
	  temppose = teleop_path.front();
	  teleop_path.pop_front();
	  targetx=temppose.pose.position.x;
	  targety=temppose.pose.position.y;
	  double xerr = (targetx-ardrone_lclzrx);
	  double yerr = (targety-ardrone_lclzry);
	  double thzerr = atan2(yerr,xerr);
	  double errdist = sqrt(xerr*xerr+yerr*yerr);

	  if(fabs(thzerr)<0.001) thzerr=0.0;
	  if(fabs(ardrone_lclzrthz)<0.001) ardrone_lclzrthz=0.0;
	  if((thzerr>(PI/2)) && (ardrone_lclzrthz<(-PI/2)))
	    {
	      double errval = (thzerr-ardrone_lclzrthz);
	      thzerr = (errval-(2*PI));
	    }
	  else if((thzerr<(-PI/2)) && (ardrone_lclzrthz>(PI/2)))
	    {
	      double errval = (thzerr-ardrone_lclzrthz);
	      thzerr = (errval+(2*PI));
	    }
	  else
	    {
	      thzerr = (thzerr-ardrone_lclzrthz);
	    }
	  /*
	  if((fabs(thzerr)<0.008726)&&(errdist<0.01))
	    {
	      teleop_path.pop_front();
	      temppose = teleop_path.front();
	      targetx=temppose.pose.position.x;
	      targety=temppose.pose.position.y;
	    }
	  */

	}

      /*
      double cthz = cos(ardrone_lclzrthz);
      double sthz = sin(ardrone_lclzrthz);
      
      double tx=((cthz*targetx)-(sthz*targety));
      double ty=((sthz*targetx)+(cthz*targety));
      
      ex_prev=ex;
      ey_prev=ey;
      ethz_prev=ethz;
      
      ex=(tx-ardrone_lclzrx);
      ey=(ty-ardrone_lclzry);
      */

#ifdef __DEBUG_LEVEL_2
      cout << "teleop tracker. 2. teleop_path.size(): " << teleop_path.size() << "\n";
      cout << "teleop tracker. 3. targetx, targety: " << targetx << "," << targety << "\n";
      cout << "teleop tracker. 4. lclzrx,lclzry: " << ardrone_lclzrx << "," << ardrone_lclzry << "\n";
      cout << "teleop tracker. 5. lclzrthz: " << ardrone_lclzrthz << "\n";
#endif

      double xerr = (targetx-ardrone_lclzrx);
      double yerr = (targety-ardrone_lclzry);
      double thzerr = atan2(yerr,xerr);

#ifdef __DEBUG_LEVEL_2
      cout << "teleop tracker. 6. xerr,yerr,thzerr: " << xerr << "," << yerr << "," << thzerr << "\n";
#endif

      
      if(fabs(thzerr)<0.001) thzerr=0.0;
      if(fabs(ardrone_lclzrthz)<0.001) ardrone_lclzrthz=0.0;
      if((thzerr>(PI/2)) && (ardrone_lclzrthz<(-PI/2)))
	{
	  double errval = (thzerr-ardrone_lclzrthz);
	  thzerr = (errval-(2*PI));
	}
      else if((thzerr<(-PI/2)) && (ardrone_lclzrthz>(PI/2)))
	{
	  double errval = (thzerr-ardrone_lclzrthz);
	  thzerr = (errval+(2*PI));
	}
      else
	{
	  thzerr = (thzerr-ardrone_lclzrthz);
	}

      ethz=thzerr;

      //note, have to map ex, ey from previous frame to current frame
      double cdthz = cos(ardrone_lclzrdthz);
      double sdthz = sin(ardrone_lclzrdthz);
      ex_prev=((cdthz*ex)+(sdthz*ey));
      ey_prev=((-sdthz*ex)+(cdthz*ey));
      ethz_prev=ethz;

      //Map xerr and yerr to drone coordinate frame
      double cthz = cos(ardrone_lclzrthz);
      double sthz = sin(ardrone_lclzrthz);
      ex = ((cthz*xerr)-(sthz*yerr));
      ey = ((sthz*xerr)+(cthz*yerr));

      double drone_velx = kpx*ex;
      double drone_vely = kpy*ey;
      double drone_velthz = kpthz*ethz;

#ifdef __DEBUG_LEVEL_2
      cout << "teleop tracker. 7. cmdtwist. velx,vely,velthz: " << drone_velx << "," << drone_vely << "," << drone_velthz << "\n\n";
#endif
      
      geometry_msgs::Twist cmdtwist;
      cmdtwist.linear.x=drone_velx;
      cmdtwist.linear.y=drone_vely;
      cmdtwist.linear.z=0.0;
      cmdtwist.angular.x=0.0;
      cmdtwist.angular.y=0.0;
      cmdtwist.angular.z=drone_velthz;
      
      cmdvel_pub.publish(cmdtwist);


    }
  return 1;


}

void traj_tracking_callback(const std_msgs::Empty &msg)
{

  cout << "Inside trajectory tracking callback.\n\n";
  traj_track_mode=true;
  teleop_mode=false;

  ex=0.0;
  ey=0.0;
  ethz=0.0;
  ex_prev=0.0;
  ey_prev=0.0;
  ethz_prev=0.0;
  eix=0.0;
  eiy=0.0;
  eithz=0.0;
  cntr=0;

  //send a zero twist command to the drone
  geometry_msgs::Twist cmdtwist;
  cmdtwist.linear.x=0.0;
  cmdtwist.linear.y=0.0;
  cmdtwist.linear.z=0.0;
  cmdtwist.angular.x=0.0;
  cmdtwist.angular.y=0.0;
  cmdtwist.angular.z=0.0;
  cmdvel_pub.publish(cmdtwist);

  /*
  unsigned int path_size = teleop_path.size();

  if(path_size>0)
    {
      for(unsigned int i=0; i<path_size; i++)
	teleop_path.pop_front();
      teleop_path.clear();
      cout << "ardrone_cntrlr. teleop trajectory cleared.\n";
    }
  else
    {
      cout << "ardrone_cntrlr: No teleop trajectory.\n";
    }
  */

}

void stop_tracking_callback(const std_msgs::Empty &msg)
{

  cout << "Inside stop_tracking_callback.\n";
  teleop_mode=false;
  traj_track_mode=false;

  ex=0.0;
  ey=0.0;
  ethz=0.0;
  ex_prev=0.0;
  ey_prev=0.0;
  ethz_prev=0.0;
  eix=0.0;
  eiy=0.0;
  eithz=0.0;
  cntr=0;

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

  path_size = teleop_path.size();
  if(path_size>0)
    {
      for(unsigned int i=0; i<path_size; i++)
	teleop_path.pop_front();
      teleop_path.clear();
      cout << "ardrone_cntrlr. planner path cleared.\n";
    }

}

int teleop_relay(const geometry_msgs::TwistConstPtr &msg)
{
#ifdef __TEST1_CNTRLR_
  teleop_mode=false;
#else
  teleop_mode=true;
#endif
  traj_track_mode=false;

  /*
  unsigned int path_size = planner_path.size();

  if(path_size>0)
    {
      for(unsigned int i=0; i<path_size; i++)
	planner_path.pop_front();
      planner_path.clear();
      cout << "ardrone_cntrlr. planner path cleared.\n";
    }
  else
    {
      cout << "ardrone_cntrlr: No planner path to clear.\n";
    }
  */

#ifdef __DEBUG_LEVEL_2
  cout << "teleop_relay. cmd_vel msg: \n";
  cout << "msg->linear.(x,y,z): " << msg->linear.x << "," << msg->linear.y << "," << msg->linear.z << "\n";
  cout << "msg->angular.(x,y,z): " << msg->angular.x << "," << msg->angular.y << "," << msg->angular.z << "\n\n";
#endif
  
#ifdef __TEST1_CNTRLR_

  geometry_msgs::Twist cmdtwist;
  cmdtwist=(*msg);
  cmdvel_pub.publish(cmdtwist);

#else

  if(fabs(msg->linear.z)>0)
    {
      geometry_msgs::Twist cmdtwist;
      cmdtwist=(*msg);
      cmdvel_pub.publish(cmdtwist);
      return 1;
    }
      
  geometry_msgs::PoseStamped temppose;
  temppose.pose.position.x=ardrone_lclzrx;
  temppose.pose.position.y=ardrone_lclzry;
  btVector3 rotaxis(0,0,1);
  btQuaternion qrot(rotaxis,ardrone_lclzrthz);
  temppose.pose.orientation.x=qrot.x();
  temppose.pose.orientation.y = qrot.y();
  temppose.pose.orientation.z = qrot.z();
  temppose.pose.orientation.w = qrot.w();
  
  if(fabs(msg->linear.x)>0)
    {
      //push back a pose offset by +/-5cm in the x direction in the teleop_path queue
      if((msg->linear.x)>0)
	temppose.pose.position.x+=0.05;
      else
	temppose.pose.position.x-=0.05;
    }
  if(fabs(msg->linear.y)>0)
    {
      //push back a pose offset by +/-5cm in the y direction in the teleop_path queue
      if((msg->linear.y)>0)
	temppose.pose.position.y+=0.05;
      else
	temppose.pose.position.y-=0.05;
    }
  if(fabs(msg->angular.z)>0)
    {
      //push back a pose offset by +/-5deg in the z direction in the teleop_path queue
      double rthz = (5/180.0*PI);
      double newdrone_thz;
      if((msg->angular.z)>0)
	newdrone_thz = (ardrone_lclzrthz+rthz);
      else
	newdrone_thz = (ardrone_lclzrthz-rthz);
      
      btVector3 rotaxis(0,0,1);
      btQuaternion qrot(rotaxis,newdrone_thz);
      temppose.pose.orientation.x=qrot.x();
      temppose.pose.orientation.y = qrot.y();
      temppose.pose.orientation.z = qrot.z();
      temppose.pose.orientation.w = qrot.w();
    }
  
  teleop_path.push_back(temppose);
#endif

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
  teleop_path.clear();

  //PID controller variables
  ex=0.0;
  ey=0.0;
  ethz=0.0;
  ex_prev=0.0;
  ey_prev=0.0;
  ethz_prev=0.0;
  eix=0.0;
  eiy=0.0;
  eithz=0.0;
  kpx=0.01;
  kpy=0.01;
  kpthz=0.01;
  kvx=0.06;
  kvy=0.06;
  kvthz=0.06;
  kix=0.09;
  kiy=0.09;
  kithz=0.09;
  
  lclzr_callback_occurred=false;

  ardrone_lclzrdthz=0.0;
  ardrone_lclzrx=0.0;
  ardrone_lclzry=0.0;
  ardrone_lclzrthz=0.0;

  cntr=0;
  traj_track_mode=false;
  teleop_mode=false;

  ros::Subscriber lclzr_sub = n.subscribe("/ardrone_mean_state",1,&lclzr_callback);
  ros::Subscriber plan_sub = n.subscribe("/plan",1,&path_callback);
  cmdvel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel",0);

  ros::Subscriber traj_tracking_sub = n.subscribe("/traj_tracking_start",1,&traj_tracking_callback);
  ros::Subscriber stop_tracking_sub = n.subscribe("/stop_tracking",1,&stop_tracking_callback);
  ros::Subscriber cmdvel_sub = n.subscribe("/ardrone_cntrlr/cmd_vel",1,&cmd_vel_callback);

  ros::Rate loop_rate(40);

  while (ros::ok())
    {
      track_trajectory();
      ros::spinOnce();
      loop_rate.sleep();

    }

  return 0;

}
