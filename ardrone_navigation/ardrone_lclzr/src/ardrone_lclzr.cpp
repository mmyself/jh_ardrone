/* 
This software is meant to localize an ARDrone
*/

#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <LinearMath/btQuaternion.h>

#include <ardrone_brown2/Navdata.h>
#include <ardrone_lclzr/drone_transform.h>

#include "ardrone_state.h"
#include <ar_recog/Tags.h>
#include <ar_recog/Tag.h>
#include "tagserver.h"
#include "ardrone_pf.h"
#include "ardrone_lclzr/ardrone_mean_state.h"

#ifndef PI
#define PI 3.1415926535
#endif

#define _REDUCED_ROTATION 1

ros::Publisher dronetransform_pub;
//tf::TransformBroadcaster *br;
ros::Publisher particles_visualization;
ros::Publisher pub_mean_state;
ardrone_state drone_state;
ardrone_particle_filter ardrone_pfilter;
vector<tagPose> tagposes;
unsigned int vizpub_count;

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

// This gives the XYZ Euler angle rotation matrix as per Craig's book pg 442 Appendix B
// phi is the roll about the body fixed X axis
// theta is the pitch about the body fixed Y axis
// psi is the yaw about the body fixed Z axis
// Input angles are in radians
void getBodyFixedEulerRPYRotationMatrix(double rphi, double rth, double rpsi, btMatrix3x3 &retMat)
{
  double xx,xy,xz,yx,yy,yz,zx,zy,zz;
  double cphi = cos(rphi);
  double cth = cos(rth);
  double cpsi = cos(rpsi);
  double sphi = sin(rphi);
  double sth = sin(rth);
  double spsi = sin(rpsi);
  double sphisth = sphi*sth;
  double cphispsi = cphi*spsi;
  double cphicpsi = cphi*cpsi;

  xx = cth*cpsi;
  xy = -cth*spsi;
  xz = sth;
  yx = sphisth*cpsi+cphispsi;
  yy = -sphisth*spsi+cphicpsi;
  yz = -sphi*cth;
  zx = -cphicpsi*sth+sphi*spsi;
  zy = cphispsi*sth+sphi*cpsi;
  zz = cphi*cth;
  
  retMat.setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
}

// This returns a yaw(z axis) rotation by rpsi radians
void getYawRotationMatrix(double rpsi, btMatrix3x3 &retMat)
{
  double xx,xy,xz,yx,yy,yz,zx,zy,zz;
  double cpsi = cos(rpsi);
  double spsi = sin(rpsi);

  xx = cpsi;
  xy = -spsi;
  xz = 0.0;
  yx = spsi;
  yy = cpsi;
  yz = 0.0;
  zx = 0.0;
  zy = 0.0;
  zz = 1.0;
  
  retMat.setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
}

// This returns a Pitch(Y axis) rotation by rth radians
void getPitchRotationMatrix(double rth, btMatrix3x3 &retMat)
{
  double xx,xy,xz,yx,yy,yz,zx,zy,zz;
  double cth = cos(rth);
  double sth = sin(rth);

  xx = cth;
  xy = 0.0;
  xz = sth;
  yx = 0.0;
  yy = 1.0;
  yz = 0.0;
  zx = -sth;
  zy = 0.0;
  zz = cth;
  
  retMat.setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
}

// This returns a Roll(X axis) rotation by rphi radians
void getRollRotationMatrix(double rphi, btMatrix3x3 &retMat)
{
  double xx,xy,xz,yx,yy,yz,zx,zy,zz;
  double cphi = cos(rphi);
  double sphi = sin(rphi);

  xx = 1.0;
  xy = 0.0;
  xz = 0.0;
  yx = 0.0;
  yy = cphi;
  yz = -sphi;
  zx = 0.0;
  zy = sphi;
  zz = cphi;
  
  retMat.setValue(xx, xy, xz, yx, yy, yz, zx, zy, zz);
}

void getYawPitchRollRotationMatrix(double rpsi, double rth, double rphi, btMatrix3x3 &retMat)
{
  btMatrix3x3 rpsiRot, rthRot, rphiRot;
  getYawRotationMatrix(rpsi, rpsiRot);
  getPitchRotationMatrix(rth, rthRot);
  getRollRotationMatrix(rphi, rphiRot);
  retMat = (rpsiRot*(rthRot*rphiRot));
}

void publish_particles()
{
  visualization_msgs::MarkerArray marray;
  vector<ardrone_particle>::iterator pf_it = ardrone_pfilter.vec_pf->begin();
  int i=0;
  btVector3 rotaxis(0,0,1);

  btVector3 avg_pos(0,0,0);
  btVector3 avg_rpy(0,0,0);

  ardrone_lclzr::ardrone_mean_state mean_state;

  for(;pf_it<ardrone_pfilter.vec_pf->end();pf_it++)
    {
      visualization_msgs::Marker tm;
      tm.header.frame_id = "/map";
      tm.header.stamp = ros::Time();
      tm.ns = "ardrone_particle";
      tm.id = i;
      tm.type = visualization_msgs::Marker::ARROW;
      tm.action = visualization_msgs::Marker::ADD;
      tm.pose.position.x = (*pf_it).w_pos[0];
      tm.pose.position.y = (*pf_it).w_pos[1];
      tm.pose.position.z = 1;
      btQuaternion qrot(rotaxis,((*pf_it).drone_rpy[2]));
      tm.pose.orientation.x = qrot.x();
      tm.pose.orientation.y = qrot.y();
      tm.pose.orientation.z = qrot.z();
      tm.pose.orientation.w = qrot.w();
      tm.scale.x = 1.20;
      tm.scale.y = 1.20;
      tm.scale.z = 1.20;
      tm.color.a = 1.0;
      tm.color.r = 0.0;
      tm.color.g = 1.0;
      tm.color.b = 0.0;
      marray.markers.push_back(tm);
      avg_pos[0]+=(*pf_it).w_pos[0];
      avg_pos[1]+=(*pf_it).w_pos[1];
      avg_rpy[2]+=(*pf_it).drone_rpy[2];
      /*
      if(((*pf_it).drone_rpy[2]<0)&&(fabs((*pf_it).drone_rpy[2])>(PI/2)))
	{
	  avg_rpy[2]+=((2*PI)+(*pf_it).drone_rpy[2]);
	}
      else 
	{
	  avg_rpy[2]+=(*pf_it).drone_rpy[2];
	}
      */
      i++;
    }

  avg_pos[0]=(avg_pos[0]/i);
  avg_pos[1]=(avg_pos[1]/i);
  avg_rpy[2]=(avg_rpy[2]/i);

  double avg_rpy_deg = (avg_rpy[2]/PI*180);

  cout << "publish_particles: \n";
  cout << "avg_pos[0]: " << avg_pos[0] << "\n";
  cout << "avg_pos[1]: " << avg_pos[1] << "\n";
  cout << "avg_rpy[2]: " << avg_rpy[2] << "\n";
  cout << "avg_rpy_deg[2]: " << avg_rpy_deg << "\n\n";


  mean_state.x = avg_pos[0];
  mean_state.y = avg_pos[1];
  mean_state.thz = avg_rpy[2];
  mean_state.vx_droneframe = drone_state.b_lin_vel[0];
  mean_state.vy_droneframe = drone_state.b_lin_vel[1];
  mean_state.dt = drone_state.dt;

  pub_mean_state.publish(mean_state);
  particles_visualization.publish(marray);
}

void navdataCallback(const ardrone_brown2::Navdata::ConstPtr& navmsg)
{
  std_msgs::String msg;
  std::stringstream ss;

  ardrone_lclzr::drone_transform trmsg;
  
  if (drone_state.seqcnt==0)
    {
      drone_state.seqcnt++;
      ss << "First update to drone state.\n";
      
      //converting orientation data to radians
      double phi = navmsg->rotX;
      double theta = navmsg->rotY;
      double psi = navmsg->rotZ;
      double rphi = phi/180.0*PI;
      double rth = theta/180.0*PI;
      double rpsi = psi/180.0*PI;

      drone_state.drone_rpy[0]=0.0;
      drone_state.drone_rpy[1]=0.0;
      drone_state.drone_rpy[2]=0.0;
      drone_state.init_rpy_offset[0]=rphi;
      drone_state.init_rpy_offset[1]=rth;
      drone_state.init_rpy_offset[2]=rpsi;      
      drone_state.tm=navmsg->tm;
      drone_state.dt=0.0;
      drone_state.raw_rpy_prev[0]=0.0;
      drone_state.raw_rpy_prev[1]=0.0;
      drone_state.raw_rpy_prev[2]=0.0;
      drone_state.yaw_delta_mag=0.0;
      for(int i=0; i<3; i++)
	{
	  drone_state.w_lin_vel[i]=0.0;
	  drone_state.b_lin_vel[i]=0.0;
	}
    }
  else
    {
      double dt = navmsg->tm - drone_state.tm;
      if(dt<0)
	{
	  // navdata message received out of sync, so ignore it
	  ss << "\n\nnavdata message received with out of sync time stamp.\n";
	  ss << "dt(usec): " << dt << "\n\n";
	  msg.data=ss.str();
	  ROS_INFO("%s", msg.data.c_str());
	}
      /*
      else if(abs(dt)<100) //dt is in microseconds
	{
	  // the data points are too close to each other, so ignore it
	  ss << "Rejecting data point, since dt is too small, dt(usec): " << dt << "\n\n";
	  msg.data=ss.str();
	  ROS_INFO("%s", msg.data.c_str());
	}
      */
      else
	{
	  drone_state.seqcnt++;
	  
	  //convert dt from microseconds to seconds
	  dt=dt/1000000.0;
	  drone_state.dt=dt;
	  
	  double phi = navmsg->rotX;
	  double theta = navmsg->rotY;
	  double psi = navmsg->rotZ;
	  //convert orientation data to radians
	  double tempphi = phi/180.0*PI;
	  double tempth = theta/180.0*PI;
	  double temppsi = psi/180.0*PI;
	  //double rtempphi=tempphi, rtempth=tempth, rtemppsi=temppsi;

	  //correct orientation with initial offset
	  double rphi = (tempphi-drone_state.init_rpy_offset[0]);
	  double rth = (tempth-drone_state.init_rpy_offset[1]);
	  double rpsi = (temppsi-drone_state.init_rpy_offset[2]);

	  drone_state.drone_rpy_delta[0] = (rphi - drone_state.drone_rpy[0]);
	  drone_state.drone_rpy_delta[1] = (rth - drone_state.drone_rpy[1]);
	  drone_state.drone_rpy_delta[2] = (rpsi - drone_state.drone_rpy[2]);

	  bool special_delta=false;

	  //handle the case when the raw yaw angle odometry changes sign
	  //case 1. it goes from the first quadrant to second quadrant and vice versa
	  if((drone_state.raw_rpy_prev[2]<0)&&(temppsi>0))
	    {
	      if((fabs(drone_state.raw_rpy_prev[2])<(PI/2.0))&&(fabs(temppsi)<(PI/2.0)))
		{
		  drone_state.yaw_delta_mag = (temppsi - drone_state.raw_rpy_prev[2]);
		  special_delta=true;
		}
	    }
	  else if((drone_state.raw_rpy_prev[2]>0)&&(temppsi<0))
	    {
	      if((fabs(drone_state.raw_rpy_prev[2])<(PI/2.0))&&(fabs(temppsi)<(PI/2.0)))
		{
		  drone_state.yaw_delta_mag = (temppsi - drone_state.raw_rpy_prev[2]);
		  special_delta=true;
		}
	    }

	  //case 2. it goes from the fourth quadrant to third quadrant and vice versa
	  if((drone_state.raw_rpy_prev[2]<0)&&(temppsi>0))
	    {
	      if((fabs(drone_state.raw_rpy_prev[2])>(PI/2.0))&&(fabs(temppsi)>(PI/2.0)))
		{
		  drone_state.yaw_delta_mag = (temppsi - drone_state.raw_rpy_prev[2]);
		  drone_state.yaw_delta_mag = (2*PI - drone_state.yaw_delta_mag);
		  special_delta=true;
		}
	    }
	  else if((drone_state.raw_rpy_prev[2]>0)&&(temppsi<0))
	    {
	      if((fabs(drone_state.raw_rpy_prev[2])>(PI/2.0))&&(fabs(temppsi)>(PI/2.0)))
		{
		  drone_state.yaw_delta_mag = (temppsi - drone_state.raw_rpy_prev[2]);
		  drone_state.yaw_delta_mag = (2*PI + drone_state.yaw_delta_mag);
		  special_delta=true;
		}
	    }
	  
	  if(!special_delta)
	    {
	      drone_state.yaw_delta_mag = drone_state.drone_rpy_delta[2];
	    }

	  drone_state.raw_rpy_prev[0]=tempphi;
	  drone_state.raw_rpy_prev[1]=tempth;
	  drone_state.raw_rpy_prev[2]=temppsi;

	  double cphi = cos(rphi);
	  double cth = cos(rth);
	  double cpsi = cos(rpsi);
	  
	  double sphi = sin(rphi);
	  double sth = sin(rth);
	  double spsi = sin(rpsi);

	  drone_state.b_lin_vel[0]=(navmsg->vx/1000.0);
	  drone_state.b_lin_vel[1]=(navmsg->vy/1000.0);
	  drone_state.b_lin_vel[2]=(navmsg->vz/1000.0);

#ifndef _REDUCED_ROTATION
	  btMatrix3x3 w_drone_rotation;
	  getBodyFixedEulerRPYRotationMatrix(rphi,rth,rpsi,w_drone_rotation);
	  
	  double rphidot = (rphi - drone_state.drone_rpy[0])/dt;
	  double rthdot = (rth - drone_state.drone_rpy[1])/dt;
	  double rpsidot = (rpsi - drone_state.drone_rpy[2])/dt;
	  
	  // angular velocity of the drone in the world frame
	  btVector3 w_ang_vel;
	  w_ang_vel[0] = rphidot+rpsidot*sth;
	  w_ang_vel[1] = rthdot*cphi-rpsidot*sphi*cth;
	  w_ang_vel[2] = rthdot*sphi+rpsidot*cphi*cth;
	  
	  btMatrix3x3 ang_vel_ssmat;
	  getSkewSymMatrix(w_ang_vel,ang_vel_ssmat);
	  
	  btVector3 drone_vel((navmsg->vx/1000.0),(navmsg->vy/1000.0),(navmsg->vz/1000.0));
	  //btVector3 w_drone_vel = w_drone_rotation*drone_vel + ang_vel_ssmat*drone_state.w_pos;
	  btVector3 w_drone_vel = w_drone_rotation*drone_vel;
#else
	  ss << "\n_REDUCED_ROTATION is defined.";
	  btMatrix3x3 w_drone_rotation;
	  getYawRotationMatrix(rpsi,w_drone_rotation);
	  ////getYawPitchRollRotationMatrix(rpsi,rth,rphi,w_drone_rotation);
	  ////getBodyFixedEulerRPYRotationMatrix(rphi,rth,rpsi,w_drone_rotation);
	  ////double rpsidot = (rpsi - drone_state.drone_rpy[2])/dt;
	  
	  // angular velocity of the drone in the world frame
	  ////btVector3 w_ang_vel;
	  ////w_ang_vel[0] = 0.0;
	  ////w_ang_vel[1] = 0.0;
	  ////w_ang_vel[2] = rpsidot;
	  
	  ////btMatrix3x3 ang_vel_ssmat;
	  ////getSkewSymMatrix(w_ang_vel,ang_vel_ssmat);
	  
	  btVector3 drone_vel((navmsg->vx/1000.0),(navmsg->vy/1000.0),(navmsg->vz/1000.0));
	  ////btVector3 w_drone_vel = w_drone_rotation*drone_vel + ang_vel_ssmat*drone_state.w_pos;
	  btVector3 w_drone_vel = w_drone_rotation*drone_vel;
#endif

	  btVector3 w_avg_vel = ((drone_state.w_lin_vel + w_drone_vel)/2.0);
	  btVector3 w_drone_disp = dt*w_avg_vel;
	  
	  drone_state.w_pos += w_drone_disp;
	  drone_state.drone_rpy[0]=rphi;
	  drone_state.drone_rpy[1]=rth;
	  drone_state.drone_rpy[2]=rpsi;
	  for(int i=0; i<3; i++)
	    drone_state.w_lin_vel[i]=w_drone_vel[i];
	  drone_state.tm=navmsg->tm;

	  ardrone_pfilter.perform_motion_update(drone_state);
	  vizpub_count++;

	  //if(vizpub_count%20==0)
	  publish_particles();

#ifdef __PF_TEST
	  ardrone_pfilter.perform_motion_update2(drone_state);
#endif

	  /*
	  trmsg.xx = w_drone_rotation[0][0];
	  trmsg.xy = w_drone_rotation[0][1];
	  trmsg.xz = w_drone_rotation[0][2];
	  trmsg.yx = w_drone_rotation[1][0];
	  trmsg.yy = w_drone_rotation[1][1];
	  trmsg.yz = w_drone_rotation[1][2];
	  trmsg.zx = w_drone_rotation[2][0];
	  trmsg.zy = w_drone_rotation[2][1];
	  trmsg.zz = w_drone_rotation[2][2];
	  
	  dronetransform_pub.publish(trmsg);
	  */
	  ////tf::Transform transform(w_drone_rotation, btVector3(0.0,0.0,0.0));
	  //tf::Transform transform(btMatrix3x3(xx, xy, xz, yx, yy, yz, zx, zy, zz), btVector3(0.0,0.0,0.0));
	  //transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
	  //transform.setRotation(tf::Quaternion(navmsg->rotX, navmsg->rotY, navmsg->rotZ));
	  ////br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ardrone"));

	  /*
	  ss << "\nARDrone Lclzr, navdataCallback, seqcnt: " << drone_state.seqcnt << "\n";
	  ss << "battery percentage: " << navmsg->batteryPercent << "\n";
	  ss << "left/right tilt: " << navmsg->rotX << "\n";
	  ss << "forward/backward tilt: " << navmsg->rotY << "\n";
	  ss << "orientation, Zrot: " << navmsg->rotZ << "\n";
	  ss << "altitude in cm: " << navmsg->altd << "\n";
	  ss << "linear velocity, x: " << navmsg->vx << "\n";
	  ss << "linear velocity, y: " << navmsg->vy << "\n";
	  ss << "linear velocity, z: " << navmsg->vz << "\n";
	  ss << "timestamp, tm: " << navmsg->tm << "\n";
	  ss << "dt(sec): " << dt << "\n";
	  ss << "drone position, x: " << drone_state.w_pos[0] << "\n";
	  ss << "drone position, y: " << drone_state.w_pos[1] << "\n";
	  ss << "drone position, z: " << drone_state.w_pos[2] << "\n";
	  ss << "w_velocity, x: " << w_drone_vel[0] << "\n";
	  ss << "w_velocity, y: " << w_drone_vel[1] << "\n";
	  ss << "w_velocity, z: " << w_drone_vel[2] << "\n";
	  */
	  /*
	  ss << "\ndrone position, x: " << drone_state.w_pos[0] << "\n";
	  ss << "drone position, y: " << drone_state.w_pos[1] << "\n";
	  ss << "drone position, z: " << drone_state.w_pos[2] << "\n";
	  ss << "orientation, Zrot: " << navmsg->rotZ << "\n";
	  */
	}
    }
  /*
  msg.data=ss.str();      
  ROS_INFO("%s", msg.data.c_str());
  */

}

void tagscallback(const ar_recog::Tags::ConstPtr& tagsmsg)
{
  ardrone_pfilter.perform_measurement_update(tagsmsg,tagposes);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "ardrone_lclzr");
  /*
  drone_state.state_initialize(btVector3(0.0,0.0,0.0),btVector3(0.0,0.0,0.0), \
			       btVector3(0.0,0.0,0.0),btVector3(0.0,0.0,0.0),\
			       btVector3(0.0,0.0,0.0),btVector3(0.0,0.0,0.0),\
			       0.0,0);
  */
  //br = new tf::TransformBroadcaster();
  drone_state.initialize_pose(btVector3(6.85,43.05,0.0),btVector3(0.0,0.0,0.0));

  ardrone_pfilter.initialize_particles(btVector3(6.85,43.05,0.0),btVector3(0.0,0.0,0.0));

  ros::NodeHandle n;

  vizpub_count=0;

  //initialize tagposes
  tagPose tag1=tagPose(0,9.21,42.65,-PI/2);
  tagposes.push_back(tag1);
  
  tagPose tag2=tagPose(1,13.33,41.8,-PI/2);
  tagposes.push_back(tag2);

  /*
  tagPose tag2=tagPose(1,5.85,43.05,-PI/2);
  tagposes.push_back(tag2);
  */

  tagPose tag3=tagPose(6,17.8514,41.773,-PI/2);
  tagposes.push_back(tag3);

  //tagPose tag2=tagPose(1,1,1,45);
  //tagposes.push_back(tag2);

  ros::Subscriber navdata_sub = n.subscribe("/ardrone/navdata", 0, &navdataCallback);
  //ros::Publisher dronestate_pub = n.advertise<std_msgs::String>("/ardrone_lclzr/dronestate",100);

  ros::Subscriber tagsmsg_sub = n.subscribe("tags", 0, &tagscallback);

  //particles_pub = n.advertise<ardrone_lclzr::ardrone_pf_particles>("/ardrone/pf_particles",1);

  particles_visualization = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array",0);

  pub_mean_state = n.advertise<ardrone_lclzr::ardrone_mean_state>("ardrone_mean_state",0);
  //dronetransform_pub = n.advertise<ardrone_lclzr::drone_transform>("/ardrone/drone_transform",100);

  ros::Rate loop_rate(60);

  //int count=0;
  while (ros::ok())
    {

      //dronestate_pub.publish(msg);
      ros::spinOnce();

      loop_rate.sleep();
      //++count;
    }

  return 0;
}


