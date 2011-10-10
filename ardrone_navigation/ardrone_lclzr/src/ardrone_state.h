#ifndef __ARDRONE_STATE_H
#define __ARDRONE_STATE_H

#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>

//#include "motion_sample.h"

class ardrone_state
{
 public:
  //Drone position in world coordinates
  btVector3 w_pos;

  //Drone linear velocity in Drone body coordinates
  btVector3 b_lin_vel;

  //Drone linear velocity in world coordinates
  btVector3 w_lin_vel;

  //Drone orientation - 
  //This gives the XYZ Euler angle rotation matrix as per Craig's book pg 442 Appendix B
  //phi or drone_rpy[0] is the roll about the body fixed X axis
  //theta or drone_rpy[1] is the pitch about the body fixed Y axis
  //psi or drone_rpy[2] is the yaw about the body fixed Z axis
  btVector3 drone_rpy;

  btVector3 raw_rpy_prev;
  
  double yaw_delta_mag;

  //Difference in previous and current odom readings for rpy orientation
  btVector3 drone_rpy_delta;

  

  //Time stamp from the drone in microseconds
  double tm;

  //delta_t in seconds
  double dt;

  //Initial offset of drone position from origin of world frame, represented in world frame
  btVector3 w_init_pos_offset;

  //Initial orientation offset of drone from world frame, in rpy euler angles
  btVector3 init_rpy_offset;

  //ARDrone flight status
  //0 - on the ground
  //1 - in the air
  //2 - battery out
  //3 - crashed
  int flight_status;

  //local sequence counter
  unsigned int seqcnt;

  ardrone_state()
    {
      tm=0.0;
      dt=0.0;
      flight_status=0;
      seqcnt=0;
      yaw_delta_mag=0.0;
      for(int i=0; i<3; i++)
	{
	  w_pos[i]=w_lin_vel[i]=drone_rpy[i]=w_init_pos_offset[i]=init_rpy_offset[i]=0.0;
	  drone_rpy_delta[i]=0.0;
	  b_lin_vel[i]=0.0;
	  raw_rpy_prev[i]=0.0;
	}
    }

 ardrone_state(const btVector3& pos, const btVector3& rpy,		\
	       const btVector3& init_pos, const btVector3& init_rpy,	\
	       double start_tm=0.0, int f_status=0)
    :tm(start_tm), flight_status(f_status)
  {
    dt=0.0;
    seqcnt=0;
    yaw_delta_mag=0.0;
    for(int i=0; i<3; i++)
      {
	w_pos[i]=pos[i];
	drone_rpy[i]=rpy[i];
	w_init_pos_offset[i]=init_pos[i];
	init_rpy_offset[i]=init_rpy[i];
	w_lin_vel[i]=0.0;
	drone_rpy_delta[i]=0.0;
	b_lin_vel[i]=0.0;
	raw_rpy_prev[i]=rpy[i];
      }
  }

  void state_initialize(const btVector3& pos, const btVector3& rpy, \
			const btVector3& init_pos, const btVector3& init_rpy, \
			double start_tm=0.0, int f_status=0)
  {
    dt=0.0;
    tm=start_tm;
    flight_status=f_status;
    yaw_delta_mag=0.0;
    for(int i=0; i<3; i++)
      {
	w_pos[i]=pos[i];
	drone_rpy[i]=rpy[i];
	w_init_pos_offset[i]=init_pos[i];
	init_rpy_offset[i]=init_rpy[i];
	w_lin_vel[i]=0.0;
	drone_rpy_delta[i]=0.0;	
	b_lin_vel[i]=0.0;
	raw_rpy_prev[i]=rpy[i];
      }
  }

  void initialize_pose(const btVector3& init_pos, const btVector3& init_rpy)
  {
    yaw_delta_mag=0.0;
    for(int i=0; i<3; i++)
      {
	w_init_pos_offset[i]=init_pos[i];
	init_rpy_offset[i]=init_rpy[i];
	raw_rpy_prev[i]=0.0;
	w_pos[i]=0.0;
	drone_rpy_delta[i]=0.0;	
	drone_rpy[i]=0.0;
	w_lin_vel[i]=0.0;
	b_lin_vel[i]=0.0;
      }
  }
};

#endif





