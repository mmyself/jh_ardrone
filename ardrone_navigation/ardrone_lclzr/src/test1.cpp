void ardrone_particle_filter::perform_motion_update(ardrone_state& drone_state)
{
  //For each particle in vec_pf perform a motion update
  //Compute the prior probability
  vector<ardrone_particle>::iterator it=vec_pf->begin();

  //standard deviation for theta_z(yaw) odometry reading  
  double thz_sd = fabs(drone_state.yaw_delta_mag*odom_rpy_error[2]);
  
  //standard deviation for lin_vel_x odometry reading
  double vx_sd = fabs(drone_state.b_lin_vel[0]*odom_linvel_error[0]);

  //standard deviation for lin_vel_y odometry reading
  double vy_sd = fabs(drone_state.b_lin_vel[1]*odom_linvel_error[1]);

  double gaussian_error_thz=0.0;
  double delta_thz_sample=0.0;
  double thz_sample=0.0;
  double cthz=0.0,sthz=0.0;
  double gaussian_error_vx=0.0;
  double vx_sample=0.0;
  double w_vx_sample=0.0;
  double gaussian_error_vy=0.0;
  double vy_sample=0.0;
  double w_vy_sample=0.0;
  double delta_x_sample=0.0;
  double delta_y_sample=0.0;
  double x_sample=0.0;
  double y_sample=0.0;

  for(;it<vec_pf->end();it++)
    {
      //compute thetaz sample
      gaussian_error_thz = pf_ran_gaussian(thz_sd);
      delta_thz_sample = (drone_state.drone_rpy_delta[2] - gaussian_error_thz);
      thz_sample = ((*it).drone_rpy[2] + delta_thz_sample);

      //cout << "thz_sample: " << thz_sample << "\n";

      //change thz_sample to {[0,+PI] U [0,-PI]}
      if(thz_sample>PI)
	{
	  thz_sample = (thz_sample-2*PI);
	}
      else if(thz_sample<-PI)
	{
	  thz_sample = (2*PI+thz_sample);
	}

      cthz = cos(thz_sample);
      sthz = sin(thz_sample);

      //compute lin_vel_x sample
      gaussian_error_vx = pf_ran_gaussian(vx_sd);
      vx_sample = (drone_state.b_lin_vel[0] - gaussian_error_vx);

      //compute lin_vel_y sample
      gaussian_error_vy = pf_ran_gaussian(vy_sd);
      vy_sample = (drone_state.b_lin_vel[1] - gaussian_error_vy);

      //compute linear velocities in the world frame (simple yaw rotation)
      w_vx_sample = (cthz*vx_sample - sthz*vy_sample);
      w_vy_sample = (+sthz*vx_sample + cthz*vy_sample);

      delta_x_sample = (drone_state.dt*((w_vx_sample + (*it).w_linvel[0])/2.0));
      delta_y_sample = (drone_state.dt*((w_vy_sample + (*it).w_linvel[1])/2.0));      
      
      x_sample = ((*it).w_pos[0] + delta_x_sample);
      y_sample = ((*it).w_pos[1] + delta_y_sample);

      (*it).w_linvel[0] = w_vx_sample;
      (*it).w_linvel[1] = w_vy_sample;
      (*it).w_pos[0] = x_sample;
      (*it).w_pos[1] = y_sample;
      (*it).drone_rpy[2] = thz_sample;
      (*it).seqcnt = drone_state.seqcnt;
    }

}
